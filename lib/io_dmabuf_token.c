/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common infrastructure for supporing dma-buf in the I/O path.
 *
 * Copyright (C) 2026 Pavel Begunkov <asml.silence@gmail.com>
 */
#include <linux/io_dmabuf_token.h>
#include <linux/dma-resv.h>

static void io_dmabuf_token_destroy_work(struct work_struct *work)
{
	struct io_dmabuf_token *token = container_of(work, struct io_dmabuf_token,
				  release_work);

	if (WARN_ON_ONCE(refcount_read(&token->refs)))
		return;

	token->dev_ops->release(token);
	dma_buf_put(token->dmabuf);
	kfree(token);
}


static void io_dmabuf_map_refs_release(struct percpu_ref *ref)
{
	struct io_dmabuf_map *map = container_of(ref, struct io_dmabuf_map, refs);

	/* Unblock io_dmabuf_drop_map() which is waiting for requests to drain. */
	complete(&map->drain);
}

int io_dmabuf_init_map(struct io_dmabuf_token *token, struct io_dmabuf_map *map)
{
	int ret;

	ret = percpu_ref_init(&map->refs, io_dmabuf_map_refs_release, 0, GFP_KERNEL);
	if (ret)
		return ret;

	init_completion(&map->drain);
	map->token = token;
	return 0;
}
EXPORT_SYMBOL_NS_GPL(io_dmabuf_init_map, "DMA_BUF");

struct io_dmabuf_map *io_dmabuf_create_map(struct io_dmabuf_token *token)
{
	struct dma_buf *dmabuf = token->dmabuf;
	struct io_dmabuf_map *map;
	long ret;

retry:
	/*
	 * Wait for any in-progress GPU moves to complete before mapping.
	 * dma_buf_map_attachment() requires the resv held and a stable BO
	 * placement; waiting here avoids taking the resv only to drop it
	 * immediately when a fence is still pending.
	 */
	ret = dma_resv_wait_timeout(dmabuf->resv, DMA_RESV_USAGE_WRITE,
				    true, MAX_SCHEDULE_TIMEOUT);
	if (!ret)
		ret = -EAGAIN;
	if (ret < 0)
		return ERR_PTR(ret);

	dma_resv_lock(dmabuf->resv, NULL);
	map = io_dmabuf_get_map(token);
	if (map) {
		ret = 0;
		goto out;
	}

	if (dma_resv_wait_timeout(dmabuf->resv, DMA_RESV_USAGE_WRITE,
				  true, 0) <= 0) {
		dma_resv_unlock(dmabuf->resv);
		goto retry;
	}

	map = token->dev_ops->map(token);
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		goto out;
	}

	percpu_ref_get(&map->refs);
	rcu_assign_pointer(token->map, map);
out:
	dma_resv_unlock(dmabuf->resv);
	if (ret < 0)
		return ERR_PTR(ret);
	return map;
}

static void io_dmabuf_drop_map(struct io_dmabuf_token *token)
{
	struct dma_buf *dmabuf = token->dmabuf;
	struct io_dmabuf_map *map;

	dma_resv_assert_held(dmabuf->resv);

	map = rcu_dereference_protected(token->map,
					dma_resv_held(dmabuf->resv));
	if (!map)
		return;
	rcu_assign_pointer(token->map, NULL);

	/*
	 * Drain all in-flight I/O requests that hold a reference to this map.
	 * percpu_ref_kill() drops the initial reference; once all per-request
	 * references are also dropped, io_dmabuf_map_refs_release() fires and
	 * signals map->drain.
	 *
	 * We hold the reservation lock (a sleeping ww_mutex) so sleeping here
	 * is allowed.  NVMe completions do not need the reservation lock, so
	 * they proceed freely and the wait is bounded by in-flight request
	 * lifetime.
	 */
	percpu_ref_kill(&map->refs);
	wait_for_completion(&map->drain);

	/*
	 * All DMA has stopped.  Call unmap with the reservation lock held as
	 * required by the dma-buf dynamic-attachment protocol.
	 */
	token->dev_ops->unmap(token, map);

	percpu_ref_exit(&map->refs);
	kfree(map);
}

void io_dmabuf_token_invalidate_mappings(struct io_dmabuf_token *token)
{
	io_dmabuf_drop_map(token);
}
EXPORT_SYMBOL_NS_GPL(io_dmabuf_token_invalidate_mappings, "DMA_BUF");

static void io_dmabuf_token_release_work(struct work_struct *work)
{
	struct io_dmabuf_token *token = container_of(work, struct io_dmabuf_token,
						  release_work);
	struct dma_buf *dmabuf = token->dmabuf;

	dma_resv_lock(dmabuf->resv, NULL);
	io_dmabuf_drop_map(token);
	dma_resv_unlock(dmabuf->resv);

	if (refcount_dec_and_test(&token->refs))
		io_dmabuf_token_destroy_work(&token->release_work);
}

void io_dmabuf_token_release(struct io_dmabuf_token *token)
{
	INIT_WORK(&token->release_work, io_dmabuf_token_release_work);
	queue_work(system_wq, &token->release_work);
}

int io_dmabuf_token_create(struct file *file,
			   struct io_dmabuf_token *token,
			   struct dma_buf *dmabuf,
			   enum dma_data_direction dir)
{
	int ret;

	if (!file->f_op->create_dmabuf_token)
		return -EOPNOTSUPP;

	memset(token, 0, sizeof(*token));
	token->dir = dir;
	token->dmabuf = dmabuf;
	refcount_set(&token->refs, 1);
	get_dma_buf(dmabuf);

	ret = file->f_op->create_dmabuf_token(file, token);
	if (ret) {
		memset(token, 0, sizeof(*token));
		dma_buf_put(dmabuf);
		return ret;
	}

	if (WARN_ON_ONCE(!token->dev_ops ||
			 !token->dev_ops->map ||
			 !token->dev_ops->unmap ||
			 !token->dev_ops->release))
		return -EINVAL;

	return ret;
}
