/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common infrastructure for supporing dma-buf in the I/O path.
 *
 * Copyright (C) 2026 Pavel Begunkov <asml.silence@gmail.com>
 */
#include <linux/dma-buf-io.h>
#include <linux/dma-resv.h>
#include <linux/file.h>

static const char *dma_buf_io_fence_drv_name(struct dma_fence *fence)
{
	return "dma-buf-io-ctx";
}

static const char *dma_buf_io_fence_timeline_name(struct dma_fence *fence)
{
	return "dma-buf-io-ctx";
}

const struct dma_fence_ops dma_buf_io_fence_ops = {
	.get_driver_name = dma_buf_io_fence_drv_name,
	.get_timeline_name = dma_buf_io_fence_timeline_name,
};

static void dma_buf_io_ctx_destroy_work(struct work_struct *work)
{
	struct dma_buf_io_ctx *ctx = container_of(work, struct dma_buf_io_ctx,
						  destroy_work);
	struct file *file = ctx->file;

	if (WARN_ON_ONCE(refcount_read(&ctx->refs)))
		return;

	ctx->dev_ops->release(ctx);
	dma_buf_put(ctx->dmabuf);
	kfree(ctx);
	fput(file);
}

/*
 * Drop a ctx reference, deferring the final free to a worker. Callers may be
 * holding dma_resv, which ctx->dev_ops->release() -> dma_buf_detach() takes
 * as well, so the free can never run inline from here.
 */
static void dma_buf_io_ctx_put(struct dma_buf_io_ctx *ctx)
{
	if (refcount_dec_and_test(&ctx->refs))
		queue_work(system_wq, &ctx->destroy_work);
}

static void dma_buf_io_map_free_rcu(struct rcu_head *rcu)
{
	struct dma_buf_io_map *map = container_of(rcu, struct dma_buf_io_map, rcu);
	struct dma_buf_io_ctx *ctx = map->ctx;

	percpu_ref_exit(&map->active);
	kfree(map);
	dma_buf_io_ctx_put(ctx);
}

void __dma_buf_io_map_free(struct kref *kref)
{
	struct dma_buf_io_map *map = container_of(kref, struct dma_buf_io_map, refs);

	call_rcu(&map->rcu, dma_buf_io_map_free_rcu);
}
EXPORT_SYMBOL_NS_GPL(__dma_buf_io_map_free, "DMA_BUF");

static void dma_buf_io_map_release_work(struct work_struct *work)
{
	struct dma_buf_io_map *map = container_of(work, struct dma_buf_io_map,
						  release_work);
	struct dma_fence *fence = map->fence;
	struct dma_buf_io_ctx *ctx = map->ctx;
	struct dma_buf *dmabuf = ctx->dmabuf;

	dma_resv_lock(dmabuf->resv, NULL);
	ctx->dev_ops->unmap(ctx, map);
	dma_resv_unlock(dmabuf->resv);

	dma_fence_put(fence);
	kref_put(&map->refs, __dma_buf_io_map_free);
}

static void dma_buf_io_map_active_release(struct percpu_ref *ref)
{
	struct dma_buf_io_map *map = container_of(ref, struct dma_buf_io_map, active);

	if (map->release_mode == DMA_BUF_IO_RELEASE_SYNC) {
		complete(&map->drain);
		return;
	}

	dma_fence_signal(map->fence);
	INIT_WORK(&map->release_work, dma_buf_io_map_release_work);
	queue_work(system_wq, &map->release_work);
}

int dma_buf_io_init_map(struct dma_buf_io_ctx *ctx, struct dma_buf_io_map *map)
{
	struct dma_fence *fence = NULL;
	int ret;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return -ENOMEM;

	ret = percpu_ref_init(&map->active, dma_buf_io_map_active_release, 0,
			      GFP_KERNEL);
	if (ret) {
		kfree(fence);
		return ret;
	}

	map->fence = fence;
	map->ctx = ctx;
	kref_init(&map->refs);
	init_completion(&map->drain);

	/* Keep ctx alive until the map's final release. */
	refcount_inc(&ctx->refs);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(dma_buf_io_init_map, "DMA_BUF");

struct dma_buf_io_map *dma_buf_io_create_map(struct dma_buf_io_ctx *ctx)
{
	struct dma_buf *dmabuf = ctx->dmabuf;
	struct dma_buf_io_map *map;
	long ret;

	ret = dma_resv_lock_interruptible(dmabuf->resv, NULL);
	if (ret)
		return ERR_PTR(ret);

	map = dma_buf_io_get_map(ctx);
	if (map) {
		ret = 0;
		goto out;
	}

	/*
	 * ->map() will call dma_buf_map_attachment(), which requires the
	 * exporter's outstanding fences to have retired first. Wait here,
	 * under the reservation lock.
	 */
	ret = dma_resv_wait_timeout(dmabuf->resv, DMA_RESV_USAGE_KERNEL, true,
				    MAX_SCHEDULE_TIMEOUT);
	if (ret == 0)
		ret = -ETIMEDOUT;
	if (ret < 0)
		goto out;

	map = ctx->dev_ops->map(ctx);
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		goto out;
	}

	if (WARN_ON_ONCE(!map->seg_shift)) {
		ctx->dev_ops->unmap(ctx, map);
		/* The fence has not been initialized. */
		kfree(map->fence);
		percpu_ref_exit(&map->active);
		kref_put(&map->refs, __dma_buf_io_map_free);
		ret = -EFAULT;
		goto out;
	}

	/* Return a caller reference in addition to the publication reference. */
	kref_get(&map->refs);
	rcu_assign_pointer(ctx->map, map);
out:
	dma_resv_unlock(dmabuf->resv);
	if (ret < 0)
		return ERR_PTR(ret);
	return map;
}

static void dma_buf_io_drop_map(struct dma_buf_io_ctx *ctx)
{
	struct dma_buf *dmabuf = ctx->dmabuf;
	struct dma_buf_io_map *map;
	int ret;

	dma_resv_assert_held(dmabuf->resv);

	map = rcu_dereference_protected(ctx->map,
					dma_resv_held(dmabuf->resv));
	if (!map)
		return;
	rcu_assign_pointer(ctx->map, NULL);

	ret = dma_resv_reserve_fences(dmabuf->resv, 1);
	if (WARN_ON_ONCE(ret)) {
		/* No fence can be published, so drain synchronously. */
		map->release_mode = DMA_BUF_IO_RELEASE_SYNC;
		percpu_ref_kill(&map->active);
		wait_for_completion(&map->drain);

		ctx->dev_ops->unmap(ctx, map);
		kfree(map->fence);
		map->fence = NULL;
		kref_put(&map->refs, __dma_buf_io_map_free);
		return;
	}

	dma_fence_init(map->fence, &dma_buf_io_fence_ops, NULL, ctx->fence_ctx,
		       atomic_inc_return(&ctx->fence_seq));
	dma_resv_add_fence(dmabuf->resv, map->fence, DMA_RESV_USAGE_KERNEL);
	map->release_mode = DMA_BUF_IO_RELEASE_FENCED;
	/* Delay unmap until all active users are gone. */
	percpu_ref_kill(&map->active);
}

void dma_buf_io_invalidate_mappings(struct dma_buf_io_ctx *ctx)
{
	dma_buf_io_drop_map(ctx);
}
EXPORT_SYMBOL_NS_GPL(dma_buf_io_invalidate_mappings, "DMA_BUF");

static void dma_buf_io_ctx_release_work(struct work_struct *work)
{
	struct dma_buf_io_ctx *ctx = container_of(work, struct dma_buf_io_ctx,
						  release_work);
	struct dma_buf *dmabuf = ctx->dmabuf;
	long ret;

	dma_resv_lock(dmabuf->resv, NULL);
	/* Remove the last map, there should be no new ones going forward. */
	dma_buf_io_drop_map(ctx);
	dma_resv_unlock(dmabuf->resv);

	/* Wait until all maps are destroyed. */
	ret = dma_resv_wait_timeout(dmabuf->resv, DMA_RESV_USAGE_KERNEL,
				    false, MAX_SCHEDULE_TIMEOUT);

	if (WARN_ON_ONCE(ret <= 0))
		return;
	if (WARN_ON_ONCE(rcu_dereference_protected(ctx->map, true)))
		return;

	dma_buf_io_ctx_put(ctx);
}

void dma_buf_io_ctx_release(struct dma_buf_io_ctx *ctx)
{
	queue_work(system_wq, &ctx->release_work);
}

int dma_buf_io_ctx_create(struct file *file,
			   struct dma_buf_io_ctx *ctx,
			   struct dma_buf *dmabuf,
			   enum dma_data_direction dir)
{
	int ret;

	if (!file->f_op->init_dma_buf_io_ctx)
		return -EOPNOTSUPP;

	memset(ctx, 0, sizeof(*ctx));
	ctx->fence_ctx = dma_fence_context_alloc(1);
	ctx->dir = dir;
	ctx->dmabuf = dmabuf;
	refcount_set(&ctx->refs, 1);
	INIT_WORK(&ctx->release_work, dma_buf_io_ctx_release_work);
	INIT_WORK(&ctx->destroy_work, dma_buf_io_ctx_destroy_work);
	get_dma_buf(dmabuf);
	ctx->file = get_file(file);

	ret = file->f_op->init_dma_buf_io_ctx(file, ctx);
	if (ret) {
		memset(ctx, 0, sizeof(*ctx));
		dma_buf_put(dmabuf);
		fput(file);
		return ret;
	}

	if (WARN_ON_ONCE(!ctx->dev_ops ||
			 !ctx->dev_ops->map ||
			 !ctx->dev_ops->unmap ||
			 !ctx->dev_ops->release)) {
		if (ctx->dev_ops && ctx->dev_ops->release)
			ctx->dev_ops->release(ctx);
		memset(ctx, 0, sizeof(*ctx));
		dma_buf_put(dmabuf);
		fput(file);
		return -EINVAL;
	}

	return ret;
}
