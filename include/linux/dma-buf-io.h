/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __DMA_BUF_IO_H__
#define __DMA_BUF_IO_H__

#include <linux/completion.h>
#include <linux/dma-buf.h>
#include <linux/kref.h>
#include <linux/rcupdate.h>

struct dma_buf_io_ctx;
struct dma_buf_io_map;

enum dma_buf_io_release_mode {
	DMA_BUF_IO_RELEASE_FENCED,
	DMA_BUF_IO_RELEASE_SYNC,
};

struct dma_buf_io_ops {
	/*
	 * Create a new map for the given ctx. Called with the reservation
	 * lock held.
	 */
	struct dma_buf_io_map *(*map)(struct dma_buf_io_ctx *ctx);

	/*
	 * Clean up device specific parts of the @map. Called with the
	 * reservation lock held.
	 */
	void (*unmap)(struct dma_buf_io_ctx *ctx, struct dma_buf_io_map *map);

	/*
	 * The user tries to destroy the ctx. Release all device specific
	 * parts of the token.
	 */
	void (*release)(struct dma_buf_io_ctx *);
};

struct dma_buf_io_map {
	/* Keeps the map and map->ctx alive for software pointer holders. */
	struct kref			refs;

	/*
	 * Gates DMA-map access from hardware submission through completion.
	 * Holders must not perform reclaim-capable allocations.
	 * Killed on invalidation and drained before ->unmap().
	 */
	struct percpu_ref		active;

	/*
	 * DMA segment-length granularity, as a power-of-2 shift, for bounding
	 * segments while splitting opaque dma-buf bios. Set by ->map().
	 */
	unsigned			seg_shift;

	enum dma_buf_io_release_mode	release_mode;

	struct work_struct		release_work;
	struct completion		drain;
	struct dma_fence		*fence;
	struct dma_buf_io_ctx		*ctx;
	struct rcu_head			rcu;
};

struct dma_buf_io_ctx {
	struct dma_buf_io_map __rcu		*map;
	struct dma_buf				*dmabuf;
	enum dma_data_direction			dir;

	atomic_t				fence_seq;
	u64					fence_ctx;

	struct work_struct			release_work;
	struct work_struct			destroy_work;
	refcount_t				refs;

	/* Pinned until ->release() detaches the target device. */
	struct file				*file;

	void					*dev_priv;
	const struct dma_buf_io_ops		*dev_ops;
};

int dma_buf_io_ctx_create(struct file *file,
			   struct dma_buf_io_ctx *ctx,
			   struct dma_buf *dmabuf,
			   enum dma_data_direction dir);
void dma_buf_io_ctx_release(struct dma_buf_io_ctx *ctx);

struct dma_buf_io_map *dma_buf_io_create_map(struct dma_buf_io_ctx *ctx);

void __dma_buf_io_map_free(struct kref *kref);

static inline bool dma_buf_io_map_active_tryget(struct dma_buf_io_map *map)
{
	return percpu_ref_tryget_live(&map->active);
}

static inline void dma_buf_io_map_active_put(struct dma_buf_io_map *map)
{
	percpu_ref_put(&map->active);
}

/* This acquires only a software reference; active is driver-managed. */
static inline struct dma_buf_io_map *
dma_buf_io_get_map(struct dma_buf_io_ctx *ctx)
{
	struct dma_buf_io_map *map;

	guard(rcu)();

	map = rcu_dereference(ctx->map);
	if (unlikely(!map || !kref_get_unless_zero(&map->refs)))
		return NULL;

	return map;
}

static inline void dma_buf_io_map_drop(struct dma_buf_io_map *map)
{
	kref_put(&map->refs, __dma_buf_io_map_free);
}

/*
 * Device API
 */

void dma_buf_io_invalidate_mappings(struct dma_buf_io_ctx *ctx);
int dma_buf_io_init_map(struct dma_buf_io_ctx *ctx, struct dma_buf_io_map *map);

#endif
