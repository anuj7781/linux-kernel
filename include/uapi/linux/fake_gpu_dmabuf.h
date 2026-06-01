/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_FAKE_GPU_DMABUF_H
#define _UAPI_LINUX_FAKE_GPU_DMABUF_H

/*
 * Fake GPU dma-buf exporter — ioctl interface.
 *
 * This driver mimics only the AMDGPU exporter behaviors that triggered
 * deadlocks in the io_uring dma-buf registered-buffer path:
 *
 *   BLOCK_MAP      Reproduce the original deadlock: map_dma_buf() blocks
 *                  while holding the resv, just as ttm_bo_validate() waits
 *                  on a GPU fence inside map_dma_buf() with the resv held.
 *
 *   INVAL_AND_HOLD Reproduce the async-unmap deadlock and validate the sync
 *                  fix: a kthread holds the resv, calls
 *                  dma_buf_invalidate_mappings(), then continues to hold the
 *                  resv (simulating KFD restore waiting for a VM fence after
 *                  triggering BO eviction).
 *
 *   HOLD_RESV      Reproduce KFD restore's drm_exec-held BO resv: a kthread
 *                  takes the resv and holds it until RELEASE_RESV.
 *
 * Device path: /dev/fake_gpu_dmabuf
 */

#include <linux/types.h>
#include <linux/ioctl.h>

struct fakegpu_create_arg {
	__u32 nr_pages;		/* in:  buffer size in 4 KiB pages   */
	__u32 handle;		/* out: opaque handle for all ioctls  */
	__s32 dmabuf_fd;	/* out: exported dma-buf fd           */
	__u32 pad;
};

#define FAKEGPU_IOC_MAGIC 'G'

/* Allocate a fake buffer; fills handle and dmabuf_fd. */
#define FAKEGPU_IOC_CREATE         _IOWR(FAKEGPU_IOC_MAGIC, 1, \
					 struct fakegpu_create_arg)

/* Release the kernel-side state for a handle (close dmabuf_fd separately). */
#define FAKEGPU_IOC_DESTROY        _IOW(FAKEGPU_IOC_MAGIC,  2, __u32)

/*
 * Make the next map_dma_buf() call block inside the exporter (resv still
 * held) until UNBLOCK_MAP.  Simulates ttm_bo_validate() waiting on a GPU
 * fence while the resv is held.
 */
#define FAKEGPU_IOC_BLOCK_MAP      _IOW(FAKEGPU_IOC_MAGIC,  3, __u32)

/* Unblock a blocked map_dma_buf(). */
#define FAKEGPU_IOC_UNBLOCK_MAP    _IOW(FAKEGPU_IOC_MAGIC,  4, __u32)

/*
 * Wait until map_dma_buf() has started blocking (returns once the
 * map_blocked counter is nonzero).  Useful for deterministic test ordering.
 */
#define FAKEGPU_IOC_WAIT_MAP_BLOCKED _IOW(FAKEGPU_IOC_MAGIC, 5, __u32)

/* Lock resv, call dma_buf_invalidate_mappings(), release immediately. */
#define FAKEGPU_IOC_INVALIDATE     _IOW(FAKEGPU_IOC_MAGIC,  6, __u32)

/*
 * Lock resv, call dma_buf_invalidate_mappings(), then hold the resv until
 * RELEASE_HOLD.  Returns once the resv is held and invalidation is done.
 *
 * With the old async-unmap design this deadlocks because the release worker
 * needs the resv to call unmap.  With the current synchronous fix the
 * invalidation drains inline and this ioctl returns without deadlock.
 */
#define FAKEGPU_IOC_INVAL_AND_HOLD  _IOW(FAKEGPU_IOC_MAGIC,  7, __u32)

/* Release the resv held by INVAL_AND_HOLD. */
#define FAKEGPU_IOC_RELEASE_HOLD    _IOW(FAKEGPU_IOC_MAGIC,  8, __u32)

/*
 * Take the resv from a kernel thread and hold it until RELEASE_RESV.
 * Returns once the resv is held.  Simulates drm_exec / KFD restore holding
 * a BO resv while waiting for a fence.
 */
#define FAKEGPU_IOC_HOLD_RESV       _IOW(FAKEGPU_IOC_MAGIC,  9, __u32)

/* Release the resv held by HOLD_RESV. */
#define FAKEGPU_IOC_RELEASE_RESV    _IOW(FAKEGPU_IOC_MAGIC, 10, __u32)

#endif /* _UAPI_LINUX_FAKE_GPU_DMABUF_H */
