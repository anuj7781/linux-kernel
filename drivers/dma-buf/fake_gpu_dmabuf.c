// SPDX-License-Identifier: GPL-2.0
/*
 * Fake GPU dma-buf exporter — deadlock reproduction for io_uring/NVMe.
 *
 * Copyright (C) 2026 Anuj Gupta
 *
 * This module implements a minimal dma-buf exporter that mimics the three
 * AMDGPU behaviors that created ABBA deadlocks when io_uring registers
 * dma-buf backed fixed buffers for NVMe I/O:
 *
 *   1. map_dma_buf() blocks while the resv is held (BLOCK_MAP).
 *      In AMDGPU, ttm_bo_validate() waits for a GPU fence inside
 *      map_dma_buf() without releasing the resv.
 *
 *   2. dma_buf_invalidate_mappings() is called while holding the resv,
 *      followed by holding the resv while waiting for a fence
 *      (INVAL_AND_HOLD).  In AMDGPU, amdgpu_bo_move_notify() triggers this
 *      and KFD restore then waits for a VM fence with the resv held.
 *
 *   3. A kernel thread holds the resv and waits (HOLD_RESV), reproducing
 *      drm_exec / KFD restore locking a BO resv for an extended window.
 *
 * Only the real NVMe importer (io_uring → io_dmabuf_token → nvme_pci) is
 * used; the exporter side is entirely fake.  If map_dma_buf() gets called,
 * it returns a real scatter-gather table built from kernel pages that the
 * NVMe controller can DMA to/from.
 *
 * Control interface:  /dev/fake_gpu_dmabuf  (ioctl, see fake_gpu_dmabuf.h)
 *
 * Lifetime model
 * --------------
 *   struct fake_gpu_buf.kref starts at 1 (primary ownership).
 *   fake_gpu_dmabuf_release() drops this ref when the dmabuf fd is closed.
 *   Each kthread and each ioctl call holds a temporary kref_get().
 *   Each kthread also calls get_dma_buf() so the dmabuf resv cannot be
 *   freed while the kthread holds the resv lock.
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/scatterlist.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/xarray.h>
#include <linux/file.h>
#include <uapi/linux/fake_gpu_dmabuf.h>

struct fake_gpu_buf {
	int			id;
	struct dma_buf		*dmabuf;
	struct page		**pages;
	unsigned int		nr_pages;

	/*
	 * BLOCK_MAP scenario: map_dma_buf() increments map_block_count and
	 * then waits on map_unblock.  WAIT_MAP_BLOCKED polls map_block_count.
	 */
	atomic_t		map_blocked;		/* flag: should block  */
	atomic_t		map_block_count;	/* how many are waiting */
	struct completion	map_unblock;

	/*
	 * INVAL_AND_HOLD scenario: kthread locks resv, calls
	 * dma_buf_invalidate_mappings(), signals inval_held, then waits on
	 * inval_release before unlocking.
	 */
	atomic_t		inval_active;
	struct completion	inval_go;
	struct completion	inval_held;
	struct completion	inval_release;

	/*
	 * HOLD_RESV scenario: kthread locks resv, signals hold_locked, waits
	 * on hold_release.
	 */
	atomic_t		hold_active;
	struct completion	hold_go;
	struct completion	hold_locked;
	struct completion	hold_release;

	struct kref		kref;
};

static DEFINE_XARRAY_ALLOC(fakegpu_xa);

static void fake_gpu_buf_free(struct kref *kref)
{
	struct fake_gpu_buf *buf = container_of(kref, struct fake_gpu_buf, kref);
	unsigned int i;

	for (i = 0; i < buf->nr_pages; i++)
		__free_page(buf->pages[i]);
	kvfree(buf->pages);
	kfree(buf);
}

static void fakegpu_put(struct fake_gpu_buf *buf)
{
	kref_put(&buf->kref, fake_gpu_buf_free);
}

/* Look up by handle; caller must call fakegpu_put() when done. */
static struct fake_gpu_buf *fakegpu_get(u32 handle)
{
	struct fake_gpu_buf *buf;

	xa_lock(&fakegpu_xa);
	buf = xa_load(&fakegpu_xa, handle);
	if (buf)
		kref_get(&buf->kref);
	xa_unlock(&fakegpu_xa);
	return buf;
}

/* ------------------------------------------------------------------ */
/* dma-buf exporter ops                                                */
/* ------------------------------------------------------------------ */

static struct sg_table *fake_gpu_map_dma_buf(struct dma_buf_attachment *attach,
					     enum dma_data_direction dir)
{
	struct fake_gpu_buf *buf = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	dma_resv_assert_held(attach->dmabuf->resv);

	/*
	 * Simulate ttm_bo_validate() blocking on a GPU fence while holding the
	 * resv.  This is scenario 1: the resv remains locked throughout.
	 */
	if (atomic_read(&buf->map_blocked)) {
		atomic_inc(&buf->map_block_count);
		wait_for_completion(&buf->map_unblock);
		atomic_dec(&buf->map_block_count);
	}

	sgt = kzalloc_obj(*sgt);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table_from_pages(sgt, buf->pages, buf->nr_pages,
					0, (size_t)buf->nr_pages << PAGE_SHIFT,
					GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	if (dma_map_sgtable(attach->dev, sgt, dir, 0)) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(-ENOMEM);
	}

	return sgt;
}

static void fake_gpu_unmap_dma_buf(struct dma_buf_attachment *attach,
				   struct sg_table *sgt,
				   enum dma_data_direction dir)
{
	dma_resv_assert_held(attach->dmabuf->resv);
	dma_unmap_sgtable(attach->dev, sgt, dir, 0);
	sg_free_table(sgt);
	kfree(sgt);
}

/*
 * pin/unpin must be present for dma_buf_is_dynamic() to return true.
 * Dynamic exporters have map_dma_buf() called with the resv held and
 * receive dma_buf_invalidate_mappings() notifications before BO movement.
 */
static int  fake_gpu_pin(struct dma_buf_attachment *a) { return 0; }
static void fake_gpu_unpin(struct dma_buf_attachment *a) {}

static void fake_gpu_dmabuf_release(struct dma_buf *dmabuf)
{
	struct fake_gpu_buf *buf = dmabuf->priv;

	xa_erase(&fakegpu_xa, buf->id);

	/*
	 * Drop the primary ownership reference.  Any in-flight kthreads
	 * hold their own kref + get_dma_buf(), so they keep the buf and
	 * dmabuf alive independently.  This put may trigger fake_gpu_buf_free()
	 * if no kthreads are active.
	 */
	kref_put(&buf->kref, fake_gpu_buf_free);
}

static const struct dma_buf_ops fake_gpu_dmabuf_ops = {
	.pin		= fake_gpu_pin,
	.unpin		= fake_gpu_unpin,
	.map_dma_buf	= fake_gpu_map_dma_buf,
	.unmap_dma_buf	= fake_gpu_unmap_dma_buf,
	.release	= fake_gpu_dmabuf_release,
};

/* ------------------------------------------------------------------ */
/* Buffer allocation                                                   */
/* ------------------------------------------------------------------ */

static struct fake_gpu_buf *fake_gpu_buf_alloc(unsigned int nr_pages)
{
	struct fake_gpu_buf *buf;
	unsigned int i;

	if (!nr_pages || nr_pages > (SZ_1G >> PAGE_SHIFT))
		return ERR_PTR(-EINVAL);

	buf = kzalloc_obj(*buf);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	kref_init(&buf->kref);

	atomic_set(&buf->map_blocked, 0);
	atomic_set(&buf->map_block_count, 0);
	init_completion(&buf->map_unblock);

	atomic_set(&buf->inval_active, 0);
	init_completion(&buf->inval_go);
	init_completion(&buf->inval_held);
	init_completion(&buf->inval_release);

	atomic_set(&buf->hold_active, 0);
	init_completion(&buf->hold_go);
	init_completion(&buf->hold_locked);
	init_completion(&buf->hold_release);

	buf->pages = kvcalloc(nr_pages, sizeof(*buf->pages), GFP_KERNEL);
	if (!buf->pages) {
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < nr_pages; i++) {
		buf->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!buf->pages[i]) {
			buf->nr_pages = i;
			kref_put(&buf->kref, fake_gpu_buf_free);
			return ERR_PTR(-ENOMEM);
		}
	}
	buf->nr_pages = nr_pages;
	return buf;
}

static struct dma_buf *fake_gpu_export(struct fake_gpu_buf *buf)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops   = &fake_gpu_dmabuf_ops;
	exp_info.size  = (size_t)buf->nr_pages << PAGE_SHIFT;
	exp_info.flags = O_RDWR | O_CLOEXEC;
	exp_info.priv  = buf;
	return dma_buf_export(&exp_info);
}

/* ------------------------------------------------------------------ */
/* kthreads                                                            */
/* ------------------------------------------------------------------ */

/*
 * INVAL_AND_HOLD kthread.
 *
 * Reproduces: AMDGPU amdgpu_bo_move_notify() + KFD restore pattern.
 *   amdgpu_bo_move_notify() holds the BO resv and calls
 *   dma_buf_invalidate_mappings().  Afterwards KFD restore continues to hold
 *   the resv while waiting for a GPU VM fence.
 *
 * With the old async release design, the NVMe release worker needs this same
 * resv to call dma_buf_unmap_attachment() — ABBA deadlock.
 * With the synchronous fix, io_dmabuf_drop_map() runs inline under the resv
 * already held by the caller, so there is no competing worker.
 */
static int fake_gpu_inval_fn(void *arg)
{
	struct fake_gpu_buf *buf = arg;
	struct dma_buf *dmabuf = buf->dmabuf;

	/*
	 * Hold a dmabuf reference so the resv cannot be freed while we
	 * hold the lock.  The caller already called get_dma_buf() for us
	 * before kthread_run(); we capture the pointer here and release it
	 * at the end.
	 */

	wait_for_completion(&buf->inval_go);

	dma_resv_lock(dmabuf->resv, NULL);

	/*
	 * This is the key call: same as amdgpu_bo_move_notify() calling
	 * dma_buf_invalidate_mappings() while the BO resv is held.
	 */
	dma_buf_invalidate_mappings(dmabuf);

	/*
	 * Signal the ioctl that we hold the resv and invalidation is done.
	 * With the synchronous fix, io_dmabuf_drop_map() has already drained
	 * all NVMe I/O refs and unmapped before we reach here.
	 */
	complete(&buf->inval_held);

	/*
	 * Hold the resv here.  With the old async design, the NVMe release
	 * worker would now be stuck trying to acquire this same resv.
	 * Release when the test is done.
	 */
	wait_for_completion_timeout(&buf->inval_release, msecs_to_jiffies(30000));

	dma_resv_unlock(dmabuf->resv);

	atomic_set(&buf->inval_active, 0);
	dma_buf_put(dmabuf);
	kref_put(&buf->kref, fake_gpu_buf_free);
	return 0;
}

/*
 * HOLD_RESV kthread.
 *
 * Reproduces: drm_exec / KFD restore holding a BO resv while waiting.
 * A NULL-ctx ww_mutex holder cannot be wounded by another NULL-ctx holder,
 * so any concurrent io_uring NULL-ctx lock attempt will block indefinitely.
 */
static int fake_gpu_hold_resv_fn(void *arg)
{
	struct fake_gpu_buf *buf = arg;
	struct dma_buf *dmabuf = buf->dmabuf;

	wait_for_completion(&buf->hold_go);

	dma_resv_lock(dmabuf->resv, NULL);
	complete(&buf->hold_locked);

	wait_for_completion_timeout(&buf->hold_release, msecs_to_jiffies(30000));

	dma_resv_unlock(dmabuf->resv);

	atomic_set(&buf->hold_active, 0);
	dma_buf_put(dmabuf);
	kref_put(&buf->kref, fake_gpu_buf_free);
	return 0;
}

/* Start a kthread that gets kref + dma_buf refs before running. */
static int start_kthread(struct fake_gpu_buf *buf,
			 int (*fn)(void *), const char *name,
			 atomic_t *active_flag,
			 struct completion *go, struct completion *done_held,
			 struct completion *release)
{
	struct task_struct *task;

	reinit_completion(go);
	reinit_completion(done_held);
	reinit_completion(release);

	kref_get(&buf->kref);
	get_dma_buf(buf->dmabuf);

	task = kthread_run(fn, buf, "%s/%d", name, buf->id);
	if (IS_ERR(task)) {
		dma_buf_put(buf->dmabuf);
		kref_put(&buf->kref, fake_gpu_buf_free);
		atomic_set(active_flag, 0);
		return PTR_ERR(task);
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* ioctl                                                               */
/* ------------------------------------------------------------------ */

static long fakegpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct fake_gpu_buf *buf;
	u32 handle;
	int ret = 0;

	if (cmd == FAKEGPU_IOC_CREATE) {
		struct fakegpu_create_arg a;
		struct dma_buf *dmabuf;
		u32 id;
		int fd;

		if (copy_from_user(&a, uarg, sizeof(a)))
			return -EFAULT;

		buf = fake_gpu_buf_alloc(a.nr_pages);
		if (IS_ERR(buf))
			return PTR_ERR(buf);

		dmabuf = fake_gpu_export(buf);
		if (IS_ERR(dmabuf)) {
			kref_put(&buf->kref, fake_gpu_buf_free);
			return PTR_ERR(dmabuf);
		}
		buf->dmabuf = dmabuf;

		/*
		 * Reserve an fd BEFORE copy_to_user so we can put it back if
		 * the copy fails (the fd is not yet visible to userspace).
		 */
		fd = get_unused_fd_flags(O_CLOEXEC);
		if (fd < 0) {
			dma_buf_put(dmabuf);
			return fd;
		}

		ret = xa_alloc(&fakegpu_xa, &id, buf, XA_LIMIT(1, UINT_MAX),
			       GFP_KERNEL);
		if (ret) {
			put_unused_fd(fd);
			dma_buf_put(dmabuf);
			return ret;
		}
		buf->id = id;

		a.handle    = id;
		a.dmabuf_fd = fd;
		if (copy_to_user(uarg, &a, sizeof(a))) {
			put_unused_fd(fd);
			/*
			 * dma_buf_put() triggers fake_gpu_dmabuf_release() which
			 * removes from XArray and drops the primary kref.
			 */
			dma_buf_put(dmabuf);
			return -EFAULT;
		}

		/* fd_install after copy so the fd only becomes visible on success */
		fd_install(fd, dmabuf->file);
		return 0;
	}

	/* All other ioctls take a plain u32 handle. */
	if (copy_from_user(&handle, uarg, sizeof(handle)))
		return -EFAULT;

	buf = fakegpu_get(handle);
	if (!buf)
		return -ENOENT;

	switch (cmd) {
	case FAKEGPU_IOC_DESTROY:
		/* Just drop the fd from userspace; this releases IDR. */
		break;

	case FAKEGPU_IOC_BLOCK_MAP:
		reinit_completion(&buf->map_unblock);
		atomic_set(&buf->map_blocked, 1);
		break;

	case FAKEGPU_IOC_UNBLOCK_MAP:
		atomic_set(&buf->map_blocked, 0);
		complete_all(&buf->map_unblock);
		break;

	case FAKEGPU_IOC_WAIT_MAP_BLOCKED:
		/* Spin-wait until at least one map_dma_buf() is blocking. */
		while (!atomic_read(&buf->map_block_count)) {
			if (signal_pending(current)) {
				ret = -EINTR;
				break;
			}
			cpu_relax();
		}
		break;

	case FAKEGPU_IOC_INVALIDATE:
		/*
		 * Simple path: lock resv, invalidate, unlock.
		 * Matches the fast amdgpu_bo_move_notify() case where the BO
		 * moves quickly and the resv is released immediately.
		 */
		dma_resv_lock(buf->dmabuf->resv, NULL);
		dma_buf_invalidate_mappings(buf->dmabuf);
		dma_resv_unlock(buf->dmabuf->resv);
		break;

	case FAKEGPU_IOC_INVAL_AND_HOLD:
		if (atomic_cmpxchg(&buf->inval_active, 0, 1) != 0) {
			ret = -EBUSY;
			break;
		}

		ret = start_kthread(buf, fake_gpu_inval_fn, "fakegpu-inval",
				    &buf->inval_active,
				    &buf->inval_go, &buf->inval_held,
				    &buf->inval_release);
		if (ret)
			break;

		complete(&buf->inval_go);

		/*
		 * Block until the kthread holds the resv and has called
		 * dma_buf_invalidate_mappings().  With the synchronous fix,
		 * this returns quickly because io_dmabuf_drop_map() drains
		 * inline.  With the old async design this may never return
		 * (deadlock) if NVMe I/O is in flight.
		 */
		if (!wait_for_completion_timeout(&buf->inval_held,
						 msecs_to_jiffies(10000))) {
			pr_warn("fakegpu: INVAL_AND_HOLD timed out waiting for kthread\n");
			complete(&buf->inval_release);
			ret = -ETIMEDOUT;
		}
		break;

	case FAKEGPU_IOC_RELEASE_HOLD:
		if (!atomic_read(&buf->inval_active)) {
			ret = -EINVAL;
			break;
		}
		complete(&buf->inval_release);
		break;

	case FAKEGPU_IOC_HOLD_RESV:
		if (atomic_cmpxchg(&buf->hold_active, 0, 1) != 0) {
			ret = -EBUSY;
			break;
		}

		ret = start_kthread(buf, fake_gpu_hold_resv_fn, "fakegpu-hold",
				    &buf->hold_active,
				    &buf->hold_go, &buf->hold_locked,
				    &buf->hold_release);
		if (ret)
			break;

		complete(&buf->hold_go);
		if (!wait_for_completion_timeout(&buf->hold_locked,
						 msecs_to_jiffies(5000))) {
			pr_warn("fakegpu: HOLD_RESV kthread failed to lock resv\n");
			ret = -ETIMEDOUT;
		}
		break;

	case FAKEGPU_IOC_RELEASE_RESV:
		if (!atomic_read(&buf->hold_active)) {
			ret = -EINVAL;
			break;
		}
		complete(&buf->hold_release);
		break;

	default:
		ret = -ENOTTY;
	}

	fakegpu_put(buf);
	return ret;
}

static const struct file_operations fakegpu_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = fakegpu_ioctl,
};

static struct miscdevice fakegpu_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "fake_gpu_dmabuf",
	.fops	= &fakegpu_fops,
	.mode	= 0666,
};

static int __init fake_gpu_dmabuf_init(void)
{
	return misc_register(&fakegpu_misc);
}

static void __exit fake_gpu_dmabuf_exit(void)
{
	misc_deregister(&fakegpu_misc);
	xa_destroy(&fakegpu_xa);
}

module_init(fake_gpu_dmabuf_init);
module_exit(fake_gpu_dmabuf_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fake GPU dma-buf exporter for io_uring/NVMe deadlock testing");
MODULE_IMPORT_NS("DMA_BUF");
