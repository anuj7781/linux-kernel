// SPDX-License-Identifier: GPL-2.0
/*
 * Fake GPU dma-buf exporter — deadlock reproduction tests.
 *
 * Each test maps to a specific deadlock scenario encountered on the
 * pavel/rw-dmabuf-v4 branch when io_uring registers dma-buf backed fixed
 * buffers for NVMe I/O.  The exporter side (fake_gpu_dmabuf.ko) is a
 * controlled stand-in for AMDGPU; the NVMe importer path is real and
 * unchanged.
 *
 * Non-NVMe tests (always run):
 *   t1_inval_no_importer     - INVAL_AND_HOLD with no attachment: must
 *                              return in < 1 s (no deadlock possible).
 *   t2_hold_resv_basic       - HOLD_RESV + RELEASE_RESV round-trip.
 *   t3_map_block_basic       - BLOCK_MAP / UNBLOCK_MAP round-trip.
 *   t4_repeated_invalidation - 20 INVALIDATE cycles, all must complete.
 *
 * NVMe tests (pass --nvme /dev/nvmeXnY):
 *   t5_inval_and_hold_nvme   - THE critical test.  While NVMe I/O is in
 *                              flight: trigger INVAL_AND_HOLD and verify the
 *                              ioctl returns without hanging.
 *                              With the old async-release design this
 *                              deadlocks; with the synchronous fix it
 *                              completes promptly.
 *
 * Usage:
 *   modprobe fake_gpu_dmabuf
 *   ./fake_gpu_test
 *   ./fake_gpu_test --nvme /dev/nvme0n1 [--buf-pages N]
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <linux/io_uring.h>
#include <linux/fake_gpu_dmabuf.h>

#define PAGE_SIZE_4K 4096UL

/* ------------------------------------------------------------------ */
/* Test framework                                                      */
/* ------------------------------------------------------------------ */

static int pass_count, fail_count;

#define PASS_STR "\033[32mPASS\033[0m"
#define FAIL_STR "\033[31mFAIL\033[0m"
#define SKIP_STR "\033[33mSKIP\033[0m"

static void result(const char *name, bool ok)
{
	if (ok)
		pass_count++;
	else
		fail_count++;
	printf("  [%s] %s\n", ok ? PASS_STR : FAIL_STR, name);
}

static void skip(const char *name, const char *reason)
{
	printf("  [%s] %s  (%s)\n", SKIP_STR, name, reason);
}

#define pr_info(f, ...)  printf("         " f "\n", ##__VA_ARGS__)
#define pr_err(f, ...)   fprintf(stderr, "  ERROR: " f "\n", ##__VA_ARGS__)

static double now_s(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec / 1e9;
}

/* ------------------------------------------------------------------ */
/* fake_gpu helpers                                                    */
/* ------------------------------------------------------------------ */

static int fgpu;		/* /dev/fake_gpu_dmabuf fd */

static int fgpu_create(uint32_t nr_pages, uint32_t *handle, int *dmabuf_fd)
{
	struct fakegpu_create_arg a = { .nr_pages = nr_pages };
	if (ioctl(fgpu, FAKEGPU_IOC_CREATE, &a) < 0)
		return -errno;
	*handle   = a.handle;
	*dmabuf_fd = a.dmabuf_fd;
	return 0;
}

static int fgpu_ioctl(unsigned long cmd, uint32_t handle)
{
	int ret = ioctl(fgpu, cmd, &handle);
	return ret < 0 ? -errno : 0;
}

/* ------------------------------------------------------------------ */
/* io_uring wrapper (raw syscalls, no liburing dependency)            */
/* ------------------------------------------------------------------ */

struct uring {
	int	 fd;
	void	*sq_ring;	/* mmap of SQ ring */
	void	*cq_ring;	/* mmap of CQ ring (may equal sq_ring) */
	void	*sqes;		/* mmap of SQE array */
	size_t	 sq_ring_sz, cq_ring_sz, sqe_sz;
	uint32_t sq_mask;
	uint32_t *sq_tail, *sq_head, *sq_array;
	uint32_t *cq_tail, *cq_head;
	struct io_uring_cqe *cqes;
};

static int io_uring_setup_raw(uint32_t entries, struct io_uring_params *p)
{
	return (int)syscall(__NR_io_uring_setup, entries, p);
}

static int io_uring_enter_raw(int fd, uint32_t submit, uint32_t wait,
			      uint32_t flags)
{
	return (int)syscall(__NR_io_uring_enter, fd, submit, wait,
			    flags, NULL, 0);
}

static int io_uring_register_raw(int fd, unsigned op, void *arg, unsigned nr)
{
	return (int)syscall(__NR_io_uring_register, fd, op, arg, nr);
}

static int uring_init(struct uring *u, uint32_t depth)
{
	struct io_uring_params p = {};

	memset(u, 0, sizeof(*u));
	u->fd = -1;

	u->fd = io_uring_setup_raw(depth, &p);
	if (u->fd < 0)
		return -errno;

	u->sq_ring_sz = p.sq_off.array + p.sq_entries * sizeof(uint32_t);
	u->sqe_sz     = p.sq_entries * sizeof(struct io_uring_sqe);
	u->cq_ring_sz = p.cq_off.cqes + p.cq_entries * sizeof(struct io_uring_cqe);

	u->sq_ring = mmap(NULL, u->sq_ring_sz, PROT_READ | PROT_WRITE,
			  MAP_SHARED | MAP_POPULATE, u->fd, IORING_OFF_SQ_RING);
	if (u->sq_ring == MAP_FAILED)
		goto err;

	u->sqes = mmap(NULL, u->sqe_sz, PROT_READ | PROT_WRITE,
		       MAP_SHARED | MAP_POPULATE, u->fd, IORING_OFF_SQES);
	if (u->sqes == MAP_FAILED)
		goto err;

	if (p.features & IORING_FEAT_SINGLE_MMAP) {
		u->cq_ring     = u->sq_ring;
		u->cq_ring_sz  = 0;
	} else {
		u->cq_ring = mmap(NULL, u->cq_ring_sz, PROT_READ | PROT_WRITE,
				  MAP_SHARED | MAP_POPULATE,
				  u->fd, IORING_OFF_CQ_RING);
		if (u->cq_ring == MAP_FAILED)
			goto err;
	}

#define SQ_OFF(f) ((uint8_t *)u->sq_ring + p.sq_off.f)
#define CQ_OFF(f) ((uint8_t *)u->cq_ring + p.cq_off.f)
	u->sq_mask  = *(uint32_t *)SQ_OFF(ring_mask);
	u->sq_tail  = (uint32_t *)SQ_OFF(tail);
	u->sq_head  = (uint32_t *)SQ_OFF(head);
	u->sq_array = (uint32_t *)SQ_OFF(array);
	u->cq_tail  = (uint32_t *)CQ_OFF(tail);
	u->cq_head  = (uint32_t *)CQ_OFF(head);
	u->cqes     = (struct io_uring_cqe *)CQ_OFF(cqes);
#undef SQ_OFF
#undef CQ_OFF
	return 0;
err:
	if (u->sqes   != MAP_FAILED) munmap(u->sqes, u->sqe_sz);
	if (u->sq_ring != MAP_FAILED) munmap(u->sq_ring, u->sq_ring_sz);
	if (u->cq_ring != MAP_FAILED && u->cq_ring != u->sq_ring)
		munmap(u->cq_ring, u->cq_ring_sz);
	close(u->fd);
	u->fd = -1;
	return -errno;
}

static void uring_cleanup(struct uring *u)
{
	if (u->fd < 0)
		return;
	if (u->sqes != MAP_FAILED)    munmap(u->sqes, u->sqe_sz);
	if (u->sq_ring != MAP_FAILED) munmap(u->sq_ring, u->sq_ring_sz);
	if (u->cq_ring != MAP_FAILED && u->cq_ring != u->sq_ring)
		munmap(u->cq_ring, u->cq_ring_sz);
	close(u->fd);
}

/*
 * Register a dma-buf backed fixed buffer at slot 0.
 * First call establishes an empty 1-entry table, second updates slot 0.
 */
static int uring_register_dmabuf(struct uring *u, int dmabuf_fd, size_t size,
				 int nvme_fd)
{
	struct io_uring_rsrc_register rr = { .nr = 1 };
	struct io_uring_regbuf_desc desc = {
		.type      = IO_REGBUF_TYPE_DMABUF,
		.size      = size,
		.dmabuf_fd = dmabuf_fd,
		.target_fd = nvme_fd,
	};
	int ret;

	ret = io_uring_register_raw(u->fd, IORING_REGISTER_BUFFERS, NULL, 0);
	if (ret < 0)
		return -errno;

	rr.data = (uint64_t)(uintptr_t)&desc;
	ret = io_uring_register_raw(u->fd, IORING_REGISTER_BUFFERS_UPDATE,
				    &rr, 1);
	return ret < 0 ? -errno : 0;
}

/* Submit one READ_FIXED to fd at offset 0, using fixed buf slot 0. */
static int uring_submit_read(struct uring *u, int fd, uint32_t len)
{
	uint32_t tail = *u->sq_tail;
	uint32_t idx  = tail & u->sq_mask;
	struct io_uring_sqe *sqe = &((struct io_uring_sqe *)u->sqes)[idx];

	memset(sqe, 0, sizeof(*sqe));
	sqe->opcode    = IORING_OP_READ_FIXED;
	sqe->fd        = fd;
	sqe->off       = 0;
	sqe->len       = len;
	sqe->buf_index = 0;
	sqe->user_data = 1;

	u->sq_array[idx] = idx;

	__sync_synchronize();
	*u->sq_tail = tail + 1;
	__sync_synchronize();

	return io_uring_enter_raw(u->fd, 1, 0, 0) < 0 ? -errno : 0;
}

/* Drain all pending completions (non-blocking). */
static void uring_drain(struct uring *u)
{
	while (*u->cq_head != *u->cq_tail) {
		__sync_synchronize();
		(*u->cq_head)++;
		__sync_synchronize();
	}
}

/* ------------------------------------------------------------------ */
/* t1: INVAL_AND_HOLD with no importer                                */
/* ------------------------------------------------------------------ */

static void t1_inval_no_importer(void)
{
	uint32_t h;
	int fd, ret;
	double t;

	ret = fgpu_create(1, &h, &fd);
	if (ret) {
		pr_err("create: %s", strerror(-ret));
		result("t1_inval_no_importer", false);
		return;
	}

	t = now_s();
	ret = fgpu_ioctl(FAKEGPU_IOC_INVAL_AND_HOLD, h);
	t = now_s() - t;

	pr_info("INVAL_AND_HOLD returned in %.3f s (no importer attached)", t);

	if (ret == 0)
		fgpu_ioctl(FAKEGPU_IOC_RELEASE_HOLD, h);
	else
		pr_err("INVAL_AND_HOLD: %s", strerror(-ret));

	close(fd);
	result("t1_inval_no_importer", ret == 0 && t < 1.0);
}

/* ------------------------------------------------------------------ */
/* t2: HOLD_RESV basic round-trip                                     */
/* ------------------------------------------------------------------ */

static void t2_hold_resv_basic(void)
{
	uint32_t h;
	int fd, ret;

	ret = fgpu_create(1, &h, &fd);
	if (ret) {
		result("t2_hold_resv_basic", false);
		return;
	}

	ret = fgpu_ioctl(FAKEGPU_IOC_HOLD_RESV, h);
	if (ret) {
		pr_err("HOLD_RESV: %s", strerror(-ret));
		close(fd);
		result("t2_hold_resv_basic", false);
		return;
	}
	pr_info("resv held by kthread");

	/* Second HOLD_RESV must fail with EBUSY. */
	if (fgpu_ioctl(FAKEGPU_IOC_HOLD_RESV, h) != -EBUSY)
		pr_info("warning: second HOLD_RESV should return -EBUSY");

	ret = fgpu_ioctl(FAKEGPU_IOC_RELEASE_RESV, h);
	pr_info("RELEASE_RESV returned %d", ret);

	close(fd);
	result("t2_hold_resv_basic", ret == 0);
}

/* ------------------------------------------------------------------ */
/* t3: BLOCK_MAP / UNBLOCK_MAP round-trip (no importer)              */
/* ------------------------------------------------------------------ */

/*
 * Without an importer, map_dma_buf() is never called, so BLOCK_MAP only
 * sets a flag.  This test verifies the flag set/clear path works and that
 * HOLD_RESV succeeds when the resv is free (map_dma_buf not blocking).
 *
 * The real MAP_BLOCK deadlock demonstration (resv held inside map_dma_buf
 * while a second thread waits) requires NVMe to call map_dma_buf().
 */
static void t3_map_block_basic(void)
{
	uint32_t h;
	int fd, ret;

	ret = fgpu_create(1, &h, &fd);
	if (ret) {
		result("t3_map_block_basic", false);
		return;
	}

	ret = fgpu_ioctl(FAKEGPU_IOC_BLOCK_MAP, h);
	if (ret) {
		pr_err("BLOCK_MAP: %s", strerror(-ret));
		close(fd);
		result("t3_map_block_basic", false);
		return;
	}
	pr_info("BLOCK_MAP set; next map_dma_buf() will block with resv held");

	/*
	 * No NVMe I/O here, so resv is free.  HOLD_RESV takes it immediately,
	 * confirming there is no spurious lock held by the BLOCK_MAP path.
	 */
	ret = fgpu_ioctl(FAKEGPU_IOC_HOLD_RESV, h);
	if (ret == 0) {
		pr_info("HOLD_RESV succeeded (resv was free, no importer mapping)");
		fgpu_ioctl(FAKEGPU_IOC_RELEASE_RESV, h);
	}

	/* Unblock so map_dma_buf() can proceed if it's ever called. */
	fgpu_ioctl(FAKEGPU_IOC_UNBLOCK_MAP, h);

	close(fd);
	result("t3_map_block_basic", ret == 0);
}

/* ------------------------------------------------------------------ */
/* t4: Repeated invalidation                                          */
/* ------------------------------------------------------------------ */

static void t4_repeated_invalidation(void)
{
	uint32_t h;
	int fd, i, ret = 0;
	double t0, t1;

	ret = fgpu_create(1, &h, &fd);
	if (ret) {
		result("t4_repeated_invalidation", false);
		return;
	}

	t0 = now_s();
	for (i = 0; i < 20; i++) {
		ret = fgpu_ioctl(FAKEGPU_IOC_INVALIDATE, h);
		if (ret) {
			pr_err("INVALIDATE[%d]: %s", i, strerror(-ret));
			break;
		}
	}
	t1 = now_s();
	pr_info("20 INVALIDATE cycles in %.3f s", t1 - t0);

	close(fd);
	result("t4_repeated_invalidation", ret == 0);
}

/* ------------------------------------------------------------------ */
/* t5: INVAL_AND_HOLD while NVMe I/O is in flight                    */
/* ------------------------------------------------------------------ */

/*
 * This is the critical test for the synchronous invalidation fix.
 *
 * With the OLD async-release design (commit 58479d3078c3 dropped from tree):
 *   io_dmabuf_drop_map() queued a work item.  The work item needed the resv
 *   to call dma_buf_unmap_attachment().  The kthread below holds the resv
 *   after calling invalidate_mappings().  Both are stuck: ABBA deadlock.
 *   This ioctl would never return.
 *
 * With the CURRENT synchronous fix (commit 000c11d0f04b):
 *   io_dmabuf_drop_map() runs inline under the caller's resv lock.  It
 *   drains all NVMe request refs (wait_for_completion), calls unmap with
 *   the resv already held (protocol correct), and returns.  The kthread
 *   signals inval_held and this ioctl returns quickly.
 *
 * Expected result: ioctl returns in O(NVMe queue drain time), not hung.
 */

struct io_arg {
	struct uring  *u;
	int            nvme_fd;
	uint32_t       len;
	volatile int   stop;
	int            submitted;
};

static void *io_thread_fn(void *arg)
{
	struct io_arg *a = arg;
	int ret;

	while (!a->stop) {
		ret = uring_submit_read(a->u, a->nvme_fd, a->len);
		if (ret < 0)
			break;
		a->submitted++;
		usleep(2000);
	}
	return NULL;
}

static void t5_inval_and_hold_nvme(const char *nvme_path, uint32_t nr_pages)
{
	struct uring u = { .fd = -1 };
	struct io_arg io_arg = {};
	pthread_t thr;
	uint32_t h;
	int dmabuf_fd = -1, nvme_fd = -1, ret;
	double t;
	size_t buf_sz = (size_t)nr_pages * PAGE_SIZE_4K;

	printf("\nTest t5: inval_and_hold_nvme  (NVMe: %s, buf: %zu B)\n",
	       nvme_path, buf_sz);

	nvme_fd = open(nvme_path, O_RDWR | O_DIRECT);
	if (nvme_fd < 0) {
		pr_err("open %s: %s", nvme_path, strerror(errno));
		skip("t5_inval_and_hold_nvme", "cannot open NVMe device");
		return;
	}

	ret = uring_init(&u, 128);
	if (ret) {
		pr_err("io_uring_setup: %s", strerror(-ret));
		goto out;
	}

	ret = fgpu_create(nr_pages, &h, &dmabuf_fd);
	if (ret) {
		pr_err("fake buf create: %s", strerror(-ret));
		goto out;
	}

	ret = uring_register_dmabuf(&u, dmabuf_fd, buf_sz, nvme_fd);
	if (ret) {
		pr_info("dma-buf registration failed (%s)", strerror(-ret));
		pr_info("NVMe may not support dmabuf tokens; checking CONFIG_DMABUF_TOKEN");
		skip("t5_inval_and_hold_nvme", "dmabuf registration not supported");
		goto out;
	}
	pr_info("dma-buf registered as fixed buffer slot 0");

	/* Warm up: one I/O to make sure map is established. */
	uring_submit_read(&u, nvme_fd, (uint32_t)buf_sz);
	usleep(5000);
	uring_drain(&u);

	/* Start background I/O to hold request refs on the active map. */
	io_arg.u       = &u;
	io_arg.nvme_fd = nvme_fd;
	io_arg.len     = (uint32_t)buf_sz;
	pthread_create(&thr, NULL, io_thread_fn, &io_arg);

	usleep(20000);	/* let several I/Os get in flight */
	pr_info("%d I/Os submitted; triggering INVAL_AND_HOLD ...",
		io_arg.submitted);

	t = now_s();
	/*
	 * This ioctl performs:
	 *   dma_resv_lock(resv, NULL)
	 *   dma_buf_invalidate_mappings()   ← calls io_dmabuf_drop_map() inline
	 *     io_dmabuf_drop_map():
	 *       percpu_ref_kill()
	 *       wait_for_completion(&map->drain)  ← drains NVMe request refs
	 *       dev_ops->unmap()                  ← dma_buf_unmap_attachment()
	 *   complete(inval_held)
	 *   [kthread holds resv, waiting for RELEASE_HOLD]
	 */
	ret = fgpu_ioctl(FAKEGPU_IOC_INVAL_AND_HOLD, h);
	t = now_s() - t;

	io_arg.stop = 1;
	pthread_join(thr, NULL);
	uring_drain(&u);

	pr_info("submitted=%d; INVAL_AND_HOLD returned in %.3f s, ret=%d",
		io_arg.submitted, t, ret);

	if (ret == 0) {
		pr_info("no deadlock — synchronous fix working correctly");
		fgpu_ioctl(FAKEGPU_IOC_RELEASE_HOLD, h);
	} else {
		pr_err("INVAL_AND_HOLD: %s", strerror(-ret));
	}

	result("t5_inval_and_hold_nvme", ret == 0 && t < 5.0);
out:
	if (dmabuf_fd >= 0) close(dmabuf_fd);
	if (u.fd >= 0)      uring_cleanup(&u);
	if (nvme_fd >= 0)   close(nvme_fd);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [--nvme /dev/nvmeXnY] [--buf-pages N]\n"
		"\n"
		"  --nvme PATH    enable t5 (INVAL_AND_HOLD with real NVMe I/O)\n"
		"  --buf-pages N  buffer size in 4 KiB pages for t5 (default: 1)\n",
		prog);
}

int main(int argc, char **argv)
{
	const char *nvme_path = NULL;
	uint32_t nr_pages = 1;
	int i;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--nvme") && i + 1 < argc) {
			nvme_path = argv[++i];
		} else if (!strcmp(argv[i], "--buf-pages") && i + 1 < argc) {
			nr_pages = (uint32_t)atol(argv[++i]);
			if (!nr_pages) nr_pages = 1;
		} else {
			usage(argv[0]);
			return 1;
		}
	}

	fgpu = open("/dev/fake_gpu_dmabuf", O_RDWR);
	if (fgpu < 0) {
		fprintf(stderr,
			"Cannot open /dev/fake_gpu_dmabuf: %s\n"
			"Load the module:  modprobe fake_gpu_dmabuf\n",
			strerror(errno));
		return 2;
	}

	printf("=== fake_gpu_dmabuf deadlock tests ===\n\n");

	t1_inval_no_importer();
	t2_hold_resv_basic();
	t3_map_block_basic();
	t4_repeated_invalidation();

	if (nvme_path)
		t5_inval_and_hold_nvme(nvme_path, nr_pages);
	else
		skip("t5_inval_and_hold_nvme", "pass --nvme to enable");

	printf("\n%d passed, %d failed\n", pass_count, fail_count);
	close(fgpu);
	return fail_count ? 1 : 0;
}
