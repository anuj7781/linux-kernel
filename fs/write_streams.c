// SPDX-License-Identifier: GPL-2.0
/*
 * Generic write-stream fd management.
 *
 * A write stream is a lease on a slot in a filesystem's write-stream pool.
 * It is represented as an anonymous fd; the slot is released when the fd is
 * closed.  The filesystem embeds struct write_stream_pool in its device
 * abstraction and calls write_stream_alloc_fd() to create stream fds.
 *
 * Lifetime: each struct write_stream holds a mount reference that keeps the
 * filesystem mounted for as long as any stream fd remains open.  The pool
 * (and its containing device structure) therefore cannot be freed while a
 * stream fd is live.
 */
#include <linux/anon_inodes.h>
#include <linux/bitmap.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/slab.h>
#include <linux/write_streams.h>

struct write_stream {
	struct write_stream_pool	*pool;
	struct vfsmount			*mnt;
	u16				id;	/* 1-based slot number */
};

static int write_stream_release(struct inode *inode, struct file *file)
{
	struct write_stream *ws = file->private_data;

	spin_lock(&ws->pool->lock);
	clear_bit(ws->id - 1, ws->pool->streams_in_use);
	spin_unlock(&ws->pool->lock);
	mntput(ws->mnt);
	kfree(ws);
	return 0;
}

static const struct file_operations write_stream_fops = {
	.release	= write_stream_release,
	.llseek		= noop_llseek,
};

/**
 * write_stream_pool_init - initialise a write stream pool
 * @pool: pool to initialise
 * @nr:   number of stream slots
 *
 * Returns 0 on success, -ENOMEM if the bitmap cannot be allocated.
 * Call write_stream_pool_destroy() to free resources when done.
 */
int write_stream_pool_init(struct write_stream_pool *pool, unsigned int nr)
{
	if (!nr) {
		pool->streams_in_use = NULL;
		pool->nr_streams = 0;
		spin_lock_init(&pool->lock);
		return 0;
	}

	pool->streams_in_use = bitmap_zalloc(nr, GFP_KERNEL);
	if (!pool->streams_in_use)
		return -ENOMEM;
	spin_lock_init(&pool->lock);
	pool->nr_streams = nr;
	return 0;
}
EXPORT_SYMBOL_GPL(write_stream_pool_init);

/**
 * write_stream_pool_destroy - release resources held by a write stream pool
 * @pool: pool previously initialised with write_stream_pool_init()
 *
 * Safe to call on a pool with nr_streams == 0.
 */
void write_stream_pool_destroy(struct write_stream_pool *pool)
{
	bitmap_free(pool->streams_in_use);
	pool->streams_in_use = NULL;
	pool->nr_streams = 0;
}
EXPORT_SYMBOL_GPL(write_stream_pool_destroy);

/**
 * write_stream_alloc_fd - allocate a stream slot and return an fd for it
 * @pool: stream pool to allocate from
 * @file: the file on whose behalf the stream is being allocated; its mount is
 *        retained to pin the filesystem while the stream fd is open
 *
 * Returns the new fd on success, -EOPNOTSUPP if the pool has no slots,
 * -EBUSY if all slots are in use, or a negative error code on failure.
 * The returned fd has O_RDONLY | O_CLOEXEC set.
 */
int write_stream_alloc_fd(struct write_stream_pool *pool, struct file *file)
{
	struct write_stream *ws;
	int slot, fd;

	if (!pool->nr_streams)
		return -EOPNOTSUPP;

	ws = kzalloc_obj(*ws, GFP_KERNEL);
	if (!ws)
		return -ENOMEM;

	spin_lock(&pool->lock);
	slot = find_first_zero_bit(pool->streams_in_use, pool->nr_streams);
	if (slot >= (int)pool->nr_streams) {
		spin_unlock(&pool->lock);
		kfree(ws);
		return -EBUSY;
	}
	set_bit(slot, pool->streams_in_use);
	spin_unlock(&pool->lock);

	ws->pool = pool;
	ws->mnt = mntget(file->f_path.mnt);
	ws->id = slot + 1;

	fd = anon_inode_getfd("[write_stream]", &write_stream_fops, ws,
			      O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		spin_lock(&pool->lock);
		clear_bit(slot, pool->streams_in_use);
		spin_unlock(&pool->lock);
		mntput(ws->mnt);
		kfree(ws);
	}
	return fd;
}
EXPORT_SYMBOL_GPL(write_stream_alloc_fd);

/**
 * write_stream_file_check - verify a file is a write stream for a given pool
 * @file:  candidate stream file
 * @pool:  expected pool
 *
 * Returns true if @file is a write stream fd whose slot belongs to @pool.
 * Used by SET handlers to reject fds that refer to a different filesystem.
 */
bool write_stream_file_check(struct file *file,
			     const struct write_stream_pool *pool)
{
	struct write_stream *ws;

	if (file->f_op != &write_stream_fops)
		return false;
	ws = file->private_data;
	return ws->pool == pool;
}
EXPORT_SYMBOL_GPL(write_stream_file_check);

/**
 * write_stream_get_id - return the 1-based stream id from a stream fd
 * @file: a write stream file (caller must have verified with
 *        write_stream_file_check())
 */
u16 write_stream_get_id(struct file *file)
{
	return ((struct write_stream *)file->private_data)->id;
}
EXPORT_SYMBOL_GPL(write_stream_get_id);
