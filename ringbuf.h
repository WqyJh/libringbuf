/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Derived from FreeBSD's bufring.h
 *
 **************************************************************************
 *
 * Copyright (c) 2007-2009 Kip Macy kmacy@freebsd.org
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. The name of Kip Macy nor the names of other
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#ifndef _RINGBUF_H_
#define _RINGBUF_H_

/**
 * @file
 * RTE Ring
 *
 * The Ring Manager is a fixed-size queue, implemented as a table of
 * pointers. Head and tail pointers are modified atomically, allowing
 * concurrent access to it. It has the following features:
 *
 * - FIFO (First In First Out)
 * - Maximum size is fixed; the pointers are stored in a table.
 * - Lockless implementation.
 * - Multi- or single-consumer dequeue.
 * - Multi- or single-producer enqueue.
 * - Bulk dequeue.
 * - Bulk enqueue.
 *
 * Note: the ring implementation is not preemptable. A lcore must not
 * be interrupted by another task that uses the same ring.
 *
 */

#include <asm-generic/errno-base.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <emmintrin.h>

#define CACHE_LINE_SIZE 64

#define	__compiler_barrier() do {		\
	asm volatile ("" : : : "memory");	\
} while(0)

/* true if x is a power of 2 */
#define POWEROF2(x) ((((x)-1) & (x)) == 0)

/** Number of elements in the array. */
#define	RINGBUF_DIM(a)	(sizeof (a) / sizeof ((a)[0]))

#define RINGBUF_SET_USED(x) (void)(x)

static inline int
__atomic32_cmpset(volatile uint32_t *dst, uint32_t exp, uint32_t src)
{
	uint8_t res;

	asm volatile(
			"lock ; "
			"cmpxchgl %[src], %[dst];"
			"sete %[res];"
			: [res] "=a" (res),     /* output */
			  [dst] "=m" (*dst)
			: [src] "r" (src),      /* input */
			  "a" (exp),
			  "m" (*dst)
			: "memory");            /* no-clobber list */
	return res;
}

/**
 * Combines 32b inputs most significant set bits into the least
 * significant bits to construct a value with the same MSBs as x
 * but all 1's under it.
 *
 * @param x
 *    The integer whose MSBs need to be combined with its LSBs
 * @return
 *    The combined value.
 */
static inline uint32_t
__combine32ms1b(uint32_t x)
{
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;

	return x;
}

/**
 * Aligns input parameter to the previous power of 2
 *
 * @param x
 *   The integer value to align
 *
 * @return
 *   Input parameter aligned to the previous power of 2
 */
static inline uint32_t
__align32prevpow2(uint32_t x)
{
	x = __combine32ms1b(x);

	return x - (x >> 1);
}

#ifndef likely
#define likely(x)	__builtin_expect(!!(x), 1)
#endif /* likely */

#ifndef unlikely
#define unlikely(x)	__builtin_expect(!!(x), 0)
#endif /* unlikely */

#define __RING_STAT_ADD(r, name, n) do {} while(0)

enum ringbuf_queue_behavior {
	RINGBUF_QUEUE_FIXED = 0, /* Enq/Deq a fixed number of items from a ring */
	RINGBUF_QUEUE_VARIABLE   /* Enq/Deq as many items a possible from ring */
};

#ifndef RINGBUF_PAUSE_REP_COUNT
#define RINGBUF_PAUSE_REP_COUNT 0 /** Yield after pause num of times, no yield \
                                    *   if RINGBUF_PAUSE_REP not defined. */
#endif

/**
 * An RTE ring structure.
 *
 * The producer and the consumer have a head and a tail index. The particularity
 * of these index is that they are not between 0 and size(ring). These indexes
 * are between 0 and 2^32, and we mask their value when we access the ring[]
 * field. Thanks to this assumption, we can do subtractions between 2 index
 * values in a modulo-32bit base: that's why the overflow of the indexes is not
 * a problem.
 */
struct ringbuf {
	int flags;                  /**< Flags supplied at creation. */
    uint32_t size;              /**< Size of ring. */
	uint32_t mask;              /**< Mask (size-1) of ring. */
    uint32_t capacity;          /**< Usable size of ring */

	/** Ring producer status. */
	struct prod {
		uint32_t sp_enqueue;     /**< True, if single producer. */
		volatile uint32_t head;  /**< Producer head. */
		volatile uint32_t tail;  /**< Producer tail. */
	} prod __attribute__((__aligned__(CACHE_LINE_SIZE)));

	/** Ring consumer status. */
	struct cons {
		uint32_t sc_dequeue;     /**< True, if single consumer. */
		volatile uint32_t head;  /**< Consumer head. */
		volatile uint32_t tail;  /**< Consumer tail. */
	} cons  __attribute__((__aligned__(CACHE_LINE_SIZE)));

	void *ring[] __attribute__((__aligned__(CACHE_LINE_SIZE)));
	/**< Memory space of ring starts here.
	                                     * about compiler re-ordering */
};

#define RING_F_SP_ENQ 0x0001 /**< The default enqueue is "single-producer". */
#define RING_F_SC_DEQ 0x0002 /**< The default dequeue is "single-consumer". */
/**
 * Ring is to hold exactly requested number of entries.
 * Without this flag set, the ring size requested must be a power of 2, and the
 * usable space will be that size - 1. With the flag, the requested size will
 * be rounded up to the next power of two, but the usable space will be exactly
 * that requested. Worst case, if a power-of-2 size is requested, half the
 * ring space will be wasted.
 */
#define RING_F_EXACT_SZ 0x0004
#define RINGBUF_SZ_MASK  (unsigned)(0x0fffffff) /**< Ring size mask */

/**
 * Calculate the memory size needed for a ring
 *
 * This function returns the number of bytes needed for a ring, given
 * the number of elements in it. This value is the sum of the size of
 * the structure ringbuf and the size of the memory needed by the
 * objects pointers. The value is aligned to a cache line size.
 *
 * @param count
 *   The number of elements in the ring (must be a power of 2).
 * @return
 *   - The memory size needed for the ring on success.
 *   - -EINVAL if count is not a power of 2.
 */
ssize_t ringbuf_get_memsize(unsigned count);

/**
 * Initialize a ring structure.
 *
 * The ring size is set to *count*, which must be a power of two. Water
 * marking is disabled by default. The real usable ring size is
 * *count-1* instead of *count* to differentiate a free ring from an
 * empty ring.
 *
 * @param r
 *   The pointer to the ring structure followed by the objects table.
 * @param count
 *   The number of elements in the ring (must be a power of 2).
 * @param flags
 *   An OR of the following:
 *    - RING_F_SP_ENQ: If this flag is set, the default behavior when
 *      using ``ringbuf_enqueue()`` or ``ringbuf_enqueue_bulk()``
 *      is "single-producer". Otherwise, it is "multi-producers".
 *    - RING_F_SC_DEQ: If this flag is set, the default behavior when
 *      using ``ringbuf_dequeue()`` or ``ringbuf_dequeue_bulk()``
 *      is "single-consumer". Otherwise, it is "multi-consumers".
 * @return
 *   0 on success, or a negative value on error.
 */
int ringbuf_init(struct ringbuf *r, unsigned count,
	unsigned flags);

/**
 * create a ringbuf
 *
 * The real usable ring size
 * is *count-1* instead of *count* to differentiate a free ring from an
 * empty ring.
 *
 * @param count
 *   The size of the ring (must be a power of 2).
 * @param flags
 *   An OR of the following:
 *    - RING_F_SP_ENQ: If this flag is set, the default behavior when
 *      using ``ringbuf_enqueue()`` or ``ringbuf_enqueue_bulk()``
 *      is "single-producer". Otherwise, it is "multi-producers".
 *    - RING_F_SC_DEQ: If this flag is set, the default behavior when
 *      using ``ringbuf_dequeue()`` or ``ringbuf_dequeue_bulk()``
 *      is "single-consumer". Otherwise, it is "multi-consumers".
 * @return
 *   On success, the pointer to the new allocated ring. NULL on error with
 *    rte_errno set appropriately. Possible errno values include:
 *    - E_NO_CONFIG - function could not get pointer to rte_config structure
 *    - E_SECONDARY - function was called from a secondary process instance
 *    - EINVAL - count provided is not a power of 2
 *    - ENOSPC - the maximum number of memzones has already been allocated
 *    - EEXIST - a memzone with the same name already exists
 *    - ENOMEM - no appropriate memory area found in which to create memzone
 */
struct ringbuf *ringbuf_create(unsigned count, unsigned flags);
/**
 * De-allocate all memory used by the ring.
 *
 * @param r
 *   Ring to free
 */
void ringbuf_free(struct ringbuf *r);

/**
 * Change the high water mark.
 *
 * If *count* is 0, water marking is disabled. Otherwise, it is set to the
 * *count* value. The *count* value must be greater than 0 and less
 * than the ring size.
 *
 * This function can be called at any time (not necessarily at
 * initialization).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param count
 *   The new water mark value.
 * @return
 *   - 0: Success; water mark changed.
 *   - -EINVAL: Invalid water mark value.
 */
int ringbuf_set_water_mark(struct ringbuf *r, unsigned count);

/**
 * Dump the status of the ring to a file.
 *
 * @param f
 *   A pointer to a file for output
 * @param r
 *   A pointer to the ring structure.
 */
void ringbuf_dump(FILE *f, const struct ringbuf *r);

static inline void
__ringbuf_enqueue_elems_32(struct ringbuf *r, const uint32_t size,
		uint32_t idx, const void *obj_table, uint32_t n)
{
	unsigned int i;
	uint32_t *ring = (uint32_t *)r->ring;
	const uint32_t *obj = (const uint32_t *)obj_table;
	if (likely(idx + n < size)) {
		for (i = 0; i < (n & ~0x7); i += 8, idx += 8) {
			ring[idx] = obj[i];
			ring[idx + 1] = obj[i + 1];
			ring[idx + 2] = obj[i + 2];
			ring[idx + 3] = obj[i + 3];
			ring[idx + 4] = obj[i + 4];
			ring[idx + 5] = obj[i + 5];
			ring[idx + 6] = obj[i + 6];
			ring[idx + 7] = obj[i + 7];
		}
		switch (n & 0x7) {
		case 7:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 6:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 5:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 4:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 3:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 2:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 1:
			ring[idx++] = obj[i++]; /* fallthrough */
		}
	} else {
		for (i = 0; idx < size; i++, idx++)
			ring[idx] = obj[i];
		/* Start at the beginning */
		for (idx = 0; i < n; i++, idx++)
			ring[idx] = obj[i];
	}
}

static inline void
__ringbuf_enqueue_elems_64(struct ringbuf *r, uint32_t prod_head,
		const void *obj_table, uint32_t n)
{
	unsigned int i;
	const uint32_t size = r->size;
	uint32_t idx = prod_head & r->mask;
	uint64_t *ring = (uint64_t *)&r[1];
	const uint64_t *obj = (const uint64_t *)obj_table;
	if (likely(idx + n < size)) {
		for (i = 0; i < (n & ~0x3); i += 4, idx += 4) {
			ring[idx] = obj[i];
			ring[idx + 1] = obj[i + 1];
			ring[idx + 2] = obj[i + 2];
			ring[idx + 3] = obj[i + 3];
		}
		switch (n & 0x3) {
		case 3:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 2:
			ring[idx++] = obj[i++]; /* fallthrough */
		case 1:
			ring[idx++] = obj[i++];
		}
	} else {
		for (i = 0; idx < size; i++, idx++)
			ring[idx] = obj[i];
		/* Start at the beginning */
		for (idx = 0; i < n; i++, idx++)
			ring[idx] = obj[i];
	}
}

static inline void
__ringbuf_enqueue_elems(struct ringbuf *r, uint32_t prod_head,
		const void *obj_table, uint32_t num)
{
    __ringbuf_enqueue_elems_64(r, prod_head, obj_table, num);
}

static inline void
__ringbuf_dequeue_elems_64(struct ringbuf *r, uint32_t prod_head,
		void *obj_table, uint32_t n)
{
	unsigned int i;
	const uint32_t size = r->size;
	uint32_t idx = prod_head & r->mask;
	uint64_t *ring = (uint64_t *)&r[1];
	uint64_t *obj = (uint64_t *)obj_table;
	if (likely(idx + n < size)) {
		for (i = 0; i < (n & ~0x3); i += 4, idx += 4) {
			obj[i] = ring[idx];
			obj[i + 1] = ring[idx + 1];
			obj[i + 2] = ring[idx + 2];
			obj[i + 3] = ring[idx + 3];
		}
		switch (n & 0x3) {
		case 3:
			obj[i++] = ring[idx++]; /* fallthrough */
		case 2:
			obj[i++] = ring[idx++]; /* fallthrough */
		case 1:
			obj[i++] = ring[idx++]; /* fallthrough */
		}
	} else {
		for (i = 0; idx < size; i++, idx++)
			obj[i] = ring[idx];
		/* Start at the beginning */
		for (idx = 0; i < n; i++, idx++)
			obj[i] = ring[idx];
	}
}

static inline void
__ringbuf_dequeue_elems(struct ringbuf *r, uint32_t cons_head,
		void *obj_table, uint32_t num)
{
	__ringbuf_dequeue_elems_64(r, cons_head, obj_table, num);
}

/* the actual enqueue of pointers on the ring.
 * Placed here since identical code needed in both
 * single and multi producer enqueue functions */
#define ENQUEUE_PTRS() do { \
	const uint32_t size = r->size; \
	uint32_t idx = prod_head & r->mask; \
	if (idx + n < size) { \
		for (i = 0; i < (n & ((~(unsigned)0x3))); i+=4, idx+=4) { \
			r->ring[idx] = obj_table[i]; \
			r->ring[idx+1] = obj_table[i+1]; \
			r->ring[idx+2] = obj_table[i+2]; \
			r->ring[idx+3] = obj_table[i+3]; \
		} \
		switch (n & 0x3) { \
			case 3: r->ring[idx++] = obj_table[i++]; \
			case 2: r->ring[idx++] = obj_table[i++]; \
			case 1: r->ring[idx++] = obj_table[i++]; \
		} \
	} else { \
		for (i = 0; idx < size; i++, idx++)\
			r->ring[idx] = obj_table[i]; \
		for (idx = 0; i < n; i++, idx++) \
			r->ring[idx] = obj_table[i]; \
	} \
} while(0)

/* the actual copy of pointers on the ring to obj_table.
 * Placed here since identical code needed in both
 * single and multi consumer dequeue functions */
#define DEQUEUE_PTRS() do { \
	uint32_t idx = cons_head & mask; \
	const uint32_t size = r->size; \
	if (idx + n < size) { \
		for (i = 0; i < (n & (~(unsigned)0x3)); i+=4, idx+=4) {\
			obj_table[i] = r->ring[idx]; \
			obj_table[i+1] = r->ring[idx+1]; \
			obj_table[i+2] = r->ring[idx+2]; \
			obj_table[i+3] = r->ring[idx+3]; \
		} \
		switch (n & 0x3) { \
			case 3: obj_table[i++] = r->ring[idx++]; \
			case 2: obj_table[i++] = r->ring[idx++]; \
			case 1: obj_table[i++] = r->ring[idx++]; \
		} \
	} else { \
		for (i = 0; idx < size; i++, idx++) \
			obj_table[i] = r->ring[idx]; \
		for (idx = 0; i < n; i++, idx++) \
			obj_table[i] = r->ring[idx]; \
	} \
} while (0)

static inline void
__ringbuf_update_prod_tail(struct ringbuf *r, uint32_t old_val,
		uint32_t new_val, uint32_t single, uint32_t enqueue)
{
	RINGBUF_SET_USED(enqueue);

	/*
	 * If there are other enqueues/dequeues in progress that preceded us,
	 * we need to wait for them to complete
	 */
	if (!single)
		while (unlikely(r->prod.tail != old_val))
			_mm_pause();

	__atomic_store_n(&r->prod.tail, new_val, __ATOMIC_RELEASE);
}

static inline void
__ringbuf_update_cons_tail(struct ringbuf *r, uint32_t old_val,
		uint32_t new_val, uint32_t single, uint32_t enqueue)
{
	RINGBUF_SET_USED(enqueue);

	/*
	 * If there are other enqueues/dequeues in progress that preceded us,
	 * we need to wait for them to complete
	 */
	if (!single)
		while (unlikely(r->cons.tail != old_val))
			_mm_pause();

	__atomic_store_n(&r->cons.tail, new_val, __ATOMIC_RELEASE);
}

static inline unsigned int
__ringbuf_move_prod_head(struct ringbuf *r, unsigned int is_sp,
		unsigned int n, enum ringbuf_queue_behavior behavior,
		uint32_t *old_head, uint32_t *new_head,
		uint32_t *free_entries)
{
	const uint32_t capacity = r->capacity;
	uint32_t cons_tail;
	unsigned int max = n;
	int success;

	*old_head = __atomic_load_n(&r->prod.head, __ATOMIC_RELAXED);
	do {
		/* Reset n to the initial burst count */
		n = max;

		/* Ensure the head is read before tail */
		__atomic_thread_fence(__ATOMIC_ACQUIRE);

		/* load-acquire synchronize with store-release of ht->tail
		 * in update_tail.
		 */
		cons_tail = __atomic_load_n(&r->cons.tail,
					__ATOMIC_ACQUIRE);

		/* The subtraction is done between two unsigned 32bits value
		 * (the result is always modulo 32 bits even if we have
		 * *old_head > cons_tail). So 'free_entries' is always between 0
		 * and capacity (which is < size).
		 */
		*free_entries = (capacity + cons_tail - *old_head);

		/* check that we have enough room in ring */
		if (unlikely(n > *free_entries))
			n = (behavior == RINGBUF_QUEUE_FIXED) ?
					0 : *free_entries;

		if (n == 0)
			return 0;

		*new_head = *old_head + n;
		if (is_sp)
			r->prod.head = *new_head, success = 1;
		else
			/* on failure, *old_head is updated */
			success = __atomic_compare_exchange_n(&r->prod.head,
					old_head, *new_head,
					0, __ATOMIC_RELAXED,
					__ATOMIC_RELAXED);
	} while (unlikely(success == 0));
	return n;
}

static inline unsigned int
__ringbuf_do_enqueue_elem(struct ringbuf *r, const void *obj_table, unsigned int n,
		enum ringbuf_queue_behavior behavior, unsigned int is_sp)
{
	uint32_t prod_head, prod_next;
	uint32_t free_entries;

	n = __ringbuf_move_prod_head(r, is_sp, n, behavior,
			&prod_head, &prod_next, &free_entries);
	if (n == 0)
		goto end;

	__ringbuf_enqueue_elems(r, prod_head, obj_table, n);

	__ringbuf_update_prod_tail(r, prod_head, prod_next, is_sp, 1);
end:
	// if (free_space != NULL)
	// 	*free_space = free_entries - n;
	return n;
}

static inline unsigned int
__ringbuf_move_cons_head(struct ringbuf *r, int is_sc,
		unsigned int n, enum ringbuf_queue_behavior behavior,
		uint32_t *old_head, uint32_t *new_head,
		uint32_t *entries)
{
	unsigned int max = n;
	uint32_t prod_tail;
	int success;

	/* move cons.head atomically */
	*old_head = __atomic_load_n(&r->cons.head, __ATOMIC_RELAXED);
	do {
		/* Restore n as it may change every loop */
		n = max;

		/* Ensure the head is read before tail */
		__atomic_thread_fence(__ATOMIC_ACQUIRE);

		/* this load-acquire synchronize with store-release of ht->tail
		 * in update_tail.
		 */
		prod_tail = __atomic_load_n(&r->prod.tail,
					__ATOMIC_ACQUIRE);

		/* The subtraction is done between two unsigned 32bits value
		 * (the result is always modulo 32 bits even if we have
		 * cons_head > prod_tail). So 'entries' is always between 0
		 * and size(ring)-1.
		 */
		*entries = (prod_tail - *old_head);

		/* Set the actual entries for dequeue */
		if (n > *entries)
			n = (behavior == RINGBUF_QUEUE_FIXED) ? 0 : *entries;

		if (unlikely(n == 0))
			return 0;

		*new_head = *old_head + n;
		if (is_sc)
			r->cons.head = *new_head, success = 1;
		else
			/* on failure, *old_head will be updated */
			success = __atomic_compare_exchange_n(&r->cons.head,
							old_head, *new_head,
							0, __ATOMIC_RELAXED,
							__ATOMIC_RELAXED);
	} while (unlikely(success == 0));
	return n;
}

static inline unsigned int
__ringbuf_do_dequeue_elem(struct ringbuf *r, void *obj_table, unsigned int n,
		enum ringbuf_queue_behavior behavior, unsigned int is_sc)
{
	uint32_t cons_head, cons_next;
	uint32_t entries;

	n = __ringbuf_move_cons_head(r, (int)is_sc, n, behavior,
			&cons_head, &cons_next, &entries);
	if (n == 0)
		goto end;

	__ringbuf_dequeue_elems(r, cons_head, obj_table, n);

	__ringbuf_update_cons_tail(r, cons_head, cons_next, is_sc, 0);

end:
	// if (available != NULL)
	// 	*available = entries - n;
	return n;
}

/**
 * @internal Enqueue several objects on the ring (multi-producers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * producer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @param behavior
 *   RINGBUF_QUEUE_FIXED:    Enqueue a fixed number of items from a ring
 *   RINGBUF_QUEUE_VARIABLE: Enqueue as many items a possible from ring
 * @return
 *   Depend on the behavior value
 *   if behavior = RINGBUF_QUEUE_FIXED
 *   - 0: Success; objects enqueue.
 *   - -ENOBUFS: Not enough room in the ring to enqueue, no object is enqueued.
 *   if behavior = RINGBUF_QUEUE_VARIABLE
 *   - n: Actual number of objects enqueued.
 */
static inline int __attribute__((always_inline))
__ringbuf_mp_do_enqueue(struct ringbuf *r, void * const *obj_table,
			 unsigned n, enum ringbuf_queue_behavior behavior)
{
	return __ringbuf_do_enqueue_elem(r, obj_table, n, behavior, 0);
}

/**
 * @internal Enqueue several objects on a ring (NOT multi-producers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @param behavior
 *   RINGBUF_QUEUE_FIXED:    Enqueue a fixed number of items from a ring
 *   RINGBUF_QUEUE_VARIABLE: Enqueue as many items a possible from ring
 * @return
 *   Depend on the behavior value
 *   if behavior = RINGBUF_QUEUE_FIXED
 *   - 0: Success; objects enqueue.
 *   - -ENOBUFS: Not enough room in the ring to enqueue, no object is enqueued.
 *   if behavior = RINGBUF_QUEUE_VARIABLE
 *   - n: Actual number of objects enqueued.
 */
static inline int __attribute__((always_inline))
__ringbuf_sp_do_enqueue(struct ringbuf *r, void * const *obj_table,
			 unsigned n, enum ringbuf_queue_behavior behavior)
{
	return __ringbuf_do_enqueue_elem(r, obj_table, n, behavior, 1);
}

/**
 * @internal Dequeue several objects from a ring (multi-consumers safe). When
 * the request objects are more than the available objects, only dequeue the
 * actual number of objects
 *
 * This function uses a "compare and set" instruction to move the
 * consumer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @param behavior
 *   RINGBUF_QUEUE_FIXED:    Dequeue a fixed number of items from a ring
 *   RINGBUF_QUEUE_VARIABLE: Dequeue as many items a possible from ring
 * @return
 *   Depend on the behavior value
 *   if behavior = RINGBUF_QUEUE_FIXED
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue; no object is
 *     dequeued.
 *   if behavior = RINGBUF_QUEUE_VARIABLE
 *   - n: Actual number of objects dequeued.
 */

static inline int __attribute__((always_inline))
__ringbuf_mc_do_dequeue(struct ringbuf *r, void **obj_table,
		 unsigned n, enum ringbuf_queue_behavior behavior)
{
    return __ringbuf_do_dequeue_elem(r, obj_table, n, behavior, 0);
}

/**
 * @internal Dequeue several objects from a ring (NOT multi-consumers safe).
 * When the request objects are more than the available objects, only dequeue
 * the actual number of objects
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @param behavior
 *   RINGBUF_QUEUE_FIXED:    Dequeue a fixed number of items from a ring
 *   RINGBUF_QUEUE_VARIABLE: Dequeue as many items a possible from ring
 * @return
 *   Depend on the behavior value
 *   if behavior = RINGBUF_QUEUE_FIXED
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue; no object is
 *     dequeued.
 *   if behavior = RINGBUF_QUEUE_VARIABLE
 *   - n: Actual number of objects dequeued.
 */
static inline int __attribute__((always_inline))
__ringbuf_sc_do_dequeue(struct ringbuf *r, void **obj_table,
		 unsigned n, enum ringbuf_queue_behavior behavior)
{
    return __ringbuf_do_dequeue_elem(r, obj_table, n, behavior, 1);
}

/**
 * Enqueue several objects on the ring (multi-producers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * producer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - 0: Success; objects enqueue.
 *   - -ENOBUFS: Not enough room in the ring to enqueue, no object is enqueued.
 */
static inline int __attribute__((always_inline))
ringbuf_mp_enqueue_bulk(struct ringbuf *r, void * const *obj_table,
			 unsigned n)
{
	return __ringbuf_mp_do_enqueue(r, obj_table, n, RINGBUF_QUEUE_FIXED) ? 0 : -ENOBUFS;
}

/**
 * Enqueue several objects on a ring (NOT multi-producers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - 0: Success; objects enqueued.
 *   - -ENOBUFS: Not enough room in the ring to enqueue; no object is enqueued.
 */
static inline int __attribute__((always_inline))
ringbuf_sp_enqueue_bulk(struct ringbuf *r, void * const *obj_table,
			 unsigned n)
{
	return __ringbuf_sp_do_enqueue(r, obj_table, n, RINGBUF_QUEUE_FIXED) ? 0 : -ENOBUFS;
}

/**
 * Enqueue several objects on a ring.
 *
 * This function calls the multi-producer or the single-producer
 * version depending on the default behavior that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - 0: Success; objects enqueued.
 *   - -ENOBUFS: Not enough room in the ring to enqueue; no object is enqueued.
 */
static inline int __attribute__((always_inline))
ringbuf_enqueue_bulk(struct ringbuf *r, void * const *obj_table,
		      unsigned n)
{
	if (r->prod.sp_enqueue)
		return ringbuf_sp_enqueue_bulk(r, obj_table, n);
	else
		return ringbuf_mp_enqueue_bulk(r, obj_table, n);
}

/**
 * Enqueue one object on a ring (multi-producers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * producer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj
 *   A pointer to the object to be added.
 * @return
 *   - 0: Success; objects enqueued.
 *   - -ENOBUFS: Not enough room in the ring to enqueue; no object is enqueued.
 */
static inline int __attribute__((always_inline))
ringbuf_mp_enqueue(struct ringbuf *r, void *obj)
{
	return ringbuf_mp_enqueue_bulk(r, &obj, 1);
}

/**
 * Enqueue one object on a ring (NOT multi-producers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj
 *   A pointer to the object to be added.
 * @return
 *   - 0: Success; objects enqueued.
 *   - -ENOBUFS: Not enough room in the ring to enqueue; no object is enqueued.
 */
static inline int __attribute__((always_inline))
ringbuf_sp_enqueue(struct ringbuf *r, void *obj)
{
	return ringbuf_sp_enqueue_bulk(r, &obj, 1);
}

/**
 * Enqueue one object on a ring.
 *
 * This function calls the multi-producer or the single-producer
 * version, depending on the default behaviour that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj
 *   A pointer to the object to be added.
 * @return
 *   - 0: Success; objects enqueued.
 *   - -ENOBUFS: Not enough room in the ring to enqueue; no object is enqueued.
 */
static inline int __attribute__((always_inline))
ringbuf_enqueue(struct ringbuf *r, void *obj)
{
	if (r->prod.sp_enqueue)
		return ringbuf_sp_enqueue(r, obj);
	else
		return ringbuf_mp_enqueue(r, obj);
}

/**
 * Dequeue several objects from a ring (multi-consumers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * consumer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue; no object is
 *     dequeued.
 */
static inline int __attribute__((always_inline))
ringbuf_mc_dequeue_bulk(struct ringbuf *r, void **obj_table, unsigned n)
{
	return __ringbuf_mc_do_dequeue(r, obj_table, n, RINGBUF_QUEUE_FIXED) ? 0 : -ENOENT;
}

/**
 * Dequeue several objects from a ring (NOT multi-consumers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table,
 *   must be strictly positive.
 * @return
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue; no object is
 *     dequeued.
 */
static inline int __attribute__((always_inline))
ringbuf_sc_dequeue_bulk(struct ringbuf *r, void **obj_table, unsigned n)
{
	return __ringbuf_sc_do_dequeue(r, obj_table, n, RINGBUF_QUEUE_FIXED) ? 0 : -ENOENT;
}

/**
 * Dequeue several objects from a ring.
 *
 * This function calls the multi-consumers or the single-consumer
 * version, depending on the default behaviour that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue, no object is
 *     dequeued.
 */
static inline int __attribute__((always_inline))
ringbuf_dequeue_bulk(struct ringbuf *r, void **obj_table, unsigned n)
{
	if (r->cons.sc_dequeue)
		return ringbuf_sc_dequeue_bulk(r, obj_table, n);
	else
		return ringbuf_mc_dequeue_bulk(r, obj_table, n);
}

/**
 * Dequeue one object from a ring (multi-consumers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * consumer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_p
 *   A pointer to a void * pointer (object) that will be filled.
 * @return
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue; no object is
 *     dequeued.
 */
static inline int __attribute__((always_inline))
ringbuf_mc_dequeue(struct ringbuf *r, void **obj_p)
{
	return ringbuf_mc_dequeue_bulk(r, obj_p, 1);
}

/**
 * Dequeue one object from a ring (NOT multi-consumers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_p
 *   A pointer to a void * pointer (object) that will be filled.
 * @return
 *   - 0: Success; objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue, no object is
 *     dequeued.
 */
static inline int __attribute__((always_inline))
ringbuf_sc_dequeue(struct ringbuf *r, void **obj_p)
{
	return ringbuf_sc_dequeue_bulk(r, obj_p, 1);
}

/**
 * Dequeue one object from a ring.
 *
 * This function calls the multi-consumers or the single-consumer
 * version depending on the default behaviour that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_p
 *   A pointer to a void * pointer (object) that will be filled.
 * @return
 *   - 0: Success, objects dequeued.
 *   - -ENOENT: Not enough entries in the ring to dequeue, no object is
 *     dequeued.
 */
static inline int __attribute__((always_inline))
ringbuf_dequeue(struct ringbuf *r, void **obj_p)
{
	if (r->cons.sc_dequeue)
		return ringbuf_sc_dequeue(r, obj_p);
	else
		return ringbuf_mc_dequeue(r, obj_p);
}

/**
 * Test if a ring is full.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   - 1: The ring is full.
 *   - 0: The ring is not full.
 */
static inline int
ringbuf_full(const struct ringbuf *r)
{
	uint32_t prod_tail = r->prod.tail;
	uint32_t cons_tail = r->cons.tail;
	return ((cons_tail - prod_tail - 1) & r->mask) == 0;
}

/**
 * Test if a ring is empty.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   - 1: The ring is empty.
 *   - 0: The ring is not empty.
 */
static inline int
ringbuf_empty(const struct ringbuf *r)
{
	uint32_t prod_tail = r->prod.tail;
	uint32_t cons_tail = r->cons.tail;
	return !!(cons_tail == prod_tail);
}

/**
 * Return the number of entries in a ring.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   The number of entries in the ring.
 */
static inline unsigned
ringbuf_count(const struct ringbuf *r)
{
	uint32_t prod_tail = r->prod.tail;
	uint32_t cons_tail = r->cons.tail;
	return (prod_tail - cons_tail) & r->mask;
}

/**
 * Return the number of free entries in a ring.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   The number of free entries in the ring.
 */
static inline unsigned
ringbuf_free_count(const struct ringbuf *r)
{
	uint32_t prod_tail = r->prod.tail;
	uint32_t cons_tail = r->cons.tail;
	return (cons_tail - prod_tail - 1) & r->mask;
}

/**
 * Return the size of the ring.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   The size of the data store used by the ring.
 *   NOTE: this is not the same as the usable space in the ring. To query that
 *   use ``ringbuf_get_capacity()``.
 */
static inline unsigned int
ringbuf_get_size(const struct ringbuf *r)
{
	return r->size;
}

/**
 * Return the number of elements which can be stored in the ring.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   The usable size of the ring.
 */
static inline unsigned int
ringbuf_get_capacity(const struct ringbuf *r)
{
	return r->capacity;
}

/**
 * Enqueue several objects on the ring (multi-producers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * producer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - n: Actual number of objects enqueued.
 */
static inline unsigned __attribute__((always_inline))
ringbuf_mp_enqueue_burst(struct ringbuf *r, void * const *obj_table,
			 unsigned n)
{
	return __ringbuf_mp_do_enqueue(r, obj_table, n, RINGBUF_QUEUE_VARIABLE);
}

/**
 * Enqueue several objects on a ring (NOT multi-producers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - n: Actual number of objects enqueued.
 */
static inline unsigned __attribute__((always_inline))
ringbuf_sp_enqueue_burst(struct ringbuf *r, void * const *obj_table,
			 unsigned n)
{
	return __ringbuf_sp_do_enqueue(r, obj_table, n, RINGBUF_QUEUE_VARIABLE);
}

/**
 * Enqueue several objects on a ring.
 *
 * This function calls the multi-producer or the single-producer
 * version depending on the default behavior that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - n: Actual number of objects enqueued.
 */
static inline unsigned __attribute__((always_inline))
ringbuf_enqueue_burst(struct ringbuf *r, void * const *obj_table,
		      unsigned n)
{
	if (r->prod.sp_enqueue)
		return ringbuf_sp_enqueue_burst(r, obj_table, n);
	else
		return ringbuf_mp_enqueue_burst(r, obj_table, n);
}

/**
 * Dequeue several objects from a ring (multi-consumers safe). When the request
 * objects are more than the available objects, only dequeue the actual number
 * of objects
 *
 * This function uses a "compare and set" instruction to move the
 * consumer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - n: Actual number of objects dequeued, 0 if ring is empty
 */
static inline unsigned __attribute__((always_inline))
ringbuf_mc_dequeue_burst(struct ringbuf *r, void **obj_table, unsigned n)
{
	return __ringbuf_mc_do_dequeue(r, obj_table, n, RINGBUF_QUEUE_VARIABLE);
}

/**
 * Dequeue several objects from a ring (NOT multi-consumers safe).When the
 * request objects are more than the available objects, only dequeue the
 * actual number of objects
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - n: Actual number of objects dequeued, 0 if ring is empty
 */
static inline unsigned __attribute__((always_inline))
ringbuf_sc_dequeue_burst(struct ringbuf *r, void **obj_table, unsigned n)
{
	return __ringbuf_sc_do_dequeue(r, obj_table, n, RINGBUF_QUEUE_VARIABLE);
}

/**
 * Dequeue multiple objects from a ring up to a maximum number.
 *
 * This function calls the multi-consumers or the single-consumer
 * version, depending on the default behaviour that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - Number of objects dequeued
 */
static inline unsigned __attribute__((always_inline))
ringbuf_dequeue_burst(struct ringbuf *r, void **obj_table, unsigned n)
{
	if (r->cons.sc_dequeue)
		return ringbuf_sc_dequeue_burst(r, obj_table, n);
	else
		return ringbuf_mc_dequeue_burst(r, obj_table, n);
}

/**
 * Flush a ring.
 *
 * This function flush all the elements in a ring
 *
 * @warning
 * Make sure the ring is not in use while calling this function.
 *
 * @param r
 *   A pointer to the ring structure.
 */
void
ringbuf_reset(struct ringbuf *r);

#ifdef __cplusplus
}
#endif

#endif /* _RINGBUF_H_ */
