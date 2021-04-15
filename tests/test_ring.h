/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2019 Arm Limited
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include <ringbuf.h>

/* API type to call
 * ringbuf_<sp/mp or sc/mc>_enqueue_<bulk/burst>
 * TEST_RING_THREAD_DEF - Uses configured SPSC/MPMC calls
 * TEST_RING_THREAD_SPSC - Calls SP or SC API
 * TEST_RING_THREAD_MPMC - Calls MP or MC API
 */
#define TEST_RING_THREAD_DEF 1
#define TEST_RING_THREAD_SPSC 2
#define TEST_RING_THREAD_MPMC 4

/* API type to call
 * TEST_RING_ELEM_SINGLE - Calls single element APIs
 * TEST_RING_ELEM_BULK - Calls bulk APIs
 * TEST_RING_ELEM_BURST - Calls burst APIs
 */
#define TEST_RING_ELEM_SINGLE 8
#define TEST_RING_ELEM_BULK 16
#define TEST_RING_ELEM_BURST 32

#define TEST_RING_IGNORE_API_TYPE ~0U

#define RINGBUF_MAX(a, b) \
	__extension__ ({ \
		typeof (a) _a = (a); \
		typeof (b) _b = (b); \
		_a > _b ? _a : _b; \
	})

#define RINGBUF_PTR_DIFF(ptr1, ptr2) ((uintptr_t)(ptr1) - (uintptr_t)(ptr2))

/* This function is placed here as it is required for both
 * performance and functional tests.
 */
static inline struct ringbuf*
test_ring_create(unsigned int count, unsigned int flags)
{
	return ringbuf_create(count, flags);
}

static inline void*
test_ring_inc_ptr(void *obj, unsigned int n)
{
	size_t sz;

	sz = sizeof(void *);

	return (void *)((uint32_t *)obj + (n * sz / sizeof(uint32_t)));
}

static inline void
test_ring_mem_copy(void *dst, void * const *src, unsigned int num)
{
	size_t sz;

	sz = num * sizeof(void *);

	memcpy(dst, src, sz);
}

static inline unsigned int
test_ring_enqueue(struct ringbuf *r, void **obj, unsigned int n,
			unsigned int api_type)
{
    switch (api_type) {
    case (TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE):
        return ringbuf_enqueue(r, *obj);
    case (TEST_RING_THREAD_SPSC | TEST_RING_ELEM_SINGLE):
        return ringbuf_sp_enqueue(r, *obj);
    case (TEST_RING_THREAD_MPMC | TEST_RING_ELEM_SINGLE):
        return ringbuf_mp_enqueue(r, *obj);
    case (TEST_RING_THREAD_DEF | TEST_RING_ELEM_BULK):
        return ringbuf_enqueue_bulk(r, obj, n);
    case (TEST_RING_THREAD_SPSC | TEST_RING_ELEM_BULK):
        return ringbuf_sp_enqueue_bulk(r, obj, n);
    case (TEST_RING_THREAD_MPMC | TEST_RING_ELEM_BULK):
        return ringbuf_mp_enqueue_bulk(r, obj, n);
    case (TEST_RING_THREAD_DEF | TEST_RING_ELEM_BURST):
        return ringbuf_enqueue_burst(r, obj, n);
    case (TEST_RING_THREAD_SPSC | TEST_RING_ELEM_BURST):
        return ringbuf_sp_enqueue_burst(r, obj, n);
    case (TEST_RING_THREAD_MPMC | TEST_RING_ELEM_BURST):
        return ringbuf_mp_enqueue_burst(r, obj, n);
    default:
        printf("Invalid API type\n");
        return 0;
    }

}

static inline unsigned int
test_ring_dequeue(struct ringbuf *r, void **obj, unsigned int n,
			unsigned int api_type)
{
    switch (api_type) {
    case (TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE):
        return ringbuf_dequeue(r, obj);
    case (TEST_RING_THREAD_SPSC | TEST_RING_ELEM_SINGLE):
        return ringbuf_sc_dequeue(r, obj);
    case (TEST_RING_THREAD_MPMC | TEST_RING_ELEM_SINGLE):
        return ringbuf_mc_dequeue(r, obj);
    case (TEST_RING_THREAD_DEF | TEST_RING_ELEM_BULK):
        return ringbuf_dequeue_bulk(r, obj, n);
    case (TEST_RING_THREAD_SPSC | TEST_RING_ELEM_BULK):
        return ringbuf_sc_dequeue_bulk(r, obj, n);
    case (TEST_RING_THREAD_MPMC | TEST_RING_ELEM_BULK):
        return ringbuf_mc_dequeue_bulk(r, obj, n);
    case (TEST_RING_THREAD_DEF | TEST_RING_ELEM_BURST):
        return ringbuf_dequeue_burst(r, obj, n);
    case (TEST_RING_THREAD_SPSC | TEST_RING_ELEM_BURST):
        return ringbuf_sc_dequeue_burst(r, obj, n);
    case (TEST_RING_THREAD_MPMC | TEST_RING_ELEM_BURST):
        return ringbuf_mc_dequeue_burst(r, obj, n);
    default:
        printf("Invalid API type\n");
        return 0;
    }
}

/* This function is placed here as it is required for both
 * performance and functional tests.
 */
static inline void *
test_ring_calloc(unsigned int rsize)
{
	unsigned int sz;
	void *p;

	sz = sizeof(void *);

	p = memalign(CACHE_LINE_SIZE, rsize * sz);
    memset(p, 0, rsize * sz);
	if (p == NULL)
		printf("Failed to allocate memory\n");

	return p;
}

int test_ring_negative_tests(void);
int test_ring_basic_ex(void);
int test_ring_with_exact_size(void);
int test_ring_burst_bulk_tests1(unsigned int test_idx);
int test_ring_burst_bulk_tests2(unsigned int test_idx);
int test_ring_burst_bulk_tests3(unsigned int test_idx);
int test_ring_burst_bulk_tests4(unsigned int test_idx);

#ifdef __cplusplus
}
#endif
