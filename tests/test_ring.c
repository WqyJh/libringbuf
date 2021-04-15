/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2014 Intel Corporation
 * Copyright(c) 2020 Arm Limited
 */

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <time.h>

#include "ringbuf.h"
#include "test_ring.h"

/*
 * Ring
 * ====
 *
 * #. Functional tests. Tests single/bulk/burst, default/SPSC/MPMC,
 *    legacy/custom element size (4B, 8B, 16B, 20B) APIs.
 *    Some tests incorporate unaligned addresses for objects.
 *    The enqueued/dequeued data is validated for correctness.
 *
 * #. Performance tests are in test_ring_perf.c
 */

#define RING_SIZE 4096
#define MAX_BULK 32

/*
 * Validate the return value of test cases and print details of the
 * ring if validation fails
 *
 * @param exp
 *   Expression to validate return value.
 * @param r
 *   A pointer to the ring structure.
 */
#define TEST_RING_VERIFY(exp, r, errst) do {				\
	if (!(exp)) {							\
		printf("error at %s:%d\tcondition " #exp " failed\n",	\
		    __func__, __LINE__);				\
		ringbuf_dump(stdout, (r));				\
		errst;							\
	}								\
} while (0)

#define TEST_RING_FULL_EMPTY_ITER	8

static const struct {
	const char *desc;
	uint32_t api_type;
	uint32_t create_flags;
	struct {
		unsigned int (*flegacy)(struct ringbuf *r,
			void * const *obj_table, unsigned int n,
			unsigned int *free_space);
	} enq;
	struct {
		unsigned int (*flegacy)(struct ringbuf *r,
			void **obj_table, unsigned int n,
			unsigned int *available);
	} deq;
} test_enqdeq_impl[] = {
	{
		.desc = "MP/MC sync mode",
		.api_type = TEST_RING_ELEM_BULK | TEST_RING_THREAD_DEF,
		.create_flags = 0,
		.enq = {
			.flegacy = ringbuf_enqueue_bulk,
		},
		.deq = {
			.flegacy = ringbuf_dequeue_bulk,
		},
	},
	{
		.desc = "SP/SC sync mode",
		.api_type = TEST_RING_ELEM_BULK | TEST_RING_THREAD_SPSC,
		.create_flags = RING_F_SP_ENQ | RING_F_SC_DEQ,
		.enq = {
			.flegacy = ringbuf_sp_enqueue_bulk,
		},
		.deq = {
			.flegacy = ringbuf_sc_dequeue_bulk,
		},
	},
	{
		.desc = "MP/MC sync mode",
		.api_type = TEST_RING_ELEM_BULK | TEST_RING_THREAD_MPMC,
		.create_flags = 0,
		.enq = {
			.flegacy = ringbuf_mp_enqueue_bulk,
		},
		.deq = {
			.flegacy = ringbuf_mc_dequeue_bulk,
		},
	}
};

unsigned int
test_ring_enq_impl(struct ringbuf *r, void **obj, unsigned int n,
	unsigned int test_idx)
{
	return test_enqdeq_impl[test_idx].enq.flegacy(r, obj, n, NULL);
}

unsigned int
test_ring_deq_impl(struct ringbuf *r, void **obj, unsigned int n,
	unsigned int test_idx)
{
	return test_enqdeq_impl[test_idx].deq.flegacy(r, obj, n, NULL);
}

void
test_ring_mem_init(void *obj, unsigned int count)
{
	unsigned int i;

    for (i = 0; i < count; i++)
        ((void **)obj)[i] = (void *)(uintptr_t)i;
}

#define LINE_LEN 128

void
rte_hexdump(FILE *f, const char *title, const void *buf, unsigned int len)
{
	unsigned int i, out, ofs;
	const unsigned char *data = buf;
	char line[LINE_LEN];	/* space needed 8+16*3+3+16 == 75 */

	fprintf(f, "%s at [%p], len=%u\n",
		title ? : "  Dump data", data, len);
	ofs = 0;
	while (ofs < len) {
		/* format the line in the buffer */
		out = snprintf(line, LINE_LEN, "%08X:", ofs);
		for (i = 0; i < 16; i++) {
			if (ofs + i < len)
				snprintf(line + out, LINE_LEN - out,
					 " %02X", (data[ofs + i] & 0xff));
			else
				strcpy(line + out, "   ");
			out += 3;
		}


		for (; i <= 16; i++)
			out += snprintf(line + out, LINE_LEN - out, " | ");

		for (i = 0; ofs < len && i < 16; i++, ofs++) {
			unsigned char c = data[ofs];

			if (c < ' ' || c > '~')
				c = '.';
			out += snprintf(line + out, LINE_LEN - out, "%c", c);
		}
		fprintf(f, "%s\n", line);
	}
	fflush(f);
}

static int
test_ring_mem_cmp(void *src, void *dst, unsigned int size)
{
	int ret;

	ret = memcmp(src, dst, size);
	if (ret) {
		rte_hexdump(stdout, "src", src, size);
		rte_hexdump(stdout, "dst", dst, size);
		printf("data after dequeue is not the same\n");
	}

	return ret;
}

static void
test_ring_print_test_string(const char *istr, unsigned int api_type)
{
	printf("\n%s: ", istr);

	if (api_type == TEST_RING_IGNORE_API_TYPE)
		return;

	if (api_type & TEST_RING_THREAD_DEF)
		printf(": default enqueue/dequeue: ");
	else if (api_type & TEST_RING_THREAD_SPSC)
		printf(": SP/SC: ");
	else if (api_type & TEST_RING_THREAD_MPMC)
		printf(": MP/MC: ");

	if (api_type & TEST_RING_ELEM_SINGLE)
		printf("single\n");
	else if (api_type & TEST_RING_ELEM_BULK)
		printf("bulk\n");
	else if (api_type & TEST_RING_ELEM_BURST)
		printf("burst\n");
}

/*
 * Various negative test cases.
 */
int
test_ring_negative_tests(void)
{
	struct ringbuf *rp = NULL;
	struct ringbuf *rt = NULL;

    /* Test if ring size is not power of 2 */
    rp = test_ring_create(RING_SIZE + 1, 0);
    if (rp != NULL) {
        printf("Test failed to detect odd count\n");
        goto test_fail;
    }

    /* Test if ring size is exceeding the limit */
    rp = test_ring_create(RINGBUF_SZ_MASK + 1, 0);
    
    if (rp != NULL) {
        printf("Test failed to detect limits\n");
        goto test_fail;
    }

    /* Test to if a non-power of 2 count causes the create
        * function to fail correctly
        */
    rp = test_ring_create(4097, 0);
    if (rp != NULL)
        goto test_fail;

    rp = test_ring_create(RING_SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
    if (rp == NULL) {
        printf("test_ring_negative fail to create ring\n");
        goto test_fail;
    }

    TEST_RING_VERIFY(ringbuf_empty(rp) == 1, rp, goto test_fail);

    ringbuf_free(rp);
    rp = NULL;

	return 0;

test_fail:

	ringbuf_free(rp);
	return -1;
}

/*
 * Burst and bulk operations with sp/sc, mp/mc and default (during creation)
 * Random number of elements are enqueued and dequeued.
 */
int
test_ring_burst_bulk_tests1(unsigned int test_idx)
{
	struct ringbuf *r;
	void **src = NULL, **cur_src = NULL, **dst = NULL, **cur_dst = NULL;
	int ret;
	unsigned int j, temp_sz;
	int randn;
	const unsigned int rsz = RING_SIZE - 1;

    test_ring_print_test_string(test_enqdeq_impl[test_idx].desc,
        test_enqdeq_impl[test_idx].api_type);

    /* Create the ring */
    r = test_ring_create(RING_SIZE, test_enqdeq_impl[test_idx].create_flags);

    /* alloc dummy object pointers */
    src = test_ring_calloc(RING_SIZE * 2);
    if (src == NULL)
        goto fail;
    test_ring_mem_init(src, RING_SIZE * 2);
    cur_src = src;

    /* alloc some room for copied objects */
    dst = test_ring_calloc(RING_SIZE * 2);
    if (dst == NULL)
        goto fail;
    cur_dst = dst;

    printf("Random full/empty test\n");

    srand(time(NULL));
    for (j = 0; j != TEST_RING_FULL_EMPTY_ITER; j++) {
        /* random shift in the ring */
        randn = RINGBUF_MAX(rand() % RING_SIZE, 1UL);
        printf("%s: random shift: %u;\n",
            __func__, randn);
        ret = test_ring_enq_impl(r, cur_src, randn,
                        test_idx);
        TEST_RING_VERIFY(ret != 0, r, goto fail);

        ret = test_ring_deq_impl(r, cur_dst, randn,
                        test_idx);
        TEST_RING_VERIFY(ret == randn, r, goto fail);

        /* fill the ring */
        ret = test_ring_enq_impl(r, cur_src, rsz,
                        test_idx);
        TEST_RING_VERIFY(ret != 0, r, goto fail);

        TEST_RING_VERIFY(ringbuf_free_count(r) == 0, r, goto fail);
        TEST_RING_VERIFY(rsz == ringbuf_count(r), r, goto fail);
        TEST_RING_VERIFY(ringbuf_full(r), r, goto fail);
        TEST_RING_VERIFY(ringbuf_empty(r) == 0, r, goto fail);

        /* empty the ring */
        ret = test_ring_deq_impl(r, cur_dst, rsz,
                        test_idx);
        TEST_RING_VERIFY(ret == (int)rsz, r, goto fail);

        TEST_RING_VERIFY(rsz == ringbuf_free_count(r), r, goto fail);
        TEST_RING_VERIFY(ringbuf_count(r) == 0, r, goto fail);
        TEST_RING_VERIFY(ringbuf_full(r) == 0, r, goto fail);
        TEST_RING_VERIFY(ringbuf_empty(r), r, goto fail);

        /* check data */
        temp_sz = rsz * sizeof(void *);
        TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                    temp_sz) == 0, r, goto fail);
    }

    /* Free memory before test completed */
    ringbuf_free(r);
    free(src);
    free(dst);
    r = NULL;
    src = NULL;
    dst = NULL;

	return 0;
fail:
	ringbuf_free(r);
	free(src);
	free(dst);
	return -1;
}

/*
 * Burst and bulk operations with sp/sc, mp/mc and default (during creation)
 * Sequence of simple enqueues/dequeues and validate the enqueued and
 * dequeued data.
 */
int
test_ring_burst_bulk_tests2(unsigned int test_idx)
{
	struct ringbuf *r;
	void **src = NULL, **cur_src = NULL, **dst = NULL, **cur_dst = NULL;
	int ret;
	unsigned int i;

    test_ring_print_test_string(test_enqdeq_impl[test_idx].desc,
        test_enqdeq_impl[test_idx].api_type);

    /* Create the ring */
    r = test_ring_create(RING_SIZE, test_enqdeq_impl[test_idx].create_flags);

    /* alloc dummy object pointers */
    src = test_ring_calloc(RING_SIZE * 2);
    if (src == NULL)
        goto fail;
    test_ring_mem_init(src, RING_SIZE * 2);
    cur_src = src;

    /* alloc some room for copied objects */
    dst = test_ring_calloc(RING_SIZE * 2);
    if (dst == NULL)
        goto fail;
    cur_dst = dst;

    printf("enqueue 1 obj\n");
    ret = test_ring_enq_impl(r, cur_src, 1, test_idx);
    TEST_RING_VERIFY(ret == 1, r, goto fail);
    cur_src = test_ring_inc_ptr(cur_src, 1);

    printf("enqueue 2 objs\n");
    ret = test_ring_enq_impl(r, cur_src, 2, test_idx);
    TEST_RING_VERIFY(ret == 2, r, goto fail);
    cur_src = test_ring_inc_ptr(cur_src, 2);

    printf("enqueue MAX_BULK objs\n");
    ret = test_ring_enq_impl(r, cur_src, MAX_BULK,
                    test_idx);
    TEST_RING_VERIFY(ret == MAX_BULK, r, goto fail);

    printf("dequeue 1 obj\n");
    ret = test_ring_deq_impl(r, cur_dst, 1, test_idx);
    TEST_RING_VERIFY(ret == 1, r, goto fail);
    cur_dst = test_ring_inc_ptr(cur_dst, 1);

    printf("dequeue 2 objs\n");
    ret = test_ring_deq_impl(r, cur_dst, 2, test_idx);
    TEST_RING_VERIFY(ret == 2, r, goto fail);
    cur_dst = test_ring_inc_ptr(cur_dst, 2);

    printf("dequeue MAX_BULK objs\n");
    ret = test_ring_deq_impl(r, cur_dst, MAX_BULK,
                    test_idx);
    TEST_RING_VERIFY(ret == MAX_BULK, r, goto fail);
    cur_dst = test_ring_inc_ptr(cur_dst, MAX_BULK);

    /* check data */
    TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                RINGBUF_PTR_DIFF(cur_dst, dst)) == 0,
                r, goto fail);

    /* Free memory before test completed */
    ringbuf_free(r);
    free(src);
    free(dst);
    r = NULL;
    src = NULL;
    dst = NULL;

	return 0;
fail:
	ringbuf_free(r);
	free(src);
	free(dst);
	return -1;
}

/*
 * Burst and bulk operations with sp/sc, mp/mc and default (during creation)
 * Enqueue and dequeue to cover the entire ring length.
 */
int
test_ring_burst_bulk_tests3(unsigned int test_idx)
{
	struct ringbuf *r;
	void **src = NULL, **cur_src = NULL, **dst = NULL, **cur_dst = NULL;
	int ret;
	unsigned int i, j;

    test_ring_print_test_string(test_enqdeq_impl[test_idx].desc,
        test_enqdeq_impl[test_idx].api_type);

    /* Create the ring */
    r = test_ring_create(RING_SIZE, test_enqdeq_impl[test_idx].create_flags);

    /* alloc dummy object pointers */
    src = test_ring_calloc(RING_SIZE * 2);
    if (src == NULL)
        goto fail;
    test_ring_mem_init(src, RING_SIZE * 2);
    cur_src = src;

    /* alloc some room for copied objects */
    dst = test_ring_calloc(RING_SIZE * 2);
    if (dst == NULL)
        goto fail;
    cur_dst = dst;

    printf("fill and empty the ring\n");
    for (j = 0; j < RING_SIZE / MAX_BULK; j++) {
        ret = test_ring_enq_impl(r, cur_src, MAX_BULK,
                        test_idx);
        TEST_RING_VERIFY(ret == MAX_BULK, r, goto fail);
        cur_src = test_ring_inc_ptr(cur_src,
                            MAX_BULK);

        ret = test_ring_deq_impl(r, cur_dst, MAX_BULK,
                        test_idx);
        TEST_RING_VERIFY(ret == MAX_BULK, r, goto fail);
        cur_dst = test_ring_inc_ptr(cur_dst,
                            MAX_BULK);
    }

    /* check data */
    TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                RINGBUF_PTR_DIFF(cur_dst, dst)) == 0,
                r, goto fail);

    /* Free memory before test completed */
    ringbuf_free(r);
    free(src);
    free(dst);
    r = NULL;
    src = NULL;
    dst = NULL;

	return 0;
fail:
	ringbuf_free(r);
	free(src);
	free(dst);
	return -1;
}

/*
 * Burst and bulk operations with sp/sc, mp/mc and default (during creation)
 * Enqueue till the ring is full and dequeue till the ring becomes empty.
 */
int
test_ring_burst_bulk_tests4(unsigned int test_idx)
{
	struct ringbuf *r;
	void **src = NULL, **cur_src = NULL, **dst = NULL, **cur_dst = NULL;
	int ret;
	unsigned int i, j;
	unsigned int api_type, num_elems;

	api_type = test_enqdeq_impl[test_idx].api_type;

    test_ring_print_test_string(test_enqdeq_impl[test_idx].desc,
        test_enqdeq_impl[test_idx].api_type);

    /* Create the ring */
    r = test_ring_create(RING_SIZE, test_enqdeq_impl[test_idx].create_flags);

    /* alloc dummy object pointers */
    src = test_ring_calloc(RING_SIZE * 2);
    if (src == NULL)
        goto fail;
    test_ring_mem_init(src, RING_SIZE * 2);
    cur_src = src;

    /* alloc some room for copied objects */
    dst = test_ring_calloc(RING_SIZE * 2);
    if (dst == NULL)
        goto fail;
    cur_dst = dst;

    printf("Test enqueue without enough memory space\n");
    for (j = 0; j < (RING_SIZE/MAX_BULK - 1); j++) {
        ret = test_ring_enq_impl(r, cur_src, MAX_BULK,
                        test_idx);
        TEST_RING_VERIFY(ret == MAX_BULK, r, goto fail);
        cur_src = test_ring_inc_ptr(cur_src,
                            MAX_BULK);
    }

    printf("Enqueue 2 objects, free entries = MAX_BULK - 2\n");
    ret = test_ring_enq_impl(r, cur_src, 2, test_idx);
    TEST_RING_VERIFY(ret == 2, r, goto fail);
    cur_src = test_ring_inc_ptr(cur_src, 2);

    printf("Enqueue the remaining entries = MAX_BULK - 3\n");
    /* Bulk APIs enqueue exact number of elements */
    if ((api_type & TEST_RING_ELEM_BULK) == TEST_RING_ELEM_BULK)
        num_elems = MAX_BULK - 3;
    else
        num_elems = MAX_BULK;
    /* Always one free entry left */
    ret = test_ring_enq_impl(r, cur_src, num_elems,
                    test_idx);
    TEST_RING_VERIFY(ret == MAX_BULK - 3, r, goto fail);
    cur_src = test_ring_inc_ptr(cur_src, MAX_BULK - 3);

    printf("Test if ring is full\n");
    TEST_RING_VERIFY(ringbuf_full(r) == 1, r, goto fail);

    printf("Test enqueue for a full entry\n");
    ret = test_ring_enq_impl(r, cur_src, MAX_BULK,
                    test_idx);
    TEST_RING_VERIFY(ret == 0, r, goto fail);

    printf("Test dequeue without enough objects\n");
    for (j = 0; j < RING_SIZE / MAX_BULK - 1; j++) {
        ret = test_ring_deq_impl(r, cur_dst, MAX_BULK,
                        test_idx);
        TEST_RING_VERIFY(ret == MAX_BULK, r, goto fail);
        cur_dst = test_ring_inc_ptr(cur_dst,
                            MAX_BULK);
    }

    /* Available memory space for the exact MAX_BULK entries */
    ret = test_ring_deq_impl(r, cur_dst, 2, test_idx);
    TEST_RING_VERIFY(ret == 2, r, goto fail);
    cur_dst = test_ring_inc_ptr(cur_dst, 2);

    /* Bulk APIs enqueue exact number of elements */
    if ((api_type & TEST_RING_ELEM_BULK) == TEST_RING_ELEM_BULK)
        num_elems = MAX_BULK - 3;
    else
        num_elems = MAX_BULK;
    ret = test_ring_deq_impl(r, cur_dst, num_elems,
                    test_idx);
    TEST_RING_VERIFY(ret == MAX_BULK - 3, r, goto fail);
    cur_dst = test_ring_inc_ptr(cur_dst, MAX_BULK - 3);

    printf("Test if ring is empty\n");
    /* Check if ring is empty */
    TEST_RING_VERIFY(ringbuf_empty(r) == 1, r, goto fail);

    /* check data */
    TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                RINGBUF_PTR_DIFF(cur_dst, dst)) == 0,
                r, goto fail);

    /* Free memory before test completed */
    ringbuf_free(r);
    free(src);
    free(dst);
    r = NULL;
    src = NULL;
    dst = NULL;

	return 0;
fail:
	ringbuf_free(r);
	free(src);
	free(dst);
	return -1;
}

/*
 * Test default, single element, bulk and burst APIs
 */
int
test_ring_basic_ex(void)
{
	int ret = -1;
	unsigned int i, j;
	struct ringbuf *rp = NULL;
	void **src = NULL, **cur_src = NULL, **dst = NULL, **cur_dst = NULL;

    rp = test_ring_create(RING_SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
    if (rp == NULL) {
        printf("%s: failed to create ring\n", __func__);
        goto fail_test;
    }

    /* alloc dummy object pointers */
    src = test_ring_calloc(RING_SIZE);
    if (src == NULL) {
        printf("%s: failed to alloc src memory\n", __func__);
        goto fail_test;
    }
    test_ring_mem_init(src, RING_SIZE);
    cur_src = src;

    /* alloc some room for copied objects */
    dst = test_ring_calloc(RING_SIZE);
    if (dst == NULL) {
        printf("%s: failed to alloc dst memory\n", __func__);
        goto fail_test;
    }
    cur_dst = dst;

    TEST_RING_VERIFY(ringbuf_empty(rp) == 1, rp, goto fail_test);

    printf("%u ring entries are now free\n",
        ringbuf_free_count(rp));

    for (j = 0; j < RING_SIZE - 1; j++) {
        ret = test_ring_enqueue(rp, cur_src, 1,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE);
        TEST_RING_VERIFY(ret == 0, rp, goto fail_test);
        cur_src = test_ring_inc_ptr(cur_src, 1);
    }

    TEST_RING_VERIFY(ringbuf_full(rp) == 1, rp, goto fail_test);

    for (j = 0; j < RING_SIZE - 1; j++) {
        ret = test_ring_dequeue(rp, cur_dst, 1,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE);
        TEST_RING_VERIFY(ret == 0, rp, goto fail_test);
        cur_dst = test_ring_inc_ptr(cur_dst, 1);
    }

    TEST_RING_VERIFY(ringbuf_empty(rp) == 1, rp, goto fail_test);

    /* check data */
    TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                RINGBUF_PTR_DIFF(cur_dst, dst)) == 0,
                rp, goto fail_test);

    /* Following tests use the configured flags to decide
        * SP/SC or MP/MC.
        */
    /* reset memory of dst */
    memset(dst, 0, RINGBUF_PTR_DIFF(cur_dst, dst));

    /* reset cur_src and cur_dst */
    cur_src = src;
    cur_dst = dst;

    /* Covering the ring burst operation */
    ret = test_ring_enqueue(rp, cur_src, 2,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_BURST);
    TEST_RING_VERIFY(ret == 2, rp, goto fail_test);
    cur_src = test_ring_inc_ptr(cur_src, 2);

    ret = test_ring_dequeue(rp, cur_dst, 2,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_BURST);
    TEST_RING_VERIFY(ret == 2, rp, goto fail_test);
    cur_dst = test_ring_inc_ptr(cur_dst, 2);

    /* Covering the ring bulk operation */
    ret = test_ring_enqueue(rp, cur_src, 2,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_BULK);
    TEST_RING_VERIFY(ret == 0, rp, goto fail_test);

    ret = test_ring_dequeue(rp, cur_dst, 2,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_BULK);
    TEST_RING_VERIFY(ret == 0, rp, goto fail_test);
    cur_dst = test_ring_inc_ptr(cur_dst, 2);

    /* check data */
    TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                RINGBUF_PTR_DIFF(cur_dst, dst)) == 0,
                rp, goto fail_test);

    ringbuf_free(rp);
    free(src);
    free(dst);
    rp = NULL;
    src = NULL;
    dst = NULL;

	return 0;

fail_test:
	ringbuf_free(rp);
	free(src);
	free(dst);
	return -1;
}

/*
 * Basic test cases with exact size ring.
 */
int
test_ring_with_exact_size(void)
{
	struct ringbuf *std_r = NULL, *exact_sz_r = NULL;
	void **src_orig = NULL, **dst_orig = NULL;
	void **src = NULL, **cur_src = NULL, **dst = NULL, **cur_dst = NULL;
	const unsigned int ring_sz = 16;
	unsigned int i, j;
	int ret = -1;

    test_ring_print_test_string("Test exact size ring", TEST_RING_IGNORE_API_TYPE);

    std_r = test_ring_create(ring_sz, RING_F_SP_ENQ | RING_F_SC_DEQ);
    if (std_r == NULL) {
        printf("%s: error, can't create std ring\n", __func__);
        goto test_fail;
    }
    exact_sz_r = test_ring_create(ring_sz, RING_F_SP_ENQ | RING_F_SC_DEQ | RING_F_EXACT_SZ);
    if (exact_sz_r == NULL) {
        printf("%s: error, can't create exact size ring\n",
                __func__);
        goto test_fail;
    }

    /* alloc object pointers. Allocate one extra object
        * and create an unaligned address.
        */
    src_orig = test_ring_calloc(17);
    if (src_orig == NULL)
        goto test_fail;
    test_ring_mem_init(src_orig, 17);
    src = (void **)((uintptr_t)src_orig + 1);
    cur_src = src;

    dst_orig = test_ring_calloc(17);
    if (dst_orig == NULL)
        goto test_fail;
    dst = (void **)((uintptr_t)dst_orig + 1);
    cur_dst = dst;

    /*
    * Check that the exact size ring is bigger than the
    * standard ring
    */
    TEST_RING_VERIFY(ringbuf_get_size(std_r) <=
            ringbuf_get_size(exact_sz_r),
            std_r, goto test_fail);

    /*
    * check that the exact_sz_ring can hold one more element
    * than the standard ring. (16 vs 15 elements)
    */
    for (j = 0; j < ring_sz - 1; j++) {
        ret = test_ring_enqueue(std_r, cur_src, 1,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE);
        TEST_RING_VERIFY(ret == 0, std_r, goto test_fail);
        ret = test_ring_enqueue(exact_sz_r, cur_src, 1,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE);
        TEST_RING_VERIFY(ret == 0, exact_sz_r, goto test_fail);
        cur_src = test_ring_inc_ptr(cur_src, 1);
    }
    ret = test_ring_enqueue(std_r, cur_src, 1,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE);
    TEST_RING_VERIFY(ret == -ENOBUFS, std_r, goto test_fail);
    ret = test_ring_enqueue(exact_sz_r, cur_src, 1,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_SINGLE);
    TEST_RING_VERIFY(ret != -ENOBUFS, exact_sz_r, goto test_fail);

    /* check that dequeue returns the expected number of elements */
    ret = test_ring_dequeue(exact_sz_r, cur_dst, ring_sz,
            TEST_RING_THREAD_DEF | TEST_RING_ELEM_BURST);
    TEST_RING_VERIFY(ret == (int)ring_sz, exact_sz_r, goto test_fail);
    cur_dst = test_ring_inc_ptr(cur_dst, ring_sz);

    /* check that the capacity function returns expected value */
    TEST_RING_VERIFY(ringbuf_get_capacity(exact_sz_r) == ring_sz,
                exact_sz_r, goto test_fail);

    /* check data */
    TEST_RING_VERIFY(test_ring_mem_cmp(src, dst,
                RINGBUF_PTR_DIFF(cur_dst, dst)) == 0,
                exact_sz_r, goto test_fail);

    free(src_orig);
    free(dst_orig);
    ringbuf_free(std_r);
    ringbuf_free(exact_sz_r);
    src_orig = NULL;
    dst_orig = NULL;
    std_r = NULL;
    exact_sz_r = NULL;

	return 0;

test_fail:
	free(src_orig);
	free(dst_orig);
	ringbuf_free(std_r);
	ringbuf_free(exact_sz_r);
	return -1;
}

int test_ring(void)
{
	int32_t rc;
	unsigned int i;

	/* Negative test cases */
	if (test_ring_negative_tests() < 0)
		goto test_fail;

	/* Some basic operations */
	if (test_ring_basic_ex() < 0)
		goto test_fail;

	if (test_ring_with_exact_size() < 0)
		goto test_fail;

	/* Burst and bulk operations with sp/sc, mp/mc and default.
	 * The test cases are split into smaller test cases to
	 * help clang compile faster.
	 */
	for (i = 0; i != RINGBUF_DIM(test_enqdeq_impl); i++) {


		rc = test_ring_burst_bulk_tests1(i);
		if (rc < 0)
			goto test_fail;

		rc = test_ring_burst_bulk_tests2(i);
		if (rc < 0)
			goto test_fail;

		rc = test_ring_burst_bulk_tests3(i);
		if (rc < 0)
			goto test_fail;

		rc = test_ring_burst_bulk_tests4(i);
		if (rc < 0)
			goto test_fail;
	}

	return 0;

test_fail:

	return -1;
}
