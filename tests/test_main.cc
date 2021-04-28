#include <cstdint>
#include <thread>
#include <atomic>

#include <gtest/gtest.h>

#include "ringbuf.h"
#include "test_ring.h"

TEST(test_ring, test_ring_negative_tests) {
    int rc = test_ring_negative_tests();
    EXPECT_EQ(rc, 0);
}

TEST(test_ring, test_ring_basic_ex) {
    int rc = test_ring_basic_ex();
    EXPECT_EQ(rc, 0);
}

TEST(test_ring, test_ring_with_exact_size) {
    int rc = test_ring_with_exact_size();
    EXPECT_EQ(rc, 0);
}

#define RING_SIZE 305
#define BURST_SIZE 64
TEST(ring, enq_deq_burst) {
    struct ringbuf *r = ringbuf_create(RING_SIZE, RING_F_EXACT_SZ);
    int64_t bufs[BURST_SIZE];
    int enqueued;
    int dequeued;
    for (int i = 0; i < BURST_SIZE; i++) {
        bufs[i] = i;
    }
    enqueued = ringbuf_enqueue_burst(r, (void **)bufs, BURST_SIZE);
    EXPECT_EQ(BURST_SIZE, enqueued);
    int64_t result[BURST_SIZE];
    dequeued = ringbuf_dequeue_burst(r, (void **)result, BURST_SIZE);
    EXPECT_EQ(BURST_SIZE, dequeued);
    for (int i = 0; i < BURST_SIZE; i++) {
        EXPECT_EQ(bufs[i], result[i]);
    }
    dequeued = ringbuf_dequeue_burst(r, (void **)result, BURST_SIZE);
    EXPECT_EQ(0, dequeued);
}

TEST(ring, enq_deq_full) {
    struct ringbuf *r = ringbuf_create(RING_SIZE, RING_F_EXACT_SZ);
    int64_t bufs[RING_SIZE];
    int enqueued = 0;
    int dequeued = 0;
    for (int i = 0; i < RING_SIZE; i++) {
        bufs[i] = i;
    }
    for (int i = 0; i < RING_SIZE; i++) {
        EXPECT_EQ(bufs[i], i);
    }
    for (int i = 0; i < RING_SIZE / BURST_SIZE + 1; i++) {
        int n = ringbuf_enqueue_burst(r, (void **)(bufs + enqueued), BURST_SIZE);
        enqueued += n;
    }
    EXPECT_EQ(RING_SIZE, enqueued);
    for (int i = 0; i < RING_SIZE; i++) {
        EXPECT_EQ(bufs[i], i);
    }

    int64_t result[RING_SIZE];
    for (int i = 0; i < RING_SIZE / BURST_SIZE + 1; i++) {
        int n = ringbuf_dequeue_burst(r, (void **)(result + dequeued), BURST_SIZE);
        dequeued += n;
    }
    for (int i = 0; i < RING_SIZE; i++) {
        EXPECT_EQ(bufs[i], i);
    }
    ringbuf_dump(stdout, r);
    EXPECT_EQ(RING_SIZE, dequeued);
    for (int i = 0; i < RING_SIZE; i++) {
        EXPECT_EQ(bufs[i], result[i]);
    }
    dequeued = ringbuf_dequeue_burst(r, (void **)result, BURST_SIZE);
    EXPECT_EQ(0, dequeued);
}

#define NUM_THREADS 16
#define COUNT 10000000

TEST(ring, enq_counter) {
    struct ringbuf *r = ringbuf_create(RING_SIZE, RING_F_EXACT_SZ);

    std::vector<std::thread> threads;
    std::atomic<int> counter(0);
    for (int i = 0; i < NUM_THREADS; i++) {
        threads.push_back(std::thread([&](){
            int c = 0;
            for (int j = 0; j < COUNT; j++) {
                if (ringbuf_enqueue(r, (void *)(uint64_t)i) == 0) {
                    ++c;
                }
            }
            counter.fetch_add(c);
        }));
    }
    for (auto& thread: threads) {
        thread.join();
    }
    EXPECT_EQ(RING_SIZE, counter);
}
