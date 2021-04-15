#include <string.h>
#include <benchmark/benchmark.h>

#include <ringbuf.h>

#define SIZE 8192

static void fill_ring(struct ringbuf *r)
{
    ringbuf_reset(r);
    void **a = new void*[4096];
    memset(a, 1, sizeof(void *) * 4096);
    for (int i = 0; i < SIZE / 4096; i++) {
        ringbuf_enqueue_burst(r, a, 4096);
    }
}

static void BM_enq1(benchmark::State& state) {
    struct ringbuf *r = ringbuf_create(SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
    int a = 1;
    for (auto _ : state) {
        int ret = ringbuf_enqueue(r, &a);
        if (ret < 0) ringbuf_reset(r);
    }
    ringbuf_free(r);
}

static void BM_enq64(benchmark::State& state) {
    struct ringbuf *r = ringbuf_create(SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
    void **a = new void*[64];
    memset(a, 1, sizeof(void *) * 64);
    for (auto _ : state) {
        int ret = ringbuf_enqueue_bulk(r, a, 64);
        if (ret < 0) ringbuf_reset(r);
    }
    ringbuf_free(r);
}

static void BM_deq1(benchmark::State& state) {
    struct ringbuf *r = ringbuf_create(SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
    fill_ring(r);
    void *result;
    for (auto _ : state) {
        int ret = ringbuf_dequeue(r, &result);
        if (ret < 0) {
            state.PauseTiming();
            ringbuf_reset(r);
            fill_ring(r);
            state.ResumeTiming();
        }
    }
    ringbuf_free(r);
}

static void BM_deq64(benchmark::State& state) {
    struct ringbuf *r = ringbuf_create(SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
    fill_ring(r);

    void **result = new void*[64];
    for (auto _ : state) {
        int ret = ringbuf_dequeue_burst(r, result, 64);
        if (ret < 0) {
            state.PauseTiming();
            ringbuf_reset(r);
            fill_ring(r);
            state.ResumeTiming();
        }
    }
    ringbuf_free(r);
}

static void BM_threaded_enqdeq(benchmark::State& state) {
    struct ringbuf *r = ringbuf_create(SIZE, 0);
    if (state.thread_index == 0) {
        // Setup code here.
        fill_ring(r);
    }
    for (auto _ : state) {
        void *objs[64];
        void *result[64];
        ringbuf_enqueue_burst(r, objs, 64);
        ringbuf_dequeue_burst(r, result, 64);
    }
    if (state.thread_index == 0) {
        ringbuf_free(r);
    }
}

BENCHMARK(BM_enq1);
BENCHMARK(BM_enq64);
BENCHMARK(BM_deq1);
BENCHMARK(BM_deq64);

BENCHMARK(BM_threaded_enqdeq)->Threads(4);
BENCHMARK(BM_threaded_enqdeq)->Threads(16);

BENCHMARK_MAIN();
