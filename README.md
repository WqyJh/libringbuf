# libringbuf

Lockless Ring Library from [DPDK Ring Library](https://doc.dpdk.org/guides/prog_guide/ring_lib.html).

## Install

```bash
mkdir build
cd build/
cmake ..
make
make test
make bench
sudo make install
sudo ldconfig
```

## Usage

Just like dpdk ring library. Change prefix `rte_ring_` to `ringbuf_`.

Use `memalign` to allocate memory instead of `rte_zmalloc`.

Numa awareness are not supported yet.

```c
#include <ringbuf.h>

struct ringbuf *r = ringbuf_create(SIZE, RING_F_SP_ENQ | RING_F_SC_DEQ);
int a = 1;
for (int i = 0; i < SIZE; i++) {
    int ret = ringbuf_enqueue(r, &a);
    if (ret < 0) ringbuf_reset(r);
}
ringbuf_free(r);
```
