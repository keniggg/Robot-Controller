#define _GNU_SOURCE
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <linux/videodev2.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#if !defined(__linux__) || !defined(__x86_64__)
#error This interposer has only been validated for Linux x86_64.
#endif
#ifndef TRACE_CAPACITY
#define TRACE_CAPACITY 4096u
#endif
_Static_assert((TRACE_CAPACITY & (TRACE_CAPACITY - 1)) == 0, "power of two capacity");
_Static_assert(ATOMIC_LLONG_LOCK_FREE == 2, "hot path requires lock free atomics");

typedef int (*select_fn)(int, fd_set *, fd_set *, fd_set *, struct timeval *);
typedef int (*pselect_fn)(int, fd_set *, fd_set *, fd_set *, const struct timespec *, const sigset_t *);
typedef int (*ioctl_fn)(int, unsigned long, ...);
static select_fn next_select;
static pselect_fn next_pselect;
static ioctl_fn next_ioctl;
static pthread_once_t symbols_once = PTHREAD_ONCE_INIT;
static _Thread_local int resolving;
static void resolve_symbols(void) {
    resolving = 1;
    next_select = (select_fn)dlsym(RTLD_NEXT, "select");
    next_pselect = (pselect_fn)dlsym(RTLD_NEXT, "pselect");
    next_ioctl = (ioctl_fn)dlsym(RTLD_NEXT, "ioctl");
    resolving = 0;
}
static void ensure_symbols(void) {
    int saved = errno;
    if (!resolving) pthread_once(&symbols_once, resolve_symbols);
    errno = saved;
}
static uint64_t clock_ns(clockid_t id) {
    struct timespec t;
    if (clock_gettime(id, &t)) return 0;
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

enum { SELECT_ENTER=1, SELECT_EXIT, PSELECT_ENTER, PSELECT_EXIT, IOCTL_ENTER, IOCTL_EXIT };
typedef struct {
    uint64_t mono, wall, id, duration;
    unsigned long request;
    long tid;
    int kind, fd, nfds, ret, err, sets_valid, buffer_valid;
    fd_set sets[3];
    uint32_t index, type, bytesused, flags, sequence, memory, length;
    int64_t buffer_sec, buffer_usec;
} event_t;
typedef struct { _Atomic uint64_t seq; event_t event; } slot_t;
static slot_t ring[TRACE_CAPACITY];
static _Atomic uint64_t enqueue_pos, dropped, recorded, call_ids;
static uint64_t dequeue_pos;
static _Atomic int enabled, stopping, producers;
static uint64_t deadline;
static pthread_t writer_thread;
static int writer_started, output_fd = -1;
#ifdef TRACE_TEST_BUILD
static unsigned test_writer_delay_us;
#endif

/* Bounded MPSC queue. No producer allocation, disk I/O, or unbounded spin. */
static void emit(event_t *e) {
    if (!atomic_load_explicit(&enabled, memory_order_acquire)) return;
    atomic_fetch_add_explicit(&producers, 1, memory_order_acquire);
    if (!atomic_load_explicit(&enabled, memory_order_acquire)) goto out;
    e->mono = clock_ns(CLOCK_MONOTONIC);
    if (e->mono >= deadline) {
        atomic_store_explicit(&enabled, 0, memory_order_release);
        goto out;
    }
    e->wall = clock_ns(CLOCK_REALTIME);
    e->tid = syscall(SYS_gettid);
    uint64_t p = atomic_load_explicit(&enqueue_pos, memory_order_relaxed);
    for (unsigned retry = 0; retry < 8; ++retry) {
        slot_t *s = &ring[p & (TRACE_CAPACITY - 1)];
        uint64_t seq = atomic_load_explicit(&s->seq, memory_order_acquire);
        if (seq == p && atomic_compare_exchange_weak_explicit(&enqueue_pos, &p, p + 1,
                memory_order_relaxed, memory_order_relaxed)) {
            s->event = *e;
            atomic_store_explicit(&s->seq, p + 1, memory_order_release);
            atomic_fetch_add_explicit(&recorded, 1, memory_order_relaxed);
            goto out;
        }
        if ((int64_t)(seq - p) < 0) break;
        p = atomic_load_explicit(&enqueue_pos, memory_order_relaxed);
    }
    atomic_fetch_add_explicit(&dropped, 1, memory_order_relaxed);
out:
    atomic_fetch_sub_explicit(&producers, 1, memory_order_release);
}
static int take(event_t *e) {
    slot_t *s = &ring[dequeue_pos & (TRACE_CAPACITY - 1)];
    if (atomic_load_explicit(&s->seq, memory_order_acquire) != dequeue_pos + 1) return 0;
    *e = s->event;
    atomic_store_explicit(&s->seq, dequeue_pos + TRACE_CAPACITY, memory_order_release);
    ++dequeue_pos;
    return 1;
}
static int write_all(const char *buf, size_t n) {
    while (n) {
        ssize_t written = write(output_fd, buf, n);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) return -1;
        buf += written;
        n -= (size_t)written;
    }
    return 0;
}
static size_t format_event(char *buf, size_t size, const event_t *e) {
    static const char *names[] = {"", "select_enter", "select_exit", "pselect_enter", "pselect_exit", "ioctl_enter", "ioctl_exit"};
    int n = snprintf(buf, size,
        "{\"kind\":\"%s\",\"mono_ns\":%llu,\"wall_ns\":%llu,\"tid\":%ld,\"id\":%llu,"
        "\"duration_ns\":%llu,\"fd\":%d,\"nfds\":%d,\"request\":%lu,\"ret\":%d,\"errno\":%d,\"sets_valid\":%d,\"sets_hex\":[",
        names[e->kind], (unsigned long long)e->mono, (unsigned long long)e->wall, e->tid,
        (unsigned long long)e->id, (unsigned long long)e->duration, e->fd, e->nfds, e->request, e->ret, e->err, e->sets_valid);
    for (int s = 0; s < 3; ++s) {
        n += snprintf(buf + n, size - (size_t)n, "%s\"", s ? "," : "");
        if (e->sets_valid & (1 << s)) {
            /* FD bit 0 is the rightmost bit of the rightmost byte. */
            for (int b = (int)sizeof(fd_set) - 1; b >= 0; --b)
                n += snprintf(buf + n, size - (size_t)n, "%02x", ((const unsigned char *)&e->sets[s])[b]);
        }
        n += snprintf(buf + n, size - (size_t)n, "\"");
    }
    n += snprintf(buf + n, size - (size_t)n,
        "],\"buffer_valid\":%d,\"index\":%u,\"type\":%u,\"bytesused\":%u,\"flags\":%u,\"sequence\":%u,\"memory\":%u,\"length\":%u,\"buffer_sec\":%lld,\"buffer_usec\":%lld}\n",
        e->buffer_valid, e->index, e->type, e->bytesused, e->flags, e->sequence, e->memory, e->length,
        (long long)e->buffer_sec, (long long)e->buffer_usec);
    return (size_t)n;
}
static void *writer_main(void *unused) {
    (void)unused;
    char batch[65536], line[2048];
    size_t used = 0;
    unsigned long long flushed = 0;
    int failed = 0;
    int n = snprintf(line, sizeof(line), "{\"kind\":\"header\",\"pid\":%ld,\"capacity\":%u,\"deadline_mono_ns\":%llu,\"clock\":\"CLOCK_MONOTONIC\",\"abi\":\"linux-x86_64\"}\n",
                     (long)getpid(), TRACE_CAPACITY, (unsigned long long)deadline);
    if (write_all(line, (size_t)n)) failed = 1;
    while (!failed) {
        event_t e;
#ifdef TRACE_TEST_BUILD
        if (test_writer_delay_us) usleep(test_writer_delay_us);
#endif
        unsigned count = 0;
        while (count++ < 128 && take(&e)) {
            size_t length = format_event(line, sizeof(line), &e);
            if (length >= sizeof(line)) { failed = 1; break; }
            if (used + length > sizeof(batch)) {
                if (write_all(batch, used)) { failed = 1; break; }
                used = 0;
            }
            memcpy(batch + used, line, length);
            used += length;
            ++flushed;
        }
        if (used && write_all(batch, used)) failed = 1;
        used = 0;
        int done = atomic_load_explicit(&stopping, memory_order_acquire) || clock_ns(CLOCK_MONOTONIC) >= deadline;
        if (done) atomic_store_explicit(&enabled, 0, memory_order_release);
        if (done && atomic_load_explicit(&producers, memory_order_acquire) == 0 &&
            dequeue_pos == atomic_load_explicit(&enqueue_pos, memory_order_acquire)) break;
        usleep(10000);
    }
    atomic_store_explicit(&enabled, 0, memory_order_release);
    n = snprintf(line, sizeof(line), "{\"kind\":\"summary\",\"recorded\":%llu,\"flushed\":%llu,\"dropped\":%llu,\"write_failed\":%d,\"mono_ns\":%llu}\n",
                 (unsigned long long)atomic_load(&recorded), flushed,
                 (unsigned long long)atomic_load(&dropped), failed, (unsigned long long)clock_ns(CLOCK_MONOTONIC));
    if (!failed) (void)write_all(line, (size_t)n);
    close(output_fd);
    output_fd = -1;
    return NULL;
}
static void disabled_notice(void) {
    static const char msg[] = "camera_io_trace: requested output unavailable or invalid; trace disabled, no fallback.\n";
    ssize_t ignored = write(STDERR_FILENO, msg, sizeof(msg)-1);
    (void)ignored;
}
__attribute__((constructor)) static void trace_init(void) {
    int saved = errno;
    ensure_symbols();
    const char *path = getenv("CAMERA_IO_TRACE_PATH");
    if (!path || !*path) goto out;
    const char *prefix = "/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/";
    struct stat root_st, parent_st;
    char parent[4096];
    int valid = strlen(path) < sizeof(parent) && strncmp(path, prefix, strlen(prefix)) == 0 && !strstr(path, "/../") && !strstr(path, "/./");
#ifdef TRACE_TEST_BUILD
    if (getenv("CAMERA_IO_TRACE_TEST_TMP") && strncmp(path, "/tmp/", 5) == 0) valid = 1;
    const char *delay = getenv("CAMERA_IO_TRACE_TEST_WRITER_DELAY_US");
    if (delay) test_writer_delay_us = (unsigned)strtoul(delay, NULL, 10);
#endif
    if (!valid || strlen(path) >= sizeof(parent)) { disabled_notice(); goto out; }
    strcpy(parent, path);
    char *slash = strrchr(parent, '/');
    if (!slash) { disabled_notice(); goto out; }
    *slash = '\0';
    /* Hold the verified directory FD while opening, so an unmount cannot redirect
       output into the system disk's underlying mountpoint directory. */
    int parent_fd = open(parent, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (parent_fd < 0) { disabled_notice(); goto out; }
    if (stat("/", &root_st) || fstat(parent_fd, &parent_st)) {
        close(parent_fd); disabled_notice(); goto out;
    }
#ifndef TRACE_TEST_BUILD
    if (root_st.st_dev == parent_st.st_dev) {
        close(parent_fd); disabled_notice(); goto out;
    }
#endif
    /* Exclusive creation prevents overwriting an earlier run or following an existing symlink. */
    output_fd = openat(parent_fd, slash + 1, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    close(parent_fd);
    if (output_fd < 0) { disabled_notice(); goto out; }
    unsigned seconds = 300;
    const char *seconds_str = getenv("CAMERA_IO_TRACE_SECONDS");
    if (seconds_str) {
        char *end = NULL;
        unsigned long value = strtoul(seconds_str, &end, 10);
        if (end && *end == '\0' && value >= 1 && value <= 600) seconds = (unsigned)value;
    }
    for (uint64_t i = 0; i < TRACE_CAPACITY; ++i) atomic_store(&ring[i].seq, i);
    deadline = clock_ns(CLOCK_MONOTONIC) + (uint64_t)seconds * 1000000000ull;
    atomic_store_explicit(&enabled, 1, memory_order_release);
    if (pthread_create(&writer_thread, NULL, writer_main, NULL)) {
        atomic_store(&enabled, 0);
        close(output_fd);
        output_fd = -1;
        disabled_notice();
        goto out;
    }
    writer_started = 1;
out:
    errno = saved;
}
__attribute__((destructor)) static void trace_fini(void) {
    if (!writer_started) return;
    int saved = errno;
    atomic_store_explicit(&enabled, 0, memory_order_release);
    atomic_store_explicit(&stopping, 1, memory_order_release);
    struct timespec end;
    clock_gettime(CLOCK_REALTIME, &end);
    end.tv_sec += 1;
    /* A hung removable disk must not indefinitely prevent process exit. */
    if (pthread_timedjoin_np(writer_thread, NULL, &end)) pthread_detach(writer_thread);
    errno = saved;
}
static void copy_sets(event_t *e, int nfds, fd_set *r, fd_set *w, fd_set *x) {
    if (nfds <= 0 || nfds > FD_SETSIZE) return;
    size_t bytes = ((size_t)nfds + 8 * sizeof(long) - 1) / (8 * sizeof(long)) * sizeof(long);
    fd_set *sets[] = {r, w, x};
    for (int i = 0; i < 3; ++i) if (sets[i]) { memcpy(&e->sets[i], sets[i], bytes); e->sets_valid |= 1 << i; }
}
static int call_select(int n, fd_set *r, fd_set *w, fd_set *x, struct timeval *t) {
    if (next_select) return next_select(n, r, w, x, t);
    return (int)syscall(SYS_select, n, r, w, x, t);
}
int select(int n, fd_set *r, fd_set *w, fd_set *x, struct timeval *t) {
    ensure_symbols();
    if (!atomic_load_explicit(&enabled, memory_order_acquire)) return call_select(n,r,w,x,t);
    int saved = errno;
    event_t e = {.kind=SELECT_ENTER, .fd=-1, .nfds=n, .id=atomic_fetch_add(&call_ids,1)+1};
    /* Invalid userspace addresses must retain the original syscall's EFAULT semantics.
       Only inspect fd_sets after a successful call. The entry identifies n and id. */
    emit(&e);
    uint64_t before = clock_ns(CLOCK_MONOTONIC);
    errno = saved;
    int ret = call_select(n,r,w,x,t);
    saved = errno;
    e.duration = clock_ns(CLOCK_MONOTONIC) - before;
    e.kind = SELECT_EXIT; e.ret=ret; e.err=saved;
    if (ret >= 0) copy_sets(&e,n,r,w,x);
    emit(&e);
    errno = saved;
    return ret;
}
static int call_pselect(int n, fd_set *r, fd_set *w, fd_set *x, const struct timespec *t, const sigset_t *sig) {
    if (next_pselect) return next_pselect(n,r,w,x,t,sig);
    struct { const sigset_t *set; size_t size; } arg = {sig, 8};
    struct timespec copy;
    const struct timespec *timeout = t;
    /* The fallback is only for symbol resolution recursion. Normal calls use libc. */
    if (t) { copy = *t; timeout = &copy; }
    return (int)syscall(SYS_pselect6, n,r,w,x,timeout,&arg);
}
int pselect(int n, fd_set *r, fd_set *w, fd_set *x, const struct timespec *t, const sigset_t *sig) {
    ensure_symbols();
    if (!atomic_load_explicit(&enabled, memory_order_acquire)) return call_pselect(n,r,w,x,t,sig);
    int saved = errno;
    event_t e = {.kind=PSELECT_ENTER, .fd=-1, .nfds=n, .id=atomic_fetch_add(&call_ids,1)+1};
    emit(&e);
    uint64_t before = clock_ns(CLOCK_MONOTONIC);
    errno = saved;
    int ret = call_pselect(n,r,w,x,t,sig);
    saved = errno;
    e.duration = clock_ns(CLOCK_MONOTONIC)-before;
    e.kind=PSELECT_EXIT; e.ret=ret; e.err=saved;
    if (ret >= 0) copy_sets(&e,n,r,w,x);
    emit(&e);
    errno=saved;
    return ret;
}
/* On x86_64 ioctl's third argument occupies RDX whether the source declaration is
   variadic or fixed. Capture that raw word without va_arg (which is undefined
   for no-argument requests), and never inspect it for unknown requests. */
int trace_ioctl(int fd, unsigned long request, uintptr_t arg) __asm__("ioctl");
int trace_ioctl(int fd, unsigned long request, uintptr_t arg) {
    ensure_symbols();
    int active = atomic_load_explicit(&enabled, memory_order_acquire) && (request == VIDIOC_DQBUF || request == VIDIOC_QBUF);
    int saved = errno;
    event_t e = {0};
    uint64_t before = 0;
    if (active) {
        e.kind=IOCTL_ENTER; e.fd=fd; e.request=request; e.id=atomic_fetch_add(&call_ids,1)+1;
        emit(&e);
        before=clock_ns(CLOCK_MONOTONIC);
    }
    errno=saved;
    int ret=next_ioctl ? next_ioctl(fd,request,arg) : (int)syscall(SYS_ioctl,fd,request,arg);
    saved=errno;
    if (active) {
        e.duration=clock_ns(CLOCK_MONOTONIC)-before;
        e.kind=IOCTL_EXIT; e.ret=ret; e.err=saved;
        if (ret == 0 && arg) {
            const struct v4l2_buffer *b=(const struct v4l2_buffer *)arg;
            e.buffer_valid=1; e.index=b->index; e.type=b->type; e.bytesused=b->bytesused;
            e.flags=b->flags; e.sequence=b->sequence; e.memory=b->memory; e.length=b->length;
            e.buffer_sec=b->timestamp.tv_sec; e.buffer_usec=b->timestamp.tv_usec;
        }
        emit(&e);
    }
    errno=saved;
    return ret;
}
