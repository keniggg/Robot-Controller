#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>
static uint64_t ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000000000ull+t.tv_nsec; }
static void *stress(void *arg) {
    long count=(long)arg;
    for (long i=0;i<count;++i) {
        struct timeval tv={0}; errno=E2BIG;
        assert(select(0,NULL,NULL,NULL,&tv)==0); assert(errno==E2BIG);
    }
    return NULL;
}
static void handler(int signo) { (void)signo; }
int main(int argc,char **argv) {
    if (argc>1 && !strcmp(argv[1],"stress")) {
        pthread_t t[8]; uint64_t begin=ns();
        for(int i=0;i<8;++i) assert(!pthread_create(&t[i],NULL,stress,(void*)10000L));
        for(int i=0;i<8;++i) assert(!pthread_join(t[i],NULL));
        printf("{\"stress_calls\":80000,\"elapsed_ns\":%llu}\n",(unsigned long long)(ns()-begin));
        return 0;
    }
    if (argc>1 && !strcmp(argv[1],"deadline")) {
        stress((void*)10L); usleep(1300000); stress((void*)10L); return 0;
    }
    if (argc>1 && !strcmp(argv[1],"benchmark")) {
        uint64_t begin=ns(); stress((void*)10000L);
        printf("{\"benchmark_calls\":10000,\"elapsed_ns\":%llu}\n",(unsigned long long)(ns()-begin)); return 0;
    }
    int pipefd[2]; assert(pipe(pipefd)==0);
    assert(write(pipefd[1],"xyz",3)==3);
    fd_set r; FD_ZERO(&r); FD_SET(pipefd[0],&r);
    struct timeval zero={0}; errno=EDOM;
    assert(select(pipefd[0]+1,&r,NULL,NULL,&zero)==1); assert(errno==EDOM); assert(FD_ISSET(pipefd[0],&r));
    struct timespec timeout={0}; FD_ZERO(&r); FD_SET(pipefd[0],&r); errno=ERANGE;
    assert(pselect(pipefd[0]+1,&r,NULL,NULL,&timeout,NULL)==1); assert(errno==ERANGE); assert(FD_ISSET(pipefd[0],&r));
    int count=0; errno=E2BIG;
    assert(ioctl(pipefd[0],FIONREAD,&count)==0); assert(count==3); assert(errno==E2BIG);
    errno=0; assert(ioctl(-1,0xabcdefUL,(uintptr_t)0xdeadbeef)==-1); assert(errno==EBADF);
    errno=EIO; assert(ioctl(pipefd[0],FIOCLEX)==0); assert(errno==EIO); assert(fcntl(pipefd[0],F_GETFD)&FD_CLOEXEC);
    errno=EIO; assert(ioctl(pipefd[0],FIONCLEX)==0); assert(errno==EIO); assert(!(fcntl(pipefd[0],F_GETFD)&FD_CLOEXEC));
    struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE};
    errno=0; assert(ioctl(pipefd[0],VIDIOC_DQBUF,&b)==-1); assert(errno==ENOTTY);
    errno=0; assert(ioctl(pipefd[0],VIDIOC_QBUF,(void*)1)==-1); assert(errno==ENOTTY);
    errno=0; assert(ioctl(-1,VIDIOC_DQBUF,(void*)1)==-1); assert(errno==EBADF);
    char buf[4]; assert(read(pipefd[0],buf,3)==3);
    FD_ZERO(&r); FD_SET(pipefd[0],&r); struct timeval tv={.tv_usec=20000}; errno=ENXIO;
    uint64_t before=ns(); assert(select(pipefd[0]+1,&r,NULL,NULL,&tv)==0); assert(errno==ENXIO); assert(!FD_ISSET(pipefd[0],&r));
    assert(ns()-before>=15000000); assert(tv.tv_sec==0 && tv.tv_usec==0);
    struct timespec ts={.tv_nsec=20000000}; errno=E2BIG; before=ns();
    assert(pselect(0,NULL,NULL,NULL,&ts,NULL)==0); assert(errno==E2BIG); assert(ts.tv_nsec==20000000); assert(ns()-before>=15000000);
    errno=0; tv=(struct timeval){0}; assert(select(-1,NULL,NULL,NULL,&tv)==-1); assert(errno==EINVAL);
    /* Kernel ignores pointers at nfds==0. The tracer must not dereference them. */
    tv=(struct timeval){0}; errno=E2BIG; assert(select(0,(fd_set*)1,NULL,NULL,&tv)==0); assert(errno==E2BIG);
    tv=(struct timeval){0}; errno=0; assert(select(pipefd[0]+1,(fd_set*)1,NULL,NULL,&tv)==-1); assert(errno==EFAULT);
    /* Only the kernel-required fd_set words may be read by instrumentation. */
    long page=sysconf(_SC_PAGESIZE); char *mem=mmap(NULL,(size_t)page*2,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
    assert(mem!=MAP_FAILED); assert(mprotect(mem+page,(size_t)page,PROT_NONE)==0);
    unsigned long *one=(unsigned long*)(mem+page-sizeof(long)); *one=0;
    tv=(struct timeval){0}; assert(select(1,(fd_set*)one,NULL,NULL,&tv)==0); assert(munmap(mem,(size_t)page*2)==0);
    struct sigaction sa={.sa_handler=handler}; sigemptyset(&sa.sa_mask); assert(sigaction(SIGALRM,&sa,NULL)==0);
    ualarm(20000,0); tv=(struct timeval){.tv_sec=1}; errno=0; assert(select(0,NULL,NULL,NULL,&tv)==-1); assert(errno==EINTR);
    ualarm(20000,0); ts=(struct timespec){.tv_sec=1}; errno=0; assert(pselect(0,NULL,NULL,NULL,&ts,NULL)==-1); assert(errno==EINTR);
    close(pipefd[0]); close(pipefd[1]); puts("{\"functional\":\"passed\"}"); return 0;
}
