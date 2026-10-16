#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <sys/ioctl.h>
/* Fake device in a second preloaded library. Never opens a video device. */
int mock_ioctl(int fd,unsigned long request,uintptr_t arg) __asm__("ioctl");
int mock_ioctl(int fd,unsigned long request,uintptr_t arg) {
    if(fd==9876 && (request==VIDIOC_DQBUF || request==VIDIOC_QBUF)) {
        struct v4l2_buffer *b=(struct v4l2_buffer*)arg;
        b->index=3; b->type=V4L2_BUF_TYPE_META_CAPTURE; b->sequence=77;
        b->bytesused=512; b->flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        b->timestamp.tv_sec=123; b->timestamp.tv_usec=456;
        b->length=1024; b->memory=V4L2_MEMORY_MMAP;
        return 0;
    }
    if(fd==9876 && request==0xfaceUL) return arg==(uintptr_t)0xdeadbeef ? 73 : -2;
    int (*next)(int,unsigned long,...)=dlsym(RTLD_NEXT,"ioctl");
    return next(fd,request,arg);
}
