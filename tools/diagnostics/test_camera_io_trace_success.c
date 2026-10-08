#include <assert.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
int main(void) {
    struct v4l2_buffer b={0}; errno=E2BIG;
    assert(ioctl(9876,VIDIOC_DQBUF,&b)==0); assert(errno==E2BIG); assert(b.sequence==77);
    errno=EDOM; assert(ioctl(9876,VIDIOC_QBUF,&b)==0); assert(errno==EDOM);
    errno=ENXIO; assert(ioctl(9876,0xfaceUL,(uintptr_t)0xdeadbeef)==73); assert(errno==ENXIO);
    puts("{\"mock_buffer\":\"passed\"}"); return 0;
}
