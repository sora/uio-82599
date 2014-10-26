#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <sys/user.h>

int main()
{

}

struct ixgbe_handle *ixgbe_open(const char *name, int dmafd, struct uio_dma_area *dmabuf)
{
        struct uio_ixgbe_bind_req breq;
        struct uio_ixgbe_open_req oreq;
        struct ixgbe_handle *h;
        void *m;
        int err;

        h = malloc(sizeof(*h));
        if (!h)
                return NULL;
        memset(h, 0, sizeof(*h));

        h->fd = open("/dev/uio-ixgbe", O_RDWR);
        if (h->fd < 0)
                goto failed;

        h->dmafd = dmafd;

        strcpy(breq.name, name);

        if (ioctl(h->fd, UIO_IXGBE_BIND, (unsigned long) &breq) < 0)
                goto failed;

        // Open the device (ie bring it up)
        memset(&oreq, 0, sizeof(oreq));

        if (ioctl(h->fd, UIO_IXGBE_OPEN, (unsigned long) &oreq) < 0)
                goto failed;

        h->dmadevid  = oreq.uio_dma_devid;
        h->info      = oreq.info;

        // Map IO space
        m = mmap(NULL, oreq.info.mmio_size, PROT_READ | PROT_WRITE, MAP_SHARED, h->fd, 0);
        if (m == MAP_FAILED)
                goto failed;

        h->mmio_addr = m;
        h->mmio_size = oreq.info.mmio_size;

        // Map DATA area
        h->dmabuf = uio_dma_map(h->dmafd, dmabuf, h->dmadevid, UIO_DMA_BIDIRECTIONAL);
        if (!h->dmabuf)
                goto failed;

        h->mtu       = 1500;
        h->rx_buflen = IXGBE_RXBUFFER_2048;

        ixgbe_throttle_intr(h, IXGBE_DEFAULT_ITR);
        return h;

failed:
        err = errno;
        if (h->dmabuf)
                uio_dma_unmap(h->dmafd, h->dmabuf);
        if (h->mmio_addr)
                munmap(h->mmio_addr, h->mmio_size);
        close(h->fd);
        free(h);
        errno = err;
        return NULL;
}
