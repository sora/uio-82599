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
	struct ixgbe_handle *handle;

	handle = ixgbe_open();
	ixgbe_close(handle);
}

struct ixgbe_handle *ixgbe_open()
{
	struct uio_ixgbe_info_req req_info;
	struct uio_ixgbe_up_req req_up;
	struct ixgbe_handle *handle;
	void *bar;
	int err;

	handle = malloc(sizeof(struct ixgbe_handle));
	if (!handle)
		return NULL;
	memset(handle, 0, sizeof(struct ixgbe_handle));

	h->fd = open("/dev/ixgbe0", O_RDWR);
	if (h->fd < 0)
		goto failed;

	/* Get device information */
	memset(&req_info, 0, sizeof(struct uio_ixgbe_info_req));
	if(ioctl(h->fd, UIO_IXGBE_INFO, (unsigned long(&req_info)) < 0)
		goto failed;

	/* UP the device */
	memset(&req_up, 0, sizeof(struct uio_ixgbe_up_req));
	req_up.info.num_rx_queues = req_info.info.max_rx_queues;
	req_up.info.num_tx_queues = req_info.info.max_tx_queues;
	if(ioctl(h->fd, UIO_IXGBE_UP, (unsigned long)&req_up) < 0)
		goto failed;

	h->info = req_up.info;

	/* Map PCI config register space */
	h->bar = mmap(NULL, h->info.mmio_size, PROT_READ | PROT_WRITE, MAP_SHARED, h->fd, 0);
	if(h->bar == MAP_FAILED)
		goto failed;

	h->bar_size = req_up.info.mmio_size;

	return h;

failed:
	err = errno;
	if (h->mmio_addr)
		munmap(h->mmio_addr, h->mmio_size);
	close(h->fd);
	free(h);
	errno = err;
	return NULL;
}

void ixgbe_close(struct ixgbe_handle *h)
{
	munmap(h->bar, h->bar_size);
	close(h->fd);
	free(h);
}
