/**
  UIO-IXGBE - User-space library for Intel 10Gigabit Ethernet adapters
  Copyright (C) 2009 Qualcomm Inc. All rights reserved.
  Written by Max Krasnyansky <maxk@qualcommm.com>

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
 
  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  
  3. Neither the name of the QUALCOMM Incorporated nor the
  names of any contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY QUALCOMM AND ANY OTHER CONTRIBUTORS 
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL QUALCOMM 
  AND CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED 
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define _GNU_SOURCE

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

#include "ixgbe-lib.h"
#include "ixgbe-uio.h"
#include "ixgbe-priv.h"

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

void ixgbe_close(struct ixgbe_handle *h)
{
	ixgbe_rx_disable(h);

	uio_dma_unmap(h->dmafd, h->dmabuf);
	munmap(h->mmio_addr, h->mmio_size);

	close(h->fd);
	free(h);
}

int ixgbe_reset(struct ixgbe_handle *h)
{
	return ioctl(h->fd, UIO_IXGBE_RESET, 0);
}

void ixgbe_get_macaddr(struct ixgbe_handle *h, uint8_t *addr)
{
	memcpy(addr, h->info.mac_addr, ETH_ALEN);
}

uint32_t ixgbe_get_irq(struct ixgbe_handle *h)
{
	return h->info.irq;
}

unsigned int ixgbe_rxbuflen(struct ixgbe_handle *h)
{
	return h->rx_buflen;
}

int ixgbe_get_pollfd(struct ixgbe_handle *h)
{
	return h->fd;
}

void ixgbe_enable_interrupts(struct ixgbe_handle *h, uint32_t mask)
{
	IXGBE_WRITE_REG(h->mmio_addr, IXGBE_EIMS, mask);
	IXGBE_WRITE_FLUSH(h->mmio_addr);
}

void ixgbe_disable_interrupts(struct ixgbe_handle *h, uint32_t mask)
{
	IXGBE_WRITE_REG(h->mmio_addr, IXGBE_EIMC, mask);
	IXGBE_WRITE_FLUSH(h->mmio_addr);
}

void ixgbe_trigger_interrupt(struct ixgbe_handle *h, uint32_t ics)
{
	IXGBE_WRITE_REG(h->mmio_addr, IXGBE_EICS, ics);
}

// Returns read only handle that must be freed with ixgbe_free()
struct ixgbe_handle *ixgbe_info(const char *name)
{
	struct ixgbe_handle *h;
	struct uio_ixgbe_info_req ireq;
	int err;

	h = malloc(sizeof(*h));
	if (!h)
		return NULL;
	memset(h, 0, sizeof(*h));

	h->fd = open("/dev/uio-ixgbe", O_RDWR);
	if (h->fd < 0)
		goto failed;

	strcpy(ireq.name, name);
	err = ioctl(h->fd, UIO_IXGBE_INFO, (unsigned long) &ireq);
	close(h->fd);

	if (err < 0)
		goto failed;

	h->info = ireq.info;
	return h;

failed:
	err = errno;
	free(h);
	errno = err;
	return NULL;
}

void ixgbe_free(struct ixgbe_handle *h)
{
	free(h);
}
