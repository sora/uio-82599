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
#include <signal.h>
#include <errno.h>
#include <sched.h>
#include <assert.h>

#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/user.h>
#include <sys/mman.h>
#include <sys/poll.h>

#include "ixgbe-lib.h"
#include "ixgbe-uio.h"
#include "ixgbe-priv.h"

#ifdef IXGBE_DEBUG

static void txq_dump(struct ixgbe_handle *h, unsigned int i)
{
	struct ixgbe_ring *r = txq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	uint32_t txdctl, tdbal, tdbah, tdlen, tdh, tdt;

	if (!txq_isopen(r)) {
		printf("txq%u: not open\n", i);
		return;
	}

	printf("txq%u: -- dump start -- \n", i);
	printf("txq%u: area %p mapping %p sw_desc %p hw_desc %p\n", i, r->area, r->mapping, r->sw_desc, r->hw_desc);
	printf("txq%u: count %lu head %u tail %u\n", i, r->count, r->head, r->tail);

	txdctl = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));
	tdbal  = IXGBE_READ_REG(mmio, IXGBE_TDBAL(i));
	tdbah  = IXGBE_READ_REG(mmio, IXGBE_TDBAH(i));
	tdlen  = IXGBE_READ_REG(mmio, IXGBE_TDLEN(i));
	tdh    = IXGBE_READ_REG(mmio, IXGBE_TDH(i));
	tdt    = IXGBE_READ_REG(mmio, IXGBE_TDT(i));

	printf("txq%u: regs: txdctl 0x%x tdbal 0x%x tdbah 0x%x tdlen %u tdh %u tdt %u\n", i,
			txdctl, tdbal, tdbah, tdlen, tdh, tdt);
	printf("txq%u: -- dump end -- \n", i);
}

#endif

static inline unsigned int powerof2(unsigned int size)
{
	unsigned int i;
	for (i=0; (1U << i) < size; i++);
	return 1U << i;
}

static inline struct ixgbe_ring *txq_get(struct ixgbe_handle *h, unsigned int i)
{
	assert(i < IXGBE_MAX_TX_QUEUES);
	return &h->tx_ring[i];
}

static inline struct ixgbe_ring *rxq_get(struct ixgbe_handle *h, unsigned int i)
{
	assert(i < IXGBE_MAX_RX_QUEUES);
	return &h->rx_ring[i];
}

static inline bool txq_isopen(struct ixgbe_ring *r)
{
	return r->mapping != NULL;
}

static inline bool rxq_isopen(struct ixgbe_ring *r)
{
	return r->mapping != NULL;
}

static inline struct ixgbe_legacy_tx_desc * txd_get(struct ixgbe_ring *r, unsigned int i)
{
	return (struct ixgbe_legacy_tx_desc *) r->hw_desc + i;
}

static inline struct ixgbe_legacy_rx_desc * rxd_get(struct ixgbe_ring *r, unsigned int i)
{
	return (struct ixgbe_legacy_rx_desc *) r->hw_desc + i;
}

static void do_completion(struct ixgbe_handle *h, struct ixgbe_ring *r)
{
	unsigned int sop = r->head, eop;
	if (!r->flushcb)
		return;
	eop = r->sw_desc[sop].eop;
	while (sop != r->tail) {
		r->flushcb(r->udata, r->sw_desc[eop].cdata);
		sop = (eop + 1) & r->count;
		eop = r->sw_desc[sop].eop;
	}
}

static inline unsigned int __ring_room(struct ixgbe_ring *ring)
{
	return (ring->head - ring->tail - 1) & ring->count;
}

unsigned int ixgbe_rxq_room(struct ixgbe_handle *h, unsigned int i)
{
	return __ring_room(h->rx_ring + i);
}

unsigned int ixgbe_txq_room(struct ixgbe_handle *h, unsigned int i)
{
	return __ring_room(h->tx_ring + i);
}

int ixgbe_txq_enable(struct ixgbe_handle *h, unsigned int i)
{
        struct ixgbe_ring *ring = txq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	uint32_t txdctl;

	if (!txq_isopen(ring)) {
		errno = EINVAL;
		return -1;
	}

	txdctl = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));
        txdctl |= IXGBE_TXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_TXDCTL(i), txdctl);

	return 0;
}

void ixgbe_txq_disable(struct ixgbe_handle *h, unsigned int i)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t txdctl, r;

	txdctl = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));
        txdctl &= ~IXGBE_TXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_TXDCTL(i), txdctl);

	for (r = txdctl; r & IXGBE_TXDCTL_ENABLE; )
		r = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));
}

void ixgbe_txq_flush(struct ixgbe_handle *h, unsigned int i)
{
	struct ixgbe_ring *ring = txq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	uint32_t txdctl, r;

	if (!txq_isopen(ring))
		return;

	/* Disable queue */
	txdctl = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));
	IXGBE_WRITE_REG(mmio, IXGBE_TXDCTL(i), txdctl & ~IXGBE_TXDCTL_ENABLE);
	for (r = txdctl; r & IXGBE_TXDCTL_ENABLE; )
		r = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));

	/* Run completion handlers */
	do_completion(h, ring);

	/* Reset head and tail */
	ring->head = ring->tail = 0;
	IXGBE_WRITE_REG(mmio, IXGBE_TDH(i), 0);
	IXGBE_WRITE_REG(mmio, IXGBE_TDT(i), 0);

	/* Restore queue state */
	IXGBE_WRITE_REG(mmio, IXGBE_TXDCTL(i), txdctl);
}

void ixgbe_txq_close(struct ixgbe_handle *h, unsigned int i)
{
        struct ixgbe_ring *ring = txq_get(h, i);

	if (!txq_isopen(ring))
		return;

	ixgbe_txq_flush(h, i);
	ixgbe_txq_disable(h, i);

        free(ring->sw_desc);
	uio_dma_unmap(h->dmafd, ring->mapping);
	uio_dma_free(h->dmafd, ring->area);

	ring->mapping = NULL;
}

int ixgbe_txq_open(struct ixgbe_handle *h, unsigned int i, unsigned int ntxd)
{
        struct ixgbe_ring *ring = txq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	unsigned int sw_size, hw_size;
	uint32_t txdctl;
	uint64_t dmaddr;
	int err;

	if (txq_isopen(ring)) {
		errno = EALREADY;
		return -1;
	}

	memset(ring, 0, sizeof(*ring));

	ntxd = powerof2(ntxd);
	hw_size = ntxd * sizeof(struct ixgbe_legacy_tx_desc);
	sw_size = ntxd * sizeof(struct ixgbe_sw_desc);

        ring->sw_desc = malloc(sw_size);
        if (!ring->sw_desc)
                return -1;

	// Allocate and mmap TX ring
	ring->area = uio_dma_alloc(h->dmafd, hw_size, UIO_DMA_CACHE_DISABLE, UIO_DMA_MASK(64), 0);
	if (!ring->area)
		goto failed;

	ring->mapping = uio_dma_map(h->dmafd, ring->area, h->dmadevid, UIO_DMA_BIDIRECTIONAL);
	if (!ring->mapping)
		goto failed;

	// FIXME: Make sure it's a single chunk area
	ring->hw_desc = ring->mapping->addr;
	dmaddr = uio_dma_addr(ring->mapping, ring->hw_desc, hw_size);
	if (!dmaddr) {
		errno = EFAULT;
		goto failed;
	}

        memset(ring->sw_desc, 0, sw_size);
        memset(ring->hw_desc, 0, hw_size);
        ring->head  = ring->tail = 0;
	ring->count = ntxd - 1;

	// FIXME: How do I set legacy format here ?

	/* Disable this queue while we're setting up descriptor ring */
	txdctl = IXGBE_READ_REG(mmio, IXGBE_TXDCTL(i));
        txdctl &= ~IXGBE_TXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_TXDCTL(i), txdctl);

	IXGBE_WRITE_REG(mmio, IXGBE_TDBAL(i), ((uint64_t) dmaddr & 0xFFFFFFFF));
	IXGBE_WRITE_REG(mmio, IXGBE_TDBAH(i), ((uint64_t) dmaddr >> 32));

	IXGBE_WRITE_REG(mmio, IXGBE_TDLEN(i), hw_size);

	IXGBE_WRITE_REG(mmio, IXGBE_TDH(i), 0);
	IXGBE_WRITE_REG(mmio, IXGBE_TDT(i), 0);

	/* Enable this queue */
        txdctl |= IXGBE_TXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_TXDCTL(i), txdctl);
	return 0;

failed:
	err = errno;
	if (ring->sw_desc)
        	free(ring->sw_desc);
	if (ring->mapping)
		uio_dma_unmap(h->dmafd, ring->mapping);
	if (ring->area)
		uio_dma_free(h->dmafd, ring->area);
	errno = err;
	return -1;
}

void ixgbe_txq_setflushcb(struct ixgbe_handle *h, unsigned int i, ixgbe_flushcb cb, unsigned long udata)
{
	struct ixgbe_ring *r = txq_get(h, i);
	r->flushcb = cb;
	r->udata   = udata;
}

//---

int ixgbe_rxq_enable(struct ixgbe_handle *h, unsigned int i)
{
        struct ixgbe_ring *ring = rxq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	uint32_t rxdctl;

	if (!rxq_isopen(ring)) {
		errno = EINVAL;
		return -1;
	}

        rxdctl = IXGBE_READ_REG(mmio, IXGBE_RXDCTL(i));
        rxdctl |= IXGBE_RXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_RXDCTL(i), rxdctl);

	return 0;
}

void ixgbe_rxq_disable(struct ixgbe_handle *h, unsigned int i)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t rxdctl, r;

        rxdctl = IXGBE_READ_REG(mmio, IXGBE_RXDCTL(i));
        rxdctl &= ~IXGBE_RXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_RXDCTL(i), rxdctl);

	for (r = rxdctl; r & IXGBE_RXDCTL_ENABLE; )
		r = IXGBE_READ_REG(mmio, IXGBE_RXDCTL(i));
}

void ixgbe_rxq_flush(struct ixgbe_handle *h, unsigned int i)
{
	struct ixgbe_ring *ring = rxq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	uint32_t rxdctl, r;

	if (!rxq_isopen(ring))
		return;

	/* Disable queue */
        rxdctl = IXGBE_READ_REG(mmio, IXGBE_RXDCTL(i));
        IXGBE_WRITE_REG(mmio, IXGBE_RXDCTL(i), rxdctl & ~IXGBE_RXDCTL_ENABLE);

	for (r = rxdctl; r & IXGBE_RXDCTL_ENABLE; )
		r = IXGBE_READ_REG(mmio, IXGBE_RXDCTL(i));

	/* Run completion handlers */
	do_completion(h, ring);

	/* Reset head and tail */
        ring->head = ring->tail = 0;
	IXGBE_WRITE_REG(mmio, IXGBE_RDH(i), 0);
	IXGBE_WRITE_REG(mmio, IXGBE_RDT(i), 0);

	/* Restore queue state */
        IXGBE_WRITE_REG(mmio, IXGBE_RXDCTL(i), rxdctl);
}

void ixgbe_rxq_close(struct ixgbe_handle *h, unsigned int i)
{
        struct ixgbe_ring *ring = rxq_get(h, i);

	if (!rxq_isopen(ring))
		return;

	ixgbe_rxq_disable(h, i);

        free(ring->sw_desc);
	uio_dma_unmap(h->dmafd, ring->mapping);
	uio_dma_free(h->dmafd, ring->area);

	ring->mapping = NULL;
}

int ixgbe_rxq_open(struct ixgbe_handle *h, unsigned int i, unsigned int nrxd)
{
        struct ixgbe_ring *ring = rxq_get(h, i);
	uint8_t *mmio = h->mmio_addr;
	unsigned int sw_size, hw_size;
	uint32_t rxdctl, srrctl;
	uint64_t dmaddr;
	int err;

	if (rxq_isopen(ring)) {
		errno = EALREADY;
		return -1;
	}

	memset(ring, 0, sizeof(*ring));

	nrxd = powerof2(nrxd);
	hw_size = nrxd * sizeof(struct ixgbe_legacy_rx_desc);
	sw_size = nrxd * sizeof(struct ixgbe_sw_desc);

        ring->sw_desc = malloc(sw_size);
        if (!ring->sw_desc)
                return -1;

	// Allocate and mmap TX ring
	ring->area = uio_dma_alloc(h->dmafd, hw_size, UIO_DMA_CACHE_DISABLE, UIO_DMA_MASK(64), 0);
	if (!ring->area)
		goto failed;

	ring->mapping = uio_dma_map(h->dmafd, ring->area, h->dmadevid, UIO_DMA_BIDIRECTIONAL);
	if (!ring->mapping)
		goto failed;

	// FIXME: Make sure it's a single chunk area
	ring->hw_desc = ring->mapping->addr;
	dmaddr = uio_dma_addr(ring->mapping, ring->hw_desc, hw_size);
	if (!dmaddr) {
		errno = EFAULT;
		goto failed;
	}

        memset(ring->sw_desc, 0, sw_size);
        memset(ring->hw_desc, 0, hw_size);
        ring->head  = ring->tail = 0;
	ring->count = nrxd - 1;

	/* Disable the queue while we're setting up rxring */
        rxdctl = IXGBE_READ_REG(mmio, IXGBE_RXDCTL(i));
        rxdctl &= ~IXGBE_RXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_RXDCTL(i), rxdctl);

	/* We're using legacy descriptor format for now */
	srrctl = IXGBE_SRRCTL_DESCTYPE_LEGACY;
        srrctl |= h->rx_buflen >> IXGBE_SRRCTL_BSIZEPKT_SHIFT;
        IXGBE_WRITE_REG(mmio, IXGBE_SRRCTL(i), srrctl);

	IXGBE_WRITE_REG(mmio, IXGBE_RDBAL(i), ((uint64_t) dmaddr & 0xFFFFFFFF));
	IXGBE_WRITE_REG(mmio, IXGBE_RDBAH(i), ((uint64_t) dmaddr >> 32));

	IXGBE_WRITE_REG(mmio, IXGBE_RDLEN(i), hw_size);
	IXGBE_WRITE_REG(mmio, IXGBE_RDH(i), 0);
	IXGBE_WRITE_REG(mmio, IXGBE_RDT(i), 0);

	/* Enable the queue */
        rxdctl |= 0x0020;
        rxdctl |= IXGBE_RXDCTL_ENABLE;
        IXGBE_WRITE_REG(mmio, IXGBE_RXDCTL(i), rxdctl);

	return 0;

failed:
	err = errno;

	if (ring->sw_desc)
        	free(ring->sw_desc);
	if (ring->mapping)
		uio_dma_unmap(h->dmafd, ring->mapping);
	if (ring->area)
		uio_dma_free(h->dmafd, ring->area);

	ring->mapping = NULL;

	errno = err;
	return -1;
}

void ixgbe_rxq_setflushcb(struct ixgbe_handle *h, unsigned int i, ixgbe_flushcb cb, unsigned long udata)
{
	struct ixgbe_ring *r = rxq_get(h, i);
	r->flushcb = cb;
	r->udata   = udata;
}

/* 
 * Enable general RX.
 * Does not change the state of the induvidual RX queues.
 */
void ixgbe_rx_enable(struct ixgbe_handle *h)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t hlreg0, rxctl, rdrxctl, vmdctl;
 
	/* Enable jumbo if needed */
        hlreg0 = IXGBE_READ_REG(mmio, IXGBE_HLREG0);
        if (h->mtu <= ETH_DATA_LEN)
                hlreg0 &= ~IXGBE_HLREG0_JUMBOEN;
        else
                hlreg0 |= IXGBE_HLREG0_JUMBOEN;

	/* Enable CRC strip on RX and CRC append on TX */
	hlreg0 |= IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_RXCRCSTRP;
        IXGBE_WRITE_REG(mmio, IXGBE_HLREG0, hlreg0);

	/* Enable VMDQ mode */
	vmdctl = IXGBE_VMD_CTL_VMDQ_EN;
	IXGBE_WRITE_REG(mmio, IXGBE_VMD_CTL, vmdctl);

	/* Not sure we need this one */
	rdrxctl = IXGBE_READ_REG(mmio, IXGBE_RDRXCTL);
	rdrxctl |= IXGBE_RDRXCTL_MVMEN;
	IXGBE_WRITE_REG(mmio, IXGBE_RDRXCTL, rdrxctl);

	/* Disable RSS */
	IXGBE_WRITE_REG(mmio, IXGBE_MRQC, 0);

	/* Disable managibility stuff */
	IXGBE_WRITE_REG(mmio, IXGBE_GRC, 0);

	/* Enable receiver */
        rxctl = IXGBE_READ_REG(mmio, IXGBE_RXCTRL);
        IXGBE_WRITE_REG(mmio, IXGBE_RXCTRL, rxctl | IXGBE_RXCTRL_RXEN);
}

/*
 * Disable general RX. Stops receiving frames.
 * Does not change the state of the induvidual RX queues.
 */
void ixgbe_rx_disable(struct ixgbe_handle *h)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t rxctl;

	/* Disable receiver. */
        rxctl = IXGBE_READ_REG(mmio, IXGBE_RXCTRL);
        IXGBE_WRITE_REG(mmio, IXGBE_RXCTRL, rxctl & ~IXGBE_RXCTRL_RXEN);
}

int ixgbe_wait4_interrupt(struct ixgbe_handle *h, uint32_t *icr, unsigned long timeo)
{
	if (timeo) {
		struct pollfd pf = { .fd = h->fd, .events = POLLIN | POLLERR };
		switch (poll(&pf, 1, timeo)) {
		case 1:
			break;
		case 0:
			errno = ETIMEDOUT;
		default:
			return -1;
		}
	}

	int r = read(h->fd, icr, sizeof(*icr));
	if (r < 0)
		return -1;
	return 0;
}

int ixgbe_txq_submit(struct ixgbe_handle *h, unsigned int i, void *ptr, size_t len, unsigned long cdata)
{
	struct ixgbe_ring *ring = txq_get(h, i);
	struct ixgbe_legacy_tx_desc *hw_desc;
	struct ixgbe_sw_desc *sw_desc;
	uint8_t *mmio = h->mmio_addr;
	uint64_t dmaddr;
	uint32_t txd_upper = 0;
	uint32_t txd_lower = IXGBE_TXD_CMD_EOP | IXGBE_TXD_CMD_RS  | IXGBE_TXD_CMD_IFCS;

	unsigned int room = __ring_room(ring);
	if (!room) {
		errno = ENOSPC;
		return -1;
	}

	dmaddr = uio_dma_addr(h->dmabuf, ptr, len);
	if (unlikely(!dmaddr)) {
		errno = EFAULT;
		return -1;
	}

	sw_desc = ring->sw_desc + ring->tail;
	sw_desc->cdata = cdata;
	sw_desc->eop   = ring->tail;

	hw_desc = txd_get(ring, ring->tail);
        hw_desc->buffer_addr = cpu_to_le64(dmaddr);
        hw_desc->lower.data  = cpu_to_le32(txd_lower | len);
        hw_desc->upper.data  = cpu_to_le32(txd_upper);

#ifdef IXGBE_DEBUG
	printf("ixgbe: queued tx cdata 0x%lx 0x%p size %u desc# %u\n", cdata, ptr, size, ring->tail); 
	dump_tx_desc(hw_desc);
#endif

	ring->tail = (ring->tail + 1) & ring->count;

	mem_barrier_w();
	IXGBE_WRITE_REG(mmio, IXGBE_TDT(i), ring->tail);

	return 0;
}

int ixgbe_txq_submitv(struct ixgbe_handle *h, unsigned int i, struct iovec *vec, 
		unsigned int nvec, unsigned long cdata)
{
	struct ixgbe_ring *ring = txq_get(h, i);
	struct ixgbe_legacy_tx_desc *hw_desc = NULL;
	uint8_t *mmio = h->mmio_addr;
	uint32_t txd_upper = 0;
	uint32_t txd_lower = IXGBE_TXD_CMD_IFCS;
	unsigned int n, d, sop, eop;

	if (__ring_room(ring) < nvec) {
		errno = ENOSPC;
		return -1;
	}

	/* Compute SOP and EOP (start and end of packet). */
	sop = d = ring->tail;
	eop = (sop + nvec - 1) & ring->count;

	/* Go throught the vector, map and populate tx descriptors */
	for (n=0; n < nvec; n++) {
		uint32_t len = vec[n].iov_len;
		uint64_t dma = uio_dma_addr(h->dmabuf, vec[n].iov_base, len);
		if (unlikely(!dma)) {
			errno = EFAULT;
			return -1;
		}

		hw_desc = txd_get(ring, d);
	        hw_desc->buffer_addr = cpu_to_le64(dma);
        	hw_desc->lower.data  = cpu_to_le32(txd_lower | len);
	        hw_desc->upper.data  = cpu_to_le32(txd_upper);

		d = (d + 1) & ring->count;
	}

       	hw_desc->lower.data |= cpu_to_le32(IXGBE_TXD_CMD_EOP |
			     		IXGBE_TXD_CMD_RS | IXGBE_TXD_CMD_IFCS);
	
	/* Only last descriptor needs cdata */
	ring->sw_desc[sop].eop   = eop;
	ring->sw_desc[eop].cdata = cdata;

#ifdef IXGBE_DEBUG
	printf("ixgbe: queued tx cdata 0x%lx sop %u eop %u\n", cdata, sop, eop);
	dump_tx_desc(desc);
#endif

	ring->tail = d;

	mem_barrier_w();
	IXGBE_WRITE_REG(mmio, IXGBE_TDT(i), ring->tail);

	return 0;
}

int ixgbe_txq_send(struct ixgbe_handle *h, unsigned int i, void *ptr, 
		size_t len, unsigned long cdata)
{
	return ixgbe_txq_submit(h, i, ptr, len, cdata);
}

int ixgbe_txq_sendv(struct ixgbe_handle *h, unsigned int i, 
		struct iovec *vec, unsigned int nvec, unsigned long cdata)
{
	return ixgbe_txq_submitv(h, i, vec, nvec, cdata);
}

int ixgbe_txq_complete(struct ixgbe_handle *h, unsigned int i, unsigned long *cdata)
{
	struct ixgbe_ring *ring = txq_get(h, i);
	struct ixgbe_legacy_tx_desc *hw_desc;
	unsigned int sop, eop;

	sop = ring->head;
	eop = ring->sw_desc[sop].eop;

	hw_desc = txd_get(ring, eop);
	if (hw_desc->upper.data & cpu_to_le32(IXGBE_TXD_STAT_DD)) {
#ifdef IXGBE_DEBUG
		printf("ixgbe: completed tx cdata 0x%lx desc# %u\n", 
				ring->sw_desc[eop].cdata, eop);
#endif
		hw_desc->upper.data = 0;
		ring->head = (eop + 1) & ring->count;

		*cdata = ring->sw_desc[eop].cdata;
		return IXGBE_COMPLETION_OK;
	}

	return IXGBE_COMPLETION_NONE;
}

int ixgbe_txq_poll(struct ixgbe_handle *h, unsigned int i)
{
	struct ixgbe_ring *ring = txq_get(h, i);
	struct ixgbe_legacy_tx_desc *hw_desc;
	unsigned int sop, eop;

	sop = ring->head;
	eop = ring->sw_desc[sop].eop;

	hw_desc = txd_get(ring, eop);
	return (hw_desc->upper.data & cpu_to_le32(IXGBE_TXD_STAT_DD));
}

int ixgbe_rxq_submit(struct ixgbe_handle *h, unsigned int i, void *ptr, size_t size, unsigned long cdata)
{
	struct ixgbe_ring *ring = rxq_get(h, i);
	struct ixgbe_legacy_rx_desc *hw_desc;
	struct ixgbe_sw_desc *sw_desc;
	uint8_t *mmio = h->mmio_addr;
	uint64_t dmaddr;

	if (unlikely(size < h->rx_buflen)) {
		errno = EINVAL;
		return -1;
	}

	if (!__ring_room(ring)) {
		errno = ENOSPC;
		return -1;
	}

	dmaddr = uio_dma_addr(h->dmabuf, ptr, size);
	if (unlikely(!dmaddr)) {
		errno = EFAULT;
		return -1;
	}

	sw_desc = ring->sw_desc + ring->tail;
	sw_desc->cdata = cdata;
	sw_desc->eop   = ring->tail;

	hw_desc = rxd_get(ring, ring->tail);
	hw_desc->buffer_addr = cpu_to_le64(dmaddr);
	hw_desc->length = 0;
	hw_desc->csum   = 0;
	hw_desc->status = 0;
	hw_desc->errors = 0;

#ifdef IXGBE_DEBUG
	printf("ixgbe: queued rx cdata 0x%lx 0x%p size %u desc# %u\n", cdata, ptr, size, ring->tail); 
#endif

	ring->tail = (ring->tail + 1) & ring->count;

	mem_barrier_w();
	IXGBE_WRITE_REG(mmio, IXGBE_RDT(i), ring->tail);

	return 0;
}

int ixgbe_rxq_recv(struct ixgbe_handle *h, unsigned int i, void *ptr, 
	size_t size, unsigned long cdata)
{
	return ixgbe_rxq_submit(h, i, ptr, size, cdata);
}

ssize_t ixgbe_rxq_complete(struct ixgbe_handle *h, unsigned int i, unsigned long *cdata)
{
	struct ixgbe_ring *ring = rxq_get(h, i);
	struct ixgbe_legacy_rx_desc *hw_desc;
	unsigned int sop;

	sop = ring->head;
	hw_desc = rxd_get(ring, sop);
	if (hw_desc->status & cpu_to_le32(IXGBE_RXD_STAT_DD)) {
		size_t len = le16_to_cpu(hw_desc->length);
#ifdef IXGBE_DEBUG
		printf("ixgbe: completed rx cdata 0x%lx len %u desc# %u\n", 
				ring->sw_desc[sop].cdata, len, sop);
#endif

		if (unlikely(!(hw_desc->status & cpu_to_le32(IXGBE_RXD_STAT_EOP)))) {
			/* All receives must fit into a single buffer */
			/* FIXME: Handle multi buffer receives */
			fprintf(stderr, "IXGBE: ERROR: multi-buffer receive. "
					"Increase MTU or fix the library\n");
		}

		hw_desc->status = 0;
		ring->head = (sop + 1) & ring->count;

		*cdata = ring->sw_desc[sop].cdata;
		return len;
	}

	return 0;
}

int ixgbe_rxq_poll(struct ixgbe_handle *h, unsigned int i)
{
	struct ixgbe_ring *ring = rxq_get(h, i);
	struct ixgbe_legacy_rx_desc *hw_desc;
	unsigned int sop;

	sop = ring->head;
	hw_desc = rxd_get(ring, sop);
	return (hw_desc->status & cpu_to_le32(IXGBE_RXD_STAT_DD));
}

#define MAX_STD_JUMBO_FRAME_SIZE 9234
int ixgbe_set_mtu(struct ixgbe_handle *h, unsigned int mtu)
{
	int max_frame = mtu + ENET_HEADER_SIZE + ETHERNET_FCS_SIZE;
	uint8_t *mmio = h->mmio_addr;
	uint32_t rxctl;

        rxctl = IXGBE_READ_REG(mmio, IXGBE_RXCTRL);
	if (rxctl & IXGBE_RXCTRL_RXEN) {
		errno = EBUSY;
		return -1;
	}

	errno = EINVAL;

	if ((max_frame < MINIMUM_ETHERNET_FRAME_SIZE) || 
				(max_frame > MAX_JUMBO_FRAME_SIZE))
		return -1;

	if (max_frame <= 2048)
		h->rx_buflen = 2048;
	else if(max_frame <= 4096)
		h->rx_buflen = 4096;
	else if(max_frame <= 8192)
		h->rx_buflen = 8192;
	else if(max_frame <= 16384)
		h->rx_buflen = 16384;

	h->mtu = mtu;

	errno = 0;
	return 0;
}

int ixgbe_throttle_intr(struct ixgbe_handle *h, unsigned int itr)
{
	uint8_t *mmio = h->mmio_addr;
	unsigned long r;

	/* Convert to HW units */
	if (itr)
		r = 1000000000 / (itr * 256);
	else
		r = 0; /* disabled */

	// FIXME: There are 16 EITR registers
	// IXGBE_WRITE_REG(mmio, IXGBE_EITR, r);
	return 0;
}
