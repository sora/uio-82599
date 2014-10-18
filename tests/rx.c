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
#include <getopt.h>
#include <sched.h>
#include <stdint.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/user.h>

#include <net/ethernet.h>

#include "uio-dma/uio-dma.h"
#include "uio-ixgbe/ixgbe-lib.h"

#include "misc.h"

static int  ignore;
static int  terminated;
static int  dump;
static int  promisc;
static int  nobcast;
static unsigned long npackets = ~0UL;
static int  ringsize = 256;
static int  buffsize = 1 * 1024 * 1024;
static int  mtu = ETHERMTU;

static char device[20] = "";

struct uio_dma_area *dmabuf;
int    dmafd;

#define MAX_FILTERS 32
static struct ixgbe_rafilter filter[MAX_FILTERS];
static unsigned int nfilters;

static void sig_int(int sig)
{
	terminated = 1;
}

struct _buffer {
	void         *data;
	unsigned long size;
	unsigned int  inuse;
};

struct _buffer *buffer;
static int nbuffers = 100 * 1024;

static void alloc_buffers(struct ixgbe_handle *h)
{
	unsigned long size, buflen, i;
	void *mem;

	buffer = malloc(sizeof(struct _buffer) * nbuffers);
	if (!buffer) {
		perror("Failed to allocate buffer descriptors\n");
		exit(1);
	}

	mem  = dmabuf->addr;
	size = dmabuf->size;
	buflen = ixgbe_rxbuflen(h);

	memset(mem, 0, size);

	printf("Number of rx descriptors: %u\n", ixgbe_rxq_room(h, 0));
	printf("Available buffer memory %lu rxbuflen %lu\n", size, buflen);

	for (i=0; i < nbuffers && size > buflen; i++) {
		buffer[i].data = mem;
		buffer[i].size = buflen;

		size -= buflen;
		mem  += buflen;
	}
	nbuffers = i;

	printf("Using %u buffers of %lu size\n", nbuffers, buflen);
}

static struct _buffer *get_buffer()
{
	unsigned int i;
	for (i=0; i<nbuffers; i++)
		if (!buffer[i].inuse)
			return &buffer[i];
	return NULL;
}

static void do_recv(struct ixgbe_handle *h, int qi)
{
	struct _buffer *buf;
	int err;

	while ((buf = get_buffer())) {
		buf->inuse = 1;
		err = ixgbe_rxq_recv(h, qi, buf->data, buf->size, (unsigned long) buf);
		if (err) {
			if (errno != ENOSPC)
				perror("Receive failed\n");
			buf->inuse = 0;
			break;
		}
	}
}

static void wait4_link(struct ixgbe_handle *h)
{
	struct ixgbe_link link;
	int flush;
	int err;

	while (1) {
		ixgbe_enable_interrupts(h, ~0);

		err = ixgbe_check_link(h, &link, &flush);
		if (err) {
			perror("Failed to check link state");
			exit(1);
		}

		if (terminated)
			exit(0);

		if (link.speed) {
			printf("Link is UP: speed %u, duplex %u\n", link.speed, link.duplex);
			break;
		} else {
			printf("Link is DOWN. Waiting ...\n");
			sleep(2);
		}
	}
}

static void flush_cb(unsigned long udata, unsigned long cdata)
{
	struct _buffer *buf = (struct _buffer *) cdata;

	printf("Flushed: qi %lu cdata 0x%lx\n", udata, cdata);
	buf->inuse = 0;
}

static void check_link(struct ixgbe_handle *h)
{
	struct ixgbe_link link;
	int flush;
	int err, i;

	err = ixgbe_check_link(h, &link, &flush);
	if (err) {
		perror("Failed to get link status");
		exit(1);
	}

	if (!link.speed) {
		printf("Link is down\n");

		if (flush) {
			printf("Flush FIFOs\n");
			for (i=0; i< nfilters; i++)
				ixgbe_rxq_flush(h, i);
		}

		wait4_link(h);
	}
}

static inline void rx_complete(int qi, unsigned long cdata, ssize_t status)
{
	struct _buffer *buf = (struct _buffer *) cdata;
	struct ether_header *eh;
	uint8_t *ptr;
	size_t len;

	buf->inuse = 0;

	if (!dump)
		return;

	printf("RX completed: qi %u cdata 0x%lx status %d\n", qi, cdata, (int) status);

	if (status < 0)
		return;

	len = status;

	ptr = (uint8_t *) buf->data;

	// Dump header
	eh = (void *) ptr;
	ptr += sizeof(*eh);
	len -= sizeof(*eh);

	printf("Ethernet header:\n");
	printf("\tsrc %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
		eh->ether_shost[0], eh->ether_shost[1], eh->ether_shost[2], 
		eh->ether_shost[3], eh->ether_shost[4], eh->ether_shost[5]);
	printf("\tdst %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
		eh->ether_dhost[0], eh->ether_dhost[1], eh->ether_dhost[2], 
		eh->ether_dhost[3], eh->ether_dhost[4], eh->ether_dhost[5]);
	printf("\ttype 0x%x\n", eh->ether_type);

	// Dump data
	hex_dump("\t", ptr, len, 10);
}

static void do_completion(struct ixgbe_handle *h, int qi)
{
	unsigned long cdata;
	while (1) {
		ssize_t len = ixgbe_rxq_complete(h, qi, &cdata);
		if (!len)
			break;
		rx_complete(qi, cdata, len);
	}
}

static void do_test()
{
	struct ixgbe_handle *h;
	uint8_t addr[ETH_ALEN];
	uint32_t icr;
	int err, i;

	dmafd = uio_dma_open();
	if (dmafd < 0) {
		perror("UIO-DMA open failed");
		exit(1);
	}

	dmabuf = uio_dma_alloc(dmafd, buffsize, UIO_DMA_CACHE_DEFAULT, UIO_DMA_MASK(64), 0);
	if (!dmabuf) {
		perror("Failed to allocate data buffer");
		exit(1);
	}

	h = ixgbe_open(device, dmafd, dmabuf);
	if (!h) {
		perror("IXGBE device open failed");
		exit(1);
	}

	err = ixgbe_set_mtu(h, mtu);
	if (err) {
		perror("Failed to set MTU");
		exit(1);
	}

	ixgbe_get_macaddr(h, addr);
	printf("My Ethernet address: ");
	printf("%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
		addr[0], addr[1], addr[2], 
		addr[3], addr[4], addr[5]);

	if (!nfilters) {
		// Add a dummy filter for our own address.
		// This is not normally needed but we open one RX queue 
		// per address down below so nfilters better be non zero.
		memcpy(filter[0].addr, addr, sizeof(addr));
		filter[0].qi    = 0;
		filter[0].flags = 0;
		nfilters++;
	}

	for (i=0; i < nfilters; i++) {
		err = ixgbe_rxq_open(h, i, ringsize);
		if (err) {
			perror("Failed to enable RX path");
			exit(1);
		}

		ixgbe_rxq_setflushcb(h, i, flush_cb, i);
	}

	err = ixgbe_set_rafilter(h, nfilters, filter);
	if (err) {
		perror("Failed to set RA filter");
		exit(1);
	}

	ixgbe_rx_enable(h);

	ixgbe_set_broadcast(h, !nobcast);

	err = ixgbe_set_promisc(h, promisc);
	if (err) {
		perror("Failed to set promisc mode\n");
		exit(1);
	}

	alloc_buffers(h);

	wait4_link(h);

	ixgbe_trigger_interrupt(h, IXGBE_EICS_GPI_SDP0);

	for (i=0; i < nfilters; i++)
		do_recv(h, i);

	while (!terminated) {
		ixgbe_enable_interrupts(h, ~0);

		err = ixgbe_wait4_interrupt(h, &icr, 0);
		if (err) {
			perror("Failed to get interrupt status");
			break;
		}

		// printf("Interrupt status: 0x%x\n", icr);

		/* Check for link state changes */
		if (icr & IXGBE_EICR_LSC)
			check_link(h);

		for (i=0; i < nfilters; i++) {
			do_completion(h, i);
			if (ixgbe_rxq_room(h, i))
				do_recv(h, i);
		}
	}

	print_stats(h);

	ixgbe_close(h);
	uio_dma_close(dmafd);
}

static const char *main_help =
	"UIO-IXGBE receive tester\n"
	"Usage:\n"
	"\trx [options]\n"
	"\tOptions:\n"
	"\t\t--device | -d <name>    Network device name\n"
	"\t\t--count | -C            Number of packets to receive\n"
	"\t\t--ignore | -I <N>       Ignore first N packets (default 0)\n"
	"\t\t--ring | -r <N>         Number of packets in the ring (default 100)\n"
	"\t\t--buff | -b <N>         Size of the data buffer in bytes\n"
	"\t\t--mtu | -m <N>          MTU size in bytes\n"
	"\t\t--load | -L <N>         Simulate app load (computes checksum N times)\n"
	"\t\t--dump | -D             Dump packet information\n"
	"\t\t--promisc | -P [M]      Set promisc mode (M=1 - uc, M=2 - mc, M=3 - all)\n"
	"\t\t--nobcast | -B          Disable broadcast receive\n"
	"\t\t--filter | -f [A]       Add address to the list of filters\n"
	"\t\t--help | -h             Display this help screen\n";

static struct option main_long_opts[] = {
	{"help", 0, 0, 'h'},
	{"device", 1, 0, 'd'},
	{"count", 1, 0, 'C'},
	{"ignore", 1, 0, 'I'},
	{"ring", 1, 0, 'r'},
	{"buff", 1, 0, 'b'},
	{"mtu",  1, 0, 'm'},
	{"load", 1, 0, 'L'},
	{"dump", 0, 0, 'D'},
	{"promisc", 2, 0, 'P'},
	{"nobcast", 0, 0, 'B'},
	{"filter", 1, 0, 'f'},
	{0, 0, 0, 0}
};

static char main_short_opts[] = "hd:C:I:r:L:DP::Bb:m:f:";

int main(int argc, char *argv[])
{
	int  opt;

	// Parse command line options
	while ((opt = getopt_long(argc, argv, main_short_opts, main_long_opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			strncpy(device, optarg, sizeof(device)-1);
			break;

		case 'C':
			npackets = strtoul(optarg,0,0);
			break;

		case 'I':
			ignore = atoi(optarg);
			break;

		case 'r':
			ringsize = atoi(optarg);
			break;

		case 'b':
			buffsize = atoi(optarg);
			break;

		case 'm':
			mtu = atoi(optarg);
			break;

		case 'D':
			dump = 1;
			break;

		case 'B':
			nobcast = 1;
			break;

		case 'P':
			if (optarg)
				promisc = atoi(optarg);
			else
				promisc = 3;
			break;

		case 'f':
			if (nfilters > MAX_FILTERS) {
				printf("To many filters\n");
				exit(1);
			}

			str2eth(filter[nfilters].addr, optarg);
			filter[nfilters].qi    = nfilters;
			filter[nfilters].flags = 0;
			nfilters++;
			break;

		case 'h':
		default:
			printf(main_help);
			exit(1);
		}
	}

	argc -= optind;
	argv += optind;
	optind = 0;

	if (!device[0]) {
		printf(main_help);
		return 1;
	}

	{
		struct sigaction sa = { { 0 } };

		sa.sa_handler = sig_int;
		sigaction(SIGINT, &sa, NULL);

		sa.sa_handler = sig_int;
		sigaction(SIGTERM, &sa, NULL);
	}

	do_test();

	return 0;
}
