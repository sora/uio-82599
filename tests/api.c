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

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/user.h>

#include <net/ethernet.h>

#include "uio-dma/uio-dma.h"
#include "uio-ixgbe/ixgbe-lib.h"

static int  ignore;
static int  terminated;
static int  dump;
static unsigned long npackets = ~0UL;
static int  ringsize = 1024;

char device[20] = "";

struct uio_dma_area *dmabuf;
int    dmafd;

void sig_int(int sig)
{
	terminated = 1;
}

void flush_cb(unsigned long udata, unsigned long cdata)
{
	printf("RX flush udata %lu cdata 0x%lu\n", udata, cdata);
}

void send_packet(struct ixgbe_handle *h)
{
	uint8_t dst[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	uint8_t src[ETH_ALEN] = { 0x00, 0x04, 0x03, 0x02, 0x01, 0x10 };
	struct ether_header *eh;
	uint8_t *frame, *ptr, *data;
	unsigned long size;
	int err;

	frame = ptr = dmabuf->addr;

	// Header
	eh = (void *) ptr; ptr += sizeof(*eh);

	memcpy(eh->ether_dhost, dst, ETH_ALEN);
	memcpy(eh->ether_shost, src, ETH_ALEN);
	eh->ether_type = 0x1010;

	// Data
	data = ptr; ptr += 64;
	memset(data + 00, 0x11, 16);
	memset(data + 16, 0x22, 16);
	memset(data + 32, 0x33, 16);
	memset(data + 48, 0x44, 16);

	err = ixgbe_txq_send(h, 0, frame, ptr - frame, 0);
	if (err)
		perror("Send failed\n");
}

void do_test()
{
	struct ixgbe_link link;
	struct ixgbe_handle *h;

	uint32_t icr;
	int flush;
	int err;

	dmafd = uio_dma_open();
	if (dmafd < 0) {
		perror("UIO-DMA open failed");
		exit(1);
	}

	dmabuf = uio_dma_alloc(dmafd, 10000000, UIO_DMA_CACHE_DEFAULT, UIO_DMA_MASK(64), 0);
	if (!dmabuf) {
		perror("Failed to allocate data buffer");
		exit(1);
	}

	h = ixgbe_open(device, dmafd, dmabuf);
	if (!h) {
		perror("IXGBE device open failed");
		exit(1);
	}

	err = ixgbe_txq_open(h, 0, ringsize);
	if (err) {
		perror("Failed to open TX queue 0");
		exit(1);
	}
	ixgbe_txq_setflushcb(h, 0, flush_cb, 0);

	err = ixgbe_rxq_open(h, 0, ringsize);
	if (err) {
		perror("Failed to enable RX path");
		exit(1);
	}
	ixgbe_rxq_setflushcb(h, 0, flush_cb, 0);

	ixgbe_rx_enable(h);

	ixgbe_throttle_intr(h, 0);
	ixgbe_throttle_intr(h, 5000);
	ixgbe_throttle_intr(h, 10000);
	ixgbe_throttle_intr(h, 0);

wait4_link:
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

	send_packet(h);

	while (!terminated) {
		ixgbe_enable_interrupts(h, ~0);

		err = ixgbe_wait4_interrupt(h, &icr, 0);
		if (err) {
			perror("Failed to get interrupt status");
			break;
		}

		printf("Interrupt status: 0x%x\n", icr);

		/* Check for link state changes */
		if (icr & IXGBE_EICR_LSC) {
			err = ixgbe_check_link(h, &link, &flush);
			if (err) {
				perror("Failed to get link status");
				break;
			}

			if (!link.speed) {
				printf("Link is down\n");

				if (flush) {
					printf("Flush FIFOs\n");
					ixgbe_rxq_flush(h, 0);
					ixgbe_txq_flush(h, 0);
				}

				goto wait4_link;
			}
		}

		while (1) {
			unsigned long cdata;
			int r = ixgbe_txq_complete(h, 0, &cdata);
			if (!r)
				break;
			printf("TX complete: status %u, cdata %u\n", r, cdata);
		}

		send_packet(h);
		sleep(1);
	}

	ixgbe_close(h);
	uio_dma_free(dmafd, dmabuf);
	uio_dma_close(dmafd);
}

static const char *main_help =
	"IXGBE UMD API tester\n"
	"Usage:\n"
	"\ttun [options]\n"
	"\tOptions:\n"
	"\t\t--device | -d <name>    Network device name\n"
	"\t\t--count | -C            Number of packets to receive\n"
	"\t\t--ignore | -I <N>       Ignore first N packets (default 0)\n"
	"\t\t--ring | -r <N>         Number of packets in the ring (default 100)\n"
	"\t\t--load | -L <N>         Simulate app load (computes checksum N times)\n"
	"\t\t--dump | -D             Dump packet information\n"
	"\t\t--init | -i <file>      Init script\n"
	"\t\t--help | -h             Display this help screen\n";

static struct option main_long_opts[] = {
	{"help", 0, 0, 'h'},
	{"device", 1, 0, 'd'},
	{"count", 1, 0, 'C'},
	{"ignore", 1, 0, 'I'},
	{"ring", 1, 0, 'r'},
	{"load", 1, 0, 'L'},
	{"dump", 0, 0, 'D'},
	{"init", 1, 0, 'i'},
	{0, 0, 0, 0}
};

static char main_short_opts[] = "hd:C:I:r:L:Di:";

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

		case 'D':
			dump = 1;
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
