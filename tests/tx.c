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

#include "misc.h"

static int  terminated;
static int  ringsize = 256;
static int  buffsize = 1 * 1024 * 1024;
static int  mtu = 64;

static unsigned long npackets = ~0UL;

static int  use_iovec = 0;

static char device[20] = "";

#define NQ 16

static char   *dst_str[NQ] = { "ff:ff:ff:ff:ff:ff" };
static uint8_t dst_addr[NQ][ETH_ALEN];
static int     dst_count = 1;

static char   *src_str[NQ] = { "00:11:22:33:44:55" };
static uint8_t src_addr[NQ][ETH_ALEN];
static int     src_count = 1;

static int speed;
static int duplex;

static struct uio_dma_area *dmabuf;
static int    dmafd;

void sig_int(int sig)
{
	terminated = 1;
}

static void flush_cb(unsigned long udata, unsigned long cdata)
{
	printf("Flushed queue %lu cdata 0x%lx\n", udata, cdata);
}

static void do_completion(struct ixgbe_handle *h, int si)
{
	unsigned long cdata;
	while (1) {
		ssize_t status = ixgbe_txq_complete(h, si, &cdata);
		if (!status)
			break;
		printf("TX completed: queue %u cdata 0x%lx status %d\n",
			si, cdata, (int) status);
	}
}

static inline uint8_t *pktbuf(int si, int di)
{
	unsigned int size = mtu + sizeof(struct ether_header);
	unsigned int offset = (size * NQ) * si + (size * di);
	return dmabuf->addr + offset;
}

// Populate packet buffer with the proper header and payload
void prep_packet(int si, int di)
{
	struct ether_header *eh;
	uint8_t *frame, *ptr, *data;
	unsigned long size, frag;
	int i;

	frame = ptr = pktbuf(si, di);

	// Header
	eh = (void *) ptr; ptr += sizeof(*eh);

	memcpy(eh->ether_dhost, dst_addr[di], ETH_ALEN);
	memcpy(eh->ether_shost, src_addr[si], ETH_ALEN);
	eh->ether_type = 0x1010;

	// Data
	data = ptr; ptr += mtu;

	size = mtu;
	frag = mtu / 4;

	for (i=0; i < 3; i++, size -= frag)
		memset(data + (i * frag), i, frag);
	memset(data + (i * frag), i, size);
}

void send_packet(struct ixgbe_handle *h, int si, int di)
{
	struct ether_header *eh;
	uint8_t *frame, *ptr, *data;
	unsigned long size, frag;
	int i, err;

	frame = ptr = pktbuf(si, di);

	eh = (void *) ptr; ptr += sizeof(*eh);
	data = ptr; ptr += mtu;
	size = mtu;
	frag = mtu / 4;

	for (i=0; i < 3; i++, size -= frag)
		memset(data + (i * frag), i, frag);
	memset(data + (i * frag), i, size);

	if (use_iovec) {
		struct iovec iov[5];

		iov[0].iov_base = eh; iov[0].iov_len = sizeof(*eh);

		size = mtu;
		for (i=0; i < 3; i++, size -= frag) {
			iov[i+1].iov_base = data + (i * frag); 
			iov[i+1].iov_len  = frag;
		}
		iov[i+1].iov_base = data + (i * frag);
		iov[i+1].iov_len  = size;

		err = ixgbe_txq_sendv(h, si, iov, 5, si);
		if (err)
			perror("Send failed\n");
	} else {
		err = ixgbe_txq_send(h, si, frame, ptr - frame, si);
		if (err)
			perror("Send failed\n");
	}
}

void do_test()
{
	struct ixgbe_link link;
	struct ixgbe_handle *h;

	uint32_t icr;
	int flush;
	int err, i, n;

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

	if (speed || duplex) {
		struct ixgbe_link l = {
			.autoneg = (!speed || !duplex) ? 1 : 0,
			.speed   = speed,
			.duplex  = duplex,
			.flowctl = ixgbe_fc_full,
		};

		err = ixgbe_set_link(h, &l);
		if (err) {
			perror("Failed to set link speed and stuff");
			exit(1);
		}
	}

	// Open TX queues (one for each source)
	for (i=0; i < src_count; i++) {
		err = ixgbe_txq_open(h, i, ringsize);
		if (err) {
			fprintf(stderr, "Failed to open TX queue %u. %s(%d)\n", i,
					strerror(errno), errno);
			exit(1);
		}
		ixgbe_txq_setflushcb(h, i, flush_cb, i);
	}

	// Populate packets (one per source + destination pair)
	for (i=0; i < src_count; i++)
		for (n=0; n < dst_count; n++)
			prep_packet(i, n);

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

	// Send packets (one per source + destination pair)
	for (i=0; i < src_count; i++)
		for (n=0; n < dst_count; n++)
			send_packet(h, i, n);

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
					printf("Flushing queue(s)\n");
					for (i=0; i < src_count; i++)
						ixgbe_txq_flush(h, i);
				}

				goto wait4_link;
			}
		}

		// Collect completed TX (one per source)
		for (i=0; i < src_count; i++)
			do_completion(h, i);

		// Send packets (one per source + destination pair)
		for (i=0; i < src_count; i++)
			for (n=0; n < dst_count; n++)
				send_packet(h, i, n);
	}

	print_stats(h);

	ixgbe_reset_stats(h);

	ixgbe_close(h);
}

static const char *main_help =
	"UIO-IXGBE TX API tester\n"
	"Usage:\n"
	"\tuio-ixgbe-tx [options]\n"
	"\tOptions:\n"
	"\t\t--device | -d <name>    Network device name\n"
	"\t\t--count | -C            Number of packets to send\n"
	"\t\t--ring | -r <N>         Number of packets in the ring (default 100)\n"
	"\t\t--buff | -b <N>         Size of the data buffer in bytes\n"
	"\t\t--mtu  | -m <N>         MTU in bytes\n"
	"\t\t--iovec | -i            Use iovec\n"
	"\t\t--dest | -D <A>         Use A as the destination address\n"
	"\t\t--src | -S <A>          Use A as the source address\n"
	"\t\t--speed | -s <L>        Link speed\n"
	"\t\t--duplex | -u <L>       Link duplex\n"
	"\t\t--help | -h             Display this help screen\n";

static struct option main_long_opts[] = {
	{"help", 0, 0, 'h'},
	{"device", 1, 0, 'd'},
	{"count", 1, 0, 'C'},
	{"ring", 1, 0, 'r'},
	{"buff", 1, 0, 'b'},
	{"iovec", 0, 0, 'i'},
	{"dest", 1, 0, 'D'},
	{"src", 1, 0, 'S'},
	{"mtu", 1, 0, 'm'},
	{"speed", 1, 0, 's'},
	{"duplex", 1, 0, 'u'},
	{0, 0, 0, 0}
};

static char main_short_opts[] = "hd:C:r:D:S:ib:m:s:u:";

int main(int argc, char *argv[])
{
	int nsrc = 0, ndst = 0;
	int opt, i;

	// Parse command line options
	while ((opt = getopt_long(argc, argv, main_short_opts, main_long_opts, NULL)) != -1) {
		switch (opt) {
		case 'i':
			use_iovec = 1;
			break;

		case 'd':
			strncpy(device, optarg, sizeof(device)-1);
			break;

		case 'C':
			npackets = strtoul(optarg,0,0);
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

		case 's':
			speed = atoi(optarg);
			break;

		case 'u':
			duplex = atoi(optarg);
			break;

		case 'D':
			dst_str[ndst++] = strdup(optarg);
			break;

		case 'S':
			src_str[nsrc++] = strdup(optarg);
			break;

		case 'h':
		default:
			printf(main_help);
			exit(1);
		}
	}

	src_count = nsrc ? nsrc : 1;
	dst_count = ndst ? ndst : 1;

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

	for (i=0; i < src_count; i++)
		str2eth(src_addr[i], src_str[i]);

	for (i=0; i < dst_count; i++)
		str2eth(dst_addr[i], dst_str[i]);

	do_test();

	return 0;
}
