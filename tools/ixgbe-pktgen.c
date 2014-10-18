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
#include <stdint.h>
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

#include "uio-ixgbe/ixgbe-lib.h"

static volatile int terminated;

static int   ringsize = 256;
static int   buffsize = 1 * 1024 * 1024;
static int   mtu = 64;
static ulong npackets = ~0UL;
static int   nofc;

static char device[20] = "";

static char *destination = "ff:ff:ff:ff:ff:ff";
static char *source      = "00:11:22:33:44:01";

static int speed;
static int duplex;

static struct uio_dma_area *dmabuf;
static int    dmafd;

static void str2eth(uint8_t *addr, const char *str)
{
	unsigned int a[6], i;

	sscanf(str, "%x:%x:%x:%x:%x:%x",
		&a[0], &a[1], &a[2],
		&a[3], &a[4], &a[5]);

	for (i=0; i < 6; i++)
		addr[i] = a[i];
}

static void print_stats(struct ixgbe_handle *h, uint64_t usec)
{
	struct ixgbe_stats stats;
	int err;
	double kpps, uspk;

	err = ixgbe_get_stats(h, &stats);
	if (err) {
		perror("Failed to get stats");
		return;
	}

        printf("tx_packets        %lu\n",  stats.tx_packets       );
        printf("tx_bytes          %lu\n",  stats.tx_bytes         );
        printf("collisions        %lu\n",  stats.collisions       );
        printf("tx_errors         %lu\n",  stats.tx_errors        );
        printf("tx_aborted_errors %lu\n",  stats.tx_aborted_errors);
        printf("tx_window_errors  %lu\n",  stats.tx_window_errors );
        printf("tx_carrier_errors %lu\n",  stats.tx_carrier_errors);

	kpps = stats.tx_packets;
	uspk = (double) usec / kpps;
	kpps = kpps * 1000 / usec;
	printf("tx: usec %lu pkts %lu kpps %lf (uspk %lf)\n", 
		usec, stats.tx_packets, kpps, uspk);
}

static void sig_int(int sig)
{
	terminated = 1;
}

static uint8_t dst[ETH_ALEN];
static uint8_t src[ETH_ALEN];

static uint64_t gtod()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000000ULL + tv.tv_usec;
}

static void prepare(struct ixgbe_handle *h)
{
	struct ether_header *eh;
	uint8_t *frame, *ptr, *data;
	unsigned long size, frag;
	int i;

	ixgbe_reset_stats(h);

	frame = ptr = dmabuf->addr;
	size = dmabuf->size;

	// Header
	eh = (void *) ptr; ptr += sizeof(*eh);

	memcpy(eh->ether_dhost, dst, ETH_ALEN);
	memcpy(eh->ether_shost, src, ETH_ALEN);
	eh->ether_type = 0x1010;

	// Data
	data = ptr; ptr += mtu;

	size = mtu;
	frag = mtu / 4;

	for (i=0; i < 3; i++, size -= frag)
		memset(data + (i * frag), i, frag);
	memset(data + (i * frag), i, size);


	ixgbe_enable_interrupts(h, ~0);
}

static int transmit(struct ixgbe_handle *h)
{
	struct ether_header *eh;
	uint8_t *frame, *ptr;
	unsigned long size;
	int err;

	frame = ptr = dmabuf->addr;
	size  = dmabuf->size;
	ptr += sizeof(*eh) + mtu;

	err = ixgbe_txq_send(h, 0, frame, ptr - frame, 0);
	if (err) {
		if (errno != ENOSPC)
			perror("Send failed\n");
		return -1;
	}
	return 0;
}

static void wait4link(struct ixgbe_handle *h)
{
	struct ixgbe_link link;
	int err, flush;

	while (1) {
		err = ixgbe_check_link(h, &link, &flush);

		if (terminated)
			exit(0);

		if (err) {
			perror("Failed to check link state");
			exit(1);
		}

		if (link.speed) {
			printf("Link is UP: speed %u, duplex %u\n", link.speed, link.duplex);
			break;
		} else {
			printf("Link is DOWN. Waiting ...\n");
			sleep(1);
		}
	}
}

void collect(struct ixgbe_handle *h)
{
	unsigned long cdata;

	while (1) {
		ssize_t status = ixgbe_txq_complete(h, 0, &cdata);
		if (!status)
			break;
	}
}

static int holdup(struct ixgbe_handle *h)
{
	uint32_t icr;
	int err, flush;

	err = ixgbe_wait4_interrupt(h, &icr, 0);
	if (err) {
		perror("Failed to get interrupt status");
		return -1;
	}

	/* Check for link state changes */
	if (icr & IXGBE_EICR_LSC) {
		struct ixgbe_link link;

		err = ixgbe_check_link(h, &link, &flush);
		if (err) {
			perror("Failed to get link status");
			return -1;
		}

		if (!link.speed) {
			printf("Link is down\n");

			if (flush) {
				printf("Flushing FIFOs\n");
				ixgbe_txq_flush(h, 0);
			}

			wait4link(h);
			return 0;
		}
	}

	ixgbe_enable_interrupts(h, ~0);
	return 0;
}

void generate()
{
	struct ixgbe_handle *h;

	int err;

	uint64_t t1, t2, count = 0;

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

	err = ixgbe_txq_open(h, 0, ringsize);
	if (err) {
		fprintf(stderr, "Failed to open TX queue %u. %s(%d)\n", 0,
				strerror(errno), errno);
		exit(1);
	}

	if (nofc) {
		// Disable flow control
		struct ixgbe_flowctl fc = { 0, 0 };
		ixgbe_set_flowctl(h, &fc);
	}

	prepare(h);

	wait4link(h);

	t1 = gtod();

	while (!terminated) {
		if (transmit(h) < 0) {
			// Looks like our ring is full.
			// Lets give the card a chance to drain it.
			if (holdup(h) < 0)
				break;
			collect(h);
		}

		count++;

		if (count >= npackets)
			break;

		// Sweap tx ring once in awhile
		if (!(count % 32))
			collect(h);
	}

	t2 = gtod();
	
	print_stats(h, t2 - t1);

	ixgbe_close(h);
}

static const char *main_help =
	"ixgbe-pktgen packet generator\n"
	"Usage:\n"
	"\tixgbe-pktgen [options]\n"
	"\tOptions:\n"
	"\t\t--device | -d <name>    PCI device\n"
	"\t\t--count | -C            Number of packets to generate\n"
	"\t\t--ring | -r <N>         Number of packets in the ring (default 256)\n"
	"\t\t--buff | -b <N>         Size of the data buffer in bytes\n"
	"\t\t--mtu  | -m <N>         MTU in bytes\n"
	"\t\t--dest | -D <A>         Use A as the destination address\n"
	"\t\t--src  | -S <A>         Use A as the source address\n"
	"\t\t--speed | -s <L>        Link speed\n"
	"\t\t--duplex | -u <L>       Link duplex\n"
	"\t\t--nofc | -F             Disable flow control\n"
	"\t\t--help | -h             Display this help screen\n";

static struct option main_long_opts[] = {
	{"help", 0, 0, 'h'},
	{"device", 1, 0, 'd'},
	{"count", 1, 0, 'C'},
	{"ring", 1, 0, 'r'},
	{"buff", 1, 0, 'b'},
	{"load", 1, 0, 'L'},
	{"dest", 1, 0, 'D'},
	{"src", 1, 0, 'S'},
	{"mtu", 1, 0, 'm'},
	{"speed", 1, 0, 's'},
	{"duplex", 1, 0, 'u'},
	{"nofc",  0, 0, 'F'},
	{0, 0, 0, 0}
};

static char main_short_opts[] = "hd:C:r:L:D:S:b:m:s:u:F";

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
			destination = strdup(optarg);
			break;

		case 'S':
			source = strdup(optarg);
			break;

		case 'F':
			nofc = 1;
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

	str2eth(dst, destination);
	str2eth(src, source);

	generate();

	return 0;
}
