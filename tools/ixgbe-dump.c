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
#include <time.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/user.h>

#include <net/ethernet.h>
#include <netinet/in.h>
#include <pcap.h>

#include "uio-ixgbe/ixgbe-lib.h"

static volatile int  terminated;
static unsigned long count;

static int  opt_promisc;
static int  opt_nobcast;
static int  opt_hexdump;
static int  opt_nofc;
static int  opt_blackhole;
static int  opt_count;

static int  opt_ringsize  = 512;
static int  opt_buffsize  = 2 * 1024 * 1024;
static int  opt_mtu       = ETHERMTU;
static int  opt_poll_usec = 0;
static int  opt_link_timeout_usec = 0;

static char *device;
static char *file;

#define MAX_FILTERS 32
static struct ixgbe_rafilter filter[MAX_FILTERS];
static unsigned int nfilters;

static struct uio_dma_area *dmabuf;
static int dmafd;

/* PCAP files */
static FILE *pcap;

static FILE *open_pcap(char *name)
{
	struct pcap_file_header hdr;
	FILE *f;

	f = fopen(name, "w");
	if (!f) {
		perror("Failed to create dump file");
		exit(1);
	}

	setvbuf(f, NULL, _IOFBF, 128 * 1024);

        hdr.magic         = 0xa1b2c3d4;         // uint32 
        hdr.version_major = PCAP_VERSION_MAJOR; // uint16     
        hdr.version_minor = PCAP_VERSION_MINOR; // uint16     
        hdr.thiszone      = 0;                  // int32  /* gmt to local correction */
        hdr.sigfigs       = 0;                  // uint32 /* accuracy of timestamps */
        hdr.snaplen       = opt_mtu;            // uint32 
        hdr.linktype      = 1;                  // uint32 /* data link type (LINKTYPE_*) */         
	if (!fwrite_unlocked((void *) &hdr, sizeof(hdr), 1, f)) {
		perror("Failed to write into dump file");
		exit(1);
	}

	return f;
}

static void write_pcap(FILE *f, struct timespec *ts, void *buf, unsigned long len)
{
	// Structure defined in pcap.h does not work.
	// It uses 'struct timeval' which is 16 bytes on 64bit machines but
	// tcpdump expects only 8 bytes.
	struct {
		uint32_t sec;
		uint32_t usec;
		uint32_t caplen;
		uint32_t len;
	} hdr;

        hdr.sec    = ts->tv_sec;
        hdr.usec   = ts->tv_nsec;
        hdr.caplen = len;
        hdr.len    = len;

	if (!fwrite_unlocked((void *) &hdr, sizeof(hdr), 1, f)) {
		perror("Failed to write into dump file");
		exit(1);
	}
	if (!fwrite_unlocked(buf, len, 1, f)) {
		perror("Failed to write into dump file");
		exit(1);
	}
}

void close_pcap(FILE *f)
{
	fclose(f);
}

static void hex_dump(const char *pref, uint8_t *buf, size_t len, unsigned int width)
{
	unsigned long i;

	for (i=0; i < len; i++) {
		if (!(i % width))
			printf("%s%s", !i ? "" : "\n", pref);
		printf("%2.2x ", buf[i]);
	}
	if (len)
		printf("\n");
}

static void str2eth(uint8_t *addr, const char *str)
{
	unsigned int a[6], i;

	sscanf(str, "%x:%x:%x:%x:%x:%x",
		&a[0], &a[1], &a[2],
		&a[3], &a[4], &a[5]);

	for (i=0; i < 6; i++)
		addr[i] = a[i];
}

static void print_stats(struct ixgbe_handle *h)
{
	struct ixgbe_stats stats;
	int err;

	err = ixgbe_get_stats(h, &stats);
	if (err) {
		perror("Failed to get stats");
		return;
	}

        printf("---------------------------------------------------------\n");
	printf("Processed %lu packets\n", count); 
        printf("RX stats:\n");
        printf("\tpackets %lu bytes %lu multicast %lu\n"
		"\tdropped %lu errors %lu\n",
		stats.rx_packets, stats.rx_bytes, stats.rx_multicast,
		stats.rx_dropped, stats.rx_errors);

        printf("RX errors:\n");
        printf("\tcrc %lu len %lu frame %lu fifo %lu missed %lu\n",  
        	stats.rx_crc_errors, stats.rx_length_errors, stats.rx_frame_errors, 
		stats.rx_fifo_errors, stats.rx_missed_errors );
}

static void sig_int(int sig)
{
	terminated = 1;
}

struct _buffer {
	void         *data;
	unsigned long size;
	unsigned int  index;
	unsigned int  next;
};

struct _buffer *buffer;
static unsigned int buf_head = ~0U;

static int nbuffers = 100 * 1024;

static struct _buffer *get_buffer()
{
	if (buf_head == ~0U)
		return NULL;

	struct _buffer *b = &buffer[buf_head];
	buf_head = b->next;

	return b;
}

static void put_buffer(struct _buffer *b)
{
	b->next  = buf_head;
	buf_head = b->index;
}

static void init_buffers(struct ixgbe_handle *h)
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

	printf("Number of rx descriptors: %lu\n", (unsigned long) ixgbe_rxq_room(h, 0));
	printf("Available buffer memory %lu rxbuflen %lu\n", size, buflen);

	for (i=0; i < nbuffers && size > buflen; i++) {
		buffer[i].data = mem;
		buffer[i].size = buflen;
		buffer[i].index = i;
		put_buffer(&buffer[i]);

		size -= buflen;
		mem  += buflen;
	}
	nbuffers = i;

	printf("Using %u buffers of %lu size\n", nbuffers, buflen);
}

static void do_recv(struct ixgbe_handle *h)
{
	struct _buffer *buf;
	int err;

	while ((buf = get_buffer())) {
		err = ixgbe_rxq_recv(h, 0, buf->data, buf->size, (unsigned long) buf);
		if (err) {
			if (errno != ENOSPC)
				perror("Receive failed\n");
			put_buffer(buf);
			break;
		}
	}
}

static void flush_cb(unsigned long udata, unsigned long cdata)
{
	struct _buffer *buf = (struct _buffer *) cdata;
	put_buffer(buf);
}

static void wait4_link(struct ixgbe_handle *h, unsigned int to_usec)
{
	struct ixgbe_link link;
	int flush;
	int err;

	unsigned int t = 0;

	while (1) {
		err = ixgbe_check_link(h, &link, &flush);
		if (err) {
			perror("Failed to get link status");
			exit(1);
		}

		if (link.speed) {
			printf("Link up: speed %u, duplex %u\n", link.speed, link.duplex);
			break;
		}

		if (t > to_usec) {
			printf("Link did not come up");
			exit(1);
		}
		usleep(1000); t += 1000;
	}
}

static void check_link(struct ixgbe_handle *h)
{
	struct ixgbe_link link;
	int flush;
	int err;

	err = ixgbe_check_link(h, &link, &flush);
	if (err) {
		perror("Failed to get link status");
		exit(1);
	}

	if (!link.speed) {
		printf("Link down\n");
		if (flush) {
			ixgbe_rxq_flush(h, 0);
		}
	}
}

static inline void rx_complete(unsigned long cdata, ssize_t status)
{
	struct _buffer *buf = (struct _buffer *) cdata;
	struct ether_header *eh;
	struct timespec ts;
	uint8_t *ptr;
	size_t len;

	put_buffer(buf);
	count++;

	if (status < 0)
		return;

	if (opt_blackhole)
		return;

	len = status;

	ptr = (uint8_t *) buf->data;

	clock_gettime(CLOCK_REALTIME, &ts);

	// Write to file mode ?
	if (pcap) {
		write_pcap(pcap, &ts, ptr, len);
		return;
	} 

	// Dump header
	eh = (void *) ptr;
	ptr += sizeof(*eh);
	len -= sizeof(*eh);

	printf("%lu.%lu %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x > "
		"%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, ethertype 0x%x, length %lu\n",
		ts.tv_sec, ts.tv_nsec,
		eh->ether_shost[0], eh->ether_shost[1], eh->ether_shost[2],
		eh->ether_shost[3], eh->ether_shost[4], eh->ether_shost[5],
		eh->ether_dhost[0], eh->ether_dhost[1], eh->ether_dhost[2], 
		eh->ether_dhost[3], eh->ether_dhost[4], eh->ether_dhost[5], 
		eh->ether_type, len);

	if (opt_hexdump) {
		// Dump data
		hex_dump("\t", ptr, len, 16);
	}
}

static void do_completion(struct ixgbe_handle *h)
{
	unsigned long cdata;
	while (1) {
		ssize_t len = ixgbe_rxq_complete(h, 0, &cdata);
		if (!len)
			break;
		rx_complete(cdata, len);

		if (opt_count && count >= opt_count)
			break;
	}
}

static void do_dump()
{
	struct ixgbe_handle *h;
	uint8_t addr[ETH_ALEN];
	uint32_t icr;
	int err;

	if (file) {
		pcap = open_pcap(file);
		if (!pcap)
			exit(1);
	}

	dmafd = uio_dma_open();
	if (dmafd < 0) {
		perror("UIO-DMA open failed");
		exit(1);
	}

	dmabuf = uio_dma_alloc(dmafd, opt_buffsize, UIO_DMA_CACHE_DEFAULT, UIO_DMA_MASK(64), 0);
	if (!dmabuf) {
		perror("Failed to allocate data buffer");
		exit(1);
	}

	h = ixgbe_open(device, dmafd, dmabuf);
	if (!h) {
		perror("IXGBE device open failed");
		exit(1);
	}

	err = ixgbe_set_mtu(h, opt_mtu);
	if (err) {
		perror("Failed to set MTU");
		exit(1);
	}

	ixgbe_get_macaddr(h, addr);
	printf("My Ethernet address: ");
	printf("%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
		addr[0], addr[1], addr[2], 
		addr[3], addr[4], addr[5]);

	err = ixgbe_rxq_open(h, 0, opt_ringsize);
	if (err) {
		perror("Failed to enable RX path");
		exit(1);
	}

	ixgbe_rxq_setflushcb(h, 0, flush_cb, 0);

	err = ixgbe_set_rafilter(h, nfilters, filter);
	if (err) {
		perror("Failed to set RA filter");
		exit(1);
	}

	ixgbe_set_broadcast(h, !opt_nobcast);

	err = ixgbe_set_promisc(h, opt_promisc);
	if (err) {
		perror("Failed to set promisc mode\n");
		exit(1);
	}


	if (opt_nofc) {
		// Disable flow control
		struct ixgbe_flowctl fc = { 0, 0 };
		ixgbe_set_flowctl(h, &fc);
	}

	init_buffers(h);

	if (opt_link_timeout_usec)
		wait4_link(h, opt_link_timeout_usec);

	do_recv(h);

	printf("Capturing ...\n");
	fflush(stdout);

	ixgbe_rx_enable(h);
	while (!terminated) {
		if (!opt_poll_usec) {
			/* Interrupt driven mode */
			ixgbe_enable_interrupts(h, ~0);

			err = ixgbe_wait4_interrupt(h, &icr, 0);
			if (err) {
				if (err != EINTR)
					perror("Failed to get interrupt status");
				break;
			}

			/* Check for link state changes */
			if (icr & (IXGBE_EICR_LSC))
				check_link(h);
		} else {
			/* Polling mode */

			/* FIXME: Need to check for link state changes here */

			usleep(opt_poll_usec);
		}

		do_completion(h);
		if (opt_count && count >= opt_count)
			break;

		if (ixgbe_rxq_room(h, 0))
			do_recv(h);
	}

	ixgbe_rx_disable(h);
	ixgbe_rxq_close(h, 0);

	print_stats(h);
	ixgbe_close(h);

	if (file)
		close_pcap(pcap);
}

static const char *main_help =
	"ixgbe-dump " VERSION "\n"
	"Usage:\n"
	"\tixgbe-dump <device name> [options]\n"
	"\tOptions:\n"
	"\t\t--device | -d <name>    PCI id of the IXGBE device\n"
	"\t\t--ring | -r <N>         Number of packets in the ring (default: 100)\n"
	"\t\t--buff | -b <N>         Size of the data buffer in bytes\n"
	"\t\t--poll | -p <USEC>      Use polling mode with polling interval of USEC\n"
	"\t\t--link-timeout | -L <USEC> How long to wait for the link to come up (default: 0 - no wait)\n"
	"\t\t--mtu | -m <N>          MTU size in bytes\n"
	"\t\t--promisc | -P [M]      Set promisc mode (M=1 - uc, M=2 - mc, M=3 - all)\n"
	"\t\t--nobcast | -B          Disable broadcast reception\n"
	"\t\t--filter | -f [A]       Add address to the list of filters\n"
	"\t\t--write  | -w <file>    Write dump into a file (tcpdump compatible)\n"
	"\t\t--hexdump | -x          Hexdump the payload\n"
	"\t\t--nofc | -F             Disabled flow control\n"
	"\t\t--blackhole | -H        Drop all received packets\n"
	"\t\t--count | -c <N>        Exit after receiving N packets\n"
	"\t\t--help | -h             Display this help screen\n";

static struct option main_long_opts[] = {
	{"help",    0, 0, 'h'},
	{"device",  1, 0, 'd'},
	{"ring",    1, 0, 'r'},
	{"buff",    1, 0, 'b'},
	{"poll",    1, 0, 'p'},
	{"link-timeout", 1, 0, 'L'},
	{"mtu",     1, 0, 'm'},
	{"promisc", 2, 0, 'P'},
	{"nobcast", 0, 0, 'B'},
	{"filter",  1, 0, 'f'},
	{"write",   1, 0, 'w'},
	{"hexdump", 0, 0, 'x'},
	{"nofc",    0, 0, 'F'},
	{"blackhole", 0, 0, 'H'},
	{"count",   1, 0, 'c'},
	{0, 0, 0, 0}
};

static char main_short_opts[] = "hd:r:b:p:L:m:P::Bf:w:xFHc:";

int main(int argc, char *argv[])
{
	int  opt;

	// Parse command line options
	while ((opt = getopt_long(argc, argv, main_short_opts, main_long_opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			device = strdup(optarg);
			break;

		case 'w':
			file = strdup(optarg);
			break;

		case 'r':
			opt_ringsize = atoi(optarg);
			break;

		case 'b':
			opt_buffsize = atoi(optarg);
			break;

		case 'p':
			opt_poll_usec = atoi(optarg);
			break;

		case 'L':
			opt_link_timeout_usec = atoi(optarg);
			break;

		case 'm':
			opt_mtu = atoi(optarg);
			break;

		case 'B':
			opt_nobcast = 1;
			break;

		case 'P':
			if (optarg)
				opt_promisc = atoi(optarg);
			else
				opt_promisc = 3;
			break;

		case 'f':
			if (nfilters > MAX_FILTERS) {
				printf("To many filters\n");
				exit(1);
			}
				
			str2eth(filter[nfilters].addr, optarg);
			nfilters++;
			break;

		case 'x':
			opt_hexdump = 1;
			break;

		case 'F':
			opt_nofc = 1;
			break;
		
		case 'H':
			opt_blackhole = 1;
			break;

		case 'c':
			opt_count = atoi(optarg);
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

	if (!device) {
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

	do_dump();

	return 0;
}
