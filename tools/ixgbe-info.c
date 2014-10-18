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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <dirent.h>
#include <ctype.h>

#include <sys/stat.h>

#include "uio-ixgbe/ixgbe-lib.h"

enum Action { INFO, LIST };

static uint action = INFO;
static char *device = NULL;

void show_info()
{
	struct ixgbe_handle* h;

	h = ixgbe_info(device);
	if (!h) {
	      printf("Device: %s, Open failed: %s\n", device, strerror(errno));
	      exit(1);
	}

	// Read own MAC address
	uint8_t addr[ETH_ALEN];

	ixgbe_get_macaddr(h, addr);
	printf("Device: %s\n", device);
	printf("\tMAC address: %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
	      addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	printf("\tIRQ: %u\n", ixgbe_get_irq(h));

	// close ulib device
	ixgbe_free(h);
}

#define PCI_BASEDIR_DEV "/sys/bus/pci/devices/"
#define PCI_BASEDIR_DRV "/sys/bus/pci/drivers/"

#define PCI_VENDOR_INTEL 0x8086
#define PCI_CLASS_NIC    0x20000

#define PCI_DEVID_RANGE0 0x10B6
#define PCI_DEVID_RANGE1 0x10F4

// Query network interface name the device is assosiated with.
// Returns
//   0 - no network if
//   1 - has network if (name stored in netif arg)
static int pci_dev_netif(const char *dev, char *netif)
{
	char str[255], *ptr;
	struct dirent *d;
	DIR *dir;
	int found = 0;

	sprintf(str, PCI_BASEDIR_DEV "%s", dev);
	dir = opendir(str);
	if (!dir) {
		perror("Failed to open pci device dir");
		exit(1);
	}

	while ((d = readdir(dir))) {
		// Find "net:.*" pattern
		if (strstr(d->d_name, "net:")) {
			ptr = strrchr(d->d_name, ':');
			strcpy(netif, ptr + 1);
			found = 1;
			break;
		}
	}

	closedir(dir);
	return found;
}

// Query driver name the device is bound to.
// Returns
//   0 - not bound
//   1 - bound (driver name stored)
static int pci_dev_driver(const char *dev, char *driver)
{
	int len;
	char str[255], *ptr;
	sprintf(str, PCI_BASEDIR_DEV "%s/driver", dev);

	len = readlink(str, str, sizeof(str));
	if (len < 0) {
		if (errno == ENOENT)
			return 0;

		perror("Failed to read driver link");
		exit(1);
	}
	str[len] = '\0';

	ptr = strrchr(str, '/');
	strcpy(driver, ptr + 1);
	return 1;
}

static int pci_dev_attr(const char *dev, const char *attr, void *val)
{
	char str[255];
	sprintf(str, PCI_BASEDIR_DEV "%s/%s", dev, attr);

	FILE *f = fopen(str, "r");
	if (!f) {
		perror("Failed to open device file");
		return -1;
	}

	fscanf(f, "%x", (uint *) val);
	fclose(f);
	return 0;
}

void show_list()
{
	char driver[16];
	char netif[16];
	struct dirent *d;
	DIR *dir;

	dir = opendir(PCI_BASEDIR_DEV);
	if (!dir) {
		perror("Failed to open pci base dir");
		exit(1);
	}

	while ((d = readdir(dir))) {
		uint vendor, devid, class;

		if (!isdigit(d->d_name[0]))
			continue;

		pci_dev_attr(d->d_name, "vendor", &vendor);
		pci_dev_attr(d->d_name, "device", &devid);
		pci_dev_attr(d->d_name, "class",  &class);

		// Filter out Intel NICs
		if (vendor != PCI_VENDOR_INTEL || class != PCI_CLASS_NIC)
			continue;

		// Filter out IXGBE adapters
		if (devid < PCI_DEVID_RANGE0 || devid > PCI_DEVID_RANGE1)
			continue;

		pci_dev_driver(d->d_name, driver);
		if (!pci_dev_netif(d->d_name, netif)) 
			strcpy(netif, "none");

		printf("%s: id %x:%x driver %s (netif %s)\n", d->d_name, vendor, 
			devid, driver, netif);
	}

	closedir(dir);
}

static struct option main_lopts[] = 
{
   { "help",     0,   0,   'h' },
   { "version",  0,   0,   'v' },  
   { "device",   1,   0,   'D' },
   { "list",     0,   0,   'l' },
   { "info",     0,   0,   'i' },
   { 0,          0,   0,    0 }
};

static char main_sopts[] = "hvD:li";

static char main_help[] =
   "Usage:\n"
   "\tixgbe-config [options]\n"
   "Options:\n"
   "\t--help -h               Display help text\n"
   "\t--version -v            Display version\n"
   "\t--list -l               List devices\n"
   "\t--device -D <pci_id>    PCI id of the device\n"
   "\t--info -i               Show device information (default)\n";

static char main_ver[] =
   "ixgbe-info v" VERSION "\n";

int main(int argc, char **argv)
{
	int opt;

	// Retrieve command line args
	while ((opt = getopt_long(argc, argv, main_sopts, main_lopts, NULL)) != -1) {
		switch (opt) {
		case 'v':
			printf(main_ver);
			exit(0);
		case 'D':
			device = strdup(optarg);
			break;
		case 'l':
			action = LIST;
			break;	
		case 'h':
		default:
			printf(main_ver);
			printf(main_help);
			exit(0);
		}
	}

	argc -= optind;

	if (action == LIST) {
		show_list();
		return 0;
	}

	if (!device) {
		printf(main_ver);
		printf(main_help);
		exit(0);
	}

	switch (action) {
	case INFO:
	default:
		show_info(device);
		return 0;
	}

	return 0;
}
