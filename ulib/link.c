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
#include "ixgbe-hw.h"
#include "ixgbe-priv.h"

int ixgbe_check_link(struct ixgbe_handle *h, struct ixgbe_link *link, int *flush)
{
	struct uio_ixgbe_link_req req = { 0 };

	if (ioctl(h->fd, UIO_IXGBE_CHECK_LINK, (unsigned long) &req) < 0)
		return -1;

	link->speed       = req.speed;
	link->duplex      = req.duplex;
	link->flowctl     = req.flowctl;

	*flush = req.flush;

	return 0;
}

int ixgbe_get_link(struct ixgbe_handle *h, struct ixgbe_link *link)
{
	struct uio_ixgbe_link_req req = { 0 };

	if (ioctl(h->fd, UIO_IXGBE_GET_LINK, (unsigned long) &req) < 0)
		return -1;

	link->speed       = req.speed;
	link->duplex      = req.duplex;
	link->flowctl     = req.flowctl;

	return 0;
}

int ixgbe_set_link(struct ixgbe_handle *h, struct ixgbe_link *link)
{
	struct uio_ixgbe_link_req req;

	req.speed       = link->speed;
	req.duplex      = link->duplex;
	req.flowctl     = link->flowctl;

	return ioctl(h->fd, UIO_IXGBE_SET_LINK, (unsigned long) &req);
}
