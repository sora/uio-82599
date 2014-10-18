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

int ixgbe_get_flowctl(struct ixgbe_handle *h, struct ixgbe_flowctl *fc)
{
	uint8_t *mmio = h->mmio_addr;
        uint32_t frctl;
        uint32_t rmcs;
	uint32_t fcttv, fcrtl, fcrth;

	/* Read flow control settings */
	frctl = IXGBE_READ_REG(mmio, IXGBE_FCTRL);
	rmcs  = IXGBE_READ_REG(mmio, IXGBE_RMCS);
	fcttv = IXGBE_READ_REG(mmio, IXGBE_FCTTV(0));
	fcrtl =	IXGBE_READ_REG(mmio, IXGBE_FCRTL(0));
	fcrth =	IXGBE_READ_REG(mmio, IXGBE_FCRTH(0));

	fc->rx_xoff = (frctl & IXGBE_FCTRL_RFCE);
	fc->tx_xoff = (rmcs & IXGBE_RMCS_TFCE_802_3X);
	fc->tx_xon  = (fcrtl & IXGBE_FCRTL_XONE);
	fc->xoff_time = fcttv;
	fc->low_water  = fcrtl & ~IXGBE_FCRTL_XONE;
	fc->high_water = fcrth & ~IXGBE_FCRTH_FCEN;

	return 0;
}

int ixgbe_set_flowctl(struct ixgbe_handle *h, struct ixgbe_flowctl *fc)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t frctl;
	uint32_t rmcs;

	/** Read and clear all flags first */
	frctl = IXGBE_READ_REG(mmio, IXGBE_FCTRL);
	frctl &= ~(IXGBE_FCTRL_RFCE | IXGBE_FCTRL_RPFCE);

	rmcs = IXGBE_READ_REG(mmio, IXGBE_RMCS);
	rmcs &= ~(IXGBE_RMCS_TFCE_PRIORITY | IXGBE_RMCS_TFCE_802_3X);

	if (fc->rx_xoff) {
		/** We can receive xoff pause frames */
		frctl |= IXGBE_FCTRL_RFCE;
	}

	if (fc->tx_xoff) {
		/** We can send xoff pause frames */
		rmcs  |= IXGBE_RMCS_TFCE_802_3X;
	}

	/* Update flow control settings. */
	IXGBE_WRITE_REG(mmio, IXGBE_FCTRL, frctl);
	IXGBE_WRITE_REG(mmio, IXGBE_RMCS, rmcs);

	/* Setup rx thresholds */
	if (fc->tx_xoff) {
		uint32_t fcrtl = fc->low_water;
		if (fc->tx_xon)
			fcrtl |= IXGBE_FCRTL_XONE;

		IXGBE_WRITE_REG(mmio, IXGBE_FCRTL(0), fcrtl);
		IXGBE_WRITE_REG(mmio, IXGBE_FCRTH(0), fc->high_water | IXGBE_FCRTH_FCEN);
	}

	IXGBE_WRITE_REG(mmio, IXGBE_FCTTV(0), fc->xoff_time);
	IXGBE_WRITE_REG(mmio, IXGBE_FCRTV,   (fc->xoff_time >> 1));

	return 0;
}
