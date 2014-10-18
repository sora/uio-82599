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

static int update_stats(struct ixgbe_handle *h)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t i, missed_rx = 0, mpc, bprc, lxon, lxoff, xon_off_tot;

	h->stats.crcerrs += IXGBE_READ_REG(mmio, IXGBE_CRCERRS);
	h->stats.illerrc += IXGBE_READ_REG(mmio, IXGBE_ILLERRC);
	h->stats.errbc   += IXGBE_READ_REG(mmio, IXGBE_ERRBC);
	h->stats.mspdc   += IXGBE_READ_REG(mmio, IXGBE_MSPDC);
	h->stats.rlec    += IXGBE_READ_REG(mmio, IXGBE_RLEC);
	h->stats.mlfc    += IXGBE_READ_REG(mmio, IXGBE_MLFC);
	h->stats.mrfc    += IXGBE_READ_REG(mmio, IXGBE_MRFC);

	h->stats.mpctotal = 0;
	for (i = 0; i < 8; i++) {
		/* for packet buffers not used, the register should read 0 */
		mpc = IXGBE_READ_REG(mmio, IXGBE_MPC(i));
		missed_rx += mpc;

		h->stats.mpc[i]   += mpc;
		h->stats.mpctotal += h->stats.mpc[i];

		h->stats.rnbc[i] += IXGBE_READ_REG(mmio, IXGBE_RNBC(i));
	}

	h->stats.gprc += IXGBE_READ_REG(mmio, IXGBE_GPRC);
	/* work around hardware counting issue */
	h->stats.gprc -= missed_rx;

	/* 82598 hardware only has a 32 bit counter in the high register */
	h->stats.gorc += IXGBE_READ_REG(mmio, IXGBE_GORCH);
	h->stats.gotc += IXGBE_READ_REG(mmio, IXGBE_GOTCH);
	h->stats.tor  += IXGBE_READ_REG(mmio, IXGBE_TORH);

	bprc = IXGBE_READ_REG(mmio, IXGBE_BPRC);
	h->stats.bprc += bprc;
	h->stats.mprc += IXGBE_READ_REG(mmio, IXGBE_MPRC);
	h->stats.mprc -= bprc;
	h->stats.roc += IXGBE_READ_REG(mmio, IXGBE_ROC);
	h->stats.prc64  += IXGBE_READ_REG(mmio, IXGBE_PRC64);
	h->stats.prc127 += IXGBE_READ_REG(mmio, IXGBE_PRC127);
	h->stats.prc255 += IXGBE_READ_REG(mmio, IXGBE_PRC255);
	h->stats.prc511 += IXGBE_READ_REG(mmio, IXGBE_PRC511);
	h->stats.prc1023 += IXGBE_READ_REG(mmio, IXGBE_PRC1023);
	h->stats.prc1522 += IXGBE_READ_REG(mmio, IXGBE_PRC1522);

	h->stats.lxonrxc  += IXGBE_READ_REG(mmio, IXGBE_LXONRXC);
	h->stats.lxoffrxc += IXGBE_READ_REG(mmio, IXGBE_LXOFFRXC);

	lxon  = IXGBE_READ_REG(mmio, IXGBE_LXONTXC);
	lxoff = IXGBE_READ_REG(mmio, IXGBE_LXOFFTXC);
	h->stats.lxontxc  += lxon;
	h->stats.lxofftxc += lxoff;

	h->stats.ruc  += IXGBE_READ_REG(mmio, IXGBE_RUC);
	h->stats.gptc += IXGBE_READ_REG(mmio, IXGBE_GPTC);
	h->stats.mptc += IXGBE_READ_REG(mmio, IXGBE_MPTC);

	/*
	 * 82598 errata - tx of flow control packets is included in tx counters
	 */
	xon_off_tot = lxon + lxoff;
	h->stats.gptc -= xon_off_tot;
	h->stats.mptc -= xon_off_tot;
	h->stats.gotc -= (xon_off_tot * (ETH_ZLEN + ETH_FCS_LEN));
	h->stats.ruc  += IXGBE_READ_REG(mmio, IXGBE_RUC);
	h->stats.rfc  += IXGBE_READ_REG(mmio, IXGBE_RFC);
	h->stats.rjc  += IXGBE_READ_REG(mmio, IXGBE_RJC);
	h->stats.tpr  += IXGBE_READ_REG(mmio, IXGBE_TPR);
	h->stats.ptc64 += IXGBE_READ_REG(mmio, IXGBE_PTC64);
	h->stats.ptc64 -= xon_off_tot;
	h->stats.ptc127 += IXGBE_READ_REG(mmio, IXGBE_PTC127);
	h->stats.ptc255 += IXGBE_READ_REG(mmio, IXGBE_PTC255);
	h->stats.ptc511 += IXGBE_READ_REG(mmio, IXGBE_PTC511);
	h->stats.ptc1023 += IXGBE_READ_REG(mmio, IXGBE_PTC1023);
	h->stats.ptc1522 += IXGBE_READ_REG(mmio, IXGBE_PTC1522);
	h->stats.bptc += IXGBE_READ_REG(mmio, IXGBE_BPTC);
	return 0;
}

void ixgbe_reset_stats(struct ixgbe_handle *h)
{
	update_stats(h);
	memset(&h->stats, 0, sizeof(h->stats));
}

int ixgbe_get_hwstats(struct ixgbe_handle *h, struct ixgbe_hw_stats *st)
{
	if (update_stats(h) < 0)
		return -1;
	*st = h->stats;
	return 0;
}

int ixgbe_get_stats(struct ixgbe_handle *h, struct ixgbe_stats *st)
{
	if (update_stats(h) < 0)
		return -1;
	/* Convert HW stats into the format that most people are used to */
	st->rx_packets   = h->stats.gprc;
	st->rx_multicast = h->stats.mprc;
	st->tx_packets   = h->stats.gptc;
	st->rx_bytes     = h->stats.gorc;
	st->tx_bytes     = h->stats.gotc;
	st->collisions   = 0;

	st->rx_dropped       = h->stats.mpctotal;
	st->rx_length_errors = h->stats.rlec;
	st->rx_crc_errors    = h->stats.crcerrs;
	st->rx_frame_errors  = h->stats.illerrc + h->stats.errbc;
	st->rx_fifo_errors   = h->stats.mpctotal;
	st->rx_missed_errors = h->stats.mpctotal;

	st->rx_errors = st->rx_dropped + st->rx_length_errors +
                        st->rx_crc_errors + st->rx_frame_errors +
                        st->rx_fifo_errors + st->rx_missed_errors;

	st->tx_errors = 0; 
	st->tx_aborted_errors = 0;
	st->tx_window_errors  = 0;
	st->tx_carrier_errors = 0;

	return 0;
}
