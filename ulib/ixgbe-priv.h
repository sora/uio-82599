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

/**
 * @file ixgbe-priv.h
 * UIO-IXGBE private structs
 */

#ifndef IXGBE_PRIV_H
#define IXGBE_PRIV_H

#include <sys/types.h>
#include <sys/user.h>
#include <sys/uio.h>
#include <net/ethernet.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ixgbe_sw_desc {
	unsigned int    eop;
	unsigned long   cdata;
};

struct ixgbe_ring {
	struct uio_dma_area    *area;
	struct uio_dma_mapping *mapping;

	void                   *hw_desc;
	struct ixgbe_sw_desc   *sw_desc;
	unsigned long           count;
	uint32_t                head;
	uint32_t                tail;

	ixgbe_flushcb           flushcb;
	unsigned long   	udata;
};

/**
 * Adapter handle.
 * Handle is an opaque object returned by ixgbe_open().
 */
struct ixgbe_handle {
	int             fd;
	uint8_t        *mmio_addr;
	unsigned long   mmio_size;

	int 		dmafd;
	int 		dmadevid;
	struct uio_dma_mapping *dmabuf;

	struct ixgbe_ring tx_ring[IXGBE_MAX_TX_QUEUES];
	struct ixgbe_ring rx_ring[IXGBE_MAX_RX_QUEUES];

	unsigned int    rx_buflen;
	unsigned int    mtu;

	struct ixgbe_hw_stats stats;

	struct uio_ixgbe_info info;
};

#ifdef __cplusplus
}
#endif

#endif /* IXGBE_PRIV_H */
