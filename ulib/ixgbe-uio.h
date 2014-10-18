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

#ifndef IXGBE_UIO_H
#define IXGBE_UIO_H

#include <linux/if_ether.h>
#include <linux/types.h>

/* Ioctl defines */
#define UIO_IXGBE_BIND       _IOW('E', 200, int)
struct uio_ixgbe_bind_req {
	char      name[20];
};

/* MAC and PHY info */
struct uio_ixgbe_info {
	uint32_t  irq;
	uint64_t  mmio_base;
	uint32_t  mmio_size;

        uint16_t  mac_type;
        uint8_t   mac_addr[ETH_ALEN];

        uint16_t  phy_type;
        uint32_t  phy_addr;
        uint32_t  phy_id;
        uint32_t  phy_revision;

        uint32_t  num_rx_queues;
        uint32_t  num_tx_queues;
        uint32_t  num_rx_addrs;
        uint32_t  mc_filter_type;
};

#define UIO_IXGBE_INFO       _IOW('E', 201, int)
struct uio_ixgbe_info_req {
	char              name[20];
	struct uio_ixgbe_info info;
};

#define UIO_IXGBE_OPEN       _IOW('E', 202, int)
struct uio_ixgbe_open_req {
	uint32_t          uio_dma_devid;
	struct uio_ixgbe_info info;
};

#define UIO_IXGBE_CLOSE      _IOW('E', 203, int)
#define UIO_IXGBE_RESET      _IOW('E', 204, int)

#define UIO_IXGBE_CHECK_LINK _IOW('E', 205, int)
#define UIO_IXGBE_GET_LINK   _IOW('E', 206, int)
#define UIO_IXGBE_SET_LINK   _IOW('E', 207, int)
struct uio_ixgbe_link_req {
	uint16_t  speed;
	uint16_t  duplex;
	uint16_t  flowctl;
        uint16_t  media_type;
        uint32_t  autoneg_advertised;
        uint8_t   autoneg_wait_to_complete;
	uint16_t  flush;  /* Indicates that TX/RX flush is necessary
			   * after link state changed */
};

#define IXGBESETDEBUG   _IOW('E', 220, int)

#endif /* IXGBE_UIO_H */
