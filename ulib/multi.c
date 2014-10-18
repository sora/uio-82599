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
  This file is partially based on the original ixgbe-1.6.4 driver
  for FreeBSD.

  Copyright (c) 2001-2008, Intel Corporation 
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
  
   2. Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution.
  
   3. Neither the name of the Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
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

int ixgbe_set_broadcast(struct ixgbe_handle *h, uint8_t mode)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t fctrl = IXGBE_READ_REG(mmio, IXGBE_FCTRL);

	switch (mode) {
	case IXGBE_BROADCAST_OFF:
                fctrl &= ~IXGBE_FCTRL_BAM;
		break;
	case IXGBE_BROADCAST_ON:
	default:
                fctrl |= IXGBE_FCTRL_BAM;
		break;
        }

        IXGBE_WRITE_REG(mmio, IXGBE_FCTRL, fctrl);
	return 0;
}

int ixgbe_set_promisc(struct ixgbe_handle *h, uint8_t mode)
{
	uint8_t *mmio = h->mmio_addr;
	uint32_t fctrl = IXGBE_READ_REG(mmio, IXGBE_FCTRL);

	switch (mode) {
	case IXGBE_PROMISC_UC | IXGBE_PROMISC_MC:
                fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
		break;

	case IXGBE_PROMISC_UC:
                fctrl |= IXGBE_FCTRL_UPE;
                fctrl &= ~IXGBE_FCTRL_MPE;
		break;

	case IXGBE_PROMISC_MC:
                fctrl |= IXGBE_FCTRL_MPE;
                fctrl &= ~IXGBE_FCTRL_UPE;
		break;

	default:
                fctrl &= ~(IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
		break;
        }

        IXGBE_WRITE_REG(mmio, IXGBE_FCTRL, fctrl);
	return 0;
}

static int ixgbe_mta_vector(struct ixgbe_handle *h, uint8_t *mc_addr)
{
	uint32_t vector = 0;

	switch (h->info.mc_filter_type) {
	case 0:	  /* use bits [47:36] of the address */
		vector = ((mc_addr[4] >> 4) | (((uint16_t)mc_addr[5]) << 4));
		break;
	case 1:	  /* use bits [46:35] of the address */
		vector = ((mc_addr[4] >> 3) | (((uint16_t)mc_addr[5]) << 5));
		break;
	case 2:	  /* use bits [45:34] of the address */
		vector = ((mc_addr[4] >> 2) | (((uint16_t)mc_addr[5]) << 6));
		break;
	case 3:	  /* use bits [43:32] of the address */
		vector = ((mc_addr[4]) | (((uint16_t)mc_addr[5]) << 8));
		break;
	default:  /* Invalid mc_filter_type */
		break;
	}

	/* vector can only be 12-bits or boundary will be exceeded */
	vector &= 0xFFF;
	return vector;
}

/**
 *  ixgbe_set_mta - Set bit-vector in multicast table
 *  @hw: pointer to hardware structure
 *  @hash_value: Multicast address hash value
 *
 *  Sets the bit-vector in the multicast table.
 **/
void ixgbe_set_mta(struct ixgbe_handle *h, uint8_t *mc_addr)
{
	uint8_t *mmio = h->mmio_addr;

	uint32_t vector;
	uint32_t vector_bit;
	uint32_t vector_reg;
	uint32_t mta_reg;

	vector = ixgbe_mta_vector(h, mc_addr);

	/*
	 * The MTA is a register array of 128 32-bit registers. It is treated
	 * like an array of 4096 bits.  We want to set bit
	 * BitArray[vector_value]. So we figure out what register the bit is
	 * in, read it, OR in the new bit, then write back the new value.  The
	 * register is determined by the upper 7 bits of the vector value and
	 * the bit within that register are determined by the lower 5 bits of
	 * the value.
	 */
	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	mta_reg = IXGBE_READ_REG(mmio, IXGBE_MTA(vector_reg));
	mta_reg |= (1 << vector_bit);
	IXGBE_WRITE_REG(mmio, IXGBE_MTA(vector_reg), mta_reg);
}

int ixgbe_set_rar(struct ixgbe_handle *h, unsigned int index, uint8_t *addr,
		uint32_t qi, uint8_t active)
{
	uint8_t *mmio = h->mmio_addr;
	u32 rar_low, rar_high;

	/*
	 * HW expects these in little endian so we reverse the byte order from
	 * network order (big endian) to little endian
	 */
	rar_low =  ((uint32_t)addr[0] |
		   ((uint32_t)addr[1] << 8) |
		   ((uint32_t)addr[2] << 16) |
		   ((uint32_t)addr[3] << 24));

	rar_high =  ((uint32_t)addr[4] |
		    ((uint32_t)addr[5] << 8) |
		    ((qi << IXGBE_RAH_VIND_SHIFT) & IXGBE_RAH_VIND_MASK));

	if (active)
		rar_high |= IXGBE_RAH_AV;

	IXGBE_WRITE_REG(mmio, IXGBE_RAL(index), rar_low);
	IXGBE_WRITE_REG(mmio, IXGBE_RAH(index), rar_high);

	return 0;
}

int ixgbe_set_rafilter(struct ixgbe_handle *h, unsigned int count, struct ixgbe_rafilter *filter)
{
	uint8_t *mmio = h->mmio_addr;
	unsigned int rar_entries = h->info.num_rx_addrs;
	unsigned int i;

	/* load the first set of addresses into the exact filters.
	 * RAR 0 is used for the station MAC adddress.
	 * If there are not enough addresses zero out the rest of the RAR
	 * entries.
	 */
	for (i = 1; i < rar_entries; i++) {
		if (!count) {
			/* Clear this RAR entry */
			IXGBE_WRITE_REG(mmio, IXGBE_RAL(i), 0);
			IXGBE_WRITE_REG(mmio, IXGBE_RAH(i), 0);
			continue;
		}

		ixgbe_set_rar(h, i, filter->addr, filter->qi, 1);
		count--; filter++;
	}

	/* Clear the MTA */
	for (i = 0; i < IXGBE_MC_TBL_SIZE; i++)
		IXGBE_WRITE_REG(mmio, IXGBE_MTA(i), 0);

	if (count) {
		/* Add the rest of the addresses to the MTA */
		while (count) {
			ixgbe_set_mta(h, filter->addr);
			count--; filter++;
		}

		/* Enable mta */
		IXGBE_WRITE_REG(mmio, IXGBE_MCSTCTRL, IXGBE_MCSTCTRL_MFE | h->info.mc_filter_type);
	}

	return 0;
}
