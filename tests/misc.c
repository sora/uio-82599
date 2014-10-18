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
#include <stdint.h>

#include "uio-ixgbe/ixgbe-lib.h"

void hex_dump(const char *pref, uint8_t *buf, size_t len, unsigned int width)
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

void str2eth(uint8_t *addr, const char *str)
{
	unsigned int a[6], i;

	sscanf(str, "%x:%x:%x:%x:%x:%x",
		&a[0], &a[1], &a[2],
		&a[3], &a[4], &a[5]);

	for (i=0; i < 6; i++)
		addr[i] = a[i];
}

void print_stats(struct ixgbe_handle *h)
{
	struct ixgbe_stats stats;
	int err;

	err = ixgbe_get_stats(h, &stats);
	if (err) {
		perror("Failed to get stats");
		return;
	}

        printf("rx_packets        %lu\n",  stats.rx_packets       );
        printf("rx_multicast      %lu\n",  stats.rx_multicast     );
        printf("tx_packets        %lu\n",  stats.tx_packets       );
        printf("rx_bytes          %lu\n",  stats.rx_bytes         );
        printf("tx_bytes          %lu\n",  stats.tx_bytes         );
        printf("collisions        %lu\n",  stats.collisions       );
        printf("rx_errors         %lu\n",  stats.rx_errors        );
        printf("rx_dropped        %lu\n",  stats.rx_dropped       );
        printf("rx_length_errors  %lu\n",  stats.rx_length_errors );
        printf("rx_crc_errors     %lu\n",  stats.rx_crc_errors    );
        printf("rx_frame_errors   %lu\n",  stats.rx_frame_errors  );
        printf("rx_fifo_errors    %lu\n",  stats.rx_fifo_errors   );
        printf("rx_missed_errors  %lu\n",  stats.rx_missed_errors );
        printf("tx_errors         %lu\n",  stats.tx_errors        );
        printf("tx_aborted_errors %lu\n",  stats.tx_aborted_errors);
        printf("tx_window_errors  %lu\n",  stats.tx_window_errors );
        printf("tx_carrier_errors %lu\n",  stats.tx_carrier_errors);
}
