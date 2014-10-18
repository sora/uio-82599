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
 * @file ixgbe-lib.h
 * UIO-IXGBE library api
 */

#ifndef IXGBE_LIB_H
#define IXGBE_LIB_H

#include <sys/types.h>
#include <sys/user.h>
#include <sys/uio.h>
#include <net/ethernet.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "uio-dma/uio-dma.h"
#include "uio-ixgbe/ixgbe-hw.h"

/* The sizes (in bytes) of a ethernet packet */
#define ENET_HEADER_SIZE             14
#define MAXIMUM_ETHERNET_FRAME_SIZE  1518 /* With FCS */
#define MINIMUM_ETHERNET_FRAME_SIZE  64   /* With FCS */
#define ETHERNET_FCS_SIZE            4
#define MAXIMUM_ETHERNET_PACKET_SIZE \
    (MAXIMUM_ETHERNET_FRAME_SIZE - ETHERNET_FCS_SIZE)
#define MINIMUM_ETHERNET_PACKET_SIZE \
    (MINIMUM_ETHERNET_FRAME_SIZE - ETHERNET_FCS_SIZE)
#define CRC_LENGTH                   ETHERNET_FCS_SIZE
#define MAX_JUMBO_FRAME_SIZE         0x3F00

/**
 * Maximum number of queues.
 */
enum {
 	/* Note HW actually supports more */
	IXGBE_MAX_RX_QUEUES  = 16,
	IXGBE_MAX_TX_QUEUES  = 16
};

/**
 * Completion status.
 */
enum {
	IXGBE_COMPLETION_NONE    =  0,
	IXGBE_COMPLETION_OK      =  1,
	IXGBE_COMPLETION_FAILURE = -1,
};

struct ixgbe_handle;

/** 
 * RX or TX flush handler.
 * This is an application provided function that is called
 * by the library when flush is performed.
 * @param udata private data specific by the application
 * @param cdata private data specific by the application
 */
typedef void (*ixgbe_flushcb)(unsigned long udata, unsigned long cdata);

/**
 * Adapter open.
 * Opens IXGBE adapter with a specified name, allocates resources and initializes HW.
 * @param name full PCI id of the adapter that you want 
 * to open (for example 0000:05:09.0). You can use lspci 
 * command to get this information.
 * @param data pointer to uio_dma_area which will be used for data buffers
 * @return adapter handle
 */
struct ixgbe_handle *ixgbe_open(const char *name, int uio_dmafd, struct uio_dma_area *data);

/**
 * Adapter close.
 * This function releases all allocated resources and closes IXGBE adapter.
 * It also triggers completion of all pending TX and RX requests.
 * @param h adapter handle 
 */
void ixgbe_close(struct ixgbe_handle *h);

/**
 * Adapter info.
 * Reads hw info about IXGBE adapter with the specified name.
 * @param name full PCI id of the adapter that you want 
 * to open (for example 0000:05:09.0). You can use lspci 
 * command to get this information.
 * @return readonly adapter handle that must be free with @see ixgbe_free()
 * Returned handle can be used to query things like macaddrs, irq, etc.
 */
struct ixgbe_handle *ixgbe_info(const char *name);

/**
 * Free readonly handle.
 * This function releases readonly handle.
 * @warning Must not be used to close normal handles. @see ixgbe_close().
 * @param h adapter handle 
 */
void ixgbe_free(struct ixgbe_handle *h);

/**
 * Reset.
 * Performs hard reset of the adapter. 
 * @param h adapter handle 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 * @warning does not flush pending transfers
 */
int ixgbe_reset(struct ixgbe_handle *h);

int ixgbe_txq_open(struct ixgbe_handle *h, unsigned int i, unsigned int ntxd);
void ixgbe_txq_close(struct ixgbe_handle *h, unsigned int i);

/**
 * Set flush callback.
 * These callbacks are called whenever TX or RX flush is performed.
 * @param h adapter handle 
 * @param tcb tx flush callback
 * @param tcb rx flush callback
 * @param udata user data (passed to callback)
 */
void ixgbe_txq_setflushcb(struct ixgbe_handle *h, unsigned int i, ixgbe_flushcb cb, unsigned long udata);

/**
 * Enable TX processing.
 * Maps TX ring and data buffer, and initializes TX control registers.
 * @param h adapter handle 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_txq_enable(struct ixgbe_handle *h, unsigned int i);


/**
 * Disable TX processing.
 * @warning Does not flush pending requests. Call ixgbe_flush_tx() explicitly.
 * @param h adapter handle 
 */
void ixgbe_txq_disable(struct ixgbe_handle *h, unsigned int i);

int ixgbe_rxq_open(struct ixgbe_handle *h, unsigned int i, unsigned int nrxd);
void ixgbe_rxq_close(struct ixgbe_handle *h, unsigned int i);
void ixgbe_rxq_setflushcb(struct ixgbe_handle *h, unsigned int i, ixgbe_flushcb cb, unsigned long udata);
int ixgbe_rxq_enable(struct ixgbe_handle *h, unsigned int i);
void ixgbe_rxq_disable(struct ixgbe_handle *h, unsigned int i);

/**
 * Enable RX processing.
 * Maps RX ring and data buffer, and initializes RX control registers.
 * @param h adapter handle 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
void ixgbe_rx_enable(struct ixgbe_handle *h);

/**
 * Disable RX processing.
 * @warning Does not flush pending requests. Call ixgbe_flush_tx() explicitly.
 * @param h adapter handle 
 */
void ixgbe_rx_disable(struct ixgbe_handle *h);

/**
 * Enable interrupts.
 * @param h adapter handle 
 * @param interrupt mask (values are defined in ixgbe_hw.h) 
 */
void ixgbe_enable_interrupts(struct ixgbe_handle *h, uint32_t mask);

/**
 * Disable interrupts.
 * @param h adapter handle 
 * @param interrupt mask (values are defined in ixgbe_hw.h) 
 */
void ixgbe_disable_interrupts(struct ixgbe_handle *h, uint32_t mask);

/**
 * Wait for an interrupt.
 * Waits for interrupt to be triggered by the HW.
 * @param h adapter handle 
 * @param icr set to a value of the interrupt cause register
 * @param timeo timeout in milliseconds (0 -  unlimited)
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_wait4_interrupt(struct ixgbe_handle *h, uint32_t *icr, unsigned long timeo);

/**
 * Trigger an interrupt.
 * @param h adapter handle
 * @param ics mask of interrupts to trigger (see ixgbe_hw.h *ICS*)
 */
void ixgbe_trigger_interrupt(struct ixgbe_handle *h, uint32_t ics);

/**
 * Get RX ring room.
 * @param h adapter handle 
 * @returns number of available RX descriptors
 */
unsigned int ixgbe_rxq_room(struct ixgbe_handle *h, unsigned int i);

/**
 * Get TX ring room.
 * @param h adapter handle 
 * @returns number of available TX descriptors
 */
unsigned int ixgbe_txq_room(struct ixgbe_handle *h, unsigned int i);

/**
 * Submit TX request.
 * The message must reside within the data buffer and application is 
 * responsible for providing correct Ethernet header at beginning of
 * the message.
 * @param h adapter handle 
 * @param ptr pointer to the message 
 * @param size size of the message (including Ethernet header) 
 * @param cdata private data passed to the completion handler 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_txq_submit(struct ixgbe_handle *h, unsigned int i, void *ptr, 
	size_t len, unsigned long cdata);

/**
 * Submit TX request using iovec.
 * Just like ixgbe_send() but uses iovec (gather send).
 * @param h adapter handle 
 * @param vec pointer iovec. All chunks must reside within the data buffer.
 * @param nvec number of iovecs 
 * @param cdata private data passed to the completion handler 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_txq_submitv(struct ixgbe_handle *h, unsigned int i, struct iovec *vec, 
	unsigned int nvec, unsigned long cdata);

/**
 * Check for and complete next pending TX request.
 * Use this function to check for and complete pending TX requests. For example after
 * ixgbe_wait4_interrupt() indicates that TX has completed.
 * @param h adapter handle 
 * @param cdata filled with cdata of the completed frame on return
 * @return completion status (OK, NONE, etc)
 */
int ixgbe_txq_complete(struct ixgbe_handle *h, unsigned int i, unsigned long *cdata);

/**
 * Check for pending TX completions.
 * Use this function to check for pending TX completions.
 * @param h adapter handle 
 * @param i queue index
 * @return 0 if there is nothing pending, >0 otherwise.
 */
int ixgbe_txq_poll(struct ixgbe_handle *h, unsigned int i);

/**
 * Submit RX request.
 * The message must reside somewhere in the data buffer. Allowed message buffer 
 * size depends on the MTU. Use ixgbe_get_rxbuflen() to query required buffer size.
 * @param h adapter handle 
 * @param ptr pointer to the message buffer
 * @param size size of the message buffer (including Ethernet header). 
 * @param cdata private data passed to the completion handler 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_rxq_submit(struct ixgbe_handle *h, unsigned int i, void *ptr,
	size_t size, unsigned long cdata);

/**
 * Flush pending TX requests.
 * @param h adapter handle 
 * @param i queue index
 * @param udata user data passed to callback
 */
void ixgbe_txq_flush(struct ixgbe_handle *h, unsigned int i);

/**
 * Check for and complete next pending RX request.
 * Use this function to check for and complete pending RX requests. For example 
 * after ixgbe_wait4_interrupt() indicates that RX has completed.
 * @param h adapter handle 
 * @param i queue index
 * @param cdata filled with cdata of the completed frame on return
 * @return length or 0 if there is nothing pending. 
 */
ssize_t ixgbe_rxq_complete(struct ixgbe_handle *h, unsigned int i, unsigned long *cdata);

/**
 * Check for pending RX completions.
 * Use this function to check for pending RX completions.
 * @param h adapter handle 
 * @param i queue index
 * @return 0 if there is nothing pending, >0 otherwise.
 */
int ixgbe_rxq_poll(struct ixgbe_handle *h, unsigned int i);

/**
 * Flush pending RX requests.
 * @param h adapter handle 
 * @param i queue index
 * @param udata user data passed to callback
 */
void ixgbe_rxq_flush(struct ixgbe_handle *h, unsigned int i);

/**
 * Ethernet link information.
 * @warning flowctl field is deprecated
 */
struct ixgbe_link {
	uint16_t autoneg;
	uint16_t speed;
	uint16_t duplex;
	uint16_t advertising;
	uint16_t flowctl;
};

/**
 * Check for link status.
 * Check must be called periodically while link is down because it
 * performs some critical HW management functions.
 * It's recommended to have something like a separate low priority control
 * thread that periodically calls this function.  
 * @param h adapter handle 
 * @param link filled in with link status information
 * @param flush if set to non-zero indicates that RX/TX flush must be
 * performed due to link status change.
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_check_link(struct ixgbe_handle *h, struct ixgbe_link *link, int *flush);

/**
 * Set link configuration (speed, duplex, etc).
 * @param h adapter handle 
 * @param link link options
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_set_link(struct ixgbe_handle *h, struct ixgbe_link *link);

/**
 * Get link configuration (speed, duplex, etc).
 * @param h adapter handle 
 * @param link filed in with current link options
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_get_link(struct ixgbe_handle *h, struct ixgbe_link *link);

/**
 * Flow control settings.
 */
struct ixgbe_flowctl {
	uint8_t  rx_xoff;   /** Receiving of xoff pause frames is enabled */
	uint8_t  tx_xoff;   /** Sending of xoff pause frames is enabled */
	uint8_t  tx_xon;    /** Sending of xon pause frames is enabled */
	uint16_t low_water;  /** Receive low water mark (triggers XON) */ 
	uint16_t high_water; /** Receive high water mark (triggers XOFF) */
	uint16_t xoff_time;  /** Pause time used in xoff frames sent by us */
};

/**
 * Set flow control settings.
 * @param h adapter handle 
 * @param fc new flow control settings.
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_set_flowctl(struct ixgbe_handle *h, struct ixgbe_flowctl *fc);

/**
 * Get flow control settings.
 * @param h adapter handle 
 * @param fc filed in with current flow control settings.
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_get_flowctl(struct ixgbe_handle *h, struct ixgbe_flowctl *fc);

#define IXGBE_PROMISC_OFF (0 << 0)
#define IXGBE_PROMISC_UC  (1 << 0)
#define IXGBE_PROMISC_MC  (1 << 1)
/**
 * Set promisc mode.
 * @param h adapter handle 
 * @param mode (IXGBE_PROMISC_OFF, IXGBE_PROMISC_UC, IXGBE_PROMISC_MC)
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_set_promisc(struct ixgbe_handle *h, uint8_t mode);

#define IXGBE_BROADCAST_OFF 0
#define IXGBE_BROADCAST_ON  1
/**
 * Set broadcast mode.
 * @param h adapter handle 
 * @param mode (IXGBE_BROADCAST_OFF, IXGBE_BROADCAST_ON)
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_set_broadcast(struct ixgbe_handle *h, uint8_t mode);

/**
 * Receive Address filter.
 */
struct ixgbe_rafilter {
	uint8_t  addr[ETH_ALEN];
	uint16_t qi;
	uint16_t flags;
};
/**
 * Set Receive Address filter.
 * @param h adapter handle 
 * @param count number of entries
 * @param flt pointer to filter entries
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_set_rafilter(struct ixgbe_handle *h, unsigned int count, struct ixgbe_rafilter *flt);

/**
 *  Set RX address register.
 *  Puts an ethernet address into a receive address register.
 *  This function provides direct access to the RAR register. Consider using 
 *  @see ixgbe_set_rafilter instead.
 *  @param h adapter handle
 *  @param index receive address register to write
 *  @param addr address to put into receive address register
 *  @param qi queue index
 *  @param active set flag that address is active
 */
int ixgbe_set_rar(struct ixgbe_handle *h, unsigned int index, uint8_t *addr,
		uint32_t qi, uint8_t active);

#define IXGBE_RXBUFFER_2048  2048
#define IXGBE_RXBUFFER_4096  4096
#define IXGBE_RXBUFFER_8192  8192
#define IXGBE_RXBUFFER_16384 16384

/**
 * Set MTU.
 * @param h adapter handle 
 * @param mtu MTU value 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_set_mtu(struct ixgbe_handle *h, unsigned int mtu);

/**
 * Get required length of the RX buffers.
 * @return length of the RX buffers.
 */
unsigned int ixgbe_rxbuflen(struct ixgbe_handle *h);

/** 
 * Link statistics.
 * This is similar to what you normally get from ifconfig (ie net_device_stats)
 */
struct ixgbe_stats
{
        unsigned long   rx_packets;
        unsigned long   rx_multicast;
        unsigned long   tx_packets;
        unsigned long   rx_bytes;
        unsigned long   tx_bytes;
        unsigned long   rx_errors;
        unsigned long   tx_errors;
        unsigned long   rx_dropped;
        unsigned long   tx_dropped;
        unsigned long   collisions;

        unsigned long   rx_length_errors;
        unsigned long   rx_over_errors;
        unsigned long   rx_crc_errors;
        unsigned long   rx_frame_errors;
        unsigned long   rx_fifo_errors;
        unsigned long   rx_missed_errors;

        unsigned long   tx_aborted_errors;
        unsigned long   tx_carrier_errors;
        unsigned long   tx_fifo_errors;
        unsigned long   tx_heartbeat_errors;
        unsigned long   tx_window_errors;
};
 
/**
 * Reset stat counters.
 * @param h adapter handle 
 */
void ixgbe_reset_stats(struct ixgbe_handle *h);

/**
 * Get HW stats.
 * @param h adapter handle 
 * @param stats filled in with HW stats
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_get_hwstats(struct ixgbe_handle *h, struct ixgbe_hw_stats *stats);

/**
 * Get general link stats.
 * @param h adapter handle 
 * @param stats filled up with link stats
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_get_stats(struct ixgbe_handle *h, struct ixgbe_stats *stats);

#define IXGBE_DEFAULT_ITR  0
/**
 * Throttle interrupts.
 * @param h adapter handle 
 * @param itr throttle rate (interrupts per second) 
 * @return 0 success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_throttle_intr(struct ixgbe_handle *h, unsigned int itr);

/**
 * Get pollable file descriptor.
 * Returned fd can be used in poll() or select().
 * Usable poll() events are POLLIN, POLLERR.
 * Tipically application would want to call ixgbe_wait4_interrupt() when
 * poll() returns a success.
 * @param h adapter handle 
 * @return fd number on success, -1 otherwise (errno is set to a valid error code).
 */
int ixgbe_get_pollfd(struct ixgbe_handle *h);

/**
 * Get MAC address.
 * @param h adapter handle 
 * @param addr set to current MAC address on return
 */
void ixgbe_get_macaddr(struct ixgbe_handle *h, uint8_t *addr);

/**
 * Get IRQ number.
 * @param h adapter handle 
 */
uint32_t ixgbe_get_irq(struct ixgbe_handle *h);

// Deprecated interfaces
int ixgbe_txq_send(struct ixgbe_handle *h, unsigned int i, void *ptr, size_t len, unsigned long cdata);
int ixgbe_txq_sendv(struct ixgbe_handle *h, unsigned int i, struct iovec *vec, 
	unsigned int nvec, unsigned long cdata);
int ixgbe_rxq_recv(struct ixgbe_handle *h, unsigned int i, void *ptr, size_t size, unsigned long cdata);

#ifdef __cplusplus
}
#endif

#endif /* IXGBE_LIB_H */
