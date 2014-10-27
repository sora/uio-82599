struct ixgbe_handle {
 	int             fd;
	void		*bar;
	uint32_t	bar_size;

	struct ixgbe_ring tx_ring[IXGBE_MAX_TX_QUEUES];
	struct ixgbe_ring rx_ring[IXGBE_MAX_RX_QUEUES];

	unsigned int    rx_buflen;
	unsigned int    mtu;

	struct ixgbe_hw_stats stats;
	struct uio_ixgbe_info info;
};

/* Ioctl defines */

/* MAC and PHY info */
struct uio_ixgbe_info {
        uint32_t  irq;
        uint64_t  mmio_base;
        uint32_t  mmio_size;

	uint16_t  mac_type;
	uint8_t   mac_addr[ETH_ALEN];
	uint16_t  phy_type;

	uint32_t num_rx_queues;
	uint32_t num_tx_queues;
};

#define UIO_IXGBE_INFO       _IOW('E', 201, int)
struct uio_ixgbe_info_req {
	struct uio_ixgbe_info info;
};

#define UIO_IXGBE_UP       _IOW('E', 202, int)
struct uio_ixgbe_up_req {
	struct uio_ixgbe_info info;
};

struct ixgbe_handle *ixgbe_open();
void ixgbe_close(struct ixgbe_handle *h);
