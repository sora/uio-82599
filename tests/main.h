struct ixgbe_handle {
 	int             fd;

	struct ixgbe_ring tx_ring[IXGBE_MAX_TX_QUEUES];
	struct ixgbe_ring rx_ring[IXGBE_MAX_RX_QUEUES];

	unsigned int    rx_buflen;
	unsigned int    mtu;

	struct ixgbe_hw_stats stats;
	struct uio_ixgbe_info info;
};

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

	uint32_t num_rx_queues;
	uint32_t num_tx_queues;
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
