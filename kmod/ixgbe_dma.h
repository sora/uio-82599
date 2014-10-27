int ixgbe_dma_iobase(struct uio_ixgbe_udapter *ud);
int ixgbe_dma_malloc(struct uio_ixgbe_udapter *ud, struct uio_ixgbe_malloc_req *req);
int ixgbe_dma_mfree(struct uio_ixgbe_udapter *ud, unsigned long mmap_offset);
void ixgbe_dma_mfree_all(struct uio_ixgbe_udapter *ud);
struct ixgbe_dma_area *ixgbe_dma_area_lookup(struct uio_ixgbe_udapter *ud, uint64_t offset);

enum {
	IXGBE_DMA_CACHE_DEFAULT = 0,
	IXGBE_DMA_CACHE_DISABLE,
	IXGBE_DMA_CACHE_WRITECOMBINE
};

struct ixgbe_dma_area {
	struct list_head	list;
	atomic_t		refcount;
	unsigned long		mmap_offset;
	unsigned long		size;
	uint8_t			cache;
	void			*vaddr;
	dma_addr_t		paddr;
};
