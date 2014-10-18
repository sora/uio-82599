noinst_PROGRAMS = api rx tx # shared

INCLUDES = -I $(top_srcdir)/include $(UIODMA_CFLAGS)
LDADD    = $(top_builddir)/ulib/libuio-ixgbe.la $(UIODMA_LIBS)

MISC = misc.c misc.h

api_SOURCES = api.c $(MISC)
rx_SOURCES  = rx.c $(MISC)
tx_SOURCES  = tx.c $(MISC)
#shared_SOURCES = shared.c $(MISC)
