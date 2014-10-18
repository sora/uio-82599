sbin_PROGRAMS = ixgbe-info ixgbe-dump ixgbe-pktgen

INCLUDES = -I$(top_srcdir)/include $(UIODMA_CFLAGS)
LDADD    = $(top_builddir)/ulib/libuio-ixgbe.la $(UIODMA_LIBS) -lrt

ixgbe_info_SOURCES   = ixgbe-info.c
ixgbe_dump_SOURCES   = ixgbe-dump.c
ixgbe_pktgen_SOURCES = ixgbe-pktgen.c
