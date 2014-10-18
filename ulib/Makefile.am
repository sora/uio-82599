incdir          = $(includedir)/uio-ixgbe
inc_HEADERS     = ixgbe-hw.h ixgbe-lib.h ixgbe-defs.h ixgbe-uio.h
noinst_HEADERS  = ixgbe-priv.h

lib_LTLIBRARIES = libuio-ixgbe.la

INCLUDES = -I$(top_srcdir)/include $(UIODMA_CFLAGS)
libuio_ixgbe_la_SOURCES = init.c link.c flowctl.c multi.c stats.c rxtx.c 
libuio_ixgbe_la_LDFLAGS = -version-info 0:2
