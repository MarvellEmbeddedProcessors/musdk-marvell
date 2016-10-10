AM_CPPFLAGS  = -I./ -I$(LOCATION)/usr/local/include 
AM_CPPFLAGS += -I$(top_srcdir)/src/include
AM_CPPFLAGS += -I$(top_srcdir)/src/include/drivers
AM_CPPFLAGS += -I$(builddir)

METASOURCES = AUTO

EXTRA_DIST =

AM_CFLAGS = -g3 -lpthread
AM_CXXFLAGS = -g3 -lpthread

bin_PROGRAMS = musdk_dma_mem
musdk_dma_mem_SOURCES  = tests/dma_mem.c
musdk_dma_mem_LDADD = $(top_builddir)/src/drivers/ppv2/libmusdk.la
musdk_dma_mem_LDADD += $(LOCATION)/usr/local/lib/libuio.la

