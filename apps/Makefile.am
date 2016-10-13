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
musdk_dma_mem_LDADD = $(top_builddir)/src/libmusdk.la
musdk_dma_mem_LDADD += $(LOCATION)/usr/local/lib/libmusdk.la

bin_PROGRAMS += musdk_pkt_echo
musdk_pkt_echo_SOURCES  = examples/ppv2/pkt-echo/pkt_echo.c
musdk_pkt_echo_LDADD = $(top_builddir)/src/libmusdk.la
musdk_pkt_echo_LDADD += $(LOCATION)/usr/local/lib/libmusdk.la
