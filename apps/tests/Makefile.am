# whatever flags you want to pass to the C compiler & linker
AM_CFLAGS = -Wall -std=gnu99 -O2
AM_CFLAGS += -I$(top_srcdir)/src/include
AM_CFLAGS += -I$(top_srcdir)/apps/include

AM_LDFLAGS = -lm -lpthread -lrt

METASOURCES = AUTO

EXTRA_DIST =

bin_PROGRAMS = musdk_dma_mem
musdk_dma_mem_SOURCES  = dma_mem.c
musdk_dma_mem_LDADD = $(top_builddir)/src/libmusdk.la

