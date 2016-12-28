# whatever flags you want to pass to the C compiler & linker
CL_CFLAGS = -O2

AM_CFLAGS = $(CL_CFLAGS)
AM_CFLAGS += -Wall -std=gnu99
AM_CFLAGS += -I$(top_srcdir)/src/include
AM_CFLAGS += -I$(top_srcdir)/src/include/drivers
AM_CFLAGS += -I$(top_srcdir)/apps/include

AM_LDFLAGS = -lm -lpthread -lrt

METASOURCES = AUTO

EXTRA_DIST =

bin_PROGRAMS = musdk_dma_mem
musdk_dma_mem_SOURCES  = dma_mem.c
musdk_dma_mem_LDADD = $(top_builddir)/src/libmusdk.la

if SAM_BUILD
bin_PROGRAMS += musdk_sam_kat
musdk_sam_kat_CFLAGS = $(AM_CFLAGS)
musdk_sam_kat_SOURCES  = sam_kat_suite/kat_tests.c
musdk_sam_kat_SOURCES += sam_kat_suite/common.c
musdk_sam_kat_SOURCES += sam_kat_suite/fileSets.c
musdk_sam_kat_SOURCES += sam_kat_suite/array.c
musdk_sam_kat_SOURCES += sam_kat_suite/generic_list.c
musdk_sam_kat_SOURCES += sam_kat_suite/encryptedBlock.c

musdk_sam_kat_LDADD = $(top_builddir)/src/libmusdk.la

bin_PROGRAMS += musdk_sam_single
musdk_sam_single_CFLAGS = $(AM_CFLAGS)
musdk_sam_single_SOURCES  = sam_kat_single/kat_tests_single.c

musdk_sam_single_LDADD = $(top_builddir)/src/libmusdk.la
endif

bin_PROGRAMS += musdk_cls_dbg
musdk_cls_dbg_CFLAGS = $(AM_CFLAGS)
musdk_cls_dbg_SOURCES  = ../common/lib/cli.c
musdk_cls_dbg_SOURCES  += ../common/mvapp.c
musdk_cls_dbg_SOURCES  += ../common/utils.c
musdk_cls_dbg_SOURCES  += ppv2/cls_demo.c
musdk_cls_dbg_SOURCES  += ppv2/cls_debug.c
musdk_cls_dbg_LDADD = $(top_builddir)/src/libmusdk.la
