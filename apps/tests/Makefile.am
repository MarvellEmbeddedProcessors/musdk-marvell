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

bin_PROGRAMS += musdk_dmax2_dma
musdk_dmax2_dma_SOURCES  = dmax2_dma_test.c
musdk_dmax2_dma_SOURCES  += ../common/lib/perf_mon_emu.c
musdk_dmax2_dma_LDADD = $(top_builddir)/src/libmusdk.la

if PP2_BUILD
bin_PROGRAMS += musdk_cls_demo
musdk_cls_demo_CFLAGS = $(AM_CFLAGS)
musdk_cls_demo_SOURCES  = ../common/lib/cli.c
musdk_cls_demo_SOURCES += ../common/mvapp.c
musdk_cls_demo_SOURCES += ../common/utils.c
musdk_cls_demo_SOURCES += ../common/pp2_utils.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_main.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_filter.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_qos.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_tables.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_plcr.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_logical_port.c
musdk_cls_demo_SOURCES += ppv2/cls_demo/cls_debug.c
musdk_cls_demo_LDADD = $(top_builddir)/src/libmusdk.la
endif

if SAM_BUILD
bin_PROGRAMS += musdk_sam_kat
musdk_sam_kat_CFLAGS = $(AM_CFLAGS)
musdk_sam_kat_SOURCES  = sam_kat_suite/kat_tests.c
musdk_sam_kat_SOURCES += sam_kat_suite/common.c
musdk_sam_kat_SOURCES += sam_kat_suite/fileSets.c
musdk_sam_kat_SOURCES += sam_kat_suite/array.c
musdk_sam_kat_SOURCES += sam_kat_suite/generic_list.c
musdk_sam_kat_SOURCES += sam_kat_suite/encryptedBlock.c
musdk_sam_kat_SOURCES += ../common/sam_utils.c

musdk_sam_kat_LDADD = $(top_builddir)/src/libmusdk.la

bin_SCRIPTS = sam_kat_suite/3des_tests.txt sam_kat_suite/aes_tests.txt
bin_SCRIPTS += sam_kat_suite/aes_chain_test.txt sam_kat_suite/aes_gcm.txt
bin_SCRIPTS += sam_kat_suite/aes_ctr.txt sam_kat_suite/aes_gmac.txt
bin_SCRIPTS += sam_kat_suite/hash_only.txt
bin_SCRIPTS += sam_kat_suite/hmac_tests.txt sam_kat_suite/hmac_tests_large.txt
bin_SCRIPTS += sam_kat_suite/aes_sha1_multi.txt sam_kat_suite/aes_sha1_1440_multi.txt


bin_PROGRAMS += musdk_sam_single
musdk_sam_single_CFLAGS = $(AM_CFLAGS)
musdk_sam_single_SOURCES  = sam_kat_single/kat_tests_single.c
musdk_sam_single_LDADD = $(top_builddir)/src/libmusdk.la

bin_PROGRAMS += musdk_sam_ipsec
musdk_sam_ipsec_CFLAGS = $(AM_CFLAGS)
musdk_sam_ipsec_SOURCES  = sam_kat_single/kat_ipsec_single.c
musdk_sam_ipsec_SOURCES += ../common/sam_utils.c
musdk_sam_ipsec_LDADD = $(top_builddir)/src/libmusdk.la
endif
