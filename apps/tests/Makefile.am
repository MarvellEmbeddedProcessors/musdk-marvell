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

musdk_sam_kat_SOURCES += ../common/crypto/mv_md5.c
musdk_sam_kat_SOURCES += ../common/crypto/mv_sha1.c
musdk_sam_kat_SOURCES += ../common/crypto/mv_sha2.c
musdk_sam_kat_SOURCES += ../common/crypto/mv_aes.c

musdk_sam_kat_LDADD = $(top_builddir)/src/libmusdk.la

bin_SCRIPTS = sam_kat_suite/3des_tests.txt sam_kat_suite/aes_tests.txt
bin_SCRIPTS += sam_kat_suite/aes_chain_test.txt sam_kat_suite/aes_gcm.txt
bin_SCRIPTS += sam_kat_suite/hmac_tests.txt
bin_SCRIPTS += sam_kat_suite/aes_sha1_multi.txt sam_kat_suite/aes_1440_multi.txt


bin_PROGRAMS += musdk_sam_single
musdk_sam_single_CFLAGS = $(AM_CFLAGS)
musdk_sam_single_SOURCES  = sam_kat_single/kat_tests_single.c

musdk_sam_single_LDADD = $(top_builddir)/src/libmusdk.la
endif
