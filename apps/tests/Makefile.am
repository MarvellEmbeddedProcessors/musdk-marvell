# whatever flags you want to pass to the C compiler & linker
AM_CFLAGS = -Wall -std=gnu99 -g -O2
AM_CFLAGS += -I$(top_srcdir)/src/include
AM_CFLAGS += -I$(top_srcdir)/src/include/drivers
AM_CFLAGS += -I$(top_srcdir)/apps/include

AM_LDFLAGS = -lm -lpthread -lrt

METASOURCES = AUTO

EXTRA_DIST =

bin_PROGRAMS = musdk_dma_mem
musdk_dma_mem_SOURCES  = dma_mem.c
musdk_dma_mem_LDADD = $(top_builddir)/src/libmusdk.la

#bin_PROGRAMS += musdk_sam_kat
#musdk_sam_kat_CFLAGS = $(AM_CFLAGS)
#musdk_sam_kat_SOURCES  = sam_kat_suite/kat_tests.c
#musdk_sam_kat_SOURCES += sam_kat_suite/common.c
#musdk_sam_kat_SOURCES += sam_kat_suite/fileSets.c
#musdk_sam_kat_SOURCES += sam_kat_suite/array.c
#musdk_sam_kat_SOURCES += sam_kat_suite/generic_list.c
#musdk_sam_kat_SOURCES += sam_kat_suite/encryptedBlock.c

#musdk_sam_kat_LDADD = $(top_builddir)/src/libmusdk.la
#musdk_sam_kat_LDADD += $(EIP197_DDK)/Examples/Driver/build/libdriver_197_u.a
#musdk_sam_kat_LDADD += $(EIP197_DDK)/Examples/DemoApp_PEC/build/libsa_bld_u.a
#musdk_sam_kat_LDADD += $(EIP197_DDK)/Examples/DemoApp_PEC/build/libtk_bld_u.a

