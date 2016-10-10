AM_CPPFLAGS  = -I./ -I$(LOCATION)/usr/local/include 
#AM_CPPFLAGS += -I$(top_build_prefix)src/include
AM_CPPFLAGS += -I$(top_srcdir)/src/include
AM_CPPFLAGS += -I$(builddir)



METASOURCES = AUTO

EXTRA_DIST =
EXTRA_DIST +=		\
	libtool-version

AM_CFLAGS = -g3
AM_CXXFLAGS = -g3

noinst_DATA = libtool-version

lib_LTLIBRARIES = libmusdk.la

include_HEADERS  = pp2_bm.h
include_HEADERS += pp2_port.h
include_HEADERS += pp2_dm.h
include_HEADERS += pp2_plat.h

noinst_HEADERS  = pp2_util.h
noinst_HEADERS += pp2.h
noinst_HEADERS += pp2_mem.h
noinst_HEADERS += pp2_gop.h
noinst_HEADERS += pp2_cma.h
noinst_HEADERS += mv_pp2x_hw_type.h
noinst_HEADERS += pp2_print.h
#noinst_HEADERS += pp2_locks.h

libmusdk_la_SOURCES  = pp2.c
libmusdk_la_SOURCES += pp2_bm.c
libmusdk_la_SOURCES += pp2_dm.c
libmusdk_la_SOURCES += pp2_port.c
libmusdk_la_SOURCES += pp2_mem.c
libmusdk_la_SOURCES += pp2_cma.c
libmusdk_la_SOURCES += pp2_gop.c
libmusdk_la_SOURCES += pp2_gop_dbg.c
libmusdk_la_SOURCES += pp2_qos.c
libmusdk_la_SOURCES += bpool.c
libmusdk_la_SOURCES += hif.c
libmusdk_la_SOURCES += ppio.c
libmusdk_la_SOURCES += ../../sys/sys_spinlock.c
libmusdk_la_SOURCES += ../../sys/sys_dma.c
libmusdk_la_SOURCES += ../../sys/mem_mng.c

