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

include_HEADERS  = include/drivers/mv_pp2.h

noinst_HEADERS  = drivers/ppv2/pp2_util.h
noinst_HEADERS += drivers/ppv2/pp2.h
noinst_HEADERS += drivers/ppv2/pp2_mem.h
noinst_HEADERS += drivers/ppv2/pp2_gop.h
noinst_HEADERS += drivers/ppv2/pp2_hw_type.h
noinst_HEADERS += drivers/ppv2/pp2_print.h
#noinst_HEADERS += drivers/ppv2/pp2_locks.h

libmusdk_la_SOURCES  = lib/list.c
libmusdk_la_SOURCES += lib/lib_misc.c
libmusdk_la_SOURCES += lib/mem_mng.c
libmusdk_la_SOURCES += lib/uio/uio_find_devices.c
libmusdk_la_SOURCES += lib/uio/uio_find_devices_byname.c
libmusdk_la_SOURCES += lib/uio/uio_free.c
libmusdk_la_SOURCES += lib/uio/uio_get_all_info.c
libmusdk_la_SOURCES += lib/uio/uio_get_device_attributes.c
libmusdk_la_SOURCES += lib/uio/uio_get_event_count.c
libmusdk_la_SOURCES += lib/uio/uio_get_mem_addr.c
libmusdk_la_SOURCES += lib/uio/uio_get_mem_size.c
libmusdk_la_SOURCES += lib/uio/uio_get_mem_name.c
libmusdk_la_SOURCES += lib/uio/uio_get_name.c
libmusdk_la_SOURCES += lib/uio/uio_get_version.c
libmusdk_la_SOURCES += lib/uio/uio_line_from_file.c
libmusdk_la_SOURCES += lib/uio/uio_num_from_filename.c
libmusdk_la_SOURCES += lib/uio/uio_single_mmap.c
libmusdk_la_SOURCES += lib/uio/uio_find_mem_byname.c

libmusdk_la_SOURCES += env/spinlock.c
libmusdk_la_SOURCES += env/cma.c
libmusdk_la_SOURCES += env/sys_dma.c
libmusdk_la_SOURCES += env/of.c

libmusdk_la_SOURCES += drivers/ppv2/pp2.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_bm.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_dm.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_port.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_mem.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_gop.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_gop_dbg.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_hw_cls.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_hw_cls_dbg.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_bpool.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_hif.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_ppio.c
