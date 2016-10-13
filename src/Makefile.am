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
noinst_HEADERS += drivers/ppv2/pp2_cma.h
noinst_HEADERS += drivers/ppv2/mv_pp2x_hw_type.h
noinst_HEADERS += drivers/ppv2/pp2_print.h
#noinst_HEADERS += drivers/ppv2/pp2_locks.h

libmusdk_la_SOURCES  = lib/list.c
libmusdk_la_SOURCES += lib/misc.c
libmusdk_la_SOURCES += lib/of/of.c
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

libmusdk_la_SOURCES += sys/sys_spinlock.c
libmusdk_la_SOURCES += sys/sys_dma.c
libmusdk_la_SOURCES += sys/mem_mng.c

libmusdk_la_SOURCES += drivers/ppv2/pp2.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_bm.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_dm.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_port.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_mem.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_cma.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_gop.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_gop_dbg.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_qos.c
libmusdk_la_SOURCES += drivers/ppv2/bpool.c
libmusdk_la_SOURCES += drivers/ppv2/hif.c
libmusdk_la_SOURCES += drivers/ppv2/ppio.c
