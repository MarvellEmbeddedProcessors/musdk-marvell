# whatever flags you want to pass to the C compiler & linker
AM_CFLAGS = -Wall -std=gnu99 -g -O2
AM_CFLAGS += -I$(top_srcdir)/src/include

AM_LDFLAGS = -lm

lib_LTLIBRARIES = libmusdk.la

include_HEADERS  = include/drivers/mv_pp2.h

libmusdk_la_CFLAGS = $(AM_CFLAGS)

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

# Definitions for SAM driver compilation
libmusdk_la_CFLAGS += -DDRIVER_EIP197_HW_CONF_B
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/Examples/Driver/build/
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/Examples/Driver/build/MVEBU
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/Integration/SLAD_Adapter/incl
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/Kit/DriverFramework/Basic_Defs_API/incl
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/Kit/SABuilder/incl
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/Kit/TokenBuilder/incl
libmusdk_la_CFLAGS += -I$(EIP197_DDK)/../SafeXcel-Firmware/eip197/firmware/production_firmware/api

include_HEADERS  += include/drivers/mv_sam.h
include_HEADERS  += include/drivers/mv_sam_cio.h
include_HEADERS  += include/drivers/mv_sam_session.h

libmusdk_la_SOURCES += drivers/sam/sam.c
