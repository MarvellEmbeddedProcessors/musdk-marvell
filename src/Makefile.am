# whatever flags you want to pass to the C compiler & linker
CL_CFLAGS = -O2

AM_CFLAGS = $(CL_CFLAGS)
AM_CFLAGS += -Wall -std=gnu99
AM_CFLAGS += -I$(top_srcdir)/src/include

AM_LDFLAGS = -lm

lib_LTLIBRARIES = libmusdk.la

nobase_include_HEADERS  = include/mv_std.h
nobase_include_HEADERS += include/env/mv_common.h
nobase_include_HEADERS += include/env/mv_compiler.h
nobase_include_HEADERS += include/env/mv_debug.h
nobase_include_HEADERS += include/env/mv_errno.h
nobase_include_HEADERS += include/env/mv_sys_dma.h
nobase_include_HEADERS += include/env/mv_types.h
nobase_include_HEADERS += include/env/mv_autogen_build_assert.h
nobase_include_HEADERS += include/env/mv_autogen_comp_flags.h

libmusdk_la_CFLAGS = $(AM_CFLAGS)
libmusdk_la_LDFLAGS = $(AM_LDFLAGS)

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
libmusdk_la_SOURCES += env/hugepage_mem.c
libmusdk_la_SOURCES += env/sys_dma.c
libmusdk_la_SOURCES += env/of.c
libmusdk_la_SOURCES += env/netdev.c
libmusdk_la_SOURCES += env/sys_iomem.c

if LOG_BUILD
libmusdk_la_SOURCES += env/log.c
endif

if TRACE_BUILD
libmusdk_la_SOURCES += env/trace.c
endif

if PP2_BUILD
nobase_include_HEADERS += include/drivers/mv_net.h
nobase_include_HEADERS += include/drivers/mv_pp2.h
nobase_include_HEADERS += include/drivers/mv_pp2_bpool.h
nobase_include_HEADERS += include/drivers/mv_pp2_cls.h
nobase_include_HEADERS += include/drivers/mv_pp2_hif.h
nobase_include_HEADERS += include/drivers/mv_pp2_ppio.h

libmusdk_la_SOURCES += drivers/ppv2/pp2.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_bm.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_dm.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_port.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_port_us.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_mem.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_gop.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_gop_dbg.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_bpool.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_hif.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_ppio.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_cls.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_utils_us.c
libmusdk_la_SOURCES += drivers/ppv2/pp2_txsched.c

libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_cls_common.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_cls_utils.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_cls_db.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_prs.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_plcr.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_flow_rules.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_c3.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_cls_mng.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_flow_rules_debug.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_c3_debug.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_c2.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_c2_debug.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_rss.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_hw_cls.c
libmusdk_la_SOURCES += drivers/ppv2/cls/pp2_hw_cls_dbg.c
endif

if NETA_BUILD
libmusdk_la_SOURCES += drivers/neta/neta.c
libmusdk_la_SOURCES += drivers/neta/neta_ppio.c
libmusdk_la_SOURCES += drivers/neta/neta_hw.c
libmusdk_la_SOURCES += drivers/neta/neta_bm.c
endif

if GIU_BUILD
nobase_include_HEADERS += include/drivers/mv_giu_gpio.h
nobase_include_HEADERS += include/drivers/mv_giu_bpool.h
nobase_include_HEADERS += include/drivers/mv_mqa.h
nobase_include_HEADERS += include/drivers/mv_mqa_queue.h
nobase_include_HEADERS += include/mng/mv_nmp.h
nobase_include_HEADERS += include/mng/mv_nmp_dispatch.h

libmusdk_la_SOURCES += drivers/giu/giu.c
libmusdk_la_SOURCES += drivers/giu/giu_gpio.c
libmusdk_la_SOURCES += drivers/giu/giu_bpool.c
libmusdk_la_SOURCES += drivers/giu/giu_queue_topology.c
libmusdk_la_SOURCES += drivers/giu/file_map.c

libmusdk_la_SOURCES += drivers/mqa/mqa.c
libmusdk_la_SOURCES += drivers/mqa/mqa_queue.c

libmusdk_la_SOURCES += hw_emul/gie/gie.c

libmusdk_la_SOURCES += mng/lf/pf/pf.c
libmusdk_la_SOURCES += mng/lf/pf/pf_profile.c
libmusdk_la_SOURCES += mng/lf/pf/pf_regfile.c
libmusdk_la_SOURCES += mng/lf/pf/pf_topology.c
libmusdk_la_SOURCES += mng/db.c
libmusdk_la_SOURCES += mng/dev_mng.c
libmusdk_la_SOURCES += mng/dispatch.c
libmusdk_la_SOURCES += mng/nmp.c

endif

if SAM_BUILD
# Definitions for SAM driver compilation
nobase_include_HEADERS += include/drivers/mv_sam.h
nobase_include_HEADERS += include/drivers/mv_sam_cio.h
nobase_include_HEADERS += include/drivers/mv_sam_session.h

AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/DriverFramework/Basic_Defs_API/incl
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/Log/incl
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/Log/src/printf
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/SABuilder/incl
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/SABuilder/build
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/TokenBuilder/incl
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/TokenBuilder/build
AM_CFLAGS += -I$(top_srcdir)/src/drivers/sam/safexcel/Firmware/api

libmusdk_la_SOURCES += drivers/sam/sam.c
libmusdk_la_SOURCES += drivers/sam/sam_hw.c
libmusdk_la_SOURCES += drivers/sam/sam_ipsec.c
libmusdk_la_SOURCES += drivers/sam/sam_debug.c

libmusdk_la_SOURCES += drivers/sam/crypto/mv_md5.c
libmusdk_la_SOURCES += drivers/sam/crypto/mv_sha1.c
libmusdk_la_SOURCES += drivers/sam/crypto/mv_sha2.c
libmusdk_la_SOURCES += drivers/sam/crypto/mv_aes.c

libmusdk_la_SOURCES += drivers/sam/safexcel/SABuilder/src/sa_builder.c
libmusdk_la_SOURCES += drivers/sam/safexcel/SABuilder/src/sa_builder_basic.c
libmusdk_la_SOURCES += drivers/sam/safexcel/SABuilder/src/sa_builder_ipsec.c
libmusdk_la_SOURCES += drivers/sam/safexcel/SABuilder/src/sa_builder_extended_ipsec.c
libmusdk_la_SOURCES += drivers/sam/safexcel/TokenBuilder/src/token_builder_core.c
libmusdk_la_SOURCES += drivers/sam/safexcel/TokenBuilder/src/token_builder_context.c

endif

nobase_include_HEADERS += include/drivers/mv_dmax2.h

libmusdk_la_SOURCES += drivers/dmax2/dmax2.c
libmusdk_la_SOURCES += drivers/dmax2/dmax2_mem.c
