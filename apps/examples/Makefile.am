# whatever flags you want to pass to the C compiler & linker
CL_CFLAGS = -O2

AM_CFLAGS = $(CL_CFLAGS)
AM_CFLAGS += -Wall -std=gnu99
#AM_CFLAGS += -I$(AC_PREFIX_DEFAULT)/include
AM_CFLAGS += -I$(top_srcdir)/src/include
AM_CFLAGS += -I$(top_srcdir)/src/include/drivers
AM_CFLAGS += -I$(top_srcdir)/apps/include

AM_LDFLAGS = -lm -lpthread -lrt

METASOURCES = AUTO

EXTRA_DIST =

bin_PROGRAMS =
# Link all programs in this directory with libmusdk.a
LDADD = $(top_builddir)/src/libmusdk.la

if PP2_BUILD
bin_PROGRAMS += musdk_pp2_pkt_echo
musdk_pp2_pkt_echo_SOURCES  = ../common/lib/cli.c
musdk_pp2_pkt_echo_SOURCES += ../common/mvapp.c
musdk_pp2_pkt_echo_SOURCES += ../common/utils.c
musdk_pp2_pkt_echo_SOURCES += ../common/pp2_utils.c
musdk_pp2_pkt_echo_SOURCES += ppv2/pkt_echo/pkt_echo.c

bin_PROGRAMS += musdk_cls_example
musdk_cls_example_CFLAGS = $(AM_CFLAGS)
musdk_cls_example_SOURCES  = ../common/lib/cli.c
musdk_cls_example_SOURCES += ../common/mvapp.c
musdk_cls_example_SOURCES += ../common/utils.c
musdk_cls_example_SOURCES += ../common/pp2_utils.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_main_example.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_filter_example.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_qos_example.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_tables_example.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_logical_port_example.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_policer_example.c
musdk_cls_example_SOURCES += ppv2/cls_example/cls_txsched_example.c
musdk_cls_example_SOURCES += ../tests/ppv2/cls/cls_debug.c

bin_PROGRAMS += musdk_pkt_l3fwd
musdk_pkt_l3fwd_SOURCES  = ../common/lib/cli.c
musdk_pkt_l3fwd_SOURCES += ../common/mvapp.c
musdk_pkt_l3fwd_SOURCES += ../common/utils.c
musdk_pkt_l3fwd_SOURCES += ../common/pp2_utils.c
musdk_pkt_l3fwd_SOURCES += ../common/lib/xxhash.c
musdk_pkt_l3fwd_SOURCES += ../common/lib//ezxml.c
musdk_pkt_l3fwd_SOURCES += ppv2/pkt_l3fwd/l3fwd_main.c
musdk_pkt_l3fwd_SOURCES += ppv2/pkt_l3fwd/l3fwd_db.c
musdk_pkt_l3fwd_SOURCES += ppv2/pkt_l3fwd/l3fwd_lpm.c

bin_PROGRAMS += musdk_pkt_gen
musdk_pkt_gen_SOURCES  = ../common/lib/cli.c
musdk_pkt_gen_SOURCES += ../common/mvapp.c
musdk_pkt_gen_SOURCES += ../common/utils.c
musdk_pkt_gen_SOURCES += ../common/pp2_utils.c
musdk_pkt_gen_SOURCES += ../common/lib/xxhash.c
musdk_pkt_gen_SOURCES += ../common/lib//ezxml.c
musdk_pkt_gen_SOURCES += ppv2/pkt_gen/pkt_gen.c

bin_PROGRAMS += musdk_pp2_dma_echo
musdk_pp2_dma_echo_SOURCES  = ../common/lib/cli.c
musdk_pp2_dma_echo_SOURCES += ../common/lib/mv_stack.c
musdk_pp2_dma_echo_SOURCES += ../common/mvapp.c
musdk_pp2_dma_echo_SOURCES += ../common/utils.c
musdk_pp2_dma_echo_SOURCES += ../common/pp2_utils.c
musdk_pp2_dma_echo_SOURCES += ppv2/dma_echo/dma_echo.c

if SAM_BUILD
bin_PROGRAMS += musdk_pp2_crypto_echo
musdk_pp2_crypto_echo_SOURCES  = ../common/lib/cli.c
musdk_pp2_crypto_echo_SOURCES += ../common/lib/mv_stack.c
musdk_pp2_crypto_echo_SOURCES += ../common/mvapp.c
musdk_pp2_crypto_echo_SOURCES += ../common/utils.c
musdk_pp2_crypto_echo_SOURCES += ../common/pp2_utils.c
musdk_pp2_crypto_echo_SOURCES += ../common/sam_utils.c
musdk_pp2_crypto_echo_SOURCES += ppv2/crypto_echo/crypto_echo.c
endif

endif

if GIU_BUILD
bin_PROGRAMS += musdk_giu_pkt_echo
musdk_giu_pkt_echo_SOURCES  = ../common/lib/cli.c
musdk_giu_pkt_echo_SOURCES += ../common/mvapp.c
musdk_giu_pkt_echo_SOURCES += ../common/utils.c
musdk_giu_pkt_echo_SOURCES += ../common/pp2_utils.c
musdk_giu_pkt_echo_SOURCES += ../common/nmp_guest_utils.c
musdk_giu_pkt_echo_SOURCES += ../common/giu_utils.c
musdk_giu_pkt_echo_SOURCES += giu/pkt_echo/pkt_echo.c
endif

if NMP_BUILD
bin_PROGRAMS += musdk_nmp_standalone
musdk_nmp_standalone_SOURCES  = ../common/lib/cli.c
musdk_nmp_standalone_SOURCES += ../common/mvapp.c
musdk_nmp_standalone_SOURCES += ../common/utils.c
musdk_nmp_standalone_SOURCES += nmp/nmp_standalone/nmp_standalone.c

bin_SCRIPTS = nmp/configs/musdk-nmp-config.txt
bin_SCRIPTS += nmp/configs/odp-nmp-config.txt
endif

if NETA_BUILD
bin_PROGRAMS += musdk_neta_pkt_echo
musdk_neta_pkt_echo_SOURCES  = ../common/lib/cli.c
musdk_neta_pkt_echo_SOURCES += ../common/mvapp.c
musdk_neta_pkt_echo_SOURCES += ../common/utils.c
musdk_neta_pkt_echo_SOURCES += neta/neta_utils.c
musdk_neta_pkt_echo_SOURCES += neta/pkt_echo/pkt_echo.c

if SAM_BUILD
bin_PROGRAMS += musdk_neta_crypto_echo
musdk_neta_crypto_echo_SOURCES  = ../common/lib/cli.c
musdk_neta_crypto_echo_SOURCES += ../common/lib/mv_stack.c
musdk_neta_crypto_echo_SOURCES += ../common/mvapp.c
musdk_neta_crypto_echo_SOURCES += ../common/utils.c
musdk_neta_crypto_echo_SOURCES += ../common/sam_utils.c
musdk_neta_crypto_echo_SOURCES += neta/neta_utils.c
musdk_neta_crypto_echo_SOURCES += neta/crypto_echo/crypto_echo.c
endif
endif
