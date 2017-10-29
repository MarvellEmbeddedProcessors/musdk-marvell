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

bin_PROGRAMS = musdk_pp2_pkt_echo
musdk_pp2_pkt_echo_SOURCES  = ../../common/lib/cli.c
musdk_pp2_pkt_echo_SOURCES += ../../common/mvapp.c
musdk_pp2_pkt_echo_SOURCES += ../../common/utils.c
musdk_pp2_pkt_echo_SOURCES += ../../common/pp2_utils.c
musdk_pp2_pkt_echo_SOURCES += pkt_echo/pkt_echo.c
musdk_pp2_pkt_echo_LDADD = $(top_builddir)/src/libmusdk.la

if SAM_BUILD
bin_PROGRAMS += musdk_pp2_crypto_echo
musdk_pp2_crypto_echo_SOURCES  = ../../common/lib/cli.c
musdk_pp2_crypto_echo_SOURCES += ../../common/lib/perf_mon_emu.c
musdk_pp2_crypto_echo_SOURCES += ../../common/lib/mv_stack.c
musdk_pp2_crypto_echo_SOURCES += ../../common/mvapp.c
musdk_pp2_crypto_echo_SOURCES += ../../common/utils.c
musdk_pp2_crypto_echo_SOURCES += ../../common/pp2_utils.c
musdk_pp2_crypto_echo_SOURCES += ../../common/sam_utils.c
musdk_pp2_crypto_echo_SOURCES += crypto_echo/crypto_echo.c

musdk_pp2_crypto_echo_LDADD = $(top_builddir)/src/libmusdk.la
endif

bin_PROGRAMS += musdk_cls_example
musdk_cls_example_CFLAGS = $(AM_CFLAGS)
musdk_cls_example_SOURCES  = ../../common/lib/cli.c
musdk_cls_example_SOURCES += ../../common/mvapp.c
musdk_cls_example_SOURCES += ../../common/utils.c
musdk_cls_example_SOURCES += ../../common/pp2_utils.c
musdk_cls_example_SOURCES += cls_example/cls_main_example.c
musdk_cls_example_SOURCES += cls_example/cls_filter_example.c
musdk_cls_example_SOURCES += cls_example/cls_qos_example.c
musdk_cls_example_SOURCES += cls_example/cls_tables_example.c
musdk_cls_example_SOURCES += cls_example/cls_logical_port_example.c
musdk_cls_example_SOURCES += cls_example/cls_policer_example.c
musdk_cls_example_SOURCES += cls_example/cls_txsched_example.c
musdk_cls_example_SOURCES += ../../tests/ppv2/cls/cls_debug.c
musdk_cls_example_LDADD = $(top_builddir)/src/libmusdk.la

bin_PROGRAMS += musdk_pkt_l3fwd
musdk_pkt_l3fwd_SOURCES  = ../../common/lib/cli.c
musdk_pkt_l3fwd_SOURCES += ../../common/mvapp.c
musdk_pkt_l3fwd_SOURCES += ../../common/utils.c
musdk_pkt_l3fwd_SOURCES += ../../common/pp2_utils.c
musdk_pkt_l3fwd_SOURCES += ../../common/lib/xxhash.c
musdk_pkt_l3fwd_SOURCES += ../../common/lib//ezxml.c
musdk_pkt_l3fwd_SOURCES += pkt_l3fwd/l3fwd_main.c
musdk_pkt_l3fwd_SOURCES += pkt_l3fwd/l3fwd_db.c
musdk_pkt_l3fwd_SOURCES += pkt_l3fwd/l3fwd_lpm.c
musdk_pkt_l3fwd_LDADD = $(top_builddir)/src/libmusdk.la

bin_PROGRAMS += musdk_pkt_gen
musdk_pkt_gen_SOURCES  = ../../common/lib/cli.c
musdk_pkt_gen_SOURCES += ../../common/mvapp.c
musdk_pkt_gen_SOURCES += ../../common/utils.c
musdk_pkt_gen_SOURCES += ../../common/pp2_utils.c
musdk_pkt_gen_SOURCES += ../../common/lib/xxhash.c
musdk_pkt_gen_SOURCES += ../../common/lib//ezxml.c
musdk_pkt_gen_SOURCES += pkt_gen/pkt_gen.c
musdk_pkt_gen_LDADD = $(top_builddir)/src/libmusdk.la
