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

bin_PROGRAMS = musdk_pkt_echo
musdk_pkt_echo_SOURCES  = ../common/lib/cli.c
musdk_pkt_echo_SOURCES += ../common/mvapp.c
musdk_pkt_echo_SOURCES += ../common/utils.c
musdk_pkt_echo_SOURCES += ppv2/pkt_echo/pkt_echo.c
musdk_pkt_echo_LDADD = $(top_builddir)/src/libmusdk.la

if SAM_BUILD
bin_PROGRAMS += musdk_crypto_echo
musdk_crypto_echo_SOURCES  = ../common/lib/cli.c
musdk_crypto_echo_SOURCES += ../common/lib/perf_mon_emu.c
musdk_crypto_echo_SOURCES += ../common/mvapp.c
musdk_crypto_echo_SOURCES += ../common/utils.c
musdk_crypto_echo_SOURCES += ppv2/crypto_echo/crypto_echo.c
musdk_crypto_echo_LDADD = $(top_builddir)/src/libmusdk.la
endif

bin_PROGRAMS += musdk_cls_demo
musdk_cls_demo_CFLAGS = $(AM_CFLAGS)
musdk_cls_demo_SOURCES  = ../common/lib/cli.c
musdk_cls_demo_SOURCES += ../common/mvapp.c
musdk_cls_demo_SOURCES += ../common/utils.c
musdk_cls_demo_SOURCES += ppv2/cls_demo.c
musdk_cls_demo_SOURCES += ppv2/cls_debug.c
musdk_cls_demo_LDADD = $(top_builddir)/src/libmusdk.la
