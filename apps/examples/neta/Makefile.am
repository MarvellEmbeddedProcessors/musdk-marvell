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

bin_PROGRAMS = musdk_neta_pkt_echo
musdk_neta_pkt_echo_SOURCES  = ../../common/lib/cli.c
musdk_neta_pkt_echo_SOURCES += ../../common/mvapp.c
musdk_neta_pkt_echo_SOURCES += ../../common/utils.c
musdk_neta_pkt_echo_SOURCES += neta_utils.c
musdk_neta_pkt_echo_SOURCES += pkt_echo/pkt_echo.c
musdk_neta_pkt_echo_LDADD = $(top_builddir)/src/libmusdk.la

if SAM_BUILD
bin_PROGRAMS += musdk_neta_crypto_echo
musdk_neta_crypto_echo_SOURCES  = ../../common/lib/cli.c
musdk_neta_crypto_echo_SOURCES += ../../common/lib/perf_mon_emu.c
musdk_neta_crypto_echo_SOURCES += ../../common/lib/mv_stack.c
musdk_neta_crypto_echo_SOURCES += ../../common/mvapp.c
musdk_neta_crypto_echo_SOURCES += ../../common/utils.c
musdk_neta_crypto_echo_SOURCES += ../../common/sam_utils.c
musdk_neta_crypto_echo_SOURCES += neta_utils.c
musdk_neta_crypto_echo_SOURCES += crypto_echo/crypto_echo.c

musdk_neta_crypto_echo_LDADD = $(top_builddir)/src/libmusdk.la
endif
