# whatever flags you want to pass to the C compiler & linker
AM_CFLAGS = -Wall -std=gnu99 -O2
#AM_CFLAGS += -I$(AC_PREFIX_DEFAULT)/include
AM_CFLAGS += -I$(top_srcdir)/src/include
AM_CFLAGS += -I$(top_srcdir)/src/include/drivers
AM_CFLAGS += -I$(top_srcdir)/apps/include

AM_LDFLAGS = -lm -lpthread -lrt

METASOURCES = AUTO

EXTRA_DIST =

bin_PROGRAMS = musdk_pkt_echo
musdk_pkt_echo_SOURCES  = ../common/lib/cli.c
musdk_pkt_echo_SOURCES  += ../common/mvapp.c
musdk_pkt_echo_SOURCES  += ppv2/pkt-echo/pkt_echo.c
musdk_pkt_echo_LDADD = $(top_builddir)/src/libmusdk.la
