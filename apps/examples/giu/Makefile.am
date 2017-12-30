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

if GIU_BUILD
bin_PROGRAMS = musdk_giu_pkt_echo
musdk_giu_pkt_echo_SOURCES  = ../../common/lib/cli.c
musdk_giu_pkt_echo_SOURCES += ../../common/mvapp.c
musdk_giu_pkt_echo_SOURCES += ../../common/utils.c
musdk_giu_pkt_echo_SOURCES += ../../common/pp2_utils.c
musdk_giu_pkt_echo_SOURCES += ../../common/nmp_guest_utils.c
musdk_giu_pkt_echo_SOURCES += ../../common/giu_utils.c
musdk_giu_pkt_echo_SOURCES += pkt_echo/pkt_echo.c
musdk_giu_pkt_echo_LDADD = $(top_builddir)/src/libmusdk.la

bin_SCRIPTS = pkt_echo/nmp-config

endif
