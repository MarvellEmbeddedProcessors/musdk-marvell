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
bin_PROGRAMS = musdk_nmp_standalone
musdk_nmp_standalone_SOURCES  = ../../common/lib/cli.c
musdk_nmp_standalone_SOURCES += ../../common/mvapp.c
musdk_nmp_standalone_SOURCES += ../../common/utils.c
musdk_nmp_standalone_SOURCES += nmp_standalone/nmp_standalone.c
musdk_nmp_standalone_LDADD = $(top_builddir)/src/libmusdk.la

bin_SCRIPTS = configs/musdk-nmp-config.txt
bin_SCRIPTS += configs/odp-nmp-config.txt

endif
