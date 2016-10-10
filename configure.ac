# use autogen.sh

AC_PREREQ(2.65)
AC_INIT([MUSDK Access Library],
	[0.1],
	[],
	[libmusdk],
	[])

AC_CANONICAL_BUILD
AC_CANONICAL_HOST

AC_CONFIG_SRCDIR([src/drivers/ppv2/pp2.h])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([1.10 dist-bzip2 dist-zip] [subdir-objects])
AM_MAINTAINER_MODE

AC_LANG([C])
AC_REQUIRE_CPP

CFLAGS+=" -pthread \
    -Wall \
    -Wunused-variable \
    -Wunused-function \
    -Wpointer-arith \
    -Wcast-qual \
    -Wcast-align \
    -Wundef \
    -Wwrite-strings \
    -Wstrict-prototypes \
    -Wmissing-prototypes \
    -Wmissing-declarations \
    -Wold-style-definition \
"

AC_PROG_CC_STDC
AM_PROG_AR

LT_PREREQ([2.2])
LT_INIT
AC_PROG_INSTALL

LOCATION=$HOME/target/sdk
AC_SUBST(LOCATION)

AC_CONFIG_FILES([
	Makefile
	src/drivers/ppv2/Makefile
    apps/Makefile
])

AC_OUTPUT
