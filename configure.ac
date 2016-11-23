#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([1.0])
AC_INIT([musdk], [1.0], [someone@marvell.com])
AM_INIT_AUTOMAKE([subdir-objects])

AC_CONFIG_SRCDIR([src])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# no build parameters print
AM_SILENT_RULES([yes])

# set install prefix
AC_PREFIX_DEFAULT($(pwd)/usr/local)

# Checks for programs.
AC_PROG_CC


# Checks for header files.
AC_CHECK_HEADERS([stdlib.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.

m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

CFLAGS="-Wall "
#CFLAGS+="-Werror "
CFLAGS+="-Wstrict-prototypes "
CFLAGS+="-Wmissing-prototypes "
CFLAGS+="-pthread " 

CC=${CROSS_COMPILE}gcc
LD=${CROSS_COMPILE}ld
AR=${CROSS_COMPILE}ar

# define LIBTOOL (for libraries)
LT_INIT

AC_CONFIG_FILES([Makefile
		src/Makefile
		apps/Makefile
		apps/tests/Makefile
		apps/examples/Makefile
])

##########################################################################
# distribute the changed variables among the Makefiles

AC_SUBST([LIBS])
AC_SUBST([AM_CPPFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([CFLAGS])
AC_SUBST([AM_LDFLAGS])
AC_SUBST([LDFLAGS])

AC_OUTPUT
AC_MSG_RESULT([
        libdir:                 ${libdir}
        includedir:             ${includedir}

        cc:                     ${CC}
        cppflags:               ${CPPFLAGS}
        cflags:                 ${CFLAGS}
        ldflags:                ${LDFLAGS}
        libs:                   ${LIBS}
])
