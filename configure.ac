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

AC_SUBST(includedir, '${exec_prefix}')

##########################################################################
# Set SAM_BUILD - using --enable-sam
##########################################################################
AC_ARG_ENABLE([sam],
[  --enable-sam            Enable musdk sam driver and application build],
[case "${enableval}" in
  yes) sam=true ;;
  no)  sam=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-sam]) ;;
esac],[sam=false])
AM_CONDITIONAL([SAM_BUILD], [test x$sam = xtrue])

if test x$sam = xtrue; then
##########################################################################
# Set MVCONF_SAM_STATS - using --enable-sam-statistics
##########################################################################
AC_ARG_ENABLE([sam-statistics],
[  --enable-sam-statistics    Enable musdk sam driver statistics],
[case "${enableval}" in
  yes) SAM_STATS=true ;;
  no)  SAM_STATS=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-sam-statistics]) ;;
esac],[SAM_STATS=false])
if test x$SAM_STATS = xtrue; then
	SAM_CFLAGS+="-DMVCONF_SAM_STATS "
fi

##########################################################################
# Set MVCONF_SAM_DEBUG - using --enable-sam-debug
##########################################################################
AC_ARG_ENABLE([sam-debug],
[  --enable-sam-debug    Enable musdk sam driver debug],
[case "${enableval}" in
  yes) SAM_DEBUG=true ;;
  no)  SAM_DEBUG=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-sam-debug]) ;;
esac],[SAM_DEBUG=false])
if test x$SAM_DEBUG = xtrue; then
	SAM_CFLAGS+="-DMVCONF_SAM_DEBUG "
fi
fi

##########################################################################
# Set MUSDK debug level
##########################################################################
MUSDK_DEBUG=4
AC_ARG_ENABLE([debug-level],
[  --enable-debug-level    Set musdk debug-level: Range 1-5],
[if test $enableval -ge 1 -a $enableval -le 9; then
	MUSDK_DEBUG=$enableval
fi])
MUSDK_CFLAGS+="-DMVCONF_DBG_LEVEL=$MUSDK_DEBUG "
##########################################################################
# Set BPOOL_COOKIE_SIZE
##########################################################################
BPOOL_COOKI_SZ=32
AC_ARG_ENABLE([bpool-cookie],
[  --enable-bpool-cookie    Set bpool cookie size: 32/64],
[if test $enableval -eq 32 -o $enableval -eq 64; then
	BPOOL_COOKI_SZ=$enableval
fi])
MUSDK_CFLAGS+="-DCONF_PP2_BPOOL_COOKIE_SIZE=$BPOOL_COOKI_SZ "
##########################################################################
# Set BPOOL_DMA_ADDR_USE_32B
##########################################################################
BPOOL_DMA_ADDR_USE_32B=32
AC_ARG_ENABLE([bpool-dma],
[  --enable-bpool-dma    Set bpool dma_addr size: 32/64],
[if test $enableval -eq 32 -o $enableval -eq 64; then
	BPOOL_DMA_ADDR_USE_32B=$enableval
fi])
if test $BPOOL_DMA_ADDR_USE_32B -eq 32; then
	MUSDK_CFLAGS+="-DCONF_PP2_BPOOL_DMA_ADDR_USE_32B "
fi
##########################################################################
# Set DMA_ADDR_SIZE
##########################################################################
DMA_ADDR_SIZE=64
AC_ARG_ENABLE([dma-addr],
[  --enable-dma-addr    Set dma_addr size: 32/64],
[if test $enableval -eq 32 -o $enableval -eq 64; then
	DMA_ADDR_SIZE=$enableval
fi])
if test $DMA_ADDR_SIZE -eq 64; then
	MUSDK_CFLAGS+="-DMVCONF_ARCH_DMA_ADDR_T_64BIT "
fi
##########################################################################
# Set MVCONF_SYS_DMA_UIO
##########################################################################
UIO_CMA=true
AC_ARG_ENABLE([uio-cma],
[  --enable-uio-cma     Enable dma_memory by uio-cma],
[case "${enableval}" in
  yes) UIO_CMA=true ;;
  no)  UIO_CMA=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-dmamem-uio-cma]) ;;
esac])
if test x$UIO_CMA = xtrue; then
	MUSDK_CFLAGS+="-DMVCONF_SYS_DMA_UIO "
fi
##########################################################################
# Set MVCONF_SYS_DMA_HUGE_PAGE
##########################################################################
HUGE_PG=false
AC_ARG_ENABLE([uio-hugepage],
[  --enable-uio-hugepage     Enable hugepages for dma_memory uio-cma],
[case "${enableval}" in
  yes) HUGE_PG=true ;;
  no)  HUGE_PG=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-uio-hugepage]) ;;
esac])
if test x$HUGE_PG = xtrue; then
	MUSDK_CFLAGS+="-DMVCONF_SYS_DMA_HUGE_PAGE "
fi

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
CFLAGS+="$MUSDK_CFLAGS $SAM_CFLAGS"
LDFLAGS+="$SAM_LDFLAGS"
LIBS+="$SAM_LIBS"
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
