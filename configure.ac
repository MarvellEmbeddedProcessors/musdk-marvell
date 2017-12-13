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
# Set PP2_BUILD - using --enable-pp2
##########################################################################
AC_ARG_ENABLE([pp2],
[  --enable-pp2         Enable pp2 driver and applications build],
[case "${enableval}" in
  yes) pp2=true ;;
  no)  pp2=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-pp2]) ;;
esac],[pp2=true])
AM_CONDITIONAL([PP2_BUILD], [test x$pp2 = xtrue])
if test x$pp2 = xtrue; then
##########################################################################
# Set pp2-lock - using --enable-pp2-lock
##########################################################################
AC_ARG_ENABLE([pp2-lock],
[  --enable-pp2-lock    Enable musdk pp2 driver HIF/TX locking],
[case "${enableval}" in
  yes) pp2_lock=true ;;
  no)  pp2_lock=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-pp2-lock]) ;;
esac],[pp2_lock=false])
if test x$pp2_lock = xtrue; then
	PP2_CFLAGS+="-DMVCONF_PP2_LOCK "
fi
if test x$pp2_lock = xtrue; then
##########################################################################
# Set pp2-lock-stat - using --enable-pp2-lock-stat
##########################################################################
AC_ARG_ENABLE([pp2-lock-stat],
[  --enable-pp2-lock-stat    Enable musdk pp2 driver HIF/TX locking stats],
[case "${enableval}" in
  yes) pp2_lock_stat=true ;;
  no)  pp2_lock_stat=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-pp2-lock-stat]) ;;
esac],[pp2_lock_stat=false])
if test x$pp2_lock_stat = xtrue; then
	PP2_CFLAGS+="-DMVCONF_PP2_LOCK_STAT "
fi
fi
fi
##########################################################################
# Set NETA_BUILD - using --enable-neta
##########################################################################
AC_ARG_ENABLE([neta],
[  --enable-neta         Enable neta driver and applications build],
[case "${enableval}" in
  yes) neta=true ;;
  no)  neta=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-neta]) ;;
esac],[neta=false])
AM_CONDITIONAL([NETA_BUILD], [test x$neta = xtrue])

##########################################################################
# Set NMP_BUILD - using --enable-nmp
##########################################################################
AC_ARG_ENABLE([nmp],
[  --enable-nmp         Enable nmp build],
[case "${enableval}" in
  yes) nmp=true ;;
  no)  nmp=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-nmp]) ;;
esac],[nmp=false])
AM_CONDITIONAL([NMP_BUILD], [test x$nmp = xtrue])

##########################################################################
# Set GIU_BUILD - using --enable-giu
##########################################################################
AC_ARG_ENABLE([giu],
[  --enable-giu         Enable giu driver and applications build],
[case "${enableval}" in
  yes) giu=true ;;
  no)  giu=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-giu]) ;;
esac],[giu=false])
AM_CONDITIONAL([GIU_BUILD], [test x$giu = xtrue])
#enable NMP builf if GIU build was enabled
AM_CONDITIONAL([NMP_BUILD], [test x$giu = xtrue])

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
# Set DEBUG - using --enable-debug
##########################################################################
AC_ARG_ENABLE([debug],
[  --enable-debug    Enable musdk debug],
[case "${enableval}" in
  yes) DEBUG=true ;;
  no)  DEBUG=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-debug]) ;;
esac],[DEBUG=false])
if test x$DEBUG = xtrue; then
	MUSDK_CFLAGS+="-DDEBUG "
fi

##########################################################################
# Set MUSDK debug level
##########################################################################
MUSDK_DEBUG=6
AC_ARG_ENABLE([debug-level],
[  --enable-debug-level    Set musdk debug-level: Range 0-7],
[if test $enableval -ge 0 -a $enableval -le 7; then
	MUSDK_DEBUG=$enableval
fi])
MUSDK_CFLAGS+="-DMVCONF_DBG_LEVEL=$MUSDK_DEBUG "
##########################################################################
# Enable MUSDK log facility
##########################################################################
AC_ARG_ENABLE([log],
[  --enable-log         Enable syslog utility],
[case "${enableval}" in
  yes) log=true ;;
  no)  log=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-log]) ;;
esac],[log=false])
if test x$log = xtrue; then
	MUSDK_CFLAGS+="-DMVCONF_SYSLOG "
fi
AM_CONDITIONAL([LOG_BUILD], [test x$log = xtrue])
##########################################################################
# Enable MUSDK trave facility
##########################################################################
AC_ARG_ENABLE([trace],
[  --enable-trace         Enable lttng trace utility],
[case "${enableval}" in
  yes) trace=true ;;
  no)  trace=false ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-trace]) ;;
esac],[trace=false])
AM_CONDITIONAL([TRACE_BUILD], [test x$trace = xtrue])
##########################################################################
# Set DMA_ADDR_SIZE
##########################################################################
DMA_ADDR_SIZE=64
AC_ARG_ENABLE([dma-addr],
[  --enable-dma-addr    Set dma_addr size: 32/64],
[if test $enableval -eq 32 -o $enableval -eq 64; then
	DMA_ADDR_SIZE=$enableval
fi])
MUSDK_CFLAGS+="-DMVCONF_DMA_PHYS_ADDR_T_SIZE=$DMA_ADDR_SIZE "
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

# Add MUSDK types flags. They are always defined in musdk_lib compilation,
# and are optional during user compilation.
MUSDK_CFLAGS+="-DMVCONF_TYPES_PUBLIC "
MUSDK_CFLAGS+="-DMVCONF_DMA_PHYS_ADDR_T_PUBLIC "


# Checks for programs.
AC_PROG_CC


# Checks for header files.
AC_CHECK_HEADERS([stdlib.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.

m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

CFLAGS="$CFLAGS -Wall "
#CFLAGS+="-Werror "
CFLAGS+="-Wstrict-prototypes "
CFLAGS+="-Wmissing-prototypes "
CFLAGS+="-pthread "
CFLAGS+="$MUSDK_CFLAGS $SAM_CFLAGS $PP2_CFLAGS"
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
		apps/examples/ppv2/Makefile
		apps/examples/neta/Makefile
		apps/examples/giu/Makefile
])
# Auto-created code files
AC_CONFIG_FILES([src/include/env/mv_autogen_build_assert.h])
AC_CONFIG_FILES([src/include/env/mv_autogen_comp_flags.h])

# Variables used by auto-created code files.
# Should not be distributed among Makefiles.
AC_SUBST([DMA_ADDR_SIZE])
AM_SUBST_NOTMAKE([DMA_ADDR_SIZE])

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
