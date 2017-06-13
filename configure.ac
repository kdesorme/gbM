dnl autoconf script for gbM
dnl
dnl Copyright (c) 2001-2007,2010,2013 CNRS/LAAS
dnl
dnl GPL, since some parts were copied from other configure.in
dnl 
AC_INIT(gbM,0.11)
AC_PREREQ(2.59)
AC_CONFIG_AUX_DIR([m4])
AC_CONFIG_MACRO_DIR([m4])

m4_include(swigAc.m4)

AC_CANONICAL_HOST

AC_PROG_CC

PACKAGE=gbM
VERSION=0.11
AC_SUBST(PACKAGE)
AC_SUBST(VERSION)
AC_REVISION($LAAS$)

dnl configure libtool
AC_DISABLE_STATIC
AC_LIBTOOL_DLOPEN
AC_PROG_LIBTOOL
AC_PROG_INSTALL

dnl check for LAAS mkdep
AC_PATH_PROG(MKDEP, mkdep, no, $prefix/bin:$exec_prefix/bin:$PATH)


dnl --- Embedded display facility -----------------------------------
dnl
AC_ARG_ENABLE(gbtcl,
   AC_HELP_STRING([--disable-gbtcl], [disable compilation of the gb tcl module]),
   [
      if test "x$enableval" != "xno" -a "x$enableval" != "x"; then
         GB_TCL="yes"
      else
         GB_TCL="no"
      fi
   ], [GB_TCL=yes])
AC_SUBST(GB_TCL)



dnl----------------------------------------------------------------------
dnl
dnl Tcl/Tk Config
dnl 
AC_ARG_WITH(tcl,
    [  --with-tcl=DIR          directory containing tclConfig.sh],
    [tcl_prefix=$withval],
    [for ac_dir in \
	${exec_prefix}/lib 	\
	/usr/local/lib/tcl8.4	\
	/usr/local/${usrlibdir}	\
	/usr/pkg/lib		\
	/usr/lib/tcl8.4		\
	/usr/${usrlibdir}	\
	; \
    do
	if test -r "$ac_dir/tclConfig.sh"; then
	  tcl_prefix=$ac_dir
	  break
	fi
    done])
if test "$GB_TCL" = "yes"; then
file=${tcl_prefix}/tclConfig.sh
. $file
fi

AC_ARG_WITH(tk,
    [  --with-tk=DIR           directory containing tkConfig.sh],
    [tk_prefix=$withval],
    [for ac_dir in \
	${tcl_prefix}			\
	${exec_prefix}/lib 		\
	/usr/local/lib/tk$TCL_VERSION	\
	/usr/local/${usrlibdir}		\
	/usr/pkg/lib			\
	/usr/lib/tk$TCL_VERSION		\
	/usr/${usrlibdir}		\
	; \
    do
	if test -r "$ac_dir/tkConfig.sh"; then
	  tk_prefix=$ac_dir
	  break
	fi
    done])

if test "$GB_TCL" = "yes"; then
file=${tk_prefix}/tkConfig.sh
. $file
fi

AC_SUBST(TCL_INCLUDE_SPEC)
AC_SUBST(TK_INCLUDE_SPEC)
AC_SUBST(TCL_LIB_SPEC)
AC_SUBST(TK_LIB_SPEC)


dnl  swig
dnl include acincludeswig.m4
AC_PROG_SWIG

AC_OUTPUT(Makefile 
	gbM.pc
	Init.make
	tcl/Makefile
	tcl/mkIndex
	tclsrc/Makefile
	src/Makefile
)


