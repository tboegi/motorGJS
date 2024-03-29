#!/bin/sh

#
# Shell script to generate (and run) an "st.cmd" for an IOC
#
APPXX=gjs
TOP=$(echo $PWD/.. | sed -e "s%/test/\.\.$%%")
export APPXX
DOLOG=
HOST=""
MOTORPORT=""

## functions
generate_st_cmd_classic() {
  for src in  ../../iocsh/*iocsh ../../test/startup/*iocsh; do
    dst=${src##*/}
    #echo sed PWD=$PWD src=$src dst=$dst
    sed <"$src" >"$dst" \
        -e "s%XXdbLoadRecords(\"%XXdbLoadRecords(\"./db/%" \
        -e "s%< %< ${TOP}/iocBoot/ioc${APPXX}/%"    \
        -e "s!/c/Users/!c:/Users/!" \
        -e "s%adsAsynPortDriverConfigure%#adsAsynPortDriverConfigure%"
  done &&
    rm -f $stcmddst &&
    cat >$stcmddst <<-EOF &&
#!../../gjsExApp/src/O.$EPICS_HOST_ARCH/gjs
#This file is autogenerated by run-gjs-ioc.sh - do not edit
epicsEnvSet("ARCH","$EPICS_HOST_ARCH")
epicsEnvSet("IOC","ioc${APPXX}")
epicsEnvSet("TOP","$TOP")
epicsEnvSet("EPICS_BASE","$EPICS_BASE")

cd ${TOP}
dbLoadDatabase "./dbd/gjs.dbd"
${APPXX}_registerRecordDeviceDriver pdbbase
EOF
  # Side note: st.${MOTORCFG}.iocsh needs extra patching
  #echo sed PWD=$PWD "<../../startup/st.${MOTORCFG}.iocsh >>$stcmddst"
  sed <../../test/startup/st.${MOTORCFG}.iocsh  \
      -e "s/__EPICS_HOST_ARCH/$EPICS_HOST_ARCH/" \
      -e "s%cfgFile=./%cfgFile=./test/startup/%"    \
      -e "s%< %< ${TOP}/iocBoot/ioc${APPXX}/%"    \
      -e "s%require%#require%" \
      -e "s!/c/Users/!c:/Users/!" \
    | grep -v '^  *#' >>$stcmddst &&
    cat >>$stcmddst <<-EOF &&
        iocInit
EOF
  postprocess_stcmddst &&
  # Create an xx file to be used under gdb
  chmod +x $stcmddst && egrep -v "^ *#" $stcmddst >xx
}

postprocess_stcmddst() {
  # Post-process of stcmddst
  if test -n "$HOST" ; then
    sed < $stcmddst \
        -e "s/172\.[0-9]*\.[0-9]*.[0-9]*/$MOTORIP/" \
        -e "s/127.0.0.1/$MOTORIP/" >/tmp/$$ &&
      mv -f /tmp/$$ $stcmddst
  fi
  if test -n "$MOTORPORT" ; then
    sed < $stcmddst -e "s/5000/$MOTORPORT/" >/tmp/$$ &&
      mv -f /tmp/$$ $stcmddst
  fi
  chmod +x $stcmddst
}


uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)

INSTALLED_EPICS=../../../../.epics.$(hostname).$uname_s.$uname_m

IOCDIR_CLASSIC=../iocBoot/ioc${APPXX} &&
mkdir -p  $IOCDIR_CLASSIC/ &&
if test -z "$EPICS_HOST_ARCH"; then
  RELEASELOCAL=../configure/RELEASE.local
  if test -r "$RELEASELOCAL"; then
    # Code stolen from .ci/travis/prepare.sh
    eval $(grep "EPICS_BASE=" $RELEASELOCAL)
    export EPICS_BASE
    echo "EPICS_BASE=$EPICS_BASE"
    if test -z "$EPICS_BASE"; then
      echo >&2 "EPICS_BASE" is not set
      exit 1
    fi
    [ -z "$EPICS_HOST_ARCH" -a -f $EPICS_BASE/src/tools/EpicsHostArch.pl ] && EPICS_HOST_ARCH=$(perl $EPICS_BASE/src/tools/EpicsHostArch.pl)
    [ -z "$EPICS_HOST_ARCH" -a -f $EPICS_BASE/startup/EpicsHostArch.pl ] && EPICS_HOST_ARCH=$(perl $EPICS_BASE/startup/EpicsHostArch.pl)
    export EPICS_HOST_ARCH
    echo "EPICS_HOST_ARCH=$EPICS_HOST_ARCH"
  fi
fi

if test -z "$EPICS_HOST_ARCH"; then
  echo >&2 "EPICS_HOST_ARCH" is not set
  exit 1
fi

if test "$1" = "--no-make"; then
  NOMAKE=y
  shift
fi
export NOMAKE
if test "$1" = "--no-run"; then
  NORUN=y
  shift
fi
export NORUN

MOTORCFG="$1"
export MOTORCFG
echo MOTORCFG=$MOTORCFG
(
  cd startup &&
  if ! test -f st.${MOTORCFG}.iocsh; then
    CMDS=$(echo st.*.iocsh | sed -e "s/st\.//g" -e "s/\.iocsh//g" | sort)
    #echo CMDS=$CMDS
    test -n "$1" && echo >&2 "not found st.${1}.iocsh"
    echo >&2 $0  "[--no-make][--no-run]"
    echo >&2 "try one of these:"
    for cmd in $CMDS; do
      case $cmd in
        *simulator)
          echo >&2 $0 " $cmd"
          ;;
        *)
          echo >&2 $0 " $cmd" " <ip>[:port]"
          ;;
      esac
    done
    exit 1
  fi
) || exit 1

shift

MOTORPORT=5001

if test -n "$1" && test "$1" != "-l"; then
  # allow doit.sh host:port
  PORT=${1##*:}
  HOST=${1%:*}
  echo HOST=$HOST PORT=$PORT
  if test "$PORT" != "$HOST"; then
    MOTORPORT=$PORT
  else
    MOTORPORT=5001
  fi
  echo HOST=$HOST MOTORPORT=$MOTORPORT
  MOTORIP=$HOST
  echo MOTORIP=$MOTORIP
  shift
fi
export MOTORIP MOTORPORT

# log/tee to file
if test "$1" = "-l"; then
    if test -n "$SM_PREFIX"; then
        LOG_TXT=log-$(echo $SM_PREFIX | sed -e "s/:$//g" | tr ":" "-" ).txt
    else
        LOG_TXT=log-$MOTORCFG.txt
    fi
    export LOG_TXT
  if test -f $LOG_TXT; then
    timestamp=$(date "+%y-%m-%d-%H.%M.%S")
    mkdir -p ../logs/ &&
    mv $LOG_TXT ../logs/$timestamp-$MOTORCFG.txt || exit 1
  fi
  DOLOG=" 2>&1 | tee $PWD/$LOG_TXT"
  shift
fi
export DOLOG

if test -n "$1"; then
  echo >&2 unsupported additional parameters: $@
  exit 1
fi

if test "$NOMAKE" != "y"; then
(
  if test -x ../checkws.sh; then
  (
    cd .. && ./checkws.sh
  ) || exit
  (cd .. && make install)
  fi
  stcmddst=./st.iocsh.$EPICS_HOST_ARCH &&
  cd $IOCDIR_CLASSIC && generate_st_cmd_classic
) || exit
fi
if test "$NORUN" != "y"; then
  stcmddst=./st.iocsh.$EPICS_HOST_ARCH &&
  cd $IOCDIR_CLASSIC/ &&
  echo PWD=$PWD $stcmddst DOLOG=$DOLOG
  eval $stcmddst $DOLOG
fi
