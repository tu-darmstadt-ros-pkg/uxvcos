#!/bin/sh

if [ -n "$BASH_SOURCE" ]; then
  UXVCOS_ROOT=$(cd `dirname $BASH_SOURCE`/../../../..; pwd)
fi

if [ -d "$UXVCOS_ROOT/core/uxvcos" ]; then
  UXVCOS_ROOT=$UXVCOS_ROOT 
elif [ -d "$ROS_WORKSPACE/core/uxvcos" ]; then
  UXVCOS_ROOT=$ROS_WORKSPACE
else
  echo "WARNING: Could not locate uxvcos in the file system. Falling back to the default /opt/uxvcos." >/dev/stderr
  UXVCOS_ROOT=/opt/uxvcos
fi

export UXVCOS_ROOT
export UXVCOS_LOG=$UXVCOS_ROOT/log

if [ -d $UXVCOS_ROOT/etc/setup.d ]; then
  for setup_file in $UXVCOS_ROOT/etc/setup.d/*; do
    . $setup_file
  done
fi
