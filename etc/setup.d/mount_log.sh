#!/bin/sh

if [ -e /dev/disk/by-label/log ]; then
  log=`readlink $UXVCOS_LOG || echo "$UXVCOS_LOG"`
  if mountpoint -q $log; then return 0; fi
  sudo mount /dev/disk/by-label/log $log -o noatime
fi
