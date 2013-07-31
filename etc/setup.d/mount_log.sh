#!/bin/sh

log=`readlink $UXVCOS_LOG || echo "$UXVCOS_LOG"`
if mountpoint -q $log; then return 0; fi
mount $log
