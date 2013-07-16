#!/bin/bash

cmdline=("$@")
if [ $# == 0 ]; then
    cmdline=($SHELL -i)
fi

UXVCOS_ROOT=$(cd `dirname $0`; pwd)
if [ ! -f $UXVCOS_ROOT/setup.sh ]; then
  echo "env.sh: Could not find root path of uxvcos" >/dev/stderr
  return
fi

echo "Using UXVCOS_ROOT at $UXVCOS_ROOT" >/dev/stderr
. $UXVCOS_ROOT/setup.sh
exec "${cmdline[@]}"
