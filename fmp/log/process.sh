#!/bin/bash

dir=$1
depth=3

for file in `find -L $dir -mindepth $depth -maxdepth $depth -name '*.bag'`; do
    dir=$(cd `dirname $file`; pwd)
    name=$(basename $file)
    bname=${name%.bag}

    if [[ "$bname" =~ _processed ]]; then
        continue
    fi

    if [ -f $dir/${bname}_processed.bag ]; then
        echo "Skipping $file..."
        continue
    fi
    
    echo \> roslaunch fmp post_processing.launch inbag:=$dir/${bname}.bag outbag:=$dir/${bname}_processed.bag paused:=false
    roslaunch fmp post_processing.launch inbag:=$dir/${bname}.bag outbag:=$dir/${bname}_processed.bag paused:=false

    if [ -f $dir/${bname}_processed.bag.active ]; then
        echo "Reindexing ${bname}_processed.bag.active..."
        rosbag reindex $dir/${bname}_processed.bag.active
        mv $dir/${bname}_processed.bag.active $dir/${bname}_processed.bag
    fi
done

