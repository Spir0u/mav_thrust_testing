#!/bin/bash
trap ' ' INT

STAMP="$(date +"%Y-%m-%d_%H-%M-%S")"
echo "Recording experiment "$1" starting at time $STAMP"
FILENAME="$1/$1_$STAMP"
echo "File: $FILENAME"
mkdir $1
rosparam dump > "$FILENAME.dump"

rosbag record -o "$1/$1.bag" -a

echo "Enter log:"
read log
echo "$log" > "$1/log-$STAMP.txt"
