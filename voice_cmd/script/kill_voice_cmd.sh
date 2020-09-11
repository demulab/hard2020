#!/bin/sh -e
ps aux | grep 'python /home/demulab/catkin_ws/src/hard2020/voice_cmd/src/voice_cmd.py'  | grep -v grep | awk '{ print "kill -9", $2 }' | sh &
