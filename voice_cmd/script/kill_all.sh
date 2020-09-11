#!/bin/sh -e
ps aux | grep 'catkin_ws/src/create_autonomy/ca_gazebo/worlds/empty.world'| grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/opt/ros/melodic/lib'| grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/opt/ros/melodic/bin' | grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep 'catkin_ws/devel/lib/picotts/picotts.exe'  | grep -v grep | awk '{ print "kill -9", $2 }' | sh &

