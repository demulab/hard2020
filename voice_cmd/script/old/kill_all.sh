#!/bin/sh -e
ps aux | grep 'catkin_ws/src/create_autonomy/ca_gazebo/worlds/empty.world'| grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/opt/ros/melodic/lib/rviz'| grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/opt/ros/melodic/bin/roslaunch rwt_speech_recognition' | grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/opt/ros/melodic/bin/roslaunch picotts microsoft.launch' | grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep 'catkin_ws/devel/lib/picotts/picotts.exe'  | grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/opt/ros/melodic/bin/rosrun voice_cmd voice_cmd.py' | grep -v grep | awk '{ print "kill -9", $2 }' | sh &
ps aux | grep '/usr/bin/python /opt/ros/melodic/bin/rosmaster' | grep -v grep | awk '{ print "kill -9", $2 }' | sh &

