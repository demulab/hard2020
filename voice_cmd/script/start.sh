#!/bin/bash
xterm -sb -geometry 80x10+0+0   -e "/opt/ros/melodic/bin/roslaunch ca_gazebo create_empty_world.launch" &
sleep 10s
xterm -sb -geometry 80x10+0+200 -e "/opt/ros/melodic/bin/rosrun picotts picotts.exe" &
sleep 5s
xterm -sb -geometry 80x10+0+400 -e "/opt/ros/melodic/bin/roslaunch rwt_speech_recognition rwt_speech_recognition.launch" &
sleep 3s
xterm -sb -geometry 80x20+0+600 -e "/opt/ros/melodic/bin/rosrun voice_cmd voice_cmd.py" &
#xterm -geometry 80x5+0+200 -e "/opt/ros/melodic/bin/rostopic pub -1 /picotts/engine std_msgs/String "microsoft" " &
#sleep 3s
