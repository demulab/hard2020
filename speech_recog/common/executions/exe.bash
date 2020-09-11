#!/bin/bash
tab="--tab-with-profile=roslaunch --command "
window="--window-with-profile=roslaunch --command "

cd $HOME/catkin_ws/src/common/bashes
gnome-terminal \
  $window './minami.launch.bash'\
  $window './respeaker.launch.bash'\
  $window './rwt.launch.bash'\
  $window './picotts.bash'\
  $window './spr.bash'\
  $window './stop_start.bash'\
  $window './rostopic.bash'
