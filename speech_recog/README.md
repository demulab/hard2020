speech_recog19

概要:
kobukiに取り付けたrespeaker,realsense2から、人の認識、声のする方の認識、言っている内容を理解し、返事をする

準備：

1.ros_openvino_toolkitをインストール
https://github.com/intel/ros_openvino_toolkit

2.realsense sdkをインストール
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

3.Turtlebot2のパッケージをインストール
https://demura.net/athome/15966.html

4.Web Speech APIをインストール
https://demura.net/athome/15941.html

5.英語音声合成 (Test To Speech)パッケージをインストール
https://demura.net/athome/15954.html

6.ロボット自身が話している時に聞き取らないようにするファイルをダウンロード
https://github.com/demulab/chrome_control

7.入っているcommonというフォルダををsrcの中にうつす

使い方:
1.$ roslaunch turtlebot_bringup minimal.launch

2.$ ./catkin_ws/src/common/executions/exe.bash




