#!/usr/bin/env python
# -*- coding: utf-8 -*- # 日本語を使うためのおまじない。
import rospy  # ROSでpython使う場合に必要
from geometry_msgs.msg import Twist # ロボットの速度を扱う場合は必要
from sensor_msgs.msg import LaserScan # LIDARを使うときは必要
import math   # 数学関数モジュール

# グローバル変数。先頭のアンダーバーはグローバル変数の目印。
# 0.0で初期化している。
#_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0


# callback_lidar関数
# /scanのMessage型はhttp://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.htmlを参照。
def callback_lidar(msg):

    rospy.loginfo("Number of Rays=%d", len(msg.ranges))  # レーザの本数

    # シミュレーションのPRDLIARは全周360[°] 計測可能。計測角度は-180[°]から180[°]。
    # ROSは右手系、進行方向x軸、左方向y軸、上方向がz軸（反時計まわりが正)。
    rospy.loginfo("Angle [rad] min=%f max=%f", msg.angle_min, msg.angle_max) # スキャンの開始/終了角度 [rad]

    # ROSの角度は[rad]。math.degresss関数でラジアンから°に変換している。
    rospy.loginfo("Angle [deg] increment=%.3f", math.degrees(msg.angle_increment)) # 計測間隔 [°] 
    rospy.loginfo("Range [m] min=%.3f max=%.3f", msg.range_min, msg.range_max)   # 最小/最大検出距離 [m]
    
    rospy.loginfo("  0 [deg]=%.3f [m]", msg.ranges[0])
    rospy.loginfo(" 90 [deg]=%.3f [m]", msg.ranges[180])
    rospy.loginfo("180 [deg]=%.3f [m]", msg.ranges[360])
    rospy.loginfo("-90 [deg]=%.3f [m]", msg.ranges[540])


# set_vel関数：速度の設定。
def set_vel(vel_msg, lv, av):
    vel_msg.linear.x  = lv  # 並進速度
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = av  # 角速度
    
    
# main関数
def main():
    #  ノードの初期化。引数はノード名。ROSでは同じノード名のノードを複数作ることはできない。
    rospy.init_node('lidar')

    # サブスクライバー（購読者)の生成。一番目の引数はトピック名、２番目の引数はメッセージの型、
    # オドメトリのメッセージはLaserScan型(正確にはsensor_msgs/LaserScan型だが
    # from sensor_msgs.msg import LaserScanでインポートしているのでここではLaserScanだけでよい)。
    # ３番目の引数はコールバック関数。
    odom_subscriber = rospy.Subscriber('/create1/rplidar/scan', LaserScan, callback_lidar)


    # パブリッシャーの生成。一番目の引数はトピック名、２番目の引数はメッセージの型、
    # 速度指令はTwist型。３番目の引数はメッセージバッファーのサイズ。
    vel_publisher = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)

    
    # ループの周期を設定。100Hz。つまり、1ループ10ms。
    rate = rospy.Rate(100)

    # Ctrl-Cが押されるまで無限ループ
    while not rospy.is_shutdown():
        # メッセージを配信する。つまり、ロボットに速度指令を送る。
        vel_publisher.publish(vel_msg)
        
        # 表示
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        
        # rospy.Rate()で指定した時間になるように調整してくれる。
        rate.sleep()


# このプログラムをモジュールとしてimportできるようにするおまじない。
# なお、モジュールとは他のプログラムから再利用できるようにしたファイルのこと。
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
