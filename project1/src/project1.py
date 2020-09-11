#!/usr/bin/env python
# -*- coding: utf-8 -*- # 日本語を使うためのおまじない。
import rospy  # ROSでpython使う場合に必要
from geometry_msgs.msg import Twist # ロボットの速度を扱う場合は必要
from nav_msgs.msg import Odometry   # オドメトリを使う場合は必要
import tf
from tf.transformations import euler_from_quaternion
from math import degrees

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0

# set_vel関数：速度の設定。
# １番目の引数は速度メッセージ、２番目の引数は並進速度、３番目の引数は角速度
def set_vel(vel_msg, lv, av):
    vel_msg.linear.x  = lv
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = av


# callback_odom関数
def callback_odom(msg):
    global _odom_x, _odom_y, _odon_theta # グローバル変数
    _odom_x = msg.pose.pose.position.x   # x座標 [m]
    _odom_y = msg.pose.pose.position.y   # y座標 [m]
    # クォーターニオン(4次元数) 姿勢を表す手法の一つ
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    # クォータニオンからオイラー角を取得
    e = euler_from_quaternion(q)
    _odom_theta = e[2] # e[0]:ロール角, e[1]：ピッチ角, e[2]：ヨー角 [rad]

# 自作odometry関数を作ろう！
# 距離[m] = 並進速度[m/s]　×　時間[s]。
# 回転角度[rad] = 角速度[rad/s]　×　時間[s]
# ロボットの姿勢を考えて、三角関数を使い距離はx軸、y軸に分ける。
# 速度と計測時間が必要。
def my_odom:
    pass

    
# main関数
def main():
    #  ノードの初期化。引数はノード名。ROSでは同じノード名のノードを複数作ることはできない。
    rospy.init_node('odometry')

    # パブリッシャーの生成。一番目の引数はトピック名、２番目の引数はメッセージの型、
    # 速度指令はTwist型。３番目の引数はメッセージバッファーのサイズ。
    vel_pub  = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)
    odom_sub = rospy.Subscriber('/create1/odom', Odometry, callback_odom)

    # vel_magオフジェクトの生成
    vel_msg = Twist()

    # 速度の初期化。安全のために0に設定。
    set_vel(vel_msg, 0, 0)
    
    # キーボードからの入力
    print("Let's move your robot")
    linear_vel  = input("Input linear velocity [m/s] :")
    angular_vel = input("Input angular velocity [rad/s] :")

    #　並進速度または角速度の設定rospy.is_shutdown():
    #  ROS座標系は右手系、ロボットの前進方向がX軸、右方向がy軸、上方向がz軸
    vel_msg.linear.x  = linear_vel
    vel_msg.angular.z = angular_vel

    # ループの周期を設定。10Hz。つまり、1ループ100ms。
    # ロボットを一定の周期で動かすことはとても重要
    rate = rospy.Rate(10) 
    
    # Ctrl-Cが押されるまで無限ループ
    while not rospy.is_shutdown():
        # メッセージを配信する。つまり、ロボットに速度指令を送る。
        vel_pub.publish(vel_msg)

        # 表示。
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        
        # rospy.Rate()で指定した時間になるように調整してくれる。
        rate.sleep()

# このプログラムをモジュールとしてimportできるようにするおまじない。
# なお、モジュールとは他のプログラムから再利用できるようにしたファイルのこと。
if __name__ == '__main__':
    # 例外処理。rospy.ROSInterruptExceptionを捕まえる。
    # この例外はCrl+cキーが押されるときに発生するので、
    # この例外処理によりこのノードが終了する。
    try:
        main
    except rospy.ROSInterruptException: pass
