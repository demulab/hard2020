#!/usr/bin/env python
# -*- coding: utf-8 -*- # 日本語を使うためのおまじない。
import rospy  # ROSでpython使う場合に必要
from nav_msgs.msg import Odometry   # オドメトリを使う場合は必要
import tf
from tf.transformations import euler_from_quaternion

# グローバル変数。先頭のアンダーバーはグローバル変数の目印。
# 0.0で初期化している。
_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0


# callback_odom関数
def callback_odom(msg):
    global _odom_x, _odom_y, _odon_theta # グローバル変数
    _odom_x = msg.pose.pose.position.x  # x座標 [m]
    _odom_y = msg.pose.pose.position.y  # y座標 [m]
    # クォーターニオン(4次元数) 姿勢を表す手法の一つ
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    # クォータニオンからオイラー角を取得
    e = euler_from_quaternion(q)
    _odom_theta = e[2] # e[0]:ロール角, e[1]：ピッチ角, e[2]：ヨー角 [rad]

    rospy.loginfo("Odomery: x=%s y=%s theta=%s", _odom_x, _odom_y, _odom_theta)



# odometry関数
def odometry():
    #  ノードの初期化。引数はノード名。ROSでは同じノード名のノードを複数作ることはできない。
    rospy.init_node('odometry')

    # サブスクライバー（購読者)の生成。一番目の引数はトピック名、２番目の引数はメッセージの型、
    # オドメトリのメッセージはOdometry型。３番目の引数はコールバック関数。
    # 新しいメッセージが来るたびにコールバック関数が自動的に呼び出される。
    # これは以下のrospy.spin()とは別に並列で実行される。
    odom_subscriber = rospy.Subscriber('/create1/odom', Odometry, callback_odom)

    # spin()はノードが終了するまで、このプロプグラムを終わらせないようにしている。
    # これがないとプログラムはすぐ終了しノードも死んでしまう。
    rospy.spin()

    

# このプログラムをモジュールとしてimportできるようにするおまじない。
# なお、モジュールとは他のプログラムから再利用できるようにしたファイルのこと。
if __name__ == '__main__':
    odometry()
