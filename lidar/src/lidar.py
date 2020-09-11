#!/usr/bin/env python
# -*- coding: utf-8 -*- # 日本語を使うためのおまじない。
import rospy  # ROSでpython使う場合に必要
from geometry_msgs.msg import Twist # ロボットの速度を扱う場合は必要
from sensor_msgs.msg import LaserScan # LIDARを使うときは必要
import math   # 数学関数モジュール


# RoombaRobotクラス
class RoombaRobot():
    def __init__(self):  # コンストラクタ
        #  ノードの初期化。引数はノード名。ROSでは同じノード名のノードを複数作ることはできない。
        rospy.init_node('lidar', anonymous=True)
        
        # サブスクライバー（購読者)の生成。一番目の引数はトピック名、２番目の引数はメッセージの型、
        # オドメトリのメッセージはLaserScan型(正確にはsensor_msgs/LaserScan型だが
        # from sensor_msgs.msg import LaserScanでインポートしているのでここではLaserScanだけでよい)。
        # ３番目の引数はコールバック関数。
        self.odom_sub = rospy.Subscriber('/create1/rplidar/scan', LaserScan, self.lidar_callback)

        # パブリッシャーの生成。一番目の引数はトピック名、２番目の引数はメッセージの型、
        # 速度指令はTwist型。３番目の引数はメッセージバッファーのサイズ。
        self.vel_pub = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)
        self.set_vel(0, 0)

        
    # set_vel関数：速度の設定。
    def set_vel(self, lv, av):
        vel = Twist()
        vel.linear.x  = lv  # 並進速度
        vel.linear.y  = 0
        vel.linear.z  = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = av  # 角速度

        self.vel_pub.publish(vel)

        # 表示
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel.linear.x, vel.angular.z)
        del vel

    # lidar_callback関数
    # /scanのMessage型はhttp://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.htmlを参照。
    def lidar_callback(self, msg):
        self.ranges          = msg.ranges
        self.angle_min       = msg.angle_min # スキャンの開始角度[rad]
        self.angle_max       = msg.angle_max # スキャンの終了角度[rad]
        self.angle_increment = msg.angle_increment # 計測間隔[rad]
        self.range_min       = msg.range_min # 最小検出距離[m]
        self.range_max       = msg.range_max # 最大検出距離[m]
        
    # main関数
    def main(self):


        # ループの周期を設定。100Hz。つまり、1ループ10ms。
        rate = rospy.Rate(100)

        # キーボードからの入力
        print("Let's move your robot")
        linear_vel  = input("Input linear velocity [m/s] :")
        angular_vel = input("Input angular velocity [rad/s] :")
        self.set_vel(linear_vel, angular_vel)             
        
        # Ctrl-Cが押されるまで無限ループ
        while not rospy.is_shutdown():

            rospy.loginfo("Number of Rays=%d", len(self.ranges))  # レーザの本数
        
            # シミュレーションのPRDLIARは全周360[°] 計測可能。計測角度は-180[°]から180[°]。
            # ROSは右手系、進行方向x軸、左方向y軸、上方向がz軸（反時計まわりが正)。
            rospy.loginfo("Angle [rad] min=%f max=%f", self.angle_min, self.angle_max) 
        
            # ROSの角度は[rad]。math.degresss関数でラジアンから°に変換している。
            rospy.loginfo("Angle [deg] increment=%.3f", math.degrees(self.angle_increment)) 
            rospy.loginfo("Range [m] min=%.3f max=%.3f", self.range_min, self.range_max)   
            rospy.loginfo("  0[deg]=%.3f[m]  90[deg]=%.3f[m]", self.ranges[0],self.ranges[180])
            rospy.loginfo("180[deg]=%.3f[m] -90[deg]=%.3f[m]", self.ranges[360],self.ranges[540])

            #　ミッション１：ここに、何かに0.3[m]以内に近づいたら停止するコードを書く

        
            # rospy.Rate()で指定した時間になるように調整してくれる。
            rate.sleep()


# このプログラムをモジュールとしてimportできるようにするおまじない。
# なお、モジュールとは他のプログラムから再利用できるようにしたファイルのこと。
if __name__ == '__main__':
    try:
        robot = RoombaRobot()
        robot.main()
    except rospy.ROSInterruptException: pass
