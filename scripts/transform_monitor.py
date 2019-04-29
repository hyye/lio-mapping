#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Vector3, Vector3Stamped
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix, euler_from_quaternion, euler_from_matrix, euler_matrix


class transform_monitor(object):
    def __init__(self):
        rospy.init_node('transform_monitor')
        self.laser_odom = rospy.get_param('~laser_odom', default='/local_laser_odom/')
        self.imu_odom = rospy.get_param('~imu_odom', default='/predict_odom')

    def get_odom(self, odom_msg):
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        explicit_pos = [pos.x, pos.y, pos.z]
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]

        transform_world_laser = quaternion_matrix(explicit_quat)
        transform_world_laser[0:3, 3] = explicit_pos
        euler_angles = euler_from_quaternion(explicit_quat, 'rzyx')
        z = euler_angles[0]
        y = euler_angles[1]
        x = euler_angles[2]

        v = Vector3Stamped()
        v.header = odom_msg.header
        v.vector.x = x
        v.vector.y = y
        v.vector.z = z

        self.imu_rot_pub.publish(v)

        # print('imu_rot_y', y)

    def get_laser_odom(self, odom_msg):
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        explicit_pos = [pos.x, pos.y, pos.z]
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]

        transform_world_laser = quaternion_matrix(explicit_quat)
        transform_world_laser[0:3, 3] = explicit_pos
        euler_angles = euler_from_quaternion(explicit_quat, 'rzyx')
        z = euler_angles[0]
        y = euler_angles[1]
        x = euler_angles[2]

        v = Vector3Stamped()
        v.header = odom_msg.header
        v.vector.x = x
        v.vector.y = y
        v.vector.z = z

        self.laser_rot_pub.publish(v)

        # print('laser_rot_y', y)

    def main(self):
        r = rospy.Rate(1000)
        self.odom_sub = rospy.Subscriber(self.imu_odom, Odometry, self.get_odom)
        self.laser_odom_sub = rospy.Subscriber(self.laser_odom, Odometry, self.get_laser_odom)
        print('self.imu_odom', self.imu_odom)
        print('self.laser_odom', self.laser_odom)
        self.imu_rot_pub = rospy.Publisher('/debug/imu_rot', Vector3Stamped, queue_size=10)
        self.laser_rot_pub = rospy.Publisher('/debug/laser_rot', Vector3Stamped, queue_size=10)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    monitor = transform_monitor()
    try:
        monitor.main()
    except rospy.ROSException:
        print 'Interrupted'
