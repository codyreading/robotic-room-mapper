#!/usr/bin/env python
import math

import rospy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from imu.minimu import MinIMU 
        
class ImuNode(object):

    def __init__(self):
        rospy.init_node("imu")
        self.min_imu = MinIMU()
        self.min_imu.enable()
        self.min_imu.calibrateGyroAngles()

        self.imu_msg = Imu()
        self.header = Header()
        self.imu_pub = rospy.Publisher("imu_data", Imu, queue_size=1)         
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            header.stamp = rospy.Time.now()
            imu_msg.header = header

            euler = self.min_imu.getComplementaryAngles()
            euler = tuple([ang * math.pi / 180.0 for ang in euler])
            quaternion = tf.transformations.quaternion_from_euler(*euler)
            self.imu_msg.orientation.x = quaternion[0]
            self.imu_msg.orientation.y = quaternion[1]
            self.imu_msg.orientation.z = quaternion[2]
            self.imu_msg.orientation.w = quaternion[3]
            self.imu_msg.orientation_covariance = [0, 0, 0, 
                                                   0, 0, 0, 
                                                   0, 0, 0] 

            angular_velocity = self.min_imu.getGyroRotationRates() * math.pi / 180.0
            self.imu_msg.angular_velocity.x = angular_velocity[0] 
            self.imu_msg.angular_velocity.y = angular_velocity[1] 
            self.imu_msg.angular_velocity.z = angular_velocity[2]
            self.imu_msg.angular_velocity_covariance = [0, 0, 0, 
                                                        0, 0, 0, 
                                                        0, 0, 0]  

            self.imu_msg.linear_acceleration_covariance = [-1,  0, 0, 
                                                            0, -1, 0, 
                                                            0,  0, -1]  
            self.imu_pub.publish(imu_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        imu_node = ImuNode()
        imu_node.spin()
    except rospy.ROSInterruptException:
        pass