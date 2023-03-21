#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu

class Imu_class:
    def __init__(self):
        self.starting = True
        self.rotated_angle = np.array([0.5,0.5,0.5,0.5])
        self.sub = rospy.Subscriber('/ahrs_imu/imu', Imu, self.imu_callback)
        rospy.spin()
    
    def imu_callback(self, msg):
        current_orientation = np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])
        if self.starting:
            print(f'before{current_orientation}')
            self.rotated_angle = self.rotate(current_orientation)
            print(f'after{self.rotated_angle}')
            self.starting = False
        
        self.check_ornt(current_orientation)
    
    def rotate(self, q):
        axis = np.array([0,0,1])
        axis = axis / np.linalg.norm(axis)

        angle = np.pi/2

        s = np.sin(angle / 2)
        c = np.cos(angle / 2)

        q_rot = np.array([c, axis[0] * s, axis[1] * s, axis[2] * s])
        q_new = self.quat_multipy([q_rot, q])

        return q_new
    
    def quat_multipy(self, q):
        result = np.array([1,0,0,0])
        for i in q:
            w1, x1, y1, z1 = result
            w2, x2, y2, z2, = i

            result = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*z2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2 
            ])
        
        return result

        
    
    def check_ornt(self, q):
        diff = np.dot(q, self.rotated_angle)
        if diff >= 0.995:
            print('90 degree reached')
        
        else:
            print(diff)

if __name__ == '__main__':
    rospy.init_node('imu_node', anonymous=True)
    Imu_class()