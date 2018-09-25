#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ToiletCleaning:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.getOdomCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.state = 'CHECK_SENSOR'
        self.base_pose = Twist()
        self.inversion = 1.0 

    def getOdomCB(self,msg):
        self.base_pose.linear.x = msg.pose.pose.position.x
        self.base_pose.linear.y = msg.pose.pose.position.y
        euler = tf.transformations.euler_from_quaternion((0,0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.base_pose.angular.z = euler[2] * self.inversion
        #print self.base_pose.angular.z

    def stopBase(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.cmd_vel_pub.publish(vel)

    def advanceBase(self,goal_dist):
        if goal_dist < 0:
            print "input positive number!"
            return
        init_x = self.base_pose.linear.x
        init_y = self.base_pose.linear.y
        moved_dist = 0.0
        vel = Twist()
        while moved_dist < goal_dist and not rospy.is_shutdown():
            moved_dist = np.sqrt(pow(init_x - self.base_pose.linear.x,2)
                                +pow(init_y - self.base_pose.linear.y,2))
            print moved_dist
            vel.angular.z = 0.01 # to correct
            vel.linear.x = 0.1
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stopBase()
        print "finish advancing"

    def rotateBase(self,goal_angle):
        if goal_angle < -1*np.pi or np.pi < goal_angle:
            print "goal angle must be set from -pi to pi."
            return
        init_angle = self.base_pose.angular.z
        moved_angle = 0.0
        vel = Twist()
        if goal_angle > 0:# Counter Clock Wise
            while moved_angle < goal_angle-0.04 and not rospy.is_shutdown():
                moved_angle = self.base_pose.angular.z - init_angle
                if moved_angle <= -np.pi+0.05:
                    print "over horizone"
                    moved_angle = 2*np.pi + moved_angle
                vel.angular.z = 0.1
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(0.01)
                print goal_angle,moved_angle 

        elif goal_angle < 0:# Clock Wise
            while moved_angle > goal_angle+0.04 and not rospy.is_shutdown():
                moved_angle = self.base_pose.angular.z - init_angle
                if moved_angle >= np.pi-0.05:
                    print "over horizone"
                    moved_angle = -2*np.pi + moved_angle
                vel.angular.z = -0.1
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(0.01)
                print goal_angle,moved_angle 
        self.stopBase()
        print "finish roteting"

    def checkSensor(self):
        vel = Twist()
        vel.angular.z = 0.1
        self.cmd_vel_pub.publish(vel)
        init_angle = self.base_pose.angular.z 
        rospy.sleep(3)
        print self.base_pose.angular.z - init_angle
        if self.base_pose.angular.z - init_angle > 0:
            self.inversion = 1
        if self.base_pose.angular.z - init_angle < 0:
            self.inversion = -1
            
        print "inversion is",self.inversion
        vel.angular.z = -0.1
        self.cmd_vel_pub.publish(vel)
        rospy.sleep(3)
        self.state="CLEAN_FLOOR"

    def cleanFloor(self):
        self.rotateBase(-0.5*np.pi)
        self.advanceBase(0.3)
        self.rotateBase(0.5*np.pi)
        self.advanceBase(0.9)
        self.rotateBase(0.5*np.pi)
        rospy.sleep(0.1)
        self.rotateBase(0.5*np.pi)
        self.advanceBase(0.9)
        self.rotateBase(-0.5*np.pi)
        self.advanceBase(0.8)
        self.rotateBase(-0.5*np.pi)
        self.advanceBase(0.9)
        self.rotateBase(0.5*np.pi)
        rospy.sleep(0.1)
        self.rotateBase(0.5*np.pi)
        self.advanceBase(0.9)
        self.rotateBase(0.5*np.pi)
        self.advanceBase(0.3)
        self.rotateBase(0.5*np.pi)
        exit()
    
    def loopMain(self):
        while not rospy.is_shutdown():
            if self.state=="CHECK_SENSOR":
                self.checkSensor()
            if self.state=="CLEAN_FLOOR":
                self.cleanFloor()
            rospy.sleep(0.3)

if __name__ == '__main__':
    rospy.init_node('toilet_cleaning')
    toilet_cleaning = ToiletCleaning()
    rospy.sleep(1) # wait setup roomba
    toilet_cleaning.loopMain()
