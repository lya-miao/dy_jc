#!/home/maggie/anaconda3/bin/python
from gevent import sleep
import rospy
import numpy as np
from queue import Queue
from sensor_msgs.msg import LaserScan
import time 
import threading 
import math
from geometry_msgs.msg import Twist


class robot():
    def __init__(self,obj):
        self.obj_name=obj


        self.current_v=0
        self.v_record=[]
        self.distance_record=[]


        self.angle_speed_current=0
        self.angle_speed_record=[]
        self.angle_difference_record=[]

    def cmd_pub(self,x,a,t):
        msg=Twist()
        topic="/"+self.obj_name+"/cmd_vel"
        pub = rospy.Publisher(topic, Twist, queue_size=10)
        msg.linear.x=x
        msg.angular.z=a
        rate = rospy.Rate(10)
        for i in range(t):
            pub.publish(msg)
            rate.sleep()

    def track_triangle(self):
        while not rospy.is_shutdown():
            self.cmd_pub(0.6,0,20)
            self.cmd_pub(0.1,math.radians(15),8)




if __name__ == '__main__':
    rospy.init_node('robot')
    robot2=robot("robot2")
    robot2.track_triangle
