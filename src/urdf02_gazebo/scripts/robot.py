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
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
class robot():
    def __init__(self,obj):
        self.obj_name=obj




        self.angle_dis_memory=0

        self.current_v=0
        self.v_record=[]
        self.distance_record=[]


        self.speed_pub_delay=0.1

        self.angle_speed_current=0
        self.angle_speed_record=[]
        self.angle_difference_record=[]

    def cmd_pub(self,x,a,t):
        msg=Twist()
        topic="/"+self.obj_name+"/cmd_vel"
        pub = rospy.Publisher(topic, Twist, queue_size=10)
        msg.linear.x=x
        msg.angular.z=a
        
        
        for i in range(t):
            pub.publish(msg)
            rospy.sleep(self.speed_pub_delay)


    def array_pub(self,array,top):
        topic="/"+self.obj_name+top
        pub=rospy.Publisher(topic,Float32MultiArray,queue_size=10)
        arr=Float32MultiArray(data=array)
        pub.publish(arr)



    def num_pub(self,num,top):
        topic="/"+self.obj_name+top
        pub=rospy.Publisher(topic,Float32,queue_size=10)
        num=Float32(data=num)
        pub.publish(num)        
        
    def pub_useful_messages(self,delay):
        while not rospy.is_shutdown():
            rospy.sleep(delay)
            self.num_pub(self.angle_dis_memory,"/angle_dis_memory")
            self.num_pub(self.current_v,"/current_v")
            self.num_pub(self.angle_speed_current,"/angle_speed_current")
            # rospy.loginfo(self.current_v)





    def track_triangle(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            self.cmd_pub(0.8,0,15)
            self.cmd_pub(0.0,math.radians(120),20)
            # rospy.loginfo(math.radians(120))

    def track_circle(self):
        while not rospy.is_shutdown():
            self.cmd_pub(0.5,0.8,10)

    def track_curve(self):
        while not rospy.is_shutdown():
            self.cmd_pub(-0.6,0,40)
            self.cmd_pub(-0.2,-1.5707,11)

    


    def info(self,m):
        while not rospy.is_shutdown():
            rospy.loginfo(m)

    def update_speed_rocord(self,delay):
        while not rospy.is_shutdown():
            rospy.sleep(delay)
            msg=rospy.wait_for_message("/"+self.obj_name+"/cmd_vel",Twist, timeout=None)
            self.current_v=msg.linear.x
            self.angle_speed_current=msg.angular.z
            self.v_record.append(self.current_v)
            self.angle_speed_record.append(self.angle_speed_current)
            if len(self.v_record)>20:
                del self.v_record[0]
            if len(self.angle_speed_record)>20:
                del self.angle_speed_record[0]

            # rospy.loginfo(self.v_record)
            # rospy.loginfo(self.angle_speed_record)


    def update_angle_memory(self):
        while not rospy.is_shutdown():
            msg=rospy.wait_for_message("/"+self.obj_name+"/cmd_vel",Twist, timeout=None)
            msga=msg.angular.z
            self.angle_dis_memory=self.angle_dis_memory+msga*self.speed_pub_delay

            if math.degrees(self.angle_dis_memory) >= 360:
                self.angle_dis_memory=self.angle_dis_memory - math.radians(360)
            if math.degrees(self.angle_dis_memory)<=-360:
                self.angle_dis_memory=self.angle_dis_memory + math.radians(360)


            # rospy.loginfo(self.angle_dis_memory)

        



robot2=robot('robot2')
robot3=robot("robot3")



class Thread_update_speed (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID


    def run(self):
        robot2.update_speed_rocord(0.5)




class Thread_run_robot (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID


    def run(self):
        robot2.track_curve()



class Thread_update_angle_memory(threading.Thread):   #继承父类threading.Thread
    def __init__(self,threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        

    def run(self):
        robot2.update_angle_memory()



class Thread_Pub_useful_messages(threading.Thread):   #继承父类threading.Thread
    def __init__(self,threadID,delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.delay=delay

    def run(self):
        robot2.pub_useful_messages(self.delay)



T_run_robot=Thread_run_robot(1)
T_update_rocord=Thread_update_speed(2)
T_update_angle_memory=Thread_update_angle_memory(3)
T_pub_useful_messages=Thread_Pub_useful_messages(4,0.2)
if __name__ == '__main__':
    rospy.init_node('robot')
    T_run_robot.start()
    T_update_rocord.start()
    T_update_angle_memory.start()
    T_pub_useful_messages.start()
    # robot2=robot("robot2")
    # robot2.track_triangle()


