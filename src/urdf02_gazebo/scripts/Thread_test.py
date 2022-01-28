#!/home/maggie/anaconda3/bin/python
from numpy.core.fromnumeric import argmax
from numpy.lib.function_base import median
from numpy.ma.core import get_object_signature
import rospy
from sensor_msgs.msg import LaserScan
import ros_numpy
import numpy as np

import threading
import time

exitFlag = 0
Map=[]
G=[]
G1=[]
Gg=[]
D=np.zeros(360)
scope_d = 0.1

S=np.zeros(360)





class Thread_update_Map (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
    
        self.delay = delay

    def run(self):
        update_Map(self.delay)



class Thread_update_G_S (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
   
        self.delay = delay

    def run(self):
        update_G_S(self.delay)


class Thread_update_D (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID,delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.delay = delay

    def run(self):
        update_D(self.delay)



class Thread_Publisher (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
   
        self.delay = delay

    def run(self):
        Publisher(self.delay)






def update_D(delay):
    while not rospy.is_shutdown():
        time.sleep(delay)
        D1 = [Map[i] - S[i] for i in range(len(Map))]
        for i in range(len(D1)):
            if abs(D1[i]) < scope_d:
                D[i] = 0
            else :
                D[i]= Map[i]



def publish_point(M,name):
    topic="/"+ name +"_pub"
    pub = rospy.Publisher(topic, LaserScan, queue_size=10)
    msg=LaserScan()
    msg.angle_max=3.14199995995
    msg.angle_min=-3.14199995995
    msg.angle_increment=0.0175041779876
    msg.time_increment=0.0
    msg.scan_time=0.0
    msg.range_min=0.300000011921
    msg.range_max=30.0
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id ="robot1/laser"
    msg.intensities=[0.0]*360
    msg.ranges=M

    pub.publish(msg)
   

def Publisher(delay):
    while not rospy.is_shutdown():
        time.sleep(delay)
        publish_point(S,"static")
        publish_point(D,"dynamic_point")


    #rospy.loginfo(S)





def update_G_S(delay):
    while not rospy.is_shutdown():
        time.sleep(delay)
        G.append(Map)
        if(len(G)>20):
            del G[0]

        G1.append(Map)
        if(len(G1)>50):
            del G1[0]
        G2 =np.round(G1,3)
        for i in range(360):
            g_most=(np.median(G2[:,i]))
            S[i]=g_most



def update_Map(delay):
    global Map

    while not rospy.is_shutdown():
        #rospy.sleep(0.5)
        time.sleep(delay)
        Map_or = rospy.wait_for_message("robot1/scan", LaserScan, timeout=None)
        Map=list(Map_or.ranges)

        


       
T_update_Map = Thread_update_Map(1,0.1)
T_update_G_S = Thread_update_G_S(2,0.5)
T_update_D = Thread_update_D(3,0.2)
T_Pub = Thread_Publisher(4,0.1)

if __name__ == '__main__':
    rospy.init_node('get_laser')
    T_update_Map.start()
    T_update_G_S.start()
    T_update_D.start()
    T_Pub.start()
