#!/home/maggie/anaconda3/bin/python
from numpy.core.fromnumeric import argmax
from numpy.lib.function_base import median
from numpy.ma.core import get_object_signature
from sympy import re
import rospy
from sensor_msgs.msg import LaserScan
import ros_numpy
import numpy as np

import threading
import time






Map=[]
G=[]
G1=[]
Gg=[]
D=np.zeros(360)

t=[]
K=[]



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
        


class Thread_update_D1 (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.delay = delay

    def run(self):
        update_D1(self.delay)
       

class Thread_update_C (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.delay = delay

    def run(self):
        update_C(self.delay)
       


def update_D(delay):
    
    while not rospy.is_shutdown():
        time.sleep(delay)
        D1 = [Map[i] - S[i] for i in range(len(Map))]
        for i in range(len(D1)):
            if abs(D1[i]) < scope_d:
                D[i] = 0
            else :
                D[i]= Map[i]

        # t.append(1)

        # if len(t) > 20:
        #     del t[0]
        #     update_D1_classifytrack()




            


      




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
        SS=np.round(S,3)
        rospy.loginfo(SS)
       
        # publish_point(D,"dynamic_point")
        # publish_point(C1_Map,"dynamic_1")
        # publish_point(C2_Map,"dynamic_2")

        


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

        


D1=[]




def update_D1(delay):
    # global K 
    global D1 
    while not rospy.is_shutdown():
        time.sleep(delay)
        D11=[]          
        # if D[0] < 0.3:    
        for i in range(len(D)-1):
            if (D[i]-D[i-1]) > 0.3:
                D11.append([i,D[i]])
        D1=D11
        
        # rospy.loginfo(D1)

   

C1_T=[] 
C2_T=[]
C1_Map=[]
C2_Map=[]




def update_C(delay):
    global C1_T
    global C2_T
    global C1_Map
    global C2_Map


    
    while not rospy.is_shutdown():
        rospy.sleep(delay)


        if len(D1)<1:

            continue

        # if len(C1_T)<20:
        c1_add=D1[0]
        c2_add=D1[-1]
        C1_T.append(c1_add)
        C2_T.append(c2_add)

        # rospy.loginfo(C1_T)
        # rospy.loginfo(C2_T)

        if len(C1_T) > 20:
            del C1_T[0]
        if len(C2_T) > 20:
            del C2_T[0]

        # else:

        #     c1_add = choose_num_from_D1(C1_T)
        #     c2_add = choose_num_from_D1(C2_T)

        #     rospy.loginfo(c1_add)
        #     rospy.loginfo(c2_add)

        #     C1_T.append(c1_add)
        #     C2_T.append(c2_add)

        #     if len(C1_T) > 20:
        #         del C1_T[0]
        #     if len(C2_T) > 20:
        #         del C2_T[0]

        C1_Map=extend_as_C_Map(C1_T)
        C2_Map=extend_as_C_Map(C2_T)





def choose_num_from_D1(C):
    min_d =100
    for i in range (len(D1)):
        d = abs(C[-1][1]-D1[i-1][1] )
        
        if min_d > d:
            min_d = d
            min_i = i

        if min_d >40:
            add_p=C[-1]

        else: add_p=D1[min_i]

    return add_p





def extend_as_C_Map(c):
    p_end= c[-1]
    C_Map = np.zeros(360)
    for i in range(360):
        if i != p_end[0]:
            C_Map[i] = 0
        else: C_Map[i] = D[i]

    return C_Map


# def init_C(num):
#     global D1
#     c=[]
#     while len(c)<20:
#         c.append(D1[num-1])
#     return c
    

        #         K.append(i)
        # D1=[0 for x in range(len(K))]
        # for j in range(K):
        #     D1[j] = ana(K,j)
            

        
        # rospy.loginfo(K)
        # K=[]

    # else:
    #     K.append(0)
    #     for i in range(len(D)-1):
    #         if (D[i]-D[i-1]) > 0.3:
    #             K.append(i)
    #     for j in range(K):
    #         D1[j] = ana(K,j)

    #     # rospy.loginfo(K)
    #     K=[]        

        

# def ana(k,j):
#     dm = np.zeros(360)
#     for i in range[k[j]:359]:
#         if abs(D[i]-D[i+1])<0.3:
#             dm[i]=D[i]
#         else:break







                        
        


            



            




       
T_update_Map = Thread_update_Map(1,0.1)
T_update_G_S = Thread_update_G_S(2,0.5)
T_update_D = Thread_update_D(3,0.2)
T_Pub = Thread_Publisher(4,0.1)
T_update_D1= Thread_update_D1(5,0.1)
T_update_C= Thread_update_C(5,0.2)


if __name__ == '__main__':
    rospy.init_node('get_laser')
    T_update_Map.start()
    T_update_G_S.start()
    # T_update_D.start()
    # T_update_D1.start()
    # T_update_C.start()
    T_Pub.start()





