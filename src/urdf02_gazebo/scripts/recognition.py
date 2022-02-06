#!/home/maggie/anaconda3/bin/python

from sympy import re
import rospy
from sensor_msgs.msg import LaserScan
import ros_numpy
import numpy as np
import math
import threading
import time
import cmath


from Thread_test import D1
from Thread_test import C1_Map

from robot import robot2
from geometry_msgs.msg import Twist

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32





P_Map=[]




def extend_predict_Map():
    predict_Map = np.zeros(360)
    if len(predic_layer_1)<1:
        predic_layer_1.append[0,0]
    for i in range(360):
        if i != predic_layer_1[-1][0]:
            predict_Map[i] = 0
        else: predict_Map[i] = predic_layer_1[-1][1]

    return predict_Map




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
        publish_point(P_Map,"predic_1")
        # D11=rospy.wait_for_message("/D1",Float32MultiArray, timeout=None)
        # rospy.loginfo(D11)
        # rospy.loginfo(robot2.current_v)
  



T=0.5


predic_layer_1=[]




def update_predict_layer(delay):
    global predic_layer_1
    global P_Map
    while not rospy.is_shutdown(): 
        time.sleep(delay)
        D11_msg=rospy.wait_for_message("/D1",Float32MultiArray, timeout=None)
        D11_list=np.array(D11_msg.data)
        D11= D11_list.reshape((int(len(D11_list)/2), 2))
        if len(predic_layer_1)<=20:
            if len(D11)>1:
                predic_layer_1.append(D11[0])
        if len(predic_layer_1)>20:
            predic_layer_1.append(chose_num(D11))
            del predic_layer_1[0]


        P_Map=extend_predict_Map()
        rospy.loginfo(predic_layer_1)
        


     
            
def chose_num(O):
    if len(O)>1:
        num=O[0]
    if len(O)<=1:
        num=predic_next_move(T)

    return num

# def chose_num(O):
#     if abs(O[0]-predic_layer_1[-1][0])>20:
#         num=O
    
#     else: 
#         num=predic_next_move(T)

#     return num



def coordinate_transition_2D(P):
    x=P[1]*math.cos(math.radians(P[0]))
    y=P[1]*math.sin(math.radians(P[0]))
    return [x,y]


def coordinate_transition_p(Q):
    Z=complex(Q[0],Q[1])   
    p=list(cmath. polar(Z))
    return p




def predic_next_move(T):
    current_v_2=rospy.wait_for_message("/robot2/current_v",Float32, timeout=None)
    angle_speed_current_2=rospy.wait_for_message("/robot2/angle_speed_current",Float32, timeout=None)
    angle_dis_memory_2=rospy.wait_for_message("/robot2/angle_dis_memory",Float32, timeout=None)

    now_point=coordinate_transition_2D(predic_layer_1[-1])
    delta_a_face=angle_speed_current_2.data*T
    delta_a_between_2point=delta_a_face/2
    delta_d=current_v_2.data*T
    now_face=angle_dis_memory_2.data
    d_angle=delta_a_between_2point+now_face
    delta_x=delta_d*math.cos(d_angle)
    delta_y=delta_d*math.sin(d_angle)

    next_point_2D=[delta_x+now_point[0],delta_y+now_point[1]]
    next_point_p=coordinate_transition_p(next_point_2D)

    return next_point_p







class Thread_update_predict_layer (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
    
        self.delay = delay

    def run(self):
        update_predict_layer(self.delay)




class Thread_Publisher (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
   
        self.delay = delay

    def run(self):
        Publisher(self.delay)
        

T_pub=Thread_Publisher(1,0.3)
T_update_predict=Thread_update_predict_layer(2,0.2)

if __name__ == '__main__':
    rospy.init_node('predict')
    T_update_predict.start()
    T_pub.start()

