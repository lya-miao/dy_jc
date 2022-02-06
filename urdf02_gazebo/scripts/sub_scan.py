#!/home/maggie/anaconda3/bin/python
from numpy.core.fromnumeric import argmax
from numpy.lib.function_base import median
from numpy.ma.core import get_object_signature
import rospy
from sensor_msgs.msg import LaserScan
import ros_numpy
import numpy as np

data1_map=[]
#data2=[]


G=[]

Gg=[]
D=np.zeros(360)
scope_d = 0.3

S=np.zeros(360)


def updata_D(M1,S2):
    D1 = [M1[i] - S2[i] for i in range(len(M1))]
    for i in range(len(D1)):
        if abs(D1[i]) <scope_d:
            D[i] = 0
        else :
            D[i]= M1[i]



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
   



def update_S(G_):
    
    G1 =np.array(G_)
    G2 =np.round(G1,3)
    for i in range(360):
        g_most=(np.median(G2[:,i]))
        S[i]=g_most

    
    #rospy.loginfo(S)








def update_Map():
    global data1_map

    rospy.init_node('get_laser')

    while not rospy.is_shutdown():
        rospy.sleep(0.5)
        data1_map = rospy.wait_for_message("robot1/scan", LaserScan, timeout=None)

        

        G.append(list(data1_map.ranges))
        if(len(G)>20):
            del G[0]

       
        update_S(G)
        publish_point(S,"static")

        updata_D(list(data1_map.ranges),S)
        publish_point(D,"dynamic_point")


        #c=len(G)
        


        #data2 = rospy.wait_for_message("robot2/scan", LaserScan, timeout=None)
        #rospy.loginfo(G)
        # rospy.loginfo(data2.ranges)

if __name__ == '__main__':
    update_Map()




# robot1_data_ranges=[]

# def callback(s):
#     global robot1_data_ranges
#     #LaserScan的数据结构
#     #std_msgs/Header header
#     #float32 angle_min
#     #float32 angle_max
#     #float32 angle_increment
#     #float32 time_increment
#     #float32 scan_time
#     #float32 range_min
#     #float32 range_max
#     #float32[] ranges
#     #float32[] intensities
#     #rospy.loginfo(s.ranges)
#     robot1_data_ranges=s.ranges



# def listener():

#         rospy.init_node('lasr_listener', anonymous=False)
#         rospy.Subscriber('robot1/scan', LaserScan,callback)
#         rospy.spin()




    
# class ranges():
#     def __init__(self):
#         self.scan_ranges=[]

#     def get_scan_origin(self,s):
#                         #LaserScan的数据结构
#                         #std_msgs/Header header
#                         #float32 angle_min
#                         #float32 angle_max
#                         #float32 angle_increment
#                         #float32 time_increment
#                         #float32 scan_time
#                         #float32 range_min
#                         #float32 range_max
#                         #float32[] ranges
#                         #float32[] intensities
#                             #self.scan_ranges=list(self.ranges)
#         self.scan_ranges=list(s.ranges)
#         #rospy.loginfo(self.scan_ranges)
        
        

#     def listener(self):
#         rospy.init_node('lasr_listener', anonymous=False)
#         rospy.Subscriber('robot1/scan', LaserScan,self.get_scan_origin)
#         rospy.spin()


# if __name__ == '__main__':
#     robot1_data = ranges()
#     robot1_data.listener()
#     rospy.loginfo(robot1_data.scan_ranges)
