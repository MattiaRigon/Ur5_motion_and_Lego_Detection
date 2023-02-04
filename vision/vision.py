from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2,Image
from sensor_msgs import point_cloud2
import numpy as np
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
from cv_bridge import CvBridge
import cv2
import torch
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler
from os import walk
from os.path import join
from math import cos
from gazebo_msgs.srv import SpawnModel
from pandas import array
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates,ModelState
from tf.transformations import quaternion_from_euler
import rospy, rospkg, rosservice
import sys
import time
import random
import numpy as np

from time import sleep
import xml.etree.ElementTree as ET

from math import pi
from std_msgs.msg import String
from spawnLego_pkg.msg import legoDetection
from spawnLego_pkg.msg import legoGroup
from math import pi
from math import sin 
#import ogl_viewer.viewer as gl
#import pyzed.sl as sl
#import pcl
#import pcl.pcl_visualization

import matplotlib.pyplot as plt
import math
from math import atan
from math import tan
from math import sqrt

from params import *
pub = rospy.Publisher('lego_position', legoGroup, queue_size=10)




#Resources

# native reading best explanation
# https: // answers.ros.org / question / 219876 / using - sensor_msgspointcloud2 - natively /
# https://medium.com/@jeehoahn/some-ideas-on-pointcloud2-data-type-1d1ae940ef9b
# https://answers.ros.org/question/373094/understanding-pointcloud2-data/
# https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2

#array=None
# Objects=[(640,360),(630,350),(620,340),(650,380),(740,380)]
Objects = []
point_count_for_item = []
list=[]

def distanza(p1,p2):

    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    

def trova_posizione_lego(num_lego,posizioni,pandino):  #

    cont = 0
    v1 = [0,0]
    v2 = [0,0]
    v3 = [0,0]
    ymax = 0
    ymin = 100000
    xmin = 100000
    zmax = 0
    last_pos = [posizioni[0][0],posizioni[0][1]]


    for pos in posizioni:
        if(pos[2] > 0.8661):
            if(pos[1]>ymax):
                ymax = pos[1]
                v1 = np.copy(pos)
            if(pos[1]<ymin):
                ymin = pos[1]
                v3 = np.copy(pos)
            if(pos[0]<xmin):
                xmin = pos[0]
                v2 = np.copy(pos)
            if(pos[2]>zmax):
                zmax = pos[2]


    b = distanza(v2,v3)
    p = distanza(v1,v2)

    if(b < p):
        tmp = b
        b = p
        p = tmp
    
    h = zmax -0.866       

    print("3 punti magici :")
    print(v1)
    print(v2)   
    print(v3)

    pos = [(v1[0]+v3[0])/2,(v1[1]+v3[1])/2]
    if(v2[0]-v1[0] != 0):
        alpha =  atan((v1[1]-v2[1])/(v1[0]-v2[0])) 
    else:
        alpha=0

    d12 = distanza(v1,v2)
    d23 = distanza(v2,v3)
    if(d12 > d23):
        alpha =alpha+pi/2
    
    rot = [0,0,alpha]

    print("pos : ")
    print(pos)
    print("rot : ")
    print(rot)

    initial_pose = Pose()
    initial_pose.position.x = pos[0]
    initial_pose.position.y = pos[1]
    initial_pose.position.z = 0.89

    q = quaternion_from_euler(0, 0,alpha)

    initial_pose.orientation.x = q[0]
    initial_pose.orientation.y = q[1]
    initial_pose.orientation.z = q[2]
    initial_pose.orientation.w = q[3]

    return initial_pose


def receive_pointcloud(pandino):

    msg = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)
    # read all the points
    #array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
    array = []
    pos_zed = [-0.9 ,0.24 ,-0.35]  # y forse 0.18
    pos_base_link = np.array([0.5,0.35,1.75])

    points_list = []
    for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=Objects):
        points_list.append([data[0], data[1], data[2]])
    
    cont = 1
    actual_detection = 0
    actual_lego =[]
    print("IMMAGINE 0")
    for data in points_list:
        alpha = -0.523
        # Ry = np.matrix([[cos(alpha),0,sin(alpha)],
        #             [0,1,0],
        #             [-sin(alpha),0,cos(alpha)]])
        Ry = np.matrix([[ 0.     , -0.49948,  0.86632],
        [-1.     ,  0.     ,  0.     ],
        [-0.     , -0.86632, -0.49948]])

        data_zed_rotation = np.array(data*Ry)*-1
        
        data_base_link =np.array(data_zed_rotation.tolist()[0]) + np.array(pos_zed)
        #print(data_base_link)

        #data_world = data_base_link + pos_base_link
        data_world = Ry.dot(data) + pos_zed + pos_base_link
        data_world = np.array(data_world)

        if(actual_detection>len(point_count_for_item)-1):
            break
        if(cont >= point_count_for_item[actual_detection]):
            print("IMMAGINE " + str(actual_detection+1))
            initial_pose = trova_posizione_lego(actual_detection,actual_lego,pandino)

            list.append(legoDetection("prova",initial_pose))
            actual_detection = actual_detection +1
            cont = 1
            actual_lego = []
        
        cont = cont +1
        actual_lego.append(data_world[0])
        #print(data_world[0])
    
    print()



def receive_image():

    msg = rospy.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)
    
    rgb = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    table = [[558*1.5, 278*1.5], [460*1.5, 552*1.5], [957*1.5,535*1.5], [777*1.5, 267*1.5]]
    mask = np.array(table, dtype=np.int32)

    background = np.zeros((rgb.shape[0], rgb.shape[1]), np.int8)
    cv2.fillPoly(background, [mask],255)
    mask_background = cv2.inRange(background, 1, 255)

    img = cv2.bitwise_and(rgb, rgb, mask=mask_background)
    
    cv2.imwrite(LAST_PHOTO_PATH, img)
    riconoscimento()



def riconoscimento():
    model = torch.hub.load(YOLO_PATH, 'custom', path=BEST_PATH, source='local')  # local repo
    
    im1 = cv2.imread(LAST_PHOTO_PATH)[..., ::-1]
    results = model([im1], size=640) # batch of images

    # Results
    results.print()  
    results.save() 

    results.render()
    
    img=results.ims[0]
    # cv2.imshow("prova",img)
    # cv2.waitKey(0)

    prova = results.xyxy[0]  # im1 predictions (tensor)
    #print(prova)
    pandino=results.pandas().xyxy[0]  # im1 predictions (pandas)
    print(pandino)
    for k in range(0,pandino.shape[0]):
        cont = 0
        if(pandino.confidence[k]<0.5):
            continue
        for j in range(int(pandino.ymin[k]),int(pandino.ymax[k])):
            for i in range(int(pandino.xmin[k]),int(pandino.xmax[k])):
                tupla=(i,j)
                Objects.append(tupla)
                cont = cont +1 
        point_count_for_item.append(cont)


 
    receive_pointcloud(pandino)


if __name__ == '__main__':

    rospy.init_node('custom_joint_pub_node')
    #sub_pointcloud = rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, callback = receive_pointcloud, queue_size=1)
    #sub_image = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback = receive_image, queue_size=1)
    loop_rate = rospy.Rate(1.)
    #spawn()
    #receive_pointcloud()
    receive_image()
    message = legoGroup("Assigment 1",list)   

    pub.publish(message)


    
