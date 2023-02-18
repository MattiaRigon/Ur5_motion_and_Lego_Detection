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
from math import sin,atan2
import matplotlib.pyplot as plt
import math
from math import atan
from math import tan
from math import sqrt

from params import *

pub = rospy.Publisher('lego_position', legoGroup, queue_size=10)

Objects = []
point_count_for_item = []
list = []
DIM_BLOCK = 0.03

    
def isUp(h):
    """return true if the block is up false if it si relaxed

    Args:
        h (int): max height of lego

    Returns:
        boolean: return true if the block is up false if it si relaxed
    """
    ret = True
    if(h < 0.04):
        ret = False
    return ret


def distanza(p1,p2):
    """this function calculate the distance between two point

    Args:
        p1 (array): first point cordinates
        p2 (array): second point cordinates

    Returns:
        int: the distance between the two points
    """
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    
def find_dimension(v1,v2,v3,zmax):
    """This function return the three lego's dimensions (width, length and height)

    Args:
        v1 (array): the cordinate of leftmost points
        v2 (array): the cordinate the lowest point at the bottom
        v3 (array): the cordinate of rightmost points
        zmax (int): the max height of lego

    Returns:
        array: the three dimension of lego (width, length and height)
    """
    #find dimension of blocks
    dimension = [0,0,0]
    d12 = distanza(v1,v2)
    d23 = distanza(v2,v3)
    if(d12 >= d23):
        dimension[0] = d12
        dimension[1] = d23
        dimension[2] = zmax - 0.866
    else:
        dimension[0] = d23
        dimension[1] = d12
        dimension[2] = zmax - 0.866
    
    return dimension


def find_orientation(dimension,v1,v2,v3):
    """This function find lego's position and orientation 

    Args:
        dimension (array): the three dimension of lego (width, length and height)
        v1 (array): the cordinate of leftmost points
        v2 (array): the cordinate the lowest point at the bottom
        v3 (array): the cordinate of rightmost points

    Returns:
        pos,rot: return the cordinate of position and rotation
    """

    rot = [0,0,0]
    if(not(isUp(dimension[2]))):
        print("Posizione --> sono appoggiato in un fianco")
        rot[1] = pi/2
    elif(dimension[2] >= 0.065):
        print("sono un Y" + str(int(dimension[2]/0.035)+1) + " in piedi")
        rot[0] = pi/2

    #find block center
    pos = [(v1[0]+v3[0])/2,(v1[1]+v3[1])/2]
    if(v2[0]-v1[0] != 0):
        alpha =  atan((v1[1]-v2[1])/(v1[0]-v2[0])) 
    else:
        alpha=0

    d12 = distanza(v1,v2)
    d23 = distanza(v2,v3)
    
    if(d12 > d23):
        alpha = alpha + pi/2
    
    rot[2] = alpha

    return pos,rot

def correction(dimension,nome,v3):
    """this function correct the name for classification of blocks based on his dimensions

    Args:
        dimension (array): the three dimension of lego (width, length and height)
        nome (String): the name of classification for lego
        v3 (array): the cordinate of rightmost points (we correct this if the block is perfectly perpendicular to the camera)

    Returns:
        string,array,array: the name of lego, the dimension and the cordinate of rightmost points
    """
    split = nome.split("-")

    #Correction name with height 
    if(dimension[2] >= 0.04 and dimension[2] < 0.045):
        split[2] = "Z1"
    elif(dimension[2] >= 0.06 and dimension[2] < 0.07):
        split[2] = "Z2"
    elif(dimension[2] >= 0.07):
        split[1] = "Y" + str(int(dimension[2]/0.035)+1)

    #Correction dimension   correction case of block is in the same direction of the camera and general correction dimension
    if(dimension[1] < 0.01):
        if(isUp):
            if(dimension[0] < DIM_BLOCK):
                dimension[1] = DIM_BLOCK * int(split[1][1])
            else:
                dimension[1] = DIM_BLOCK
        else:
            if((dimension[0] >= 0.06 and dimension[0] < 0.07) or (dimension[0] >= 0.04 and dimension[0] < 0.045)):
                dimension[1] = dimension[1] = DIM_BLOCK * int(split[1][1])
            else:
                if(int(split[2][1]) == 1):
                    dimension[1] = 0.041
                else:
                    dimension[1] = 0.065  
        v3[1] = dimension[1]  
        #correggo ordine di dimension
        if(dimension[1] > dimension[0]):
            tmp = dimension[1]
            dimension[1] = dimension[0]
            dimension[0] = tmp

    if(int(split[1][1]) != int(dimension[0]/DIM_BLOCK)):
        print("ERRORE DIM")
    if(int(split[0][1]) != int(dimension[1]/DIM_BLOCK)):
        print("ERRORE DIM")

    if(len(split)==4):
        retName = split[0] + "-" + split[1] + "-" + split[2] + "-" + split[3]
    else:
        retName = split[0] + "-" + split[1] + "-" + split[2]
    
    return retName,dimension,v3


def lego_processing(actual_detection,posizioni,results_data):  
    """this function take the point of Pointcloud and calculate the position and rotation of the lego and made some corrections

    Args:
        actual_detection (_type_): _description_
        posizioni (_type_): _description_
        results_data (_type_): _description_

    Returns:
        string, : _description_
    """

    orientation = 0   #0 normale 1 girato 2 appoggiato a terra 3 appoggiato in piedi
    dimension = [0, 0, 0]   #lato lungo, lato corto e altezza
    v1 = [0,0,0]   #left point of block
    v2 = [0,0,0]   #lowest point of blocks
    v3 = [0,0,0]   #right point of block


    yleft = 0
    yright = 100000
    xmin = 100000

    zmax = 0
        
    for pos in posizioni:
        if(pos[2] > 0.875):  #find the three point of blocks
            if(pos[1] > yleft):
                yleft = pos[1]
                v1 = np.copy(pos)
            if(pos[1] < yright):
                yright = pos[1]
                v3 = np.copy(pos)
            if(pos[0] < xmin):
                xmin = pos[0]
                v2 = np.copy(pos)

        if(pos[2]>zmax):    #find z max
                zmax = pos[2]
    

    dimension = find_dimension(v1,v2,v3,zmax)
    print("PRIMA" + str(dimension))
    nome,dimension,v3 = correction(dimension, results_data["name"][actual_detection],v3)

    

    pos,rot = find_orientation(dimension,v1,v2,v3)
    print("DOPO" + str(dimension))
    print("Correzione: " + results_data["name"][actual_detection] + " --> " + nome)
    


    print("pos : " + str(pos))
    print("rot : " + str(rot))

    initial_pose = Pose()
    initial_pose.position.x = pos[0]
    initial_pose.position.y = pos[1]
    initial_pose.position.z = 0.89              #0.89

    q = quaternion_from_euler(rot[0], rot[1], rot[2])

    initial_pose.orientation.x = q[0]
    initial_pose.orientation.y = q[1]
    initial_pose.orientation.z = q[2]
    initial_pose.orientation.w = q[3]

    return nome,initial_pose


def receive_pointcloud(results_data):
    """For each pixel of the list created by recognition(), this function takes all the corresponding points of the pointcloud  

    Args:
        results_data (ArrayList): the results of the recognition: class and bounding box's vertices
    """

    msg = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)
    # read all the points
    #array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
    pos_zed = [-0.9 ,0.24 ,-0.35]  # y forse 0.18
    pos_base_link = np.array([0.5,0.35,1.75])

    points_list = []
    for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=Objects):
        points_list.append([data[0], data[1], data[2]])
    
    cont = 1
    actual_detection = 0
    actual_lego =[]
    
    for data in points_list:
        alpha = -0.523
        # Ry = np.matrix([[cos(alpha),0,sin(alpha)],
        #             [0,1,0],
        #             [-sin(alpha),0,cos(alpha)]])
        Ry = np.matrix([[ 0.     , -0.49948,  0.86632],
                        [-1.     ,  0.     ,  0.     ],
                        [-0.     , -0.86632, -0.49948]])

        data_zed_rotation = np.array( data * Ry ) * -1
        
        data_base_link = np.array(data_zed_rotation.tolist()[0]) + np.array(pos_zed)

        data_world = Ry.dot(data) + pos_zed + pos_base_link
        data_world = np.array(data_world)

        if(actual_detection>len(point_count_for_item)-1):
            break
        if(cont >= point_count_for_item[actual_detection]):
            print("\nIMMAGINE " + str(actual_detection+1) + " --> " + results_data["name"][actual_detection])

            nome,initial_pose = lego_processing(actual_detection,actual_lego,results_data)

            list.append(legoDetection(nome,initial_pose))
            actual_detection = actual_detection +1
            cont = 1
            actual_lego = []
        
        cont = cont +1
        actual_lego.append(data_world[0])


def recognition():
    """This function use the photo saved before and recognizes lego with YOLOv5, then it saves all the inner points of the bounding box into a list  
    """
    model = torch.hub.load(YOLO_PATH, 'custom', path=BEST_PATH, source='local')  # local repo
    
    img = cv2.imread(LAST_PHOTO_PATH)[..., ::-1]
    
    results = model([img], size=640) # batch of images
    
    # Results
    results.print()  
    results.save() 
    results.render()
    results.show()
    

    results_data = results.pandas().xyxy[0]  # im1 predictions (pandas)
    print(results_data)
    for k in range(0,results_data.shape[0]):
        
        #Allargo scontorno
        results_data.loc[k,"ymin"] -= 10
        results_data.loc[k,"ymax"] += 10
        results_data.loc[k,"xmin"] -= 10
        results_data.loc[k,"xmax"] += 10

        ymin = int(results_data.ymin[k])
        ymax = int(results_data.ymax[k])
        xmin = int(results_data.xmin[k])
        xmax = int(results_data.xmax[k])
        
        cont = 0
        if(results_data.confidence[k]<0.5):
            continue
        for j in range(int(results_data.ymin[k]),int(results_data.ymax[k])):
            for i in range(int(results_data.xmin[k]),int(results_data.xmax[k])):
                tupla=(i,j)
                Objects.append(tupla)
                cont = cont +1 
        point_count_for_item.append(cont)

    receive_pointcloud(results_data)


def receive_image(msg):
    """This function take a photo with ZED2, cropped it and finally save the result

    Args:
        msg (_type_): zed message
    """
    #msg = rospy.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)
    rgb = CvBridge().imgmsg_to_cv2(msg, "bgr8")



    #Creo la mashera
    table = [[558*1.5, 278*1.5], [450*1.5, 590*1.5], [970*1.5,610*1.5], [777*1.5, 267*1.5]]       #sim
    #table = [[310, 180], [210, 410], [800, 410], [650, 180]]
    mask = np.array(table, dtype=np.int32)

    background = np.zeros((rgb.shape[0], rgb.shape[1]), np.int8)
    cv2.fillPoly(background, [mask],255)
    mask_background = cv2.inRange(background, 1, 255)

    img = cv2.bitwise_and(rgb, rgb, mask=mask_background)
    
    cv2.imwrite(LAST_PHOTO_PATH, img)
    
    #print("IMMAGINE ACQUISITA")



if __name__ == '__main__':

    rospy.init_node('custom_joint_pub_node')
    msg = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback = receive_image, queue_size=1)
    loop_rate = rospy.Rate(1.)
    while True:
        loop_rate.sleep()
        break
        pass
    recognition()

    message = legoGroup("Assigment 1",list)   

    pub.publish(message)
