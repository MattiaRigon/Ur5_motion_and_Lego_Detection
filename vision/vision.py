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

#pub = rospy.Publisher('lego_position', legoGroup, queue_size=10)

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
list = []
DIM_BLOCK = 0.03
class_list = ["X1-Y1-Z2","X1-Y2-Z1","X1-Y2-Z2","X1-Y1-Z2-CHAMFER","X1-Y1-Z2-TWINFILLET","X1-Y3-Z2","X1-Y1-Z2-FILLET","X1-Y4-Z1","X1-Y4-Z2","X2-Y2-Z2","X2-Y2-Z2-FILLET"]

def isUp(h):
    ret = True
    if(h < 0.04):
        ret = False
    return ret


def distanza(p1,p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def find_dimension(v1,v2,v3,zmax):
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


def find_orientation(dimension,v1,v1_1,v2,v3,v3_1):

    if(not(isUp(dimension[2]))):
        print("Posizione --> sono appoggiato in un fianco")
        print(distanza(v1,v1_1))
        print(distanza(v3,v3_1))
        if(distanza(v1,v1_1)>0.01):
            if(v1[0]>v1_1[0]):
                print("ho il pisello a sinistra in basso")
            else:
                print("ho il pisello a sinistra in alto")
        elif(distanza(v3,v3_1)>0.01):
            if(v1[0]>v1_1[0]):
                print("ho il pisello a destra in basso")
            else:
                print("ho il pisello a destra in alto")
    # elif(dimension[2] >= 0.04 and dimension[2] < 0.045):
    #     print("sono un Z1 in piedi")
    # elif(dimension[2] >= 0.06 and dimension[2] < 0.07):
    #     print("sono un Z2 in piedi")
    elif(dimension[2] >= 0.07):
        print("sono un Y" + str(int(dimension[2]/0.035)+1) + " in piedi")
        print(distanza(v1,v1_1))
        print(distanza(v3,v3_1))
        if(distanza(v1,v1_1)>0.01):
            if(v1[0]>v1_1[0]):
                print("ho il pisello a sinistra in basso")
            else:
                print("ho il pisello a sinistra in alto")
        elif(distanza(v3,v3_1)>0.01):
            if(v1[0]>v1_1[0]):
                print("ho il pisello a destra in basso")
            else:
                print("ho il pisello a destra in alto")


def correction(dimension,nome):

    #altezza di un blocchetto Y1 piegato a terra = 0.035000936615467104
    #altezza di un blocchetto Z2 in piedi = 0.06100125036597259
    #altezza di un blocchetto Z1 in piedi = 0.04200111413598073
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
    
    return retName,dimension


def trova_posizione_lego(actual_detection,posizioni,results_data):  

    orientation = 0   #0 normale 1 girato 2 appoggiato a terra 3 appoggiato in piedi
    dimension = [0, 0, 0]   #lato lungo, lato corto e altezza
    v1 = [0,0]   #left point of block
    v1_1 = [0,0] #leftmost point of block 
    v2 = [0,0]   #lowest point of blocks
    v3 = [0,0]   #right point of block
    v3_1 = [0,0] #rightmost point of block

    yleft = 0
    yMaxleft = 0
    yright = 100000
    yMaxright = 100000
    xmin = 100000
    zmax = 0

    for pos in posizioni:
        if(pos[2] > 0.8661 and pos[2] < 0.872):  #find the three point of blocks
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
        
        if(pos[2] >= 0.872 and pos[2] <= 0.93): #find the three point in case the block is relaxed on one side
            if(pos[1] > yMaxleft):
                yMaxleft = pos[1]
                v1_1 = np.copy(pos)
            if(pos[1] < yMaxright):
                yMaxright = pos[1]
                v3_1 = np.copy(pos)

    dimension = find_dimension(v1_1,v2,v3_1,zmax)
    find_orientation(dimension,v1,v1_1,v2,v3,v3_1)
    print("PRIMA" + str(dimension))
    nome,dimension = correction(dimension, results_data["name"][actual_detection])
    print("DOPO" + str(dimension))
    print("Correzione: " + results_data["name"][actual_detection] + " --> " + nome)
    #print("Oggetto di dimension:\nLato lungo--> " + str(dimension[0]) + "\nLato corto--> " + str(dimension[1]) + "\nAltezza--> " + str(dimension[2]))


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
    
    rot = [0,0,alpha]

    print("pos : " + str(pos))
    print("rot : " + str(rot))

    initial_pose = Pose()
    initial_pose.position.x = pos[0]
    initial_pose.position.y = pos[1]
    initial_pose.position.z = 0.89              #0.89

    q = quaternion_from_euler(0, 0, alpha)

    initial_pose.orientation.x = q[0]
    initial_pose.orientation.y = q[1]
    initial_pose.orientation.z = q[2]
    initial_pose.orientation.w = q[3]

    return initial_pose


def receive_pointcloud(results_data):

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

            initial_pose = trova_posizione_lego(actual_detection,actual_lego,results_data)

            list.append(legoDetection(results_data["name"][actual_detection],initial_pose))
            actual_detection = actual_detection +1
            cont = 1
            actual_lego = []
        
        cont = cont +1
        actual_lego.append(data_world[0])


def drawAxis(img, p_, q_, color, scale):
    p = list(p_)
    q = list(q_)

    ## [visualization1]
    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
    ## [visualization1]


def getOrientation(pts, img):
    ## [pca]
    # Construct a buffer used by the pca analysis
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]

    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

    # Store the center of the object
    cntr = (int(mean[0,0]), int(mean[0,1]))
    ## [pca]

    ## [visualization]
    # Draw the principal components
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
    drawAxis(img, cntr, p1, (255, 255, 0), 1)
    drawAxis(img, cntr, p2, (0, 0, 255), 5)

    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
    ## [visualization]

    # Label with the rotation angle
    #label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
    #textbox = cv.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
    #cv.putText(img, label, (cntr[0], cntr[1]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv.LINE_AA)

    return angle


def riconoscimento():
    model = torch.hub.load(YOLO_PATH, 'custom', path=BEST_PATH, source='local')  # local repo
    
    img = cv2.imread(LAST_PHOTO_PATH)[..., ::-1]
    
    results = model([img], size=640) # batch of images
    
    # Results
    results.print()  
    #results.save() 
    results.render()
    #results.show()
    

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

        img1 = img[ymin : ymax , xmin : xmax]
        cv2.imwrite("pp.jpg",img1)
        img1 = cv2.imread("pp.jpg")
        # caricamento dell'immagine da utilizzare
        gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
 
        # Convert image to binary
        _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        
        # Find all the contours in the thresholded image
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        
        for i, c in enumerate(contours):
        
            # Calculate the area of each contour
            area = cv2.contourArea(c)
            
            if((ymax-ymin)*(xmax-xmin)-area<1000):
                continue
            # Ignore contours that are too small or too large
            # if area < 3700 or 100000 < area:
            #     continue
            
            # Draw each contour only for visualisation purposes
            cv2.drawContours(img1, contours, i, (0, 0, 255), 2)
            #getOrientation(c, img1)
            
            cv2.imwrite("pp.jpg",img1)
        
    exit(0)  
        
    #     cont = 0
    #     if(results_data.confidence[k]<0.5):
    #         continue
    #     for j in range(int(results_data.ymin[k]),int(results_data.ymax[k])):
    #         for i in range(int(results_data.xmin[k]),int(results_data.xmax[k])):
    #             tupla=(i,j)
    #             Objects.append(tupla)
    #             cont = cont +1 
    #     point_count_for_item.append(cont)

    # receive_pointcloud(results_data)


def receive_image(msg):

    #msg = rospy.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)
    rgb = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    #Creo la mashera
    table = [[558*1.5, 278*1.5], [450*1.5, 590*1.5], [970*1.5,610*1.5], [777*1.5, 267*1.5]]
    mask = np.array(table, dtype=np.int32)

    background = np.zeros((rgb.shape[0], rgb.shape[1]), np.int8)
    cv2.fillPoly(background, [mask],255)
    mask_background = cv2.inRange(background, 1, 255)

    img = cv2.bitwise_and(rgb, rgb, mask=mask_background)
    
    cv2.imwrite(LAST_PHOTO_PATH, img)
    
    print("IMMAGINE ACQUISITA")

    


if __name__ == '__main__':

    rospy.init_node('custom_joint_pub_node')
    msg = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback = receive_image, queue_size=1)
    loop_rate = rospy.Rate(1.)
    while True:
        loop_rate.sleep()
        break
        pass
    riconoscimento()
    #sub_pointcloud = rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, callback = receive_pointcloud, queue_size=1)
    #sub_image = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback = receive_image, queue_size=1)
    #Take Zed picture
    #receive_image(msg)
    #recognition models
    #riconoscimento()

    #message = legoGroup("Assigment 1",list)   

    #pub.publish(message)

    #pub.publish(message)
