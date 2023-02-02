#!/usr/bin/python3
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

pub = rospy.Publisher('lego_position', legoGroup, queue_size=10)


models_path = "/home/mattia/ros_ws/src/locosim/ros_impedance_controller/worlds/models/models"
models = ["X1-Y1-Z2", "X1-Y2-Z1", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET", "X1-Y2-Z2", "X1-Y3-Z2", "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2-FILLET", "X2-Y2-Z2"] 
cont = 0
colorList = ['Gazebo/Indigo', 'Gazebo/Gray', 'Gazebo/Orange','Gazebo/Red', 'Gazebo/Purple', 'Gazebo/SkyBlue','Gazebo/DarkYellow', 'Gazebo/White', 'Gazebo/Green']

list = []

def get_random_model():
    return random.choice(models)

def spawn_model(model, pos, name=None, ref_frame='world'):

	global cont,list

	if(name == None):
		name = model

	name = model + str(cont)
	cont = cont +1

	model_xml = models_path + "/" + model + "/model.sdf"
	model_xml = open(model_xml, 'r').read() 
	color = random.choice(colorList)

	if color is not None:

		model_xml = changeModelColor(model_xml, color)

	list.append(legoDetection(model,pos))
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)


	return spawn_model_client(model_name=name,model_xml=model_xml,initial_pose=pos,reference_frame=ref_frame)

def randNum(min, max):
    num = round(random.uniform(min, max), 2)
    return num

def random_position(rotation = False):

	x = randNum(0, 0.98)   #0.42 + 0.5 
	y = randNum(0.35, 0.79)   #0.2 + 0.35
	z = 0.866 

	initial_pose = Pose()
	initial_pose.position.x = x
	initial_pose.position.y = y
	initial_pose.position.z = z
	# se metti pi/2 sulla x sono sorti ma in piedi 
	# con pi/2 sulla y sono storti ma di lato
	if(rotation):
		q = quaternion_from_euler(0, pi/2, randNum(0,2*pi))
	else:
		q = quaternion_from_euler(0, 0,randNum(0,2*pi))

	initial_pose.orientation.x = q[0]
	initial_pose.orientation.y = q[1]
	initial_pose.orientation.z = q[2]
	initial_pose.orientation.w = q[3]


	print(initial_pose)

	return initial_pose



def changeModelColor(model_xml, color):
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')	



if __name__ == "__main__":
		
		

		#rospy.init_node('turtlebot3_replay1')
		rospy.init_node("spawn")
		color = random.choice(colorList)
		print("Che assigment vuoi eseguire ? \n Inserire un numero tra 1 e 2 e 3 ")
		scelta = input("Scelta : ")
		if(scelta == '1'):	
			print(spawn_model("X1-Y2-Z2",pos=random_position()))
			message = legoGroup("Assigment 1",list)   

			#pub.publish(message)

		elif(scelta =='2'):
			for i in range(0,5):
				print(spawn_model(get_random_model(),pos=random_position()))
				i=i+1
			message = legoGroup("Assigment 2",list)   
			#pub.publish(message)
		elif(scelta =='3'):
		
			print(spawn_model("X1-Y2-Z2",pos=random_position(rotation=True)))
			message = legoGroup("Assigment 3",list)   
			#pub.publish(message)

		else :
			print("scelta sbagliata")

		#sleep(5)
		#mov_lego()



