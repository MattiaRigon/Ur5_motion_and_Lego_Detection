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
import math
import numpy as np

from time import sleep
import xml.etree.ElementTree as ET

from math import pi
from std_msgs.msg import String
from spawnLego_pkg.msg import legoDetection
from spawnLego_pkg.msg import legoGroup
import os

pub = rospy.Publisher('lego_position', legoGroup, queue_size=10)


models_path = os.path.dirname(os.path.abspath(__file__))
models_str = "models"
models_str = models_str [::-1]
src_str = "src"
src_str = src_str[::-1]
models_path = models_path[::-1].replace(src_str, models_str, 1)[::-1]

models = ["X1-Y1-Z2", "X1-Y2-Z1", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET", "X1-Y2-Z2", "X1-Y3-Z2", "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2-FILLET", "X2-Y2-Z2", "X1-Y3-Z2-FILLET"] 
cont = 0
colorList = ['Gazebo/Indigo', 'Gazebo/Gray', 'Gazebo/Orange','Gazebo/Red', 'Gazebo/Purple', 'Gazebo/SkyBlue','Gazebo/DarkYellow',  'Gazebo/Green']

list = []

spawned_lego = []

models_spawned = []

def get_random_model():
	"""this function return me a random lego block's class

	Returns:
	string : the name of the lego model 
	"""
	return random.choice(models)

def spawn_model(model, pos, name=None, ref_frame='world'):
	"""This function spawns the model in the position given 

	Args:
	model (string): the name of the lego model
	pos (struct): it contains pall the parameters for the position of the block and his orientation too
	name (string, optional): the name of the model. Defaults to None.
	ref_frame (string, optional): the reference frame. Defaults to 'world'.

	Returns:
	_type_: _description_
	"""
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
	"""generates a random number 

	Args:
	min (int): the minimum number
	max (int): the maximum number

	Returns:
	int : the random number generated
	"""
	num = round(random.uniform(min, max), 2)
	return num

def random_position(rotation = False):
	"""It generates a random position in the spawning zone, and if rotation=True it generates a random rotation too

	Args:
	rotation (bool, optional): if is equal to True, it generates also a random rotation. Defaults to False.

	Returns:
	Pose: _description_
	"""
	s = lego.split("-")

	r = int(s[1][1])*1.5*0.01

	while 1:
		x = randNum(0.03, 0.4)   #0.42 + 0.5 
		y = randNum(0.35, 0.74)   #0.2 + 0.35
		if x - r >= 0 or y + r <= 0.75:
			break
	z = 0.91

	initial_pose = Pose()
	initial_pose.position.x = x
	initial_pose.position.y = y
	initial_pose.position.z = z
	# se metti pi/2 sulla x sono sorti ma in piedi 
	# con pi/2 sulla y sono storti ma di lato
	if(rotation):
		num = randNum(0, 2)
		if num == 0:
			q = quaternion_from_euler(pi/2,0,randNum(0,2*pi))
		elif num == 1:
			q = quaternion_from_euler(0,pi/2,randNum(0,2*pi))
		elif num == 2:
			q = quaternion_from_euler(pi/2,pi/2,randNum(0,2*pi))
	else:
		q = quaternion_from_euler(0, 0,randNum(0,2*pi))

	initial_pose.orientation.x = q[0]
	initial_pose.orientation.y = q[1]
	initial_pose.orientation.z = q[2]
	initial_pose.orientation.w = q[3]

	print(initial_pose)

	return initial_pose



def changeModelColor(model_xml, color):
	"""it changes the color of model

	Args:
		model_xml (xml): xml of model
		color (string): color to apply

	Returns:
		string: color
	"""
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')	

def check_sovrapposizioni(pos, lego):
	"""this function check if there is conflict in spawn with other lego

	Args:
		pos (array): position of lego
		lego (_type_): lego

	Returns:
		_type_: _description_
	"""
	s = lego.split("-")

	r = int(s[1][1])*1.5*0.01
	print("\n\n" + lego + " -> y = " + str(r) + "\n\n")

	if not spawned_lego: #empty list
		spawned_lego.append(pos)
		rint("Qui può spawnare \n")
		return False
	else:
		for p in spawned_lego:
			if (p.position.x - pos.position.x)**2 + (p.position.y - pos.position.y)**2 <= r:
				print("Qui c'è gia un blocchetto! \n")
				return True 
	spawned_lego.append(pos)
	print("Qui può spawnare \n")
	return False


if __name__ == "__main__":
	#rospy.init_node('turtlebot3_replay1')
	rospy.init_node("spawn")
	color = random.choice(colorList)
	print("Che assigment vuoi eseguire ? \n Inserire un numero tra 1 e 2 e 3 ")
	scelta = input("Scelta : ")
	if(scelta == '1'):	
		lego = get_random_model()
		pos=random_position()
		print(spawn_model(lego, pos))
		message = legoGroup("Assigment 1",list)   
		#pub.publish(message)
	elif(scelta =='2'):
		for i in range(0,5):
			count = 0
			while True:
				while True:
					lego = get_random_model()
					print("--> " + lego + "\n")
					for it in models_spawned:
						print(it + " ")
					if lego not in models_spawned:   #non possono esserci due lego della stessa classe
						break
				pos=random_position()
				if not check_sovrapposizioni(pos, lego): 
					print(spawn_model(lego, pos))
					i=i+1
					break
			models_spawned.append(lego)
			print("---------> spawnato")
		message = legoGroup("Assigment 2",list)   
		#pub.publish(message)
	elif(scelta =='3'):
		for i in range(0,1):
			while True:
				lego = "X1-Y4-Z2"#get_random_model()
				pos=random_position(rotation=True)
				if not check_sovrapposizioni(pos, lego): 
					print(spawn_model(lego, pos))
					i=i+1
			break
		message = legoGroup("Assigment 3",list)   
		#pub.publish(message)
	else :
		print("scelta sbagliata")




