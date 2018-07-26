import gym
from gym import error, spaces, utils
from gym.utils import seeding
import rospy
import os
import signal
import subprocess
import time
from os import path
from std_srvs.srv import Empty
import random
import roslaunch
import sys

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
from cv_bridge import CvBridge, CvBridgeError
import cv2

import skimage as skimage
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer
import time



class DangEnv(gym.Env):
	metadata = {'render.modes': ['human']}

 	def __init__(self):
   	 	#start roscore
		subprocess.Popen(["roscore"])
		time.sleep(1)
		print ("Roscore launched!")

		# Launch the simulation with the given launchfile name
		rospy.init_node('gym', anonymous=True)
		self.launchfile = "/home/dangkieu/catkin_ws/src/dang_gazebo/launch/turtlebot3_world.launch"
		if not path.exists(self.launchfile):
			raise IOError(launchfile+" does not exist")

		subprocess.Popen(["roslaunch", self.launchfile])
		print ("Gazebo launched!")

		self.gzclient_pid = 0
		
		#recall service and topic
			
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)



	def config_launchfile(self, file_name):
		self.launchfile = file_name
		print ("Config file launch to:", file_name)

	def calculate_observation(self,data):
		min_range = 0.21
		done = False
		for i, item in enumerate(data.ranges):
			if (min_range > data.ranges[i] > 0):
				done = True
		return done

	def step(self, action):
		state = None # return image
		reward = -10000 # return value 
		done = False # stop or not

		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		if action == 0: #FORWARD
			vel_cmd = Twist()
			vel_cmd.linear.x = 0.2
			vel_cmd.angular.z = 0.0
			self.vel_pub.publish(vel_cmd)
		elif action == 1: #LEFT
			vel_cmd = Twist()
			vel_cmd.linear.x = 0.05
			vel_cmd.angular.z = 0.2
			self.vel_pub.publish(vel_cmd)
		elif action == 2: #RIGHT
			vel_cmd = Twist()
			vel_cmd.linear.x = 0.05
			vel_cmd.angular.z = -0.2
			self.vel_pub.publish(vel_cmd)    

		return state, reward, done

	def render(self, mode="human", close=False):
		if close:
			tmp = os.popen("ps -Af").read()
			proccount = tmp.count('gzclient')
			if proccount > 0:
				if self.gzclient_pid != 0:
					os.kill(self.gzclient_pid, signal.SIGTERM)
					os.wait()
			return

		tmp = os.popen("ps -Af").read()
		proccount = tmp.count('gzclient')
		if proccount < 1:
			subprocess.Popen("gzclient")
			self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
		else:
			self.gzclient_pid = 0

 	def close(self):
	# Kill gzclient, gzserver and roscore
		tmp = os.popen("ps -Af").read()
		gzclient_count = tmp.count('gzclient')
		gzserver_count = tmp.count('gzserver')
		roscore_count = tmp.count('roscore')
		rosmaster_count = tmp.count('rosmaster')

		if gzclient_count > 0:
			os.system("killall -9 gzclient")
		if gzserver_count > 0:
		    os.system("killall -9 gzserver")
		if rosmaster_count > 0:
		    os.system("killall -9 rosmaster")
		if roscore_count > 0:
		    os.system("killall -9 roscore")

		if (gzclient_count or gzserver_count or roscore_count or rosmaster_count >0):
			os.wait()

	def reset(self):
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
			#reset_proxy.call()
			self.reset_proxy()
		except (rospy.ServiceException) as e:
			print ("/gazebo/reset_simulation service call failed")

		# Unpause simulation to make observation
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		time.sleep(5)
		#TODO
		counter = 0
		data = None
		while counter <= 200 and data == None:
			counter += 1
			data = rospy.wait_for_message("/image_raw_dang", Image, timeout=10)
			image = cvtMsg_Img(data) 
		if counter > 200:
			print ("/image_raw_dang topic get image failed")
		#TODO

		observation = image

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		return observation


def cvtMsg_Img(data):
	try:
		cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	# (rows,cols,channels) = cv_image.shape
	# cv2.imshow("Image window", cv_image)
	# cv2.waitKey(0)
	return cv_image