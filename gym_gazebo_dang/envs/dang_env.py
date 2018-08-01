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
import math

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
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
			
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

		## set position of goal 
		self.goal = [[-4.6, -0.01, 0],[3.95, 0.06, 0]]


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

	def reset_vel(self):
		vel_cmd = Twist()
		vel_cmd.linear.x = 0.0
		vel_cmd.angular.z = 0.0
		self.vel_pub.publish(vel_cmd)

	def rotate(self, value):
		# value = 1 LEFT, -1 RIGHT
		vel_cmd = Twist()
		PI = 3.1415926535897
		angular_speed = PI/16
		relative_angle = PI/8
		vel_cmd.linear.x=0
		vel_cmd.linear.y=0
		vel_cmd.linear.z=0
		vel_cmd.angular.x = 0
		vel_cmd.angular.y = 0
		vel_cmd.angular.z = angular_speed*value
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
		counter = 0
		while(current_angle < relative_angle):
			counter += 1
			self.publish_vel_cmd(vel_cmd, counter)
			t1=rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1-t0)
		vel_cmd.angular.z = 0
		self.publish_vel_cmd(vel_cmd, -1)

	def publish_vel_cmd(self, value_vel, counter):
		self.vel_pub.publish(value_vel)

	def step(self, action):
		observation = None # return image
		reward = -10000 # return value 
		done = False # stop or not

		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")
		distance = 1
		t0 = rospy.Time.now().to_sec()
		if action == 0: #FORWARD
			vel_cmd = Twist()
			vel_cmd.linear.x = -0.25
			vel_cmd.linear.y=0
			vel_cmd.linear.z=0
			vel_cmd.angular.x = 0
			vel_cmd.angular.y = 0
			speed = 1
			vel_cmd.angular.z = 0
			current_distance = 0
			counter = 0 
			while(current_distance < distance):
				self.publish_vel_cmd(vel_cmd, counter)
				t1=rospy.Time.now().to_sec()
				current_distance= speed*(t1-t0)
				counter += 1 
			vel_cmd.linear.x = 0
			self.publish_vel_cmd(vel_cmd, -1)
		elif action == 1: #LEFT
			self.rotate(1)
		elif action == 2: #RIGHT
			self.rotate(-1)
		# calculate done value using laser data
		data = None 
		while data == None :
			try:
				data = rospy.wait_for_message("/scan", LaserScan, timeout=10)
			except Exception as e:
				raise e

		done = self.calculate_observation(data)

		# retrieval image from camera in robot
		rospy.wait_for_service('/gazebo/pause_physics')
		# try:
		# 	self.pause()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/pause_physics service call failed")
		counter = 0
		data = None
		while counter <= 200 and data == None:
			counter += 1
			data = rospy.wait_for_message("/image_raw_dang", Image, timeout=10)
			image = cvtMsg_Img(data) 
		if counter > 200:
			print ("/image_raw_dang topic get image failed")
		
		
		done, reward = self.check_reward()
		observation = image
		print reward
		return observation, reward, done

	def check_reward(self):
		done = False
		reward = -10000
		data = rospy.wait_for_message("/odom", Odometry, timeout=10)
		xx = data.pose.pose.position.x
		yy = data.pose.pose.position.y
		for goalI in self.goal:
			vx = goalI[0]-xx
			vy = goalI[1]-yy
			tmp = math.sqrt(vx*vx + vy*vy)
			if tmp <= 0.5: 
				done = True
				reward = 10000
				break
		return done, reward

	def get_action_list(self):
		return self.action_list

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

## TO DO 
