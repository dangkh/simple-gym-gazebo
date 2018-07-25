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

import skimage as skimage
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer




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

		#TODO
		#TODO
		#TODO
		#TODO
		#TODO
		#TODO
		#TODO



	def config_launchfile(self, file_name):
		self.launchfile = file_name
		print ("Config file launch to:", file_name)

	def step(self, action):
		state = None # return image
		reward = -10000 # return value 
		done = False # stop or not

		return state, reward, done

	def _render(self, mode="human", close=False):
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

 	def _close(self):
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