import gym_gazebo_dang
import gym
import matplotlib.pyplot as plt
import cv2 

env = gym.make('dang-v0')
print('Run gym gazebo example')
for i in range(3):
	print('Starting run episode:', i)
	observation = env.reset()
	cv2.imwrite('image_'+str(i)+'.png',observation)