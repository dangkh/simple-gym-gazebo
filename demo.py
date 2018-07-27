import gym_gazebo_dang
import gym
import matplotlib.pyplot as plt
import cv2 

env = gym.make('dang-v0')
print('Run gym gazebo example')
for i in range(3):
	print('Starting run episode:', i)
	observation = env.reset()
	# cv2.imwrite('image_'+str(i)+'.png',observation)
	for i in range(100):
		try:
			mode = input("Enter action: ")
			mode = int(mode)
			mode = mode if mode < 4 else 1
		except Exception as e:
			raise e
		observation, reward, done = env.step(mode)	
		cv2.imwrite('image_'+str(i)+'.png',observation)
