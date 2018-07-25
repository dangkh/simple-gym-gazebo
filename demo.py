import gym_gazebo_dang
import gym

env = gym.make('dang-v0')
print('Run gym gazebo example')
for i in range(3):
	print('Starting run episode:', i)