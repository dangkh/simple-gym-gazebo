from gym.envs.registration import register

register(
    id='dang-v0',
    entry_point='gym_gazebo_dang.envs:DangEnv',
)
