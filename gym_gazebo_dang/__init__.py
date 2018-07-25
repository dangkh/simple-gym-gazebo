from gym.envs.registration import register

register(
    id='dang-v0',
    entry_point='gym_dang_gazebo.envs:DangEnv',
)
