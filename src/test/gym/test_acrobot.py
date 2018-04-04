import gym
env = gym.make('Acrobot-v1')
env.reset()

for i in range(1000):
    env.step(env.action_space.sample())
    env.render()
