import gym
env = gym.make('BipedalWalker-v2')
env.reset()

for i in range(1000):
    env.step(env.action_space.sample())
    env.render()
