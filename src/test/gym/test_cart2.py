import gym
env = gym.make('CartPole-v0')
env.reset()

for i in range(1000):
    observation, reward, done, info = env.step(env.action_space.sample())
    env.render()
    # if done:
        # break
