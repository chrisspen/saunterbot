"""
Based on http://kvfrans.com/simple-algoritms-for-solving-cartpole/
"""
import random
import sys
import numpy as np
import gym
env = gym.make('CartTable-v0')
# from carttable import CartTableEnv; env = CartTableEnv()

FINISHED_STEPS = 1000

def demo(parameters=None):
    # env = CartTableEnv()
    run_episode(env, parameters, max_steps=FINISHED_STEPS*10, show=True)


def run_episode(env, parameters, max_steps=FINISHED_STEPS, show=False):
    if parameters is None:
        parameters = np.random.rand(4) * 2 - 1
    observation = env.reset()
    totalreward = 0
    for _ in range(max_steps):
        action = 0 if np.matmul(parameters, observation) < 0 else 1
        observation, reward, done, info = env.step(action)
        totalreward += reward
        if show:
            env.render()
        if done:
            break
    if show:
        print('totalreward:', totalreward)
    return totalreward


def random_search():
    # env = CartTableEnv()
    bestparams = None  
    bestreward = 0
    max_episodes = 10000
    for _ in range(max_episodes):
        sys.stdout.write('\rEvaluating %i of %i' % (_, max_episodes))
        sys.stdout.flush()
        parameters = np.random.rand(4) * 2 - 1
        reward = run_episode(env, parameters)
        if reward > bestreward:
            bestreward = reward
            bestparams = parameters
            # considered solved if the agent lasts 200 timesteps
            if reward == FINISHED_STEPS:
                break

    print()
    print('bestreward:', bestreward)
    print('bestparams:', list(bestparams))
    return bestreward, bestparams


score, params = random_search()

# Perfect.
# params = [0.41209637496354823, 0.25646063685734277, 0.38971277755298406, 0.6932977114472907]

# Oscillating
# params = [0.08103526868575472, 0.258454589596842, 0.207211081933806, 0.9400320442911136]

demo(parameters=params)
