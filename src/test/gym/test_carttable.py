#!/usr/bin/env python
"""
Based on http://kvfrans.com/simple-algoritms-for-solving-cartpole/
"""
import random
import sys
import numpy as np
import gym
# env = gym.make('CartTable-v0')
from carttable import CartTableEnv; env = CartTableEnv()

# REWARD_THRESHOLD = 2000
REWARD_THRESHOLD = 5000

def demo(parameters=None):
    # env = CartTableEnv()
    run_episode(env, parameters, max_steps=REWARD_THRESHOLD*10, show=True)


def run_episode(env, parameters, max_steps=REWARD_THRESHOLD, show=False):
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
            if reward >= REWARD_THRESHOLD:
                break

    print()
    print('bestreward:', bestreward)
    print('bestparams:', list(bestparams))
    return bestreward, bestparams


def hill_search():
    noise_scaling = 0.1  
    bestparams = np.random.rand(4) * 2 - 1  
    bestreward = 0  
    max_episodes = 10000
    for _ in range(max_episodes): 
        sys.stdout.write('\rEvaluating %i of %i' % (_, max_episodes))
        sys.stdout.flush() 
        newparams = bestparams + (np.random.rand(4) * 2 - 1)*noise_scaling
        reward = run_episode(env, newparams)
        if reward > bestreward:
            bestreward = reward
            bestparams = newparams
            if reward >= REWARD_THRESHOLD:
                break

    print()
    print('bestreward:', bestreward)
    print('bestparams:', list(bestparams))
    return bestreward, bestparams


def multi_hill_search():
    noise_scaling = 0.1
    bestparams = None#
    bestreward = 0  
    max_episodes = 10000
    
    # Create initial population.
    population = 10
    param_pop = [] # [(reward, params)]
    for _y in range(population):
        params = np.random.rand(4) * 2 - 1  
        reward = run_episode(env, params)
        param_pop.append((reward, params))

    for _ in range(int(max_episodes/population)):
        sys.stdout.write('\rEvaluating %i of %i' % (_, max_episodes))
        sys.stdout.flush()
    
        # Crossover.
        new_param_pop = [] # [(reward, params)]    
        for _y in range(population-1):
            oldparams = random.choice(param_pop)[1]
            newparams = oldparams + (np.random.rand(4) * 2 - 1) * noise_scaling
            newreward = run_episode(env, newparams)
            new_param_pop.append((newreward, newparams))
        
        # Mutation.
        newparams = np.random.rand(4) * 2 - 1
        newreward = run_episode(env, newparams)
        new_param_pop.append((newreward, newparams))

        # Discrimination.
        param_pop = param_pop + new_param_pop
        param_pop.sort(key=lambda o: (o[0], list(o[1])))
        param_pop = param_pop[-population:]

        # Check for end condition.
        bestreward, bestparams = new_param_pop[-1]
        if bestreward >= REWARD_THRESHOLD:
            break

    print()
    print('bestreward:', bestreward)
    print('bestparams:', list(bestparams))
    return bestreward, bestparams

# With reward+angle equal weight.
#   Average steps until solution is found = (6+11+16+65+20)/5.=23.6
#   Fatal = 1/5.
# With reward+angle*2 weight.
#   Average steps until solution is found = (48+27+4+40+11)/5.=26
#   Fatal = 2/5.
score, params = random_search()

# With reward+angle equal weight.
#   Average steps until solution is found = (9999+11+0+9999+9999)/5.=6002
#   Fatal = 5/5.
# score, params = hill_search()

# With reward+angle equal weight.
#   Average steps until solution is found = (33+18+19+6+9)/5.=17
#   Fatal = 2/5.
# With reward+angle*2 weight.
#   Average steps until solution is found = (99+28+0+4+5)/5.=27
#   Fatal = 2/5.
# score, params = multi_hill_search()

# Perfect off-center.
# params = [0.41209637496354823, 0.25646063685734277, 0.38971277755298406, 0.6932977114472907]

# Perfect upright.
# params = [0.08106616824157165, 0.014354243708667891, 0.3847436956637298, 0.21260153934738635]

# Oscillating
# params = [0.08103526868575472, 0.258454589596842, 0.207211081933806, 0.9400320442911136]

# params = [-0.5613529384922931, -0.2873107168263489, -0.3866124699733069, -0.2620484249535875]
demo(parameters=params)
