"""
Based on http://kvfrans.com/simple-algoritms-for-solving-cartpole/
"""
import random
import sys
import numpy as np
import gym
# env = gym.make('CartTable-v0')
from carttableangular import CartTableAngularEnv; env = CartTableAngularEnv()

# REWARD_THRESHOLD = 2000
REWARD_THRESHOLD = 5000

def demo(parameters=None, keep=False):
    run_episode(env, parameters, max_steps=REWARD_THRESHOLD*10, show=True, keep=keep)


def run_episode(env, parameters, max_steps=REWARD_THRESHOLD, show=False, keep=False):
    if parameters is None:
        parameters = np.random.rand(4) * 2 - 1
    observation = env.reset()
    totalreward = 0
    action = 0
    for _ in range(max_steps):
        action = 0 if np.matmul(parameters, observation) < 0 else 1
        observation, reward, done, info = env.step(action)
        totalreward += reward
        if show:
            env.render()
        if done and not keep:
            break
    if show:
        print('totalreward:', totalreward)
    return totalreward


def random_search():
    bestparams = None  
    bestreward = 0
    max_episodes = 10000
    for _ in range(max_episodes):
        sys.stdout.write('\rEvaluating %i of %i' % (_, max_episodes))
        sys.stdout.flush()
        
        # Test each parameter set several times to see how it reacts to different inputs.
        reward_runs = 10
        reward_sum = 0
        for _ in range(reward_runs):
            parameters = np.random.rand(4) * 2 - 1
            reward = run_episode(env, parameters)
            reward_sum += reward
        reward = reward_sum/float(reward_runs)

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

# score, params = random_search()
# demo(params)

# score, params = hill_search()

# score, params = multi_hill_search()

demo([-0.6741055081288734, 0.12195637193897135, 0.0502067848682084, 0.46858856540807947],
    keep=True)

# Perfect with zero weight offset.
# demo([0.699945500154137, 0.9978736341836465, -0.02037230500207654, -0.4869280256800115])
