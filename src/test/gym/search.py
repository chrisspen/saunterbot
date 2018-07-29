#!/usr/bin/env python
"""
Based on http://kvfrans.com/simple-algoritms-for-solving-cartpole/
"""
import random
import sys
import numpy as np
import gym
# env = gym.make('CartTable-v0')
# from carttableangular import CartTableAngularEnv; env = CartTableAngularEnv()


REWARD_THRESHOLD = 2000


def demo(env, parameters=None, keep=False, noise=0, eval_cls=None):
    run_episode(env, parameters, max_steps=REWARD_THRESHOLD*10, show=True, keep=keep, noise=noise, eval_cls=eval_cls)


def run_episode(env, parameters, max_steps=REWARD_THRESHOLD, show=False, keep=False, eval_cls=None, noise=0):
    if parameters is None:
        parameters = np.random.rand(4) * 2 - 1
    observation = env.reset()
    env.verbose = show
    totalreward = 0
    action = 0
    evaluator = None
    if eval_cls:
        evaluator = eval_cls(*parameters)
    for _ in range(max_steps):
        
        # Get the action.
        if evaluator:
            action = evaluator(observation)
        else:
            # print('actions:', env.actions)
            if env.actions == 2:
                action = 0 if np.matmul(parameters, observation) < 0 else 1
            elif env.actions == 3:
                # Assuming each value is bounded between [-1:+1], then we divide range into three equal sub-ranges [-1:-0.33], [-.33,+.33], [+.33,+1]
                _v = np.matmul(parameters, observation)/float(len(parameters))
                # print('v:', _v)
                if _v < -0.001:
                    action = 0
                elif _v < +0.001:
                    action = 1
                else:
                    action = 2
            else:
                raise NotImplementedError
        
        # If we're simulating slippage and noise, then randomly flip the action if noise exceeds the given threshold.
        if noise and random.random() <= noise:
            action = 1 - action
            
        # action = 1 #TODO:revert
        # action = _ % 2 #TODO:revert
        # action = int(bool(_ % 2)) #TODO:revert
        observation, reward, done, info = env.step(action)
        totalreward += reward
        if show:
            env.render()
        if show:
            print('done:', done)
            print('keep:', keep)
        if done and not keep:
            break
    if show:
        print('totalreward:', totalreward)
    return totalreward

def run_episodes(*args, **kwargs):
    """
    Executes run_episode() N times and returns the average reward.
    """
    n = kwargs.pop('n', 1)
    totalreward = 0
    for _ in range(n):
        totalreward += run_episode(*args, **kwargs)
    return totalreward/float(n)

def random_search(env, max_episodes=10000):
    bestparams = []
    bestreward = 0
    for cnt in range(max_episodes):
        sys.stdout.write(('\rEvaluating %i of %i (best reward=%s => %s)' % (cnt, max_episodes, bestreward, list(bestparams))).ljust(100))
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
            if reward >= REWARD_THRESHOLD*2:
                break

    print()
    print('bestreward:', bestreward)
    print('bestparams:', list(bestparams))
    return bestreward, bestparams, cnt


# def hill_search(env):
    # noise_scaling = 0.1  
    # bestparams = np.random.rand(4) * 2 - 1  
    # bestreward = 0  
    # max_episodes = 10000
    # for cnt in range(max_episodes):  
        # sys.stdout.write(('\rEvaluating %i of %i (best reward=%s => %s)' % (cnt, max_episodes, bestreward, list(bestparams))).ljust(100))
        # sys.stdout.flush()
        # newparams = bestparams + (np.random.rand(4) * 2 - 1)*noise_scaling
        # reward = run_episode(env, newparams)
        # if reward > bestreward:
            # bestreward = reward
            # bestparams = newparams
            # if reward >= REWARD_THRESHOLD:
                # break

    # print()
    # print('bestreward:', bestreward)
    # print('bestparams:', list(bestparams))
    # return bestreward, bestparams, cnt


def multi_hill_search(env, max_episodes=10000, seed=None, reward_threshold=REWARD_THRESHOLD, eval_cls=None):
    noise_scaling = 0.1
    bestparams = []
    bestreward = 0
    n = 3
    
    # Create initial population.
    population = 10
    param_pop = [] # [(reward, params)]
    if seed:
        reward = run_episodes(env, seed, n=n, eval_cls=eval_cls)
        param_pop.append((reward, seed))
    for _y in range(population):
        if eval_cls:
            params = eval_cls.get_random()
            assert params is not None
        else:
            params = np.random.rand(4) * 2 - 1
        reward = run_episodes(env, params, n=n, eval_cls=eval_cls)
        param_pop.append((reward, params))

    _max_episodes = int(max_episodes/population)
    for cnt in range(_max_episodes):
        sys.stdout.write(('\rEvaluating %i of %i (best reward=%s => %s)' % (cnt, _max_episodes, bestreward, list(bestparams))).ljust(100))
        sys.stdout.flush()
    
        new_param_pop = [] # [(reward, params)]
    
        # Crossover.
        # print('param_pop:', param_pop)
        for _y in range(population-1):
            oldparams = random.choice(param_pop)[1]
            if eval_cls:
                newparams = eval_cls.get_crossover(oldparams)
            else:
                newparams = oldparams + (np.random.rand(4) * 2 - 1) * noise_scaling
            newreward = run_episodes(env, newparams, n=n, eval_cls=eval_cls)
            new_param_pop.append((newreward, newparams))
        
        # Mutation.
        if eval_cls:
            newparams = eval_cls.get_random()
        else:
            newparams = np.random.rand(4) * 2 - 1
        newreward = run_episodes(env, newparams, n=n, eval_cls=eval_cls)
        new_param_pop.append((newreward, newparams))

        # Discrimination.
        param_pop = param_pop + new_param_pop
        param_pop.sort(key=lambda o: (o[0], list(o[1])))
        param_pop = param_pop[-population:]

        # Check for end condition.
        bestreward, bestparams = param_pop[-1]
        if bestreward >= reward_threshold:
            break

    print()
    print('bestreward:', bestreward)
    print('bestparams:', list(bestparams))
    return bestreward, bestparams, cnt
