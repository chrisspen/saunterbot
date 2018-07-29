#!/usr/bin/env python
"""
Based on http://kvfrans.com/simple-algoritms-for-solving-cartpole/
"""

import random
import time
from math import pi

import numpy as np

from carttableangular import CartTableAngularEnv
from search import multi_hill_search, demo

# MAXRANGE = 2
MAXRANGE = 100

class PID(object):
    
    def __init__(self, p, i, d, expected=0):
        self.p = p
        self.i = i
        self.d = d
        self.p_term = 0
        self.i_term = 0
        self.d_term = 0
        self.expected = expected
        self.last_time = None
        self.last_actual = None
    
    @classmethod
    def get_random(cls):
        # return [random.random(), random.random(), random.random()]
        return np.random.rand(3) * MAXRANGE - MAXRANGE/2
        # return [random.random(), 0, 0]
    
    @classmethod
    def get_crossover(cls, oldparams, noise_scaling=0.1):
        # print('oldparams:', oldparams)
        newparams = oldparams + (np.random.rand(3) * MAXRANGE - MAXRANGE/2) * noise_scaling
        return newparams
    
    def step(self, actual):
        value = 0
        this_time = time.time()
        if self.last_time:
            time_change = this_time - self.last_time
            
            error = self.expected - actual
            
            self.p_term = self.p * error
            self.i_term += self.i * error * time_change
            self.d_term = self.d * (actual - self.last_actual) / time_change
            
            value = self.p_term + self.i_term + self.d_term
            
        self.last_time = this_time
        self.last_actual = actual
        
        return value

    def __call__(self, obs):
        # print('obs:', obs)
        yeta, yeta_dot, theta, theta_dot = obs
        # theta = angle from foot from center => 0=perfectly upright
        # +90 = full CCW, -90 = full CW
        
        value = self.step(theta)
        # print('yeta deg:', yeta*180./pi)
        # print('theta deg:', theta*180./pi)
        # return 1 # rotate hip full CW
        # return 0 # rotate hip full CCW
        # if theta > 0:
            # return 1
        # return 0
        # Our domain expects a boolean output.
        if value >= 0:
            return 1
        return 0

if __name__ == '__main__':

    env = CartTableAngularEnv()

    seed = None
    seed = [19.883364107899475, -0.0001, 6.14581940379808]
    env.cog_offset = 0
    score, params, episodes = multi_hill_search(env=env, seed=seed, eval_cls=PID, reward_threshold=2000)
    try:
        demo(env, params, eval_cls=PID)
    except KeyboardInterrupt:
        print()
    print('bestscore:', score)
    print('bestparams:', list(params))
    print('episodes:', episodes)
