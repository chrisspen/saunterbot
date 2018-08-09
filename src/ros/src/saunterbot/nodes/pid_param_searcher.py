#!/usr/bin/env python
"""
Waits for the platform to be uprighted, then enters new balance controller weight parameters and enables balancing.
Once the platform falls over, it records how long the balancing lasts, and uses this as the feedback parameter for a search of new parameters.

Run with:

    rosrun saunterbot param_searcher.py

"""
from __future__ import print_function
import os
import sys
from time import sleep, time
import random
from functools import partial

import rospy
from std_msgs.msg import Bool, Int16, Float32MultiArray, Empty

import yaml

import numpy as np

def here(*dirs):
    return os.path.join(os.path.dirname(__file__), *dirs)

DATA_DIR = os.path.abspath(here('../data'))

# Format: [r1,[p,i,d]]
DATA_FN = os.path.join(DATA_DIR, 'pid_balancing_params.yaml')

os.system('touch "%s"' % DATA_FN)

# Minimum seconds to balance in order to have found successful parameters.
REWARD_THRESHOLD = 60

NUMBER_OF_PARAMS = 3

# The highest value each parameter can take.
PARAM_SCALE = 50

class TimeoutException(Exception):
    pass


def wait_until(cb, timeout=30):
    """
    Waits until the given timeout until given callback returns true.
    """
    t0 = time()
    while time() - t0 < timeout:
        if cb():
            return
        sleep(0.1)
    raise TimeoutException


class ParamSearcherNode(object):

    def __init__(self):
        rospy.init_node("pid_param_searcher")

        # Initialize parameter database.
        self.data = []
        with open(DATA_FN) as fin:
            data = yaml.load(fin)
            if data:
                assert isinstance(data, list)
                assert len(data[0]) == 2
                self.data.extend(data)

        ########################################################################
        # Initialize state variables.



        ########################################################################
        # Subscribe to topics.

        self.balancing_enabled = None
        rospy.Subscriber('/torso_arduino/balancing/get', Bool, partial(self.on_data, key='balancing_enabled'))

        self.upright = None
        rospy.Subscriber('/torso_arduino/upright/get', Bool, partial(self.on_data, key='upright'))

        self.button = None # False=pressed, True=unpressed
        rospy.Subscriber('/torso_arduino/button', Bool, partial(self.on_data, key='button'))

        self.current_params = None
        rospy.Subscriber('/torso_arduino/hip/pid/get', Float32MultiArray, partial(self.on_data, key='current_params'))

        self.balance_button_enabled = None
        rospy.Subscriber('/torso_arduino/balancing/button/get', Bool, partial(self.on_data, key='balance_button_enabled'))

        ########################################################################
        # Setup publishers.

        self.upright_pub = rospy.Publisher('/torso_arduino/upright/set', Empty, queue_size=None)

        self.balancing_set_pub = rospy.Publisher('/torso_arduino/balancing/set', Bool, queue_size=None)

        self.balancing_button_set_pub = rospy.Publisher('/torso_arduino/balancing/button/set', Bool, queue_size=None)

        self.balancing_param_pub = rospy.Publisher('/torso_arduino/hip/pid/set', Float32MultiArray, queue_size=None)

        self.hip_set_pub = rospy.Publisher('/torso_arduino/servo/hip/set', Int16, queue_size=None)

        ########################################################################
        # Main process loop.

        rospy.on_shutdown(self.on_shutdown)

        self.multi_hill_search()

    def multi_hill_search(self, max_episodes=10000, population_size=10):
        """
        Uses a primitive genetic algorithm to search for optimal parameters.
        """

        noise_scaling = 0.1
        bestparams = []
        bestreward = 0
        current_population = [] # [(reward, params)]

        rospy.loginfo('Initializing population...')
        current_population.extend(sorted(self.data)[-population_size:])
        #while len(current_population) < population_size:
            #current_population.append(self.get_random_params())
        #for line in current_population:
            #print(line)
        #raw_input('enter')
        rospy.loginfo('Population initialized.')

        # Evaluate each member.
        _max_episodes = int(max_episodes/population_size)
        for cnt in range(_max_episodes):
            rospy.loginfo('Evaluating episode %i of %i (best reward=%s => %s)', cnt, _max_episodes, bestreward, list(bestparams))

            pending_params = []

            # Crossover.
            if current_population:
                for _y in range(population_size-1):
                    oldparams = random.choice(current_population)[1]
                    pending_params.append(list(oldparams + self.get_random_params() * noise_scaling))

            # Mutation.
            pending_params.append(self.get_random_params())

            # Evaluation.
            i = 0
            for newparams in pending_params:
                i += 1
                rospy.loginfo('Evaluating param %i of %i (best reward=%s => %s)', i, len(pending_params), bestreward, list(bestparams))
                newreward = self.run_episode(newparams)
                bestreward, bestparams = max((bestreward, bestparams), (newreward, newparams))
                self.save_feedback(params=newparams, reward=newreward)
                current_population.append([newreward, newparams])
                # Check for end condition.
                if bestreward >= REWARD_THRESHOLD:
                    rospy.loginfo('Passed reward threshold! %s', bestreward)
                    return

            # Discrimination.
            current_population.sort(key=lambda o: (o[0], o[1])) # best at end of list
            current_population = current_population[-population_size:]

    def run_episode(self, parameters):
        """
        Sets up a balancing session and measures the effectiveness of the given parameters.
        """
        assert len(parameters) == NUMBER_OF_PARAMS
        rospy.loginfo('Running episode for params: %s', str(parameters))

        rospy.loginfo('Disabling balancing button...')
        self.balance_button_enabled = None
        #self.balancing_button_set_pub.publish(Bool(False))#TODO:fix
        os.system('printf "data: False\n---\n" | rostopic pub --once /torso_arduino/balancing/button/set std_msgs/Bool > /dev/null')
        wait_until(lambda: self.balance_button_enabled is not None, timeout=10)
        assert not self.balance_button_enabled, 'Failed to disable balancing button.'
        rospy.loginfo('Balance button disabled.')

        # Upload parameters.
        rospy.loginfo('Setting parameters...')
        self.current_params = []
        #msg = Float32MultiArray()
        #msg.data = list(parameters)
        #self.balancing_param_pub.publish(msg)
        for _ in range(3):
            cmd = 'rostopic pub --once /torso_arduino/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:%s}"' \
                % list(parameters)
            print(cmd)
            os.system(cmd)
            wait_until(lambda: self.current_params, timeout=10)
            expected = [round(_, 4) for _ in parameters]
            actual = [round(_, 4) for _ in self.current_params]
            print('expected params:', expected)
            print('actual params:', actual)
            if expected == actual:
                break
        assert expected == actual, 'Unable to set parameters!'
        rospy.loginfo('Parameters set.')

        # Tell user to upright platform.
        self.upright = None
        self.upright_pub.publish(Empty())
        wait_until(lambda: self.upright is not None, timeout=10)
        if not self.upright:
            rospy.loginfo('ATENTION USER, please upright platform.')
        while not self.upright and not rospy.is_shutdown():
            if self.upright:
                break
            sleep(0.1)
        if rospy.is_shutdown():
            sys.exit(1)

        # Wait for user to press signal button.
        if self.button:
            rospy.loginfo('ATENTION USER, press button.')
            while self.button and not rospy.is_shutdown():
                if not self.button:
                    break
                sleep(0.1)
        if rospy.is_shutdown():
            sys.exit(1)
        
        # Ensure legs are reset to starting position.
        #os.system('rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1500')
        self.center_legs()

        # Wait for user to release button to begin balancing.
        rospy.loginfo('ATENTION USER, release button.')
        while not self.button and not rospy.is_shutdown():
            if self.button:
                break
            sleep(0.001)
        if rospy.is_shutdown():
            sys.exit(1)

        # Enable balancing.
        self.balancing_enabled = None # Clear old value so we know when we get a confirmation.
        self.balancing_set_pub.publish(Bool(True))

        # Record roughly when balancing started.
        t0 = time()

        # Wait for balancing enabling to be confirmed.
        wait_until(lambda: self.balancing_enabled is not None, timeout=10)
        assert self.balancing_enabled, 'Failed to enable balancing.'

        rospy.loginfo('Waiting until platform has toppled...')
        try:
            wait_until(lambda: not self.upright, timeout=REWARD_THRESHOLD)
            total_time = time() - t0
            rospy.loginfo('total_time: %s', total_time)
            self.center_legs()
            return total_time
        except TimeoutException:
            return REWARD_THRESHOLD

    def center_legs(self):
        os.system('printf "data: 1500\n---\n" | rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 > /dev/null')

    def get_random_params(self):
        """
        Generates random parameters.

        Each parameter is a value between -16 and +16.
        """
        v = np.random.rand(NUMBER_OF_PARAMS) * PARAM_SCALE * 2 - PARAM_SCALE
        return v

    def on_shutdown(self):
        #os.system('rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1500')
        self.center_legs()

    def on_data(self, msg, key):
        #rospy.loginfo('on_%s: %s', key, msg.data)
        assert hasattr(self, key), 'Invalid state key: %s' % key
        setattr(self, key, msg.data)

    def save_feedback(self, params, reward):
        """
        Records (params, reward) feedback tuple, used for training.
        """
        if hasattr(params, 'tolist'):
            params = params.tolist()
        params = [float(_) for _ in params]
        assert len(params) == NUMBER_OF_PARAMS
        with open(DATA_FN, 'a') as fout:
            print(yaml.dump([[reward, params]], width=1000), file=fout)
        self.data.append([reward, params])

if __name__ == "__main__":
    ParamSearcherNode()
