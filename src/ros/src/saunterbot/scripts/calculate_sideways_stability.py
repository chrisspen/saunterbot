#!/usr/bin/env python
"""
CS 2018.8.19
Calculates the stability of the platform given leg extension positions in 2 dimensions.
"""
from __future__ import print_function

import unittest
from math import *

# All lengths in cm

# We assume center of gravity is in the exact center of the body box.
body_width = 14
body_height = 20

leg_length_full_extension = 22.5
leg_length_full_retraction = 13.5

def calculate_sideways_stability(left_length, right_length):
    leg_diff_cm = abs(left_length - right_length)
    #lean_left = left_length < right_length
    lean_angle_radians = atan(leg_diff_cm/float(body_width))
    print('lean_angle_radians:', lean_angle_radians, lean_angle_radians*180/pi)
    
    fatal_angle_radians = pi/2. - atan((min(left_length, right_length)+body_height/2.)/(body_width/2.))
    print('fatal_angle_radians:', fatal_angle_radians, fatal_angle_radians*180/pi)
    
    return lean_angle_radians <= fatal_angle_radians
    

class Tests(unittest.TestCase):
    
    def test_stability(self):
        self.assertEqual(calculate_sideways_stability(22.5, 22.5), True)
        self.assertEqual(calculate_sideways_stability(13.5, 22.5), False)
        self.assertEqual(calculate_sideways_stability(13.5, 13.5), True)
        self.assertEqual(calculate_sideways_stability(13.5, 14.5), True)
        self.assertEqual(calculate_sideways_stability(13.5, 17), True)
        self.assertEqual(calculate_sideways_stability(13.5, 18), False)

if __name__ == '__main__':
    unittest.main()
