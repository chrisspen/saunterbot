"""
Waits for the platform to be uprighted, then enters new balance controller weight parameters and enables balancing.
Once the platform falls over, it records how long the balancing lasts, and uses this as the feedback parameter for a search of new parameters.

Run with:

    rosrun saunterbot param_searcher.py

"""
from time import sleep

import rospy
from std_msgs.msg import Bool

class ParamSearcherNode(object):
    
    def __init__(self):
        rospy.init_node("param_searcher")

        # Initialize parameter database.

        # Subscribe to balancer get topic.
        self.balancing_enabled = None
        rospy.Subscriber('/torso_arduino/balancing/get', Bool, on_balancing_get)
        
        # Subscribe to balancer set topic.

        while not rospy.is_shutdown():
            try:
                # Wait until platform has toppled.
                # If we were timing a weight, record weight and topple timer.
                # Get new pseudo random weights.
                # Upload new weights.
                # Tell user to upright platform.
                # Enable balancing.
                rospy.spinOnce()
                sleep(1)
            except KeyboardInterrupt:
                break

    def on_balancing_get(self, msg):
        self.balancing_enabled = msg.data

if __name__=="__main__":
    ParamSearcherNode()
