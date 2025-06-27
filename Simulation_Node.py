#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String
import random
import threading
from utils.error_logger import ErrorLogger

class SimulationNode:
    def __init__(self):
        rospy.init_node('simulation_node')
        self.namespace = rospy.get_param('~namespace', '/sentience')

        self.proximity_pub = rospy.Publisher(f'{self.namespace}/raw_sensors/proximity', Range, queue_size=10)
        self.state_pub = rospy.Publisher(f'{self.namespace}/world_model_state', String, queue_size=10)

        self.error_logger = ErrorLogger()
        self.rate = rospy.Rate(5)  # 5 Hz update rate

        rospy.loginfo("Simulation Node initialized.")

        self.thread = threading.Thread(target=self.simulate_loop)
        self.thread.daemon = True
        self.thread.start()

    def simulate_proximity(self):
        # Simulated range sensor data between 0.2m and 4.0m
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.5
        msg.min_range = 0.2
        msg.max_range = 4.0
        msg.range = random.uniform(0.2, 4.0)
        return msg

    def simulate_state(self):
        # Simulated state string, can be expanded
        states = ['idle', 'exploring', 'charging', 'processing']
        return random.choice(states)

    def simulate_loop(self):
        while not rospy.is_shutdown():
            try:
                prox_msg = self.simulate_proximity()
                self.proximity_pub.publish(prox_msg)

                state_msg = self.simulate_state()
                self.state_pub.publish(state_msg)

                rospy.loginfo(f"Published proximity: {prox_msg.range:.2f}m, state: {state_msg}")

                self.rate.sleep()
            except Exception as e:
                self.error_logger.log(f"Exception in simulate_loop: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SimulationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
