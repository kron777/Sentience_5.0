#!/usr/bin/env python3
import rospy
import json
import random
import time
from std_msgs.msg import String

class DriveSystemNode:
    def __init__(self):
        rospy.init_node('drive_system_node', anonymous=False)
        self.node_name = rospy.get_name()

        self.drives = {
            'curiosity': 0.4,
            'energy': 0.5,
            'social_contact': 0.3,
            'safety': 0.6
        }

        self.decay_rate = rospy.get_param("~decay_rate", 0.02)
        self.drive_threshold = rospy.get_param("~drive_threshold", 0.5)

        self.pub_motivation = rospy.Publisher('/motivation_state', String, queue_size=10)

        rospy.Subscriber('/memory_feedback', String, self.memory_feedback_callback)
        rospy.Subscriber('/battery_state', String, self.battery_callback)
        rospy.Subscriber('/surprise_detector', String, self.surprise_callback)

        rospy.Timer(rospy.Duration(4), self.update_drives)
        rospy.loginfo(f"{self.node_name}: Drive system initialized.")

    def memory_feedback_callback(self, msg):
        """Increase curiosity if repeated inputs detected (low novelty)."""
        data = json.loads(msg.data)
        novelty_score = data.get("novelty_score", 0.5)
        self.drives['curiosity'] += (0.5 - novelty_score) * 0.1

    def battery_callback(self, msg):
        """Drive for energy management."""
        data = json.loads(msg.data)
        level = data.get("battery_level", 1.0)
        self.drives['energy'] = 1.0 - level  # Low battery = high energy drive

    def surprise_callback(self, msg):
        """Boost safety drive if surprises are frequent."""
        data = json.loads(msg.data)
        intensity = data.get("surprise_intensity", 0.0)
        self.drives['safety'] += intensity * 0.1

    def update_drives(self, event):
        # Decay all drives slowly
        for key in self.drives:
            self.drives[key] = max(0.0, self.drives[key] - self.decay_rate)

        dominant_drive = max(self.drives, key=self.drives.get)
        urgency = self.drives[dominant_drive]

        active_goals = []
        if urgency > self.drive_threshold:
            if dominant_drive == 'curiosity':
                active_goals = ['explore', 'seek_novelty']
            elif dominant_drive == 'energy':
                active_goals = ['locate_power', 'reduce_activity']
            elif dominant_drive == 'social_contact':
                active_goals = ['initiate_dialogue']
            elif dominant_drive == 'safety':
                active_goals = ['scan_environment', 'avoid_threats']

        state_msg = {
            "timestamp": time.time(),
            "dominant_drive": dominant_drive,
            "overall_drive_level": round(urgency, 2),
            "active_goals_json": json.dumps(active_goals)
        }

        rospy.loginfo(f"{self.node_name}: Dominant Drive: {dominant_drive} (Urgency: {urgency:.2f})")
        self.pub_motivation.publish(json.dumps(state_msg))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DriveSystemNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
