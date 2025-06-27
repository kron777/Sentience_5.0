#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import time

class MetaAwarenessNode:
    def __init__(self):
        rospy.init_node('meta_awareness_node', anonymous=False)
        self.node_name = rospy.get_name()

        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.4)
        self.conflict_threshold = rospy.get_param("~conflict_threshold", 0.6)

        self.meta_state_pub = rospy.Publisher('/meta_awareness_state', String, queue_size=10)
        self.directive_pub = rospy.Publisher('/cognitive_directives', String, queue_size=10)

        rospy.Subscriber('/prediction_state', String, self.prediction_callback)
        rospy.Subscriber('/internal_narrative', String, self.narrative_callback)
        rospy.Subscriber('/emotion_state', String, self.emotion_callback)

        self.last_confidence = 1.0
        self.dissonance_level = 0.0
        self.last_emotion = {'mood': 'neutral', 'intensity': 0.0}

        rospy.Timer(rospy.Duration(3), self.evaluate_meta_state)
        rospy.loginfo(f"{self.node_name} online. Watching inner confidence.")

    def prediction_callback(self, msg):
        data = json.loads(msg.data)
        confidence = data.get("confidence", 1.0)
        accurate = data.get("is_accurate", True)

        if not accurate:
            delta = self.last_confidence - confidence
            self.dissonance_level += abs(delta)
        self.last_confidence = confidence

    def narrative_callback(self, msg):
        data = json.loads(msg.data)
        if "conflict" in data.get("main_theme", "").lower():
            self.dissonance_level += 0.2

    def emotion_callback(self, msg):
        data = json.loads(msg.data)
        self.last_emotion = {'mood': data.get('mood', 'neutral'), 'intensity': data.get('mood_intensity', 0.0)}

    def evaluate_meta_state(self, event):
        meta_status = {
            'confidence_level': self.last_confidence,
            'dissonance': self.dissonance_level,
            'emotional_intensity': self.last_emotion['intensity'],
            'mood': self.last_emotion['mood'],
            'timestamp': time.time()
        }

        rospy.loginfo(f"{self.node_name}: Meta-awareness snapshot: {meta_status}")
        self.meta_state_pub.publish(json.dumps(meta_status))

        if self.last_confidence < self.confidence_threshold or self.dissonance_level > self.conflict_threshold:
            directive = {
                'directive_type': 'TriggerReflection',
                'target_node': '/reflection_node',
                'command_payload': json.dumps({'reason': 'low_confidence_or_conflict'}),
                'timestamp': time.time()
            }
            self.directive_pub.publish(json.dumps(directive))
            rospy.logwarn(f"{self.node_name}: Triggering self-reflection due to uncertainty.")

        self.dissonance_level *= 0.8  # decay dissonance

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MetaAwarenessNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
