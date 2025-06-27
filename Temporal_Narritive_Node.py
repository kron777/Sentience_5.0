#!/usr/bin/env python3
import rospy
import json
import time
from std_msgs.msg import String
from collections import deque

class TemporalNarrativeNode:
    def __init__(self):
        rospy.init_node('temporal_narrative_node', anonymous=False)
        self.node_name = rospy.get_name()

        self.narrative_window = rospy.get_param("~narrative_window", 10)  # Number of past events to track
        self.memory_events = deque(maxlen=self.narrative_window)
        self.emotional_trace = deque(maxlen=5)
        self.goal_history = deque(maxlen=5)

        rospy.Subscriber('/memory_log', String, self.memory_callback)
        rospy.Subscriber('/emotional_state', String, self.emotion_callback)
        rospy.Subscriber('/goal_updates', String, self.goal_callback)

        self.pub_narrative = rospy.Publisher('/narrative_state', String, queue_size=10)

        rospy.Timer(rospy.Duration(6), self.generate_narrative)
        rospy.loginfo(f"{self.node_name}: Temporal narrative system online.")

    def memory_callback(self, msg):
        data = json.loads(msg.data)
        self.memory_events.append({
            "event": data.get("event", "unknown"),
            "timestamp": data.get("timestamp", time.time())
        })

    def emotion_callback(self, msg):
        data = json.loads(msg.data)
        self.emotional_trace.append({
            "mood": data.get("mood", "neutral"),
            "intensity": data.get("mood_intensity", 0.0),
            "timestamp": time.time()
        })

    def goal_callback(self, msg):
        data = json.loads(msg.data)
        self.goal_history.append({
            "goal": data.get("goal", ""),
            "status": data.get("status", "in_progress"),
            "timestamp": time.time()
        })

    def generate_narrative(self, event):
        recent_events = list(self.memory_events)
        recent_goals = list(self.goal_history)
        mood_summary = self._summarize_emotion()

        story = {
            "timestamp": time.time(),
            "narrative_theme": self._infer_theme(recent_events, recent_goals),
            "recent_memory": recent_events[-3:] if len(recent_events) >= 3 else recent_events,
            "mood_trend": mood_summary,
            "goal_trail": recent_goals[-3:] if len(recent_goals) >= 3 else recent_goals
        }

        rospy.loginfo(f"{self.node_name}: Story frame: {story['narrative_theme']}")
        self.pub_narrative.publish(json.dumps(story))

    def _summarize_emotion(self):
        if not self.emotional_trace:
            return "neutral"
        mood_counts = {}
        for e in self.emotional_trace:
            mood = e['mood']
            mood_counts[mood] = mood_counts.get(mood, 0) + 1
        dominant = max(mood_counts, key=mood_counts.get)
        return dominant

    def _infer_theme(self, events, goals):
        if not events:
            return "awakening"
        if any("conflict" in e["event"].lower() for e in events):
            return "resilience"
        if any(g["status"] == "completed" for g in goals):
            return "progress"
        if any(g["status"] == "abandoned" for g in goals):
            return "redirection"
        return "continuity"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TemporalNarrativeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
