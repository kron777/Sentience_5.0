import logging
import json
import rospy
from typing import Dict, Optional, List
from std_msgs.msg import String
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class OptimizationNode:
    def __init__(self):
        self.optimization_suggestions: List[Dict] = []
        self.pub = rospy.Publisher("optimization_suggestions", String, queue_size=10)
        self.subscribers = {
            "monitoring": rospy.Subscriber("monitoring_output", String, self.callback_monitoring),
            "memory": rospy.Subscriber("memory_status", String, self.callback_memory),
            "self_awareness": rospy.Subscriber("self_awareness_status", String, self.callback_self_awareness)
        }
        logger.info(f"{rospy.get_name()}: OptimizationNode initialized")

    def callback_monitoring(self, data: String) -> None:
        """Callback to process monitoring data for optimization."""
        try:
            monitoring_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received monitoring data")
            self.analyze_performance(monitoring_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing monitoring data: {e}")

    def callback_memory(self, data: String) -> None:
        """Callback to process memory status for optimization."""
        try:
            memory_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received memory data")
            self.analyze_memory_usage(memory_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing memory data: {e}")

    def callback_self_awareness(self, data: String) -> None:
        """Callback to process self-awareness data for optimization."""
        try:
            awareness_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received self-awareness data")
            self.analyze_coherence(awareness_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing self-awareness data: {e}")

    def analyze_performance(self, monitoring_data: Dict) -> None:
        """Analyze performance metrics and suggest optimizations."""
        try:
            if monitoring_data.get("status") == "alert":
                suggestion = {
                    "timestamp": time.time(),
                    "type": "performance",
                    "action": "reallocate_resources",
                    "priority": "high",
                    "reason": monitoring_data.get("message", "Performance issue detected")
                }
                self.optimization_suggestions.append(suggestion)
                logger.info(f"{rospy.get_name()}: Suggested optimization: {json.dumps(suggestion)}")
                self.publish_suggestions()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error analyzing performance: {e}")

    def analyze_memory_usage(self, memory_data: Dict) -> None:
        """Analyze memory usage and suggest optimizations."""
        try:
            total_entries = memory_data.get("total_entries", 0)
            if total_entries >= 90:  # Near max capacity (assuming max_entries = 100)
                suggestion = {
                    "timestamp": time.time(),
                    "type": "memory",
                    "action": "clear_old_entries",
                    "priority": "medium",
                    "reason": f"Memory at {total_entries} entries, nearing limit"
                }
                self.optimization_suggestions.append(suggestion)
                logger.info(f"{rospy.get_name()}: Suggested optimization: {json.dumps(suggestion)}")
                self.publish_suggestions()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error analyzing memory usage: {e}")

    def analyze_coherence(self, awareness_data: Dict) -> None:
        """Analyze coherence and suggest optimizations."""
        try:
            coherence = awareness_data.get("coherence", 0.8)
            if coherence < 0.5:
                suggestion = {
                    "timestamp": time.time(),
                    "type": "coherence",
                    "action": "recalibrate_system",
                    "priority": "high",
                    "reason": f"Low coherence detected: {coherence}"
                }
                self.optimization_suggestions.append(suggestion)
                logger.info(f"{rospy.get_name()}: Suggested optimization: {json.dumps(suggestion)}")
                self.publish_suggestions()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error analyzing coherence: {e}")

    def publish_suggestions(self) -> None:
        """Publish the latest optimization suggestions."""
        try:
            if self.optimization_suggestions:
                latest_suggestion = self.optimization_suggestions[-1]
                self.pub.publish(json.dumps(latest_suggestion))
                logger.info(f"{rospy.get_name()}: Published optimization suggestion: {json.dumps(latest_suggestion)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error publishing suggestions: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("optimization_node", anonymous=True)
        node = OptimizationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")