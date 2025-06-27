import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ContextAwarenessNode:
    def __init__(self):
        self.context: Dict = {"environment": "neutral", "time_of_day": "unknown", "priority": "normal"}
        self.pub = rospy.Publisher("context_status", String, queue_size=10)
        self.subscribers = {
            "control": rospy.Subscriber("control_output", String, self.callback_control),
            "health": rospy.Subscriber("health_status", String, self.callback_health)
        }
        logger.info(f"{rospy.get_name()}: ContextAwarenessNode initialized")

    def callback_control(self, data: String) -> None:
        """Callback to process control output for context awareness."""
        try:
            control_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received control data")
            self.update_context_from_control(control_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing control data: {e}")

    def callback_health(self, data: String) -> None:
        """Callback to process health status for context awareness."""
        try:
            health_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received health data")
            self.update_context_from_health(health_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing health data: {e}")

    def update_context_from_control(self, control_data: Dict) -> None:
        """Update context based on control actions."""
        try:
            action = control_data.get("action", "idle")
            if action == "execute_task":
                self.context["environment"] = "active"
                self.context["priority"] = "high"
            elif action == "wait":
                self.context["environment"] = "idle"
                self.context["priority"] = "low"
            self.context["time_of_day"] = time.strftime("%H:%M:%S")
            logger.info(f"{rospy.get_name()}: Updated context from control: {json.dumps(self.context)}")
            self.publish_context()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating context from control: {e}")

    def update_context_from_health(self, health_data: Dict) -> None:
        """Update context based on health status."""
        try:
            if health_data.get("cpu_usage", 0) > 80 or health_data.get("memory_usage", 0) > 80:
                self.context["environment"] = "stressed"
                self.context["priority"] = "high"
            self.publish_context()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating context from health: {e}")

    def publish_context(self) -> None:
        """Publish the current context status."""
        try:
            self.pub.publish(json.dumps(self.context))
            logger.info(f"{rospy.get_name()}: Published context status: {json.dumps(self.context)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error publishing context: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("context_awareness_node", anonymous=True)
        node = ContextAwarenessNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")