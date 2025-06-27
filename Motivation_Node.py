import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MotivationNode:
    def __init__(self):
        self.motivation_level = 0.5  # Default motivation level (0 to 1)
        self.goal = "default_goal"
        self.subscriber = rospy.Subscriber("integration_output", String, self.callback)
        logger.info(f"{rospy.get_name()}: MotivationNode initialized")

    def callback(self, data: String) -> None:
        """Callback to process integrated output from other nodes."""
        try:
            integrated_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received integrated data: {json.dumps(integrated_data)}")
            self.update_motivation(integrated_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing callback data: {e}")

    def update_motivation(self, integrated_data: Dict) -> None:
        """Update motivation level based on integrated data."""
        try:
            confidence = integrated_data.get("components", {}).get("decision", {}).get("confidence", 0)
            status = integrated_data.get("components", {}).get("system_status", "unknown")
            
            if status == "alert" or confidence < 0.3:
                self.motivation_level = max(0.0, self.motivation_level - 0.1)
                self.goal = "recovery"
            elif confidence > 0.7:
                self.motivation_level = min(1.0, self.motivation_level + 0.1)
                self.goal = "optimization"
            else:
                self.motivation_level = 0.5
                self.goal = "maintenance"

            logger.info(f"{rospy.get_name()}: Motivation updated - Level: {self.motivation_level}, Goal: {self.goal}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating motivation: {e}")

    def run(self):
        """Run the node and publish motivation status."""
        pub = rospy.Publisher("motivation_status", String, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            try:
                message = json.dumps({"motivation_level": self.motivation_level, "goal": self.goal})
                pub.publish(message)
                logger.info(f"{rospy.get_name()}: Published motivation status: {message}")
                rate.sleep()
            except Exception as e:
                logger.error(f"{rospy.get_name()}: Error in run loop: {e}")

if __name__ == "__main__":
    try:
        rospy.init_node("motivation_node", anonymous=True)
        node = MotivationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")