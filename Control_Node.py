import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ControlNode:
    def __init__(self):
        self.current_action = {"action": "idle", "priority": "low"}
        self.subscribers = {
            "integration": rospy.Subscriber("integration_output", String, self.callback_integration),
            "motivation": rospy.Subscriber("motivation_status", String, self.callback_motivation),
            "adaptation": rospy.Subscriber("adaptation_output", String, self.callback_adaptation)
        }
        self.pub = rospy.Publisher("control_output", String, queue_size=10)
        logger.info(f"{rospy.get_name()}: ControlNode initialized")

    def callback_integration(self, data: String) -> None:
        try:
            integrated_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received integrated data")
            self.update_action(integrated_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing integration data: {e}")

    def callback_motivation(self, data: String) -> None:
        try:
            motivation_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received motivation data")
            self.adjust_priority(motivation_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing motivation data: {e}")

    def callback_adaptation(self, data: String) -> None:
        try:
            adaptation_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received adaptation data")
            self.apply_adaptation(adaptation_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing adaptation data: {e}")

    def update_action(self, integrated_data: Dict) -> None:
        try:
            action = integrated_data.get("final_action", {})
            self.current_action["action"] = action.get("action", "idle")
            logger.info(f"{rospy.get_name()}: Updated action to {self.current_action['action']}")
            self.execute_action()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating action: {e}")

    def adjust_priority(self, motivation_data: Dict) -> None:
        try:
            motivation_level = motivation_data.get("motivation_level", 0.5)
            if motivation_level < 0.3:
                self.current_action["priority"] = "low"
            elif motivation_level > 0.7:
                self.current_action["priority"] = "high"
            else:
                self.current_action["priority"] = "medium"
            logger.info(f"{rospy.get_name()}: Adjusted priority to {self.current_action['priority']}")
            self.execute_action()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error adjusting priority: {e}")

    def apply_adaptation(self, adaptation_data: Dict) -> None:
        try:
            strategy = adaptation_data.get("strategy", "default")
            if strategy == "optimized":
                self.current_action["priority"] = "high" if self.current_action["priority"] != "low" else "medium"
            elif strategy == "conservative":
                self.current_action["priority"] = "low" if self.current_action["priority"] != "high" else "medium"
            logger.info(f"{rospy.get_name()}: Applied adaptation, priority: {self.current_action['priority']}")
            self.execute_action()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error applying adaptation: {e}")

    def execute_action(self) -> None:
        try:
            output = json.dumps(self.current_action)
            self.pub.publish(output)
            logger.info(f"{rospy.get_name()}: Executed action: {output}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error executing action: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("control_node", anonymous=True)
        node = ControlNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")