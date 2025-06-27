import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class InteractionInterfaceNode:
    def __init__(self):
        self.interaction_log: List[Dict] = []
        self.pub = rospy.Publisher("interaction_response", String, queue_size=10)
        self.subscriber = rospy.Subscriber("control_output", String, self.callback_control)
        logger.info(f"{rospy.get_name()}: InteractionInterfaceNode initialized")

    def callback_control(self, data: String) -> None:
        """Callback to process control output for interaction."""
        try:
            control_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received control data")
            self.handle_interaction(control_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing control data: {e}")

    def handle_interaction(self, control_data: Dict) -> None:
        """Handle interaction based on control output."""
        try:
            action = control_data.get("action", "idle")
            response = {"timestamp": time.time(), "action": action, "response": "acknowledged"}

            if action == "respond_emotionally":
                response["response"] = "Expressing appropriate emotion"
            elif action == "execute_task":
                response["response"] = "Task execution in progress"

            self.interaction_log.append(response)
            self.pub.publish(json.dumps(response))
            logger.info(f"{rospy.get_name()}: Handled interaction: {json.dumps(response)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error handling interaction: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("interaction_interface_node", anonymous=True)
        node = InteractionInterfaceNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")