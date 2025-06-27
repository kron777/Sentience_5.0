import logging
import json
import rospy
from typing import Dict, List
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CoordinationNode:
    def __init__(self):
        self.agent_status: Dict[str, Dict] = {}
        self.pub = rospy.Publisher("coordination_commands", String, queue_size=10)
        self.subscribers = {
            "control": rospy.Subscriber("control_output", String, self.callback_control),
            "motivation": rospy.Subscriber("motivation_status", String, self.callback_motivation),
            "health": rospy.Subscriber("health_status", String, self.callback_health)
        }
        logger.info(f"{rospy.get_name()}: CoordinationNode initialized")

    def callback_control(self, data: String) -> None:
        """Callback to process control output for coordination."""
        try:
            control_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received control data")
            self.update_agent_status("control", control_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing control data: {e}")

    def callback_motivation(self, data: String) -> None:
        """Callback to process motivation status for coordination."""
        try:
            motivation_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received motivation data")
            self.update_agent_status("motivation", motivation_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing motivation data: {e}")

    def callback_health(self, data: String) -> None:
        """Callback to process health status for coordination."""
        try:
            health_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received health data")
            self.update_agent_status("health", health_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing health data: {e}")

    def update_agent_status(self, source: str, data: Dict) -> None:
        """Update the status of agents based on received data."""
        try:
            self.agent_status[source] = data
            logger.info(f"{rospy.get_name()}: Updated {source} status: {json.dumps(data)}")
            self.coordinate_agents()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating agent status: {e}")

    def coordinate_agents(self) -> None:
        """Coordinate actions among agents based on their status."""
        try:
            if not self.agent_status:
                logger.warning(f"{rospy.get_name()}: No agent status available")
                return

            command = {"timestamp": time.time(), "commands": {}}
            if self.agent_status.get("health", {}).get("error_rate", 0) > 0.1:
                command["commands"]["control"] = "pause_and_diagnose"
            elif self.agent_status.get("motivation", {}).get("motivation_level", 0.5) > 0.7:
                command["commands"]["control"] = "increase_effort"

            if command["commands"]:
                self.pub.publish(json.dumps(command))
                logger.info(f"{rospy.get_name()}: Published coordination command: {json.dumps(command)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error coordinating agents: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("coordination_node", anonymous=True)
        node = CoordinationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")