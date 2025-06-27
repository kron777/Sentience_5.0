import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class IntegrationNode:
    def __init__(self):
        self.node_outputs: Dict[str, Dict] = {}
        self.pub = rospy.Publisher("integration_output", String, queue_size=10)
        self.subscribers = {
            "decision_making": rospy.Subscriber("decision_making_output", String, self.callback_decision),
            "learning": rospy.Subscriber("learning_output", String, self.callback_learning),
            "communication": rospy.Subscriber("communication_output", String, self.callback_communication),
            "monitoring": rospy.Subscriber("monitoring_output", String, self.callback_monitoring),
            "adaptation": rospy.Subscriber("adaptation_output", String, self.callback_adaptation)
        }
        logger.info(f"{rospy.get_name()}: IntegrationNode initialized")

    def callback_decision(self, data: String) -> None:
        try:
            self.node_outputs["decision_making"] = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received decision data")
            self.integrate_outputs()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing decision data: {e}")

    def callback_learning(self, data: String) -> None:
        try:
            self.node_outputs["learning"] = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received learning data")
            self.integrate_outputs()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing learning data: {e}")

    def callback_communication(self, data: String) -> None:
        try:
            self.node_outputs["communication"] = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received communication data")
            self.integrate_outputs()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing communication data: {e}")

    def callback_monitoring(self, data: String) -> None:
        try:
            self.node_outputs["monitoring"] = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received monitoring data")
            self.integrate_outputs()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing monitoring data: {e}")

    def callback_adaptation(self, data: String) -> None:
        try:
            self.node_outputs["adaptation"] = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received adaptation data")
            self.integrate_outputs()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing adaptation data: {e}")

    def integrate_outputs(self) -> None:
        try:
            if not self.node_outputs:
                logger.warning(f"{rospy.get_name()}: No outputs received for integration")
                return

            integrated_response = {"status": "integrated", "components": {}}

            if "decision_making" in self.node_outputs:
                integrated_response["components"]["decision"] = self.node_outputs["decision_making"]
            if "learning" in self.node_outputs:
                integrated_response["components"]["suggestion"] = self.node_outputs["learning"].get("suggestion", "none")
            if "communication" in self.node_outputs:
                integrated_response["components"]["last_message"] = self.node_outputs["communication"].get("message", {})
            if "monitoring" in self.node_outputs:
                integrated_response["components"]["system_status"] = self.node_outputs["monitoring"].get("status", "unknown")
            if "adaptation" in self.node_outputs:
                integrated_response["components"]["strategy"] = self.node_outputs["adaptation"].get("strategy", "default")

            priority = max((self.node_outputs.get(n, {}).get("priority", "low") for n in self.node_outputs), default="low")
            integrated_response["final_action"] = {
                "priority": priority,
                "action": self.node_outputs.get("decision_making", {}).get("action", "wait")
            }

            self.pub.publish(json.dumps(integrated_response))
            logger.info(f"{rospy.get_name()}: Integrated response published: {json.dumps(integrated_response)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error integrating outputs: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("integration_node", anonymous=True)
        node = IntegrationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")