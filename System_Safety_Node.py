import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SystemSafetyNode:
    def __init__(self):
        self.safety_status = {"safe": True, "shutdown_triggered": False}
        self.pub = rospy.Publisher("safety_status", String, queue_size=10)
        self.subscribers = {
            "health": rospy.Subscriber("health_status", String, self.callback_health),
            "prediction": rospy.Subscriber("prediction_output", String, self.callback_prediction)
        }
        logger.info(f"{rospy.get_name()}: SystemSafetyNode initialized")

    def callback_health(self, data: String) -> None:
        """Callback to process health status for safety check."""
        try:
            health_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received health data")
            self.check_safety(health_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing health data: {e}")

    def callback_prediction(self, data: String) -> None:
        """Callback to process prediction data for safety check."""
        try:
            prediction_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received prediction data")
            self.check_safety(prediction_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing prediction data: {e}")

    def check_safety(self, data: Dict) -> None:
        """Check system safety based on health and prediction data."""
        try:
            if data.get("cpu_usage", 0) > 95 or data.get("memory_usage", 0) > 95:
                self.safety_status["safe"] = False
                self.safety_status["shutdown_triggered"] = True
                logger.warning(f"{rospy.get_name()}: Safety breach detected, triggering shutdown")
            elif data.get("predicted_cpu_usage", 0) > 90 or data.get("predicted_memory_usage", 0) > 90:
                self.safety_status["safe"] = False
                logger.warning(f"{rospy.get_name()}: Predicted safety breach")
            self.publish_safety_status()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error checking safety: {e}")

    def publish_safety_status(self) -> None:
        """Publish the current safety status."""
        try:
            self.pub.publish(json.dumps(self.safety_status))
            logger.info(f"{rospy.get_name()}: Published safety status: {json.dumps(self.safety_status)}")
            if self.safety_status["shutdown_triggered"]:
                rospy.signal_shutdown("Safety shutdown triggered")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error publishing safety status: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("system_safety_node", anonymous=True)
        node = SystemSafetyNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")