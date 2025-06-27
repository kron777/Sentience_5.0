import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ResourceAllocationNode:
    def __init__(self):
        self.allocation_plan = {"cpu_share": 0.5, "memory_share": 0.5}
        self.pub = rospy.Publisher("resource_allocation", String, queue_size=10)
        self.subscribers = {
            "health": rospy.Subscriber("health_status", String, self.callback_health),
            "prediction": rospy.Subscriber("prediction_output", String, self.callback_prediction),
            "optimization": rospy.Subscriber("optimization_suggestions", String, self.callback_optimization)
        }
        logger.info(f"{rospy.get_name()}: ResourceAllocationNode initialized")

    def callback_health(self, data: String) -> None:
        """Callback to process health status for resource allocation."""
        try:
            health_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received health data")
            self.adjust_allocation(health_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing health data: {e}")

    def callback_prediction(self, data: String) -> None:
        """Callback to process prediction data for resource allocation."""
        try:
            prediction_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received prediction data")
            self.adjust_allocation(prediction_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing prediction data: {e}")

    def callback_optimization(self, data: String) -> None:
        """Callback to process optimization suggestions for resource allocation."""
        try:
            optimization_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received optimization data")
            self.adjust_allocation(optimization_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing optimization data: {e}")

    def adjust_allocation(self, data: Dict) -> None:
        """Adjust resource allocation based on health, prediction, or optimization data."""
        try:
            if data.get("cpu_usage", 0) > 80 or data.get("predicted_cpu_usage", 0) > 85:
                self.allocation_plan["cpu_share"] = 0.7
                self.allocation_plan["memory_share"] = 0.3
            elif data.get("action", "") == "reallocate_resources":
                self.allocation_plan["cpu_share"] = 0.6
                self.allocation_plan["memory_share"] = 0.4

            self.publish_allocation()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error adjusting allocation: {e}")

    def publish_allocation(self) -> None:
        """Publish the current resource allocation plan."""
        try:
            self.pub.publish(json.dumps(self.allocation_plan))
            logger.info(f"{rospy.get_name()}: Published resource allocation: {json.dumps(self.allocation_plan)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error publishing allocation: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("resource_allocation_node", anonymous=True)
        node = ResourceAllocationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")