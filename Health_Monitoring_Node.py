import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String
import psutil  # For system resource monitoring (install via pip if needed)
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class HealthMonitoringNode:
    def __init__(self):
        self.health_status: Dict = {
            "cpu_usage": 0.0,
            "memory_usage": 0.0,
            "error_rate": 0.0,
            "last_check": 0.0
        }
        self.error_count = 0
        self.pub = rospy.Publisher("health_status", String, queue_size=10)
        self.subscribers = {
            "control": rospy.Subscriber("control_output", String, self.callback_control),
            "monitoring": rospy.Subscriber("monitoring_output", String, self.callback_monitoring)
        }
        logger.info(f"{rospy.get_name()}: HealthMonitoringNode initialized")

    def callback_control(self, data: String) -> None:
        """Callback to process control output for health assessment."""
        try:
            control_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received control data")
            self.check_execution_health(control_data)
        except Exception as e:
            self.error_count += 1
            logger.error(f"{rospy.get_name()}: Error processing control data: {e}")

    def callback_monitoring(self, data: String) -> None:
        """Callback to process monitoring data for health assessment."""
        try:
            monitoring_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received monitoring data")
            self.update_system_health(monitoring_data)
        except Exception as e:
            self.error_count += 1
            logger.error(f"{rospy.get_name()}: Error processing monitoring data: {e}")

    def update_system_health(self, monitoring_data: Dict) -> None:
        """Update system health metrics based on monitoring data."""
        try:
            if time.time() - self.health_status["last_check"] >= 5:  # Update every 5 seconds
                self.health_status["cpu_usage"] = psutil.cpu_percent()
                self.health_status["memory_usage"] = psutil.virtual_memory().percent
                self.health_status["error_rate"] = (self.error_count / (time.time() - self.health_status["last_check"])) if self.health_status["last_check"] > 0 else 0.0
                self.health_status["last_check"] = time.time()
                logger.info(f"{rospy.get_name()}: Updated health status: {json.dumps(self.health_status)}")
                self.publish_health_status()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating system health: {e}")

    def check_execution_health(self, control_data: Dict) -> None:
        """Check health based on control execution."""
        try:
            if control_data.get("action") == "idle" and time.time() - self.health_status["last_check"] > 60:
                self.health_status["error_rate"] += 0.1  # Penalize prolonged idleness
                logger.warning(f"{rospy.get_name()}: Prolonged idle state detected")
            self.publish_health_status()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error checking execution health: {e}")

    def publish_health_status(self) -> None:
        """Publish the current health status."""
        try:
            self.pub.publish(json.dumps(self.health_status))
            logger.info(f"{rospy.get_name()}: Published health status: {json.dumps(self.health_status)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error publishing health status: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("health_monitoring_node", anonymous=True)
        node = HealthMonitoringNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")