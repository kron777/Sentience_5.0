import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SelfAwarenessNode:
    def __init__(self):
        self.internal_state: Dict = {"coherence": 0.8, "uptime": 0.0, "anomalies": []}
        self.pub = rospy.Publisher("self_awareness_status", String, queue_size=10)
        self.subscribers = {
            "integration": rospy.Subscriber("integration_output", String, self.callback_integration),
            "monitoring": rospy.Subscriber("monitoring_output", String, self.callback_monitoring)
        }
        self.start_time = time.time()
        logger.info(f"{rospy.get_name()}: SelfAwarenessNode initialized")

    def callback_integration(self, data: String) -> None:
        """Callback to process integrated output for self-assessment."""
        try:
            integrated_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received integrated data")
            self.assess_coherence(integrated_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing integration data: {e}")

    def callback_monitoring(self, data: String) -> None:
        """Callback to process monitoring data for self-assessment."""
        try:
            monitoring_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received monitoring data")
            self.detect_anomalies(monitoring_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing monitoring data: {e}")

    def assess_coherence(self, integrated_data: Dict) -> None:
        """Assess the system's coherence based on integrated outputs."""
        try:
            confidence_sum = sum(d.get("confidence", 0) for d in integrated_data.get("components", {}).values() if isinstance(d, dict))
            component_count = len([d for d in integrated_data.get("components", {}).values() if isinstance(d, dict)])
            coherence = confidence_sum / component_count if component_count > 0 else 0.5
            self.internal_state["coherence"] = max(0.0, min(1.0, coherence))
            logger.info(f"{rospy.get_name()}: Assessed coherence: {self.internal_state['coherence']}")
            self.update_status()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error assessing coherence: {e}")

    def detect_anomalies(self, monitoring_data: Dict) -> None:
        """Detect anomalies based on monitoring data."""
        try:
            if monitoring_data.get("status") == "alert":
                anomaly = {
                    "timestamp": time.time(),
                    "message": monitoring_data.get("message", "Unknown anomaly"),
                    "severity": "high"
                }
                self.internal_state["anomalies"].append(anomaly)
                logger.warning(f"{rospy.get_name()}: Detected anomaly: {json.dumps(anomaly)}")
            self.update_status()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error detecting anomalies: {e}")

    def update_status(self) -> None:
        """Update and publish the internal state."""
        try:
            self.internal_state["uptime"] = time.time() - self.start_time
            status = json.dumps(self.internal_state)
            self.pub.publish(status)
            logger.info(f"{rospy.get_name()}: Published self-awareness status: {status}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating status: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("self_awareness_node", anonymous=True)
        node = SelfAwarenessNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")