import logging
import json
import rospy
from typing import Dict, List
import time
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MonitoringNode:
    def __init__(self):
        self.metrics: List[Dict] = []
        self.alert_threshold = 0.3
        self.last_check = 0
        self.pub = rospy.Publisher("monitoring_output", String, queue_size=10)
        self.subscribers = {
            "decision_making": rospy.Subscriber("decision_making_output", String, self.callback_decision),
            "learning": rospy.Subscriber("learning_output", String, self.callback_learning),
            "communication": rospy.Subscriber("communication_output", String, self.callback_communication)
        }
        logger.info(f"{rospy.get_name()}: MonitoringNode initialized")

    def callback_decision(self, data: String) -> None:
        try:
            self.collect_metric("decision_making", json.loads(data.data))
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing decision data: {e}")

    def callback_learning(self, data: String) -> None:
        try:
            self.collect_metric("learning", json.loads(data.data))
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing learning data: {e}")

    def callback_communication(self, data: String) -> None:
        try:
            self.collect_metric("communication", json.loads(data.data))
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing communication data: {e}")

    def collect_metric(self, node_name: str, metric: Dict) -> None:
        try:
            entry = {
                "node": node_name,
                "timestamp": time.time(),
                "metric": metric,
                "status": "ok"
            }
            self.metrics.append(entry)
            logger.info(f"{rospy.get_name()}: Collected metric from {node_name}: {json.dumps(metric)}")
            self.check_performance()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error collecting metric: {e}")

    def check_performance(self) -> None:
        try:
            if not self.metrics or (time.time() - self.last_check < 5):
                return

            self.last_check = time.time()
            total_metrics = len(self.metrics)
            if total_metrics == 0:
                return

            avg_performance = sum(m["metric"].get("confidence", 0) for m in self.metrics) / total_metrics
            logger.info(f"{rospy.get_name()}: Average performance: {avg_performance:.2f}")

            if avg_performance < self.alert_threshold:
                alert = {
                    "status": "alert",
                    "message": f"Performance below threshold ({avg_performance:.2f} < {self.alert_threshold})",
                    "recommendation": "retrain_or_restart"
                }
                self.pub.publish(json.dumps(alert))
                logger.warning(f"{rospy.get_name()}: {alert['message']}")
            else:
                self.pub.publish(json.dumps({"status": "normal", "alert": False}))
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error checking performance: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("monitoring_node", anonymous=True)
        node = MonitoringNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")