import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AdaptationNode:
    def __init__(self):
        self.current_strategy = "default"
        self.adjustment_factor = 0.1
        self.pub = rospy.Publisher("adaptation_output", String, queue_size=10)
        self.subscribers = {
            "learning": rospy.Subscriber("learning_output", String, self.callback_learning),
            "monitoring": rospy.Subscriber("monitoring_output", String, self.callback_monitoring)
        }
        logger.info(f"{rospy.get_name()}: AdaptationNode initialized")

    def callback_learning(self, data: String) -> None:
        try:
            suggestion = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received learning suggestion")
            self.receive_learning_suggestion(suggestion)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing learning data: {e}")

    def callback_monitoring(self, data: String) -> None:
        try:
            alert = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received monitoring alert")
            self.receive_monitoring_alert(alert)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing monitoring data: {e}")

    def receive_learning_suggestion(self, suggestion: Dict) -> None:
        try:
            if suggestion.get("suggestion") == "adjust_strategy" and suggestion.get("confidence", 0) > 0.5:
                self.update_strategy(suggestion)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing learning suggestion: {e}")

    def receive_monitoring_alert(self, alert: Dict) -> None:
        try:
            if alert.get("status") == "alert":
                self.update_strategy(alert)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing monitoring alert: {e}")

    def update_strategy(self, input_data: Dict) -> None:
        try:
            if input_data.get("recommendation", "") == "retrain_or_restart":
                self.current_strategy = "optimized"
                self.adjustment_factor = max(0.05, self.adjustment_factor - 0.05)
            elif input_data.get("confidence", 0) < 0.3:
                self.current_strategy = "conservative"
                self.adjustment_factor = min(0.2, self.adjustment_factor + 0.05)
            else:
                self.current_strategy = "balanced"
                self.adjustment_factor = 0.1

            output = {"status": "adapted", "strategy": self.current_strategy, "adjustment_factor": self.adjustment_factor}
            self.pub.publish(json.dumps(output))
            logger.info(f"{rospy.get_name()}: Strategy updated and published: {json.dumps(output)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating strategy: {e}")

    def apply_adaptation(self, decision: Dict) -> Dict:
        try:
            if self.current_strategy == "optimized":
                decision["priority"] = "high" if decision.get("priority") != "low" else "medium"
            elif self.current_strategy == "conservative":
                decision["priority"] = "low" if decision.get("priority") != "high" else "medium"
            decision["adjustment"] = self.adjustment_factor
            logger.info(f"{rospy.get_name()}: Adapted decision: {json.dumps(decision)}")
            return decision
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error applying adaptation: {e}")
            return decision

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("adaptation_node", anonymous=True)
        node = AdaptationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")