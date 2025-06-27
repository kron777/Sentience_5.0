import logging
import json
import rospy
from typing import Dict, Optional
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class LearningUpdateNode:
    def __init__(self):
        self.learning_rate = 0.1
        self.pub = rospy.Publisher("learning_update", String, queue_size=10)
        self.subscribers = {
            "feedback": rospy.Subscriber("feedback_input", String, self.callback_feedback),
            "optimization": rospy.Subscriber("optimization_suggestions", String, self.callback_optimization),
            "memory": rospy.Subscriber("memory_status", String, self.callback_memory)
        }
        logger.info(f"{rospy.get_name()}: LearningUpdateNode initialized")

    def callback_feedback(self, data: String) -> None:
        """Callback to process feedback for learning update."""
        try:
            feedback_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received feedback data")
            self.update_learning_from_feedback(feedback_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing feedback data: {e}")

    def callback_optimization(self, data: String) -> None:
        """Callback to process optimization suggestions for learning update."""
        try:
            optimization_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received optimization data")
            self.update_learning_from_optimization(optimization_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing optimization data: {e}")

    def callback_memory(self, data: String) -> None:
        """Callback to process memory status for learning update."""
        try:
            memory_data = json.loads(data.data)
            logger.info(f"{rospy.get_name()}: Received memory data")
            self.adjust_learning_rate(memory_data)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing memory data: {e}")

    def update_learning_from_feedback(self, feedback_data: Dict) -> None:
        """Update learning based on feedback."""
        try:
            if feedback_data.get("success", False):
                self.learning_rate = min(0.2, self.learning_rate + 0.01)
            else:
                self.learning_rate = max(0.05, self.learning_rate - 0.01)
            update = {"learning_rate": self.learning_rate, "source": "feedback"}
            self.publish_update(update)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating from feedback: {e}")

    def update_learning_from_optimization(self, optimization_data: Dict) -> None:
        """Update learning based on optimization suggestions."""
        try:
            if optimization_data.get("priority", "low") == "high":
                self.learning_rate = min(0.2, self.learning_rate + 0.02)
            update = {"learning_rate": self.learning_rate, "source": "optimization"}
            self.publish_update(update)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error updating from optimization: {e}")

    def adjust_learning_rate(self, memory_data: Dict) -> None:
        """Adjust learning rate based on memory usage."""
        try:
            if memory_data.get("total_entries", 0) > 80:
                self.learning_rate = max(0.05, self.learning_rate - 0.01)
            update = {"learning_rate": self.learning_rate, "source": "memory"}
            self.publish_update(update)
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error adjusting learning rate: {e}")

    def publish_update(self, update: Dict) -> None:
        """Publish the learning update."""
        try:
            self.pub.publish(json.dumps(update))
            logger.info(f"{rospy.get_name()}: Published learning update: {json.dumps(update)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error publishing update: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("learning_update_node", anonymous=True)
        node = LearningUpdateNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")