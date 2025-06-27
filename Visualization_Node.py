import logging
import json
import rospy
from typing import Dict, List
import time
import matplotlib.pyplot as plt
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class VisualizationNode:
    def __init__(self):
        self.data_points: List[Dict] = []
        self.pub = rospy.Publisher("visualization_output", String, queue_size=10)
        self.subscriber = rospy.Subscriber("monitoring_output", String, self.callback)
        logger.info(f"{rospy.get_name()}: VisualizationNode initialized")

    def callback(self, data: String) -> None:
        try:
            self.add_data_point("monitoring", json.loads(data.data))
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error processing callback data: {e}")

    def add_data_point(self, node_name: str, data: Dict) -> None:
        try:
            data_point = {"node": node_name, "timestamp": time.time(), "data": data}
            self.data_points.append(data_point)
            logger.info(f"{rospy.get_name()}: Added data point from {node_name}: {json.dumps(data)}")
            self.generate_performance_plot()
            self.display_status_summary()
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error adding data point: {e}")

    def generate_performance_plot(self) -> None:
        try:
            if not self.data_points:
                logger.warning(f"{rospy.get_name()}: No data points available for plotting")
                return

            timestamps = [dp["timestamp"] for dp in self.data_points]
            confidences = [dp["data"].get("confidence", 0) for dp in self.data_points]

            plt.figure(figsize=(10, 6))
            plt.plot(timestamps, confidences, label="Confidence", marker='o')
            plt.xlabel("Time")
            plt.ylabel("Confidence Level")
            plt.title("System Performance Over Time")
            plt.legend()
            plt.grid(True)
            plt.savefig("performance_plot.png")
            plt.close()
            logger.info(f"{rospy.get_name()}: Performance plot generated and saved as performance_plot.png")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error generating performance plot: {e}")

    def display_status_summary(self) -> None:
        try:
            if not self.data_points:
                logger.warning(f"{rospy.get_name()}: No data points available for summary")
                return

            latest_data = max(self.data_points, key=lambda x: x["timestamp"])
            summary = {
                "node": latest_data["node"],
                "timestamp": latest_data["timestamp"],
                "status": latest_data["data"].get("status", "unknown"),
                "confidence": latest_data["data"].get("confidence", 0)
            }
            self.pub.publish(json.dumps(summary))
            logger.info(f"{rospy.get_name()}: Status summary published: {json.dumps(summary)}")
        except Exception as e:
            logger.error(f"{rospy.get_name()}: Error generating status summary: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("visualization_node", anonymous=True)
        node = VisualizationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")