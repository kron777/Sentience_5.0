import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import json

# Assuming your InsightNode class is implemented as `InsightNode` (import or define here)
# For demonstration, I’ll inline a minimal mock version

class InsightNodeCore:
    def analyze(self):
        # Replace with actual analyze logic
        return {
            "nodes": [
                {"name": "EnergyManagerNode", "spec": "Manage battery efficiently."},
                {"name": "TaskSchedulerNode", "spec": "Optimize task scheduling."}
            ]
        }

class InsightNodeROS(Node):
    def __init__(self):
        super().__init__('insight_node')
        self.insight_core = InsightNodeCore()

        self.publisher_ = self.create_publisher(String, 'insight/suggestions', 10)
        self.srv = self.create_service(Trigger, 'insight/analyze', self.handle_analyze_request)

        self.get_logger().info("InsightNode ROS started.")

    def handle_analyze_request(self, request, response):
        self.get_logger().info("Analyze service called.")
        suggestions = self.insight_core.analyze()
        suggestions_str = json.dumps(suggestions)
        response.success = True
        response.message = suggestions_str

        # Also publish suggestions
        msg = String()
        msg.data = suggestions_str
        self.publisher_.publish(msg)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = InsightNodeROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
