import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_msgs.srv import CreateNode  # Define this custom srv, or use std srv + JSON
import ast
import types
import traceback

class IngenuityNodeCore:
    def __init__(self):
        self.generated_nodes = {}

    def generate_node_code(self, node_name: str, specification: str) -> str:
        return f'''
class {node_name}:
    def __init__(self):
        self.name = "{node_name}"

    def run(self):
        print("Running {node_name}: {specification}")
'''

    def validate_code(self, code_str: str) -> bool:
        try:
            ast.parse(code_str)
            return True
        except SyntaxError:
            return False

    def integrate_node(self, code_str: str, node_name: str):
        if not self.validate_code(code_str):
            return False, "Syntax error in generated code"
        module = types.ModuleType(node_name)
        try:
            exec(code_str, module.__dict__)
            node_class = getattr(module, node_name)
            instance = node_class()
            self.generated_nodes[node_name] = instance
            return True, f"Node {node_name} integrated"
        except Exception as e:
            return False, str(e)

class IngenuityNodeROS(Node):
    def __init__(self):
        super().__init__('ingenuity_node')
        self.core = IngenuityNodeCore()

        self.publisher_ = self.create_publisher(String, 'ingenuity/status', 10)
        # Using Trigger srv here for example — you can define a custom srv for node creation
        from example_interfaces.srv import Trigger
        self.srv = self.create_service(Trigger, 'ingenuity/create_node', self.handle_create_node)

        self.get_logger().info("IngenuityNode ROS started.")

    def handle_create_node(self, request, response):
        # For demo, parse node_name and spec from a JSON string in request.message if available
        # Here we just create a dummy node for demo purposes
        node_name = "DemoNode"
        spec = "Demonstration node created by IngenuityNode."

        code = self.core.generate_node_code(node_name, spec)
        success, msg = self.core.integrate_node(code, node_name)

        response.success = success
        response.message = msg

        # Publish status update
        status_msg = String()
        status_msg.data = msg
        self.publisher_.publish(status_msg)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = IngenuityNodeROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
