import ast
import types
import traceback
import os

class IngenuityNode:
    def __init__(self, ros_workspace_path=None, messaging_system=None):
        self.ros_ws = ros_workspace_path
        self.generated_nodes = {}
        self.messaging_system = messaging_system or self.default_messaging_system

    def default_messaging_system(self, message):
        print(f"[IngenuityNode Message] {message}")

    def generate_node_code(self, node_name: str, specification: str) -> str:
        # Simple code template — extend with LLM-assisted code gen if you want
        code = f'''
class {node_name}:
    def __init__(self):
        self.name = "{node_name}"

    def run(self):
        print("Running {node_name}: {specification}")
'''
        return code

    def validate_code(self, code_str: str) -> bool:
        try:
            ast.parse(code_str)
            return True
        except SyntaxError as e:
            self.messaging_system({"status": "validation_failed", "error": str(e)})
            return False

    def integrate_node(self, code_str: str, node_name: str):
        if not self.validate_code(code_str):
            return None
        module = types.ModuleType(node_name)
        try:
            exec(code_str, module.__dict__)
            node_class = getattr(module, node_name)
            instance = node_class()
            self.generated_nodes[node_name] = instance
            self.messaging_system({"status": "integrated", "node": node_name})
            return instance
        except Exception:
            self.messaging_system({"status": "integration_error", "node": node_name})
            traceback.print_exc()
            return None

    def save_node_code(self, node_name: str, code_str: str):
        if self.ros_ws:
            package_dir = os.path.join(self.ros_ws, node_name)
            os.makedirs(package_dir, exist_ok=True)
            file_path = os.path.join(package_dir, f"{node_name}.py")
            with open(file_path, "w") as f:
                f.write(code_str)
            self.messaging_system({"status": "saved_to_disk", "path": file_path})
        else:
            self.messaging_system({"status": "no_ros_ws_defined"})

    def create_and_deploy_node(self, node_name: str, specification: str):
        code = self.generate_node_code(node_name, specification)
        self.save_node_code(node_name, code)
        return self.integrate_node(code, node_name)

    def run_node(self, node_name: str):
        node = self.generated_nodes.get(node_name)
        if node:
            try:
                node.run()
            except Exception as e:
                print(f"[IngenuityNode] Error running node {node_name}: {e}")
        else:
            print(f"[IngenuityNode] Node {node_name} not found.")

# Demo usage
if __name__ == "__main__":
    ingenuity = IngenuityNode(ros_workspace_path="/tmp/robot_ros_ws")

    node_name = "EnergyManagerNode"
    spec = "Manages battery usage efficiently."

    instance = ingenuity.create_and_deploy_node(node_name, spec)
    if instance:
        ingenuity.run_node(node_name)
