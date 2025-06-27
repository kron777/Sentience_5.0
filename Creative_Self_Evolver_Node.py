import json
import ast
import types
import os
import traceback

# -- InsightNode --

class InsightNode:
    def __init__(self, awareness, conversation, dreaming, llm):
        self.awareness = awareness
        self.conversation = conversation
        self.dreaming = dreaming
        self.llm = llm

    def gather_context(self):
        awareness_text = self.awareness.get_state_description()
        conversation_text = self.conversation.get_recent_dialogue()
        dreaming_text = self.dreaming.get_recent_dream()
        combined = (
            f"Robot Awareness:\n{awareness_text}\n\n"
            f"Recent Conversation:\n{conversation_text}\n\n"
            f"Dreaming Output:\n{dreaming_text}\n"
        )
        return combined

    def construct_prompt(self, context):
        prompt = (
            f"Given the robot's current situation and context below:\n{context}\n\n"
            "Identify areas where the robot can improve itself by creating new functional nodes. "
            "For each suggested node, provide a name and a short description. "
            "Return the response in valid JSON format as:\n"
            "{\n  \"nodes\": [\n    {\"name\": \"NodeName\", \"spec\": \"Description\"}, ... ]\n}\n"
        )
        return prompt

    def analyze(self):
        context = self.gather_context()
        prompt = self.construct_prompt(context)
        raw_response = self.llm.query(prompt)
        try:
            suggestions = json.loads(raw_response)
        except (json.JSONDecodeError, TypeError):
            print("Warning: LLM response invalid JSON, returning empty suggestions.")
            suggestions = {"nodes": []}
        return suggestions

# -- IngenuityNode --

class IngenuityNode:
    def __init__(self, ros_workspace_path=None, messaging_system=None):
        self.ros_ws = ros_workspace_path
        self.generated_nodes = {}
        self.messaging_system = messaging_system or self.default_messaging_system

    def default_messaging_system(self, message):
        print(f"[IngenuityNode Message] {message}")

    def generate_node_code(self, node_name: str, specification: str) -> str:
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

# -- Mock Components --

class MockAwareness:
    def get_state_description(self):
        return "Battery drains too quickly and task scheduling is inefficient."

class MockConversation:
    def get_recent_dialogue(self):
        return "User mentioned that tasks are often late and system overheats."

class MockDreaming:
    def get_recent_dream(self):
        return "Imagined a node balancing power usage dynamically."

class MockLLM:
    def query(self, prompt):
        print(f"LLM Prompt:\n{prompt}\n")
        return """
        {
            "nodes": [
                {"name": "EnergyManagerNode", "spec": "Manage battery usage efficiently to extend runtime."},
                {"name": "TaskSchedulerNode", "spec": "Optimize task execution order to meet deadlines."}
            ]
        }
        """

# -- Putting it all together --

if __name__ == "__main__":
    awareness = MockAwareness()
    conversation = MockConversation()
    dreaming = MockDreaming()
    llm = MockLLM()

    insight_node = InsightNode(awareness, conversation, dreaming, llm)
    ingenuity_node = IngenuityNode(ros_workspace_path="/tmp/robot_ros_ws")

    suggestions = insight_node.analyze()

    for node in suggestions.get("nodes", []):
        instance = ingenuity_node.create_and_deploy_node(node["name"], node["spec"])
        if instance:
            ingenuity_node.run_node(node["name"])
