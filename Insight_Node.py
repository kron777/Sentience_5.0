import json

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
        # Assuming raw_response is a string, try to parse JSON
        try:
            suggestions = json.loads(raw_response)
        except (json.JSONDecodeError, TypeError):
            print("Warning: LLM response invalid JSON, returning empty suggestions.")
            suggestions = {"nodes": []}
        return suggestions


# Mock Components for demonstration

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
        # Example valid JSON string response from LLM
        return """
        {
            "nodes": [
                {"name": "EnergyManagerNode", "spec": "Manage battery usage efficiently to extend runtime."},
                {"name": "TaskSchedulerNode", "spec": "Optimize task execution order to meet deadlines."}
            ]
        }
        """

# Example usage

if __name__ == "__main__":
    awareness = MockAwareness()
    conversation = MockConversation()
    dreaming = MockDreaming()
    llm = MockLLM()

    insight = InsightNode(awareness, conversation, dreaming, llm)
    suggestions = insight.analyze()

    print("InsightNode Suggestions:")
    for node in suggestions.get("nodes", []):
        print(f"- {node['name']}: {node['spec']}")
