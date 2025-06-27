from typing import Dict, Any, List

class TheoryOfMindNode:
    """
    Models other agents’ mental states for prediction and empathy.
    """

    def __init__(self):
        # Store representations of other agents keyed by their ID/name
        self.agents: Dict[str, Dict[str, Any]] = {}

    def add_or_update_agent(self, agent_id: str, beliefs: List[str], desires: List[str], intentions: List[str]):
        self.agents[agent_id] = {
            "beliefs": beliefs,
            "desires": desires,
            "intentions": intentions
        }

    def get_agent_model(self, agent_id: str) -> Dict[str, Any]:
        return self.agents.get(agent_id, {})

    def predict_agent_behavior(self, agent_id: str) -> str:
        """
        Basic heuristic prediction based on desires and intentions.
        """
        agent = self.agents.get(agent_id)
        if not agent:
            return "Unknown behavior"
        # Simple rule: if strong intention exists, predict that action
        if agent["intentions"]:
            return f"Likely to {agent['intentions'][0]}"
        elif agent["desires"]:
            return f"Probably seeking {agent['desires'][0]}"
        else:
            return "Behavior unclear"

    def summary(self) -> Dict[str, Dict[str, Any]]:
        return self.agents

# Example usage
if __name__ == "__main__":
    tom = TheoryOfMindNode()
    tom.add_or_update_agent("Alice", ["thinks it will rain"], ["wants to stay dry"], ["will carry an umbrella"])
    tom.add_or_update_agent("Bob", ["believes the meeting is canceled"], ["wants to relax"], [])

    print(tom.get_agent_model("Alice"))
    print(tom.predict_agent_behavior("Alice"))
    print(tom.predict_agent_behavior("Bob"))
