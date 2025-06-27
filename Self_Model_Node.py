from typing import Dict, Any

class SelfModelNode:
    """
    Maintains self-representation and identity.
    Tracks internal states, boundaries, and agent-specific attributes.
    """

    def __init__(self):
        self.identity: Dict[str, Any] = {
            "name": "SentienceAgent",
            "version": "4.0",
            "core_values": [],
            "physical_state": {},
            "emotional_state": {},
            "cognitive_state": {},
            "boundaries": {
                "self_vs_other": True,
                "privacy": True,
            }
        }

    def update_identity(self, key: str, value: Any):
        if key in self.identity:
            self.identity[key] = value
        else:
            # Add new key if needed
            self.identity[key] = value

    def get_identity(self, key: str) -> Any:
        return self.identity.get(key)

    def set_boundary(self, boundary_name: str, status: bool):
        if "boundaries" not in self.identity:
            self.identity["boundaries"] = {}
        self.identity["boundaries"][boundary_name] = status

    def get_boundary(self, boundary_name: str) -> bool:
        return self.identity.get("boundaries", {}).get(boundary_name, False)

    def summary(self) -> Dict[str, Any]:
        return self.identity

# Example usage
if __name__ == "__main__":
    self_model = SelfModelNode()
    self_model.update_identity("core_values", ["compassion", "equanimity"])
    self_model.update_identity("physical_state", {"battery_level": 87})
    self_model.set_boundary("self_vs_other", True)
    print(self_model.summary())
