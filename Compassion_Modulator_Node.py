from typing import Dict, Any

class CompassionModulatorNode:
    """
    Modulates system behavior by assessing suffering levels of others,
    adjusting responses to prioritize compassion and supportive action.
    """

    def __init__(self):
        # Tracks perceived suffering scores per agent/context [0..1]
        self.suffering_map: Dict[str, float] = {}
        # Compassion modulation factor [0..1], higher means more compassionate
        self.compassion_level: float = 0.5

    def update_suffering(self, agent_id: str, suffering_score: float):
        suffering_score = max(0.0, min(suffering_score, 1.0))
        self.suffering_map[agent_id] = suffering_score
        self._update_compassion_level()

    def _update_compassion_level(self):
        """
        Updates compassion level based on average perceived suffering.
        Simple approach: compassion increases with more suffering observed.
        """
        if not self.suffering_map:
            self.compassion_level = 0.5  # baseline
            return
        avg_suffering = sum(self.suffering_map.values()) / len(self.suffering_map)
        # Map average suffering to compassion in a soft manner
        self.compassion_level = min(1.0, max(0.5, avg_suffering * 1.5))

    def get_compassion_level(self) -> float:
        return self.compassion_level

    def summary(self) -> Dict[str, Any]:
        return {
            "suffering_map": self.suffering_map,
            "compassion_level": self.compassion_level,
        }

# Example usage
if __name__ == "__main__":
    compassion_node = CompassionModulatorNode()
    compassion_node.update_suffering("agent_A", 0.7)
    compassion_node.update_suffering("agent_B", 0.4)
    compassion_node.update_suffering("agent_C", 0.9)
    print(compassion_node.summary())
