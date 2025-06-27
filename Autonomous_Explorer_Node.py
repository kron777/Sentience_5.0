import random
from typing import Set, Dict, Any

class AutonomousExplorerNode:
    """
    Drives curiosity and autonomous exploration by seeking novelty.
    """

    def __init__(self):
        self.known_states: Set[Any] = set()
        self.exploration_history: Dict[Any, float] = {}  # state -> novelty score
        self.novelty_threshold = 0.5  # threshold to consider a state novel

    def evaluate_state_novelty(self, state: Any) -> float:
        """
        Simple novelty function: lower if state known, higher if unknown.
        For demonstration, novelty is 1.0 if unseen, else decays with visits.
        """
        if state not in self.known_states:
            return 1.0
        else:
            # Decay novelty based on how often visited
            visits = sum(1 for s in self.exploration_history if s == state)
            return max(0.0, 1.0 - 0.1 * visits)

    def explore(self, candidate_states: Set[Any]) -> Any:
        """
        Choose the most novel state to explore next from candidates.
        """
        novelty_scores = {state: self.evaluate_state_novelty(state) for state in candidate_states}
        novel_states = {s: score for s, score in novelty_scores.items() if score >= self.novelty_threshold}
        
        if novel_states:
            chosen = max(novel_states, key=novel_states.get)
        else:
            # No novel states, fallback to random choice
            chosen = random.choice(list(candidate_states))
        
        # Update knowledge base
        self.known_states.add(chosen)
        self.exploration_history[chosen] = novelty_scores.get(chosen, 0.0)
        return chosen

    def summary(self) -> Dict[str, Any]:
        return {
            "known_states_count": len(self.known_states),
            "exploration_history": self.exploration_history
        }

# Example usage
if __name__ == "__main__":
    explorer = AutonomousExplorerNode()
    candidates = {"state1", "state2", "state3"}

    for _ in range(5):
        next_state = explorer.explore(candidates)
        print(f"Exploring: {next_state}")

    print(explorer.summary())
