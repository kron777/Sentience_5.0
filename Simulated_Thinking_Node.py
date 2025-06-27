from typing import Callable, List, Any, Dict

class SimulatedThinkingNode:
    """
    Runs internal simulations of scenarios and predicts outcomes.
    """

    def __init__(self):
        # Store scenarios as callables that return some evaluation metric
        self.scenarios: List[Callable[[], Any]] = []

    def add_scenario(self, scenario_func: Callable[[], Any]):
        self.scenarios.append(scenario_func)

    def run_simulations(self) -> Dict[int, Any]:
        """
        Executes all stored scenarios and collects their outcomes.
        Returns a mapping from scenario index to results.
        """
        results = {}
        for idx, scenario in enumerate(self.scenarios):
            try:
                result = scenario()
                results[idx] = result
            except Exception as e:
                results[idx] = f"Error: {e}"
        return results

    def clear_scenarios(self):
        self.scenarios.clear()

# Example usage
if __name__ == "__main__":
    sim_node = SimulatedThinkingNode()

    # Add dummy scenarios
    sim_node.add_scenario(lambda: "Outcome A: success with 80% probability")
    sim_node.add_scenario(lambda: 42)  # some evaluation metric
    sim_node.add_scenario(lambda: 1 / 0)  # scenario that raises an error

    results = sim_node.run_simulations()
    for idx, outcome in results.items():
        print(f"Scenario {idx}: {outcome}")
