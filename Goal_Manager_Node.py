from typing import List, Dict, Optional, Tuple
import heapq

class Goal:
    def __init__(self, name: str, priority: float, dependencies: Optional[List[str]] = None):
        self.name = name
        self.priority = priority  # Higher value = higher priority
        self.dependencies = dependencies or []
        self.status = "pending"  # pending, active, completed, blocked

    def __lt__(self, other):
        # For priority queue (heapq), invert priority for max-heap behavior
        return self.priority > other.priority

class GoalManagerNode:
    """
    Manages and prioritizes multiple goals, handling conflicts and dependencies.
    """

    def __init__(self):
        self.goals: Dict[str, Goal] = {}
        self.active_goals: List[Goal] = []

    def add_goal(self, goal: Goal):
        self.goals[goal.name] = goal

    def update_goal_priority(self, name: str, priority: float):
        if name in self.goals:
            self.goals[name].priority = priority

    def resolve_conflicts(self):
        """
        Simple conflict resolution by priority and dependencies.
        """
        # Reset active goals list
        self.active_goals = []

        # Filter out blocked goals due to unmet dependencies
        available_goals = [
            g for g in self.goals.values()
            if all(self.goals.get(dep, Goal("", 0)).status == "completed" for dep in g.dependencies)
        ]

        # Sort by priority descending
        sorted_goals = sorted(available_goals, key=lambda g: g.priority, reverse=True)

        # Activate highest priority goals first
        for goal in sorted_goals:
            if goal.status == "pending":
                goal.status = "active"
                self.active_goals.append(goal)

    def complete_goal(self, name: str):
        if name in self.goals:
            self.goals[name].status = "completed"
            self.resolve_conflicts()  # Reevaluate goals

    def get_active_goals(self) -> List[Tuple[str, float]]:
        return [(g.name, g.priority) for g in self.active_goals]

    def summary(self) -> Dict[str, List[Tuple[str, float]]]:
        """
        Returns goals grouped by status.
        """
        summary = {
            "pending": [],
            "active": [],
            "completed": [],
            "blocked": []
        }
        for g in self.goals.values():
            summary[g.status].append((g.name, g.priority))
        return summary

# Example usage
if __name__ == "__main__":
    gm = GoalManagerNode()
    gm.add_goal(Goal("ChargeBattery", 0.9))
    gm.add_goal(Goal("RespondToUser", 1.0))
    gm.add_goal(Goal("DataBackup", 0.5, dependencies=["ChargeBattery"]))
    gm.resolve_conflicts()
    print("Active goals:", gm.get_active_goals())
    gm.complete_goal("RespondToUser")
    print("Goals summary:", gm.summary())
