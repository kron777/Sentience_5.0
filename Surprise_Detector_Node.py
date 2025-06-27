from typing import List, Optional

class SurpriseDetectorNode:
    """
    Detects surprises by comparing predicted vs actual outcomes.
    Provides a surprise score and flags anomalies.
    """

    def __init__(self, threshold: float = 0.7):
        self.threshold = threshold  # Surprise threshold
        self.surprise_history: List[float] = []

    def compute_surprise(self, predicted: float, actual: float) -> float:
        """
        Compute surprise as normalized absolute error between predicted and actual.
        Expected inputs: predicted and actual in [0,1]
        """
        error = abs(predicted - actual)
        surprise = error  # direct error as surprise metric
        self.surprise_history.append(surprise)
        return surprise

    def is_surprise(self, surprise_score: float) -> bool:
        return surprise_score >= self.threshold

    def recent_surprises(self, window: Optional[int] = 10) -> List[float]:
        return self.surprise_history[-window:]

    def summary(self) -> dict:
        count_surprises = sum(1 for s in self.surprise_history if self.is_surprise(s))
        total = len(self.surprise_history)
        return {
            "total_events": total,
            "surprises_detected": count_surprises,
            "surprise_rate": count_surprises / total if total > 0 else 0,
        }

# Example usage
if __name__ == "__main__":
    detector = SurpriseDetectorNode(threshold=0.5)
    tests = [(0.8, 0.75), (0.4, 0.9), (0.3, 0.35), (0.2, 0.9), (0.7, 0.1)]
    for pred, act in tests:
        score = detector.compute_surprise(pred, act)
        print(f"Predicted: {pred}, Actual: {act}, Surprise: {score:.2f}, Is Surprise? {detector.is_surprise(score)}")

    print(detector.summary())
