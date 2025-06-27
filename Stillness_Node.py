import time
from threading import Event, Thread

class StillnessNode:
    """
    Implements cognitive pause for reflection and consolidation.
    """

    def __init__(self):
        self.is_still = False
        self._pause_event = Event()
        self._pause_event.set()  # Initially not paused

    def enter_stillness(self, duration: float):
        """
        Enter stillness state for 'duration' seconds.
        Non-blocking, runs in background.
        """
        if self.is_still:
            print("Already in stillness.")
            return
        self.is_still = True
        self._pause_event.clear()

        def _stillness_timer():
            print("Entering stillness...")
            time.sleep(duration)
            self.exit_stillness()

        Thread(target=_stillness_timer, daemon=True).start()

    def exit_stillness(self):
        """
        Exit stillness state and resume processing.
        """
        self.is_still = False
        self._pause_event.set()
        print("Exiting stillness, resuming cognition.")

    def wait_if_still(self):
        """
        Call this in cognitive loops to pause if stillness active.
        """
        self._pause_event.wait()

# Example usage
if __name__ == "__main__":
    stillness = StillnessNode()
    stillness.enter_stillness(3)  # Pause cognition for 3 seconds

    print("Doing work before stillness...")
    for i in range(5):
        stillness.wait_if_still()
        print(f"Working... step {i+1}")
        time.sleep(1)
