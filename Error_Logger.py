import os
import threading
import datetime

class ErrorLogger:
    def __init__(self, log_path=None):
        if log_path is None:
            self.log_path = os.path.expanduser('~/.ros/conscious_robot/error_log.txt')
        else:
            self.log_path = log_path
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.lock = threading.Lock()

    def log(self, message):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with self.lock:
            with open(self.log_path, 'a') as f:
                f.write(f"[{timestamp}] {message}\n")
