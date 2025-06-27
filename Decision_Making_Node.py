import logging
import json
from typing import Dict, Optional

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DecisionMaker:
    def __init__(self):
        self.cognition_input: Optional[Dict] = None
        self.emotion_input: Optional[Dict] = None
        self.hardware_status: Optional[Dict] = None
        logger.info("DecisionMaker initialized")

    def receive_cognition(self, data: Dict) -> Dict:
        """Receive and process cognition data."""
        try:
            self.cognition_input = data
            logger.info(f"Received cognition data: {json.dumps(data)}")
            return self.process_decision()
        except Exception as e:
            logger.error(f"Error processing cognition data: {e}")
            return {"action": "error", "priority": "none"}

    def receive_emotion(self, data: Dict) -> Dict:
        """Receive and process emotion data."""
        try:
            self.emotion_input = data
            logger.info(f"Received emotion data: {json.dumps(data)}")
            return self.process_decision()
        except Exception as e:
            logger.error(f"Error processing emotion data: {e}")
            return {"action": "error", "priority": "none"}

    def receive_hardware(self, data: Dict) -> Dict:
        """Receive and process hardware status."""
        try:
            self.hardware_status = data
            logger.info(f"Received hardware status: {json.dumps(data)}")
            return self.process_decision()
        except Exception as e:
            logger.error(f"Error processing hardware data: {e}")
            return {"action": "error", "priority": "none"}

    def process_decision(self) -> Dict:
        """Process decision based on available inputs."""
        if not (self.cognition_input and self.emotion_input and self.hardware_status):
            logger.warning("Missing input data for decision making")
            return {"action": "pending", "priority": "none"}

        try:
            # Extract key values with defaults
            emotion_intensity = self.emotion_input.get("intensity", 0)
            hardware_available = self.hardware_status.get("available", False)
            task_confidence = self.cognition_input.get("confidence", 0)

            # Decision logic
            if emotion_intensity > 0.7:
                return {"action": "respond_emotionally", "priority": "high", "details": self.emotion_input}
            elif hardware_available and task_confidence > 0.5:
                return {"action": "execute_task", "priority": "medium", "details": self.cognition_input}
            else:
                return {"action": "wait", "priority": "low"}
        except Exception as e:
            logger.error(f"Error in decision processing: {e}")
            return {"action": "error", "priority": "none"}

def main():
    """Main function for testing the DecisionMaker."""
    decision_maker = DecisionMaker()
    
    # Simulate receiving data (replace with actual node communication)
    cognition_data = {"task": "analyze", "confidence": 0.9}
    emotion_data = {"intensity": 0.8, "type": "joy"}
    hardware_data = {"available": True, "status": "online"}

    print("Testing cognition input:")
    result = decision_maker.receive_cognition(cognition_data)
    print(f"Decision: {result}")

    print("Testing emotion input:")
    result = decision_maker.receive_emotion(emotion_data)
    print(f"Decision: {result}")

    print("Testing hardware input:")
    result = decision_maker.receive_hardware(hardware_data)
    print(f"Decision: {result}")

if __name__ == "__main__":
    main()