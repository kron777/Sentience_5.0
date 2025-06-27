#!/usr/bin/env python3
import rospy
import sqlite3
import os
import json
import asyncio
import aiohttp
from collections import deque
from std_msgs.msg import String
from uuid import uuid4
from datetime import datetime

try:
    from sentience.msg import EmotionState, ValueDriftState
except ImportError:
    rospy.logwarn("Custom ROS messages for 'sentience' package not found. Using String for fallback.")
    EmotionState = String
    ValueDriftState = String

from sentience.scripts.utils import parse_ros_message_data

class HardwareInterfaceNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('hardware_interface_node', anonymous=False)
        self.node_name = rospy.get_name()

        # Parameters
        self.db_path = os.path.expanduser(rospy.get_param('~db_path', '~/.ros/conscious_robot/hardware_log.db'))
        self.traits_path = os.path.expanduser(rospy.get_param('~traits_path', '~/.ros/conscious_robot/default_character_traits.json'))
        self.update_interval = rospy.get_param('~update_interval', 0.5)
        self.max_execution_history = rospy.get_param('~max_execution_history', 50)  # Prompt 3
        self.batch_size = rospy.get_param('~batch_size', 50)  # Prompt 4
        self.log_flush_interval_s = rospy.get_param('~log_flush_interval_s', 10.0)
        self.trait_update_interval_s = rospy.get_param('~trait_update_interval_s', 60.0)
        self.speech_endpoint = rospy.get_param('~speech_endpoint', 'http://localhost:8081/tts')
        self.motor_endpoint = rospy.get_param('~motor_endpoint', 'http://localhost:8082/motor')
        self.learning_rate = rospy.get_param('~learning_rate', 0.01)

        # Load character traits
        self.character_traits = self._load_character_traits()

        # Internal state
        self.current_execution = {'action_type': 'none', 'parameters': '{}', 'target': 'none'}
        self.execution_history = deque(maxlen=self.max_execution_history)  # Prompt 3
        self.log_buffer = deque(maxlen=self.batch_size)  # Prompt 4
        self.trait_update_buffer = deque(maxlen=self.batch_size)
        self.latest_states = {
            'actuator': None,
            'emotion': None,
            'value_drift': None,
        }

        # Initialize SQLite database
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS hardware_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                action_type TEXT,
                parameters TEXT,
                target TEXT,
                execution_status TEXT,
                confidence_score REAL,
                contributing_factors TEXT
            )
        ''')
        self.conn.commit()

        # Publishers
        self.pub_feedback = rospy.Publisher('/actuator_feedback', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/actuator_commands', String, self.actuator_callback)
        rospy.Subscriber('/emotion_state', EmotionState, self.emotion_state_callback)
        rospy.Subscriber('/value_drift_state', ValueDriftState, self.value_drift_state_callback)

        # Timers
        rospy.Timer(rospy.Duration(self.update_interval), self.execute_hardware_actions)
        rospy.Timer(rospy.Duration(self.log_flush_interval_s), self.flush_log_buffer)
        rospy.Timer(rospy.Duration(self.trait_update_interval_s), self.flush_trait_updates)

        # Asyncio setup (Prompt 20)
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop.run_until_complete(self._async_init())

        rospy.loginfo(f"{self.node_name}: Hardware Interface Node initialized.")

    async def _async_init(self):
        """Initialize async components."""
        self.http_session = aiohttp.ClientSession()
        rospy.logdebug(f"{self.node_name}: Async HTTP session initialized.")

    def _load_character_traits(self):
        """Load default character traits from JSON."""
        try:
            with open(self.traits_path, 'r') as f:
                traits = json.load(f)
            rospy.loginfo(f"{self.node_name}: Loaded character traits from {self.traits_path}")
            return traits
        except (FileNotFoundError, json.JSONDecodeError) as e:
            rospy.logerr(f"{self.node_name}: Failed to load character traits: {e}")
            return {
                "personality": {"precision": {"value": 0.7, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}},
                "emotional_tendencies": {"empathy": {"value": 0.8, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}},
                "behavioral_preferences": {},
                "metadata": {"version": "1.0", "created": "2025-06-26T20:00:00Z", "last_modified": "2025-06-26T20:00:00Z", "update_history": []}
            }

    def _update_character_traits(self, trait_category, trait_key, new_value, source):
        """Update character traits and log changes."""
        try:
            if trait_category not in self.character_traits:
                self.character_traits[trait_category] = {}
            if trait_key not in self.character_traits[trait_category]:
                self.character_traits[trait_category][trait_key] = {"value": 0.5, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}
            self.character_traits[trait_category][trait_key]['value'] = new_value
            self.character_traits[trait_category][trait_key]['weight'] = min(1.0, self.character_traits[trait_category][trait_key]['weight'] + self.learning_rate)
            self.character_traits[trait_category][trait_key]['last_updated'] = datetime.utcnow().isoformat() + 'Z'
            self.character_traits[trait_category][trait_key]['update_source'] = source
            self.character_traits['metadata']['last_modified'] = datetime.utcnow().isoformat() + 'Z'
            self.character_traits['metadata']['update_history'].append({
                'trait': f"{trait_category}.{trait_key}",
                'new_value': new_value,
                'source': source,
                'timestamp': datetime.utcnow().isoformat() + 'Z'
            })
            self.trait_update_buffer.append((trait_category, trait_key, new_value, source))
            rospy.loginfo(f"{self.node_name}: Updated trait {trait_category}.{trait_key} to {new_value} from {source}")
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to update trait {trait_category}.{trait_key}: {e}")

    def actuator_callback(self, msg):
        """Handle incoming actuator commands."""
        try:
            actuator_data = json.loads(msg.data or '{}')
            fields_map = {
                'timestamp': (str(rospy.get_time()), 'timestamp'),
                'action_type': ('none', 'action_type'),
                'parameters': ('{}', 'parameters'),
                'target': ('none', 'target'),
                'confidence_score': (0.0, 'confidence_score'),
            }
            self.latest_states['actuator'] = parse_ros_message_data(actuator_data, fields_map, node_name=self.node_name)
        except json.JSONDecodeError:
            rospy.logwarn(f"{self.node_name}: Failed to parse actuator data: {msg.data}")

    def emotion_state_callback(self, msg):
        """Handle incoming emotion state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'mood': ('neutral', 'mood'),
            'sentiment_score': (0.0, 'sentiment_score'),
        }
        self.latest_states['emotion'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def value_drift_state_callback(self, msg):
        """Handle incoming value drift state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'drift_score': (0.0, 'drift_score'),
        }
        self.latest_states['value_drift'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    async def _async_execute_hardware(self, action_type, parameters, target):
        """Execute hardware actions asynchronously (Prompt 20)."""
        parameters_dict = json.loads(parameters or '{}')
        execution_status = 'success'
        confidence_score = 0.8
        contributing_factors = {'action_type': action_type, 'target': target}

        # Adjust execution based on emotion and value drift
        emotion = self.latest_states['emotion'] or {'mood': 'neutral', 'sentiment_score': 0.0}
        value_drift = self.latest_states['value_drift'] or {'drift_score': 0.0}
        precision = self.character_traits['personality'].get('precision', {}).get('value', 0.7)
        empathy = self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8)

        if value_drift['drift_score'] > 0.5 and 'harm' in str(parameters_dict).lower():
            execution_status = 'blocked'
            confidence_score = 0.3
            contributing_factors['reason'] = 'ethical_violation'
            return execution_status, confidence_score, contributing_factors

        try:
            if action_type == 'speak':
                payload = {'text': parameters_dict.get('text', ''), 'tone': emotion['mood'] if emotion['mood'] != 'neutral' else parameters_dict.get('tone', 'neutral')}
                async with self.http_session.post(self.speech_endpoint, json=payload, timeout=5.0) as response:
                    if response.status != 200:
                        execution_status = 'failed'
                        confidence_score = 0.4
                        contributing_factors['error'] = f"Speech endpoint failed: {response.status}"
            elif action_type == 'move':
                payload = {'target': target, 'speed': parameters_dict.get('speed', 0.5), 'precision': precision}
                async with self.http_session.post(self.motor_endpoint, json=payload, timeout=5.0) as response:
                    if response.status != 200:
                        execution_status = 'failed'
                        confidence_score = 0.4
                        contributing_factors['error'] = f"Motor endpoint failed: {response.status}"
            else:
                execution_status = 'skipped'
                confidence_score = 0.5
                contributing_factors['reason'] = 'unsupported_action'

            if execution_status == 'success' and precision < 0.9:
                self._update_character_traits('personality', 'precision', min(1.0, precision + 0.05), 'successful_execution')
            if execution_status == 'failed':
                self._update_character_traits('personality', 'precision', max(0.0, precision - 0.05), 'failed_execution')
            if action_type == 'speak' and empathy > 0.7:
                contributing_factors['empathy_adjusted'] = True

        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            execution_status = 'failed'
            confidence_score = 0.4
            contributing_factors['error'] = str(e)
            rospy.logerr(f"{self.node_name}: Hardware execution failed: {e}")

        return execution_status, confidence_score, contributing_factors

    def execute_hardware_actions(self, event):
        """Periodic hardware action execution (Prompt 1, 3, 4)."""
        timestamp = str(rospy.get_time())
        actuator = self.latest_states['actuator'] or {'action_type': 'none', 'parameters': '{}', 'target': 'none', 'confidence_score': 0.0}
        action_type = actuator['action_type']
        parameters = actuator['parameters']
        target = actuator['target']

        execution_status, confidence_score, contributing_factors = self.loop.run_until_complete(
            self._async_execute_hardware(action_type, parameters, target)
        )

        self.current_execution = {
            'action_type': action_type,
            'parameters': parameters,
            'target': target
        }
        self.execution_history.append({
            'action_type': action_type,
            'parameters': parameters,
            'target': target,
            'execution_status': execution_status,
            'timestamp': timestamp
        })

        # Publish feedback
        self.publish_feedback(timestamp, action_type, parameters, target, execution_status, confidence_score, contributing_factors)

        # Log state
        self.log_buffer.append((
            timestamp,
            action_type,
            parameters,
            target,
            execution_status,
            confidence_score,
            json.dumps(contributing_factors)
        ))

        rospy.loginfo(f"{self.node_name}: Executed {action_type} on {target}, Status: {execution_status}, Confidence: {confidence_score}")

    def publish_feedback(self, timestamp, action_type, parameters, target, execution_status, confidence_score, contributing_factors):
        """Publish execution feedback."""
        try:
            feedback_data = {
                'timestamp': timestamp,
                'action_type': action_type,
                'parameters': parameters,
                'target': target,
                'execution_status': execution_status,
                'confidence_score': confidence_score,
                'contributing_factors': contributing_factors,
            }
            self.pub_feedback.publish(json.dumps(feedback_data))
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to publish feedback: {e}")

    def flush_log_buffer(self, event=None):
        """Flush log buffer to database (Prompt 4)."""
        if not self.log_buffer:
            return
        entries = list(self.log_buffer)
        self.log_buffer.clear()
        try:
            self.cursor.executemany('''
                INSERT INTO hardware_log (timestamp, action_type, parameters, target, execution_status, confidence_score, contributing_factors)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', entries)
            self.conn.commit()
            rospy.loginfo(f"{self.node_name}: Flushed {len(entries)} hardware log entries to DB.")
        except sqlite3.Error as e:
            rospy.logerr(f"{self.node_name}: Failed to flush log buffer: {e}")
            for entry in entries:
                self.log_buffer.append(entry)

    def flush_trait_updates(self, event=None):
        """Flush trait updates to JSON file."""
        if not self.trait_update_buffer:
            return
        try:
            with open(self.traits_path, 'w') as f:
                json.dump(self.character_traits, f, indent=2)
            entries = list(self.trait_update_buffer)
            self.trait_update_buffer.clear()
            rospy.loginfo(f"{self.node_name}: Flushed {len(entries)} trait updates to {self.traits_path}")
        except (IOError, json.JSONDecodeError) as e:
            rospy.logerr(f"{self.node_name}: Failed to flush trait updates: {e}")
            for entry in entries:
                self.trait_update_buffer.append(entry)

    def run(self):
        """Run the node with asyncio integration."""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo(f"{self.node_name}: Interrupted by ROS shutdown.")
        finally:
            self.loop.run_until_complete(self.http_session.close())
            self.flush_log_buffer()
            self.flush_trait_updates()
            if self.conn:
                self.conn.close()

if __name__ == '__main__':
    try:
        node = HardwareInterfaceNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")