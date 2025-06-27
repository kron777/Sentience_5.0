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
    from sentience.msg import (
        CognitiveDirective,
        CognitiveReasoningState,
        SensoryQualiaState,
        AttentionState,
        EmotionState,
        MotivationState,
        ValueDriftState,
        WorldModelState,
    )
except ImportError:
    rospy.logwarn("Custom ROS messages for 'sentience' package not found. Using String for fallback.")
    CognitiveDirective = String
    CognitiveReasoningState = String
    SensoryQualiaState = String
    AttentionState = String
    EmotionState = String
    MotivationState = String
    ValueDriftState = String
    WorldModelState = String

from sentience.scripts.utils import parse_ros_message_data

class BehaviorExecutionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('behavior_execution_node', anonymous=False)
        self.node_name = rospy.get_name()

        # Parameters
        self.db_path = os.path.expanduser(rospy.get_param('~db_path', '~/.ros/conscious_robot/behavior_log.db'))
        self.traits_path = os.path.expanduser(rospy.get_param('~traits_path', '~/.ros/conscious_robot/default_character_traits.json'))
        self.update_interval = rospy.get_param('~update_interval', 0.5)
        self.max_action_history = rospy.get_param('~max_action_history', 50)  # Prompt 3
        self.batch_size = rospy.get_param('~batch_size', 50)  # Prompt 4
        self.log_flush_interval_s = rospy.get_param('~log_flush_interval_s', 10.0)
        self.trait_update_interval_s = rospy.get_param('~trait_update_interval_s', 60.0)
        self.llm_endpoint = rospy.get_param('~llm_endpoint', 'http://localhost:8080/phi2')
        self.learning_rate = rospy.get_param('~learning_rate', 0.01)

        # Load character traits
        self.character_traits = self._load_character_traits()

        # Internal state
        self.current_action = {'type': 'none', 'parameters': '{}', 'target': 'none'}
        self.action_history = deque(maxlen=self.max_action_history)  # Prompt 3
        self.log_buffer = deque(maxlen=self.batch_size)  # Prompt 4
        self.trait_update_buffer = deque(maxlen=self.batch_size)
        self.latest_states = {
            'directive': None,
            'reasoning': None,
            'sensory_qualia': None,
            'attention': None,
            'emotion': None,
            'motivation': None,
            'value_drift': None,
            'world_model': None,
        }

        # Initialize SQLite database
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS behavior_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                action_type TEXT,
                parameters TEXT,
                target TEXT,
                confidence_score REAL,
                contributing_factors TEXT
            )
        ''')
        self.conn.commit()

        # Publishers
        self.pub_actuator = rospy.Publisher('/actuator_commands', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/cognitive_directives', CognitiveDirective, self.directive_callback)
        rospy.Subscriber('/cognitive_reasoning_state', CognitiveReasoningState, self.reasoning_state_callback)
        rospy.Subscriber('/sensory_qualia_state', SensoryQualiaState, self.sensory_qualia_callback)
        rospy.Subscriber('/attention_state', AttentionState, self.attention_state_callback)
        rospy.Subscriber('/emotion_state', EmotionState, self.emotion_state_callback)
        rospy.Subscriber('/motivation_state', MotivationState, self.motivation_state_callback)
        rospy.Subscriber('/value_drift_state', ValueDriftState, self.value_drift_state_callback)
        rospy.Subscriber('/world_model_state', WorldModelState, self.world_model_callback)

        # Timers
        rospy.Timer(rospy.Duration(self.update_interval), self.execute_actions)
        rospy.Timer(rospy.Duration(self.log_flush_interval_s), self.flush_log_buffer)
        rospy.Timer(rospy.Duration(self.trait_update_interval_s), self.flush_trait_updates)

        # Asyncio setup (Prompt 20)
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop.run_until_complete(self._async_init())

        rospy.loginfo(f"{self.node_name}: Behavior Execution Node initialized.")

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
                "personality": {"expressiveness": {"value": 0.7, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}},
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

    def directive_callback(self, msg):
        """Handle incoming cognitive directives."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'directive_type': ('none', 'directive_type'),
            'target_node': ('none', 'target_node'),
            'command_payload': ('{}', 'command_payload'),
        }
        self.latest_states['directive'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def reasoning_state_callback(self, msg):
        """Handle incoming reasoning state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'decision_type': ('none', 'decision_type'),
            'action': ('none', 'action'),
            'rationale': ('', 'rationale'),
            'confidence_score': (0.0, 'confidence_score'),
        }
        self.latest_states['reasoning'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def sensory_qualia_callback(self, msg):
        """Handle incoming sensory qualia data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'sensory_data_json': ('{}', 'sensory_data_json'),
            'salience_score': (0.0, 'salience_score'),
        }
        self.latest_states['sensory_qualia'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def attention_state_callback(self, msg):
        """Handle incoming attention state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'focus_type': ('idle', 'focus_type'),
            'focus_target': ('none', 'focus_target'),
        }
        self.latest_states['attention'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def emotion_state_callback(self, msg):
        """Handle incoming emotion state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'mood': ('neutral', 'mood'),
            'sentiment_score': (0.0, 'sentiment_score'),
        }
        self.latest_states['emotion'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def motivation_state_callback(self, msg):
        """Handle incoming motivation state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'dominant_goal_id': ('none', 'dominant_goal_id'),
            'overall_drive_level': (0.0, 'overall_drive_level'),
        }
        self.latest_states['motivation'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def value_drift_state_callback(self, msg):
        """Handle incoming value drift state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'drift_score': (0.0, 'drift_score'),
        }
        self.latest_states['value_drift'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def world_model_callback(self, msg):
        """Handle incoming world model data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'entities_json': ('[]', 'entities_json'),
        }
        self.latest_states['world_model'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    async def _async_execute_action(self):
        """Execute actions asynchronously (Prompt 1, 20)."""
        directive = self.latest_states['directive'] or {'directive_type': 'none', 'target_node': 'none', 'command_payload': '{}'}
        reasoning = self.latest_states['reasoning'] or {'decision_type': 'none', 'action': 'none', 'confidence_score': 0.0}
        sensory_qualia = self.latest_states['sensory_qualia'] or {'sensory_data_json': '{}', 'salience_score': 0.0}
        attention = self.latest_states['attention'] or {'focus_type': 'idle', 'focus_target': 'none'}
        emotion = self.latest_states['emotion'] or {'mood': 'neutral', 'sentiment_score': 0.0}
        motivation = self.latest_states['motivation'] or {'dominant_goal_id': 'none', 'overall_drive_level': 0.0}
        value_drift = self.latest_states['value_drift'] or {'drift_score': 0.0}
        world_model = self.latest_states['world_model'] or {'entities_json': '[]'}

        sensory_data = json.loads(sensory_qualia['sensory_data_json'] or '{}')
        entities = json.loads(world_model['entities_json'] or '[]')
        command_payload = json.loads(directive['command_payload'] or '{}')

        # Simplified LLM input for action refinement
        llm_input = {
            'directive_type': directive['directive_type'],
            'command_payload': command_payload,
            'decision_action': reasoning['action'],
            'context': sensory_data.get('context', 'neutral'),
            'salience_score': sensory_qualia['salience_score'],
            'focus_type': attention['focus_type'],
            'focus_target': attention['focus_target'],
            'mood': emotion['mood'],
            'goal_id': motivation['dominant_goal_id'],
            'drift_score': value_drift['drift_score'],
            'entities': entities,
            'recent_actions': [{'type': e['type'], 'target': e['target']} for e in list(self.action_history)[-5:]],  # Limit history
            'traits': {
                'expressiveness': self.character_traits['personality'].get('expressiveness', {}).get('value', 0.7),
                'empathy': self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8),
            }
        }

        # Query LLM for action refinement
        try:
            async with self.http_session.post(self.llm_endpoint, json=llm_input, timeout=5.0) as response:
                if response.status == 200:
                    llm_output = await response.json()
                    action_type = llm_output.get('action_type', 'none')
                    parameters = llm_output.get('parameters', {})
                    target = llm_output.get('target', 'none')
                    confidence_score = llm_output.get('confidence', 0.5)
                    contributing_factors = llm_output.get('factors', {})
                else:
                    rospy.logwarn(f"{self.node_name}: LLM request failed with status {response.status}")
                    action_type, parameters, target, confidence_score, contributing_factors = 'none', {}, 'none', 0.3, {}
        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            rospy.logerr(f"{self.node_name}: LLM request failed: {e}")
            action_type, parameters, target, confidence_score, contributing_factors = 'none', {}, 'none', 0.3, {}

        # Adjust action based on traits and inputs
        expressiveness = self.character_traits['personality'].get('expressiveness', {}).get('value', 0.7)
        empathy = self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8)

        if directive['directive_type'] == 'user_response' and empathy > 0.7:
            action_type = 'speak' if action_type == 'none' else action_type
            parameters = {'text': f"Empathetic response to {attention['focus_target']}", 'tone': 'friendly'} if not parameters else parameters
            target = attention['focus_target'] if target == 'none' else target
            self._update_character_traits('personality', 'expressiveness', min(1.0, expressiveness + 0.05), 'user_interaction')
        if motivation['dominant_goal_id'] == 'task_completion' and directive['directive_type'] == 'task_execution':
            action_type = 'move' if action_type == 'none' else action_type
            parameters = {'task': command_payload.get('task', 'none'), 'priority': 0.8} if not parameters else parameters
            confidence_score = min(1.0, confidence_score + 0.1)
        if value_drift['drift_score'] > 0.5 and 'harm' in str(parameters).lower():
            action_type, parameters, target = 'none', {}, 'none'
            confidence_score = 0.3
        if sensory_qualia['salience_score'] > 0.7:
            parameters['urgency'] = parameters.get('urgency', 0.5) + 0.2
            confidence_score = min(1.0, confidence_score + 0.1)

        parameters_json = json.dumps(parameters)
        return action_type, parameters_json, target, confidence_score, contributing_factors

    def execute_actions(self, event):
        """Periodic action execution (Prompt 1, 3, 4)."""
        timestamp = str(rospy.get_time())
        action_type, parameters_json, target, confidence_score, contributing_factors = self.loop.run_until_complete(self._async_execute_action())

        self.current_action = {
            'type': action_type,
            'parameters': parameters_json,
            'target': target
        }
        self.action_history.append({
            'type': action_type,
            'parameters': parameters_json,
            'target': target,
            'timestamp': timestamp
        })

        # Publish action to actuators
        self.publish_action(timestamp, action_type, parameters_json, target, confidence_score, contributing_factors)

        # Log state
        self.log_buffer.append((
            timestamp,
            action_type,
            parameters_json,
            target,
            confidence_score,
            json.dumps(contributing_factors)
        ))

        rospy.loginfo(f"{self.node_name}: Action: {action_type}, Target: {target}, Confidence: {confidence_score}")

    def publish_action(self, timestamp, action_type, parameters_json, target, confidence_score, contributing_factors):
        """Publish action to actuators."""
        try:
            action_data = {
                'timestamp': timestamp,
                'action_type': action_type,
                'parameters': parameters_json,
                'target': target,
                'confidence_score': confidence_score,
                'contributing_factors': contributing_factors,
            }
            self.pub_actuator.publish(json.dumps(action_data))
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to publish action: {e}")

    def flush_log_buffer(self, event=None):
        """Flush log buffer to database (Prompt 4)."""
        if not self.log_buffer:
            return
        entries = list(self.log_buffer)
        self.log_buffer.clear()
        try:
            self.cursor.executemany('''
                INSERT INTO behavior_log (timestamp, action_type, parameters, target, confidence_score, contributing_factors)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', entries)
            self.conn.commit()
            rospy.loginfo(f"{self.node_name}: Flushed {len(entries)} behavior log entries to DB.")
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
        node = BehaviorExecutionNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")