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
        SensoryQualiaState,
        EmotionState,
        MotivationState,
        ValueDriftState,
        WorldModelState,
    )
except ImportError:
    rospy.logwarn("Custom ROS messages for 'sentience' package not found. Using String for fallback.")
    CognitiveDirective = String
    SensoryQualiaState = String
    EmotionState = String
    MotivationState = String
    ValueDriftState = String
    WorldModelState = String

from sentience.scripts.utils import parse_ros_message_data

class FeedbackLoopNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('feedback_loop_node', anonymous=False)
        self.node_name = rospy.get_name()

        # Parameters
        self.db_path = os.path.expanduser(rospy.get_param('~db_path', '~/.ros/conscious_robot/feedback_log.db'))
        self.traits_path = os.path.expanduser(rospy.get_param('~traits_path', '~/.ros/conscious_robot/default_character_traits.json'))
        self.update_interval = rospy.get_param('~update_interval', 0.5)
        self.max_feedback_history = rospy.get_param('~max_feedback_history', 50)  # Prompt 3
        self.batch_size = rospy.get_param('~batch_size', 50)  # Prompt 4
        self.log_flush_interval_s = rospy.get_param('~log_flush_interval_s', 10.0)
        self.trait_update_interval_s = rospy.get_param('~trait_update_interval_s', 60.0)
        self.llm_endpoint = rospy.get_param('~llm_endpoint', 'http://localhost:8080/phi2')
        self.learning_rate = rospy.get_param('~learning_rate', 0.01)

        # Load character traits
        self.character_traits = self._load_character_traits()

        # Internal state
        self.current_feedback = {'type': 'none', 'target_node': 'none', 'adjustment': '{}'}
        self.feedback_history = deque(maxlen=self.max_feedback_history)  # Prompt 3
        self.log_buffer = deque(maxlen=self.batch_size)  # Prompt 4
        self.trait_update_buffer = deque(maxlen=self.batch_size)
        self.latest_states = {
            'actuator': None,
            'sensory_qualia': None,
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
            CREATE TABLE IF NOT EXISTS feedback_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                feedback_type TEXT,
                target_node TEXT,
                adjustment TEXT,
                confidence_score REAL,
                contributing_factors TEXT
            )
        ''')
        self.conn.commit()

        # Publishers
        self.pub_directive = rospy.Publisher('/cognitive_directives', CognitiveDirective, queue_size=10)
        self.pub_feedback = rospy.Publisher('/action_feedback', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/actuator_commands', String, self.actuator_callback)
        rospy.Subscriber('/sensory_qualia_state', SensoryQualiaState, self.sensory_qualia_callback)
        rospy.Subscriber('/emotion_state', EmotionState, self.emotion_state_callback)
        rospy.Subscriber('/motivation_state', MotivationState, self.motivation_state_callback)
        rospy.Subscriber('/value_drift_state', ValueDriftState, self.value_drift_state_callback)
        rospy.Subscriber('/world_model_state', WorldModelState, self.world_model_callback)

        # Timers
        rospy.Timer(rospy.Duration(self.update_interval), self.process_feedback)
        rospy.Timer(rospy.Duration(self.log_flush_interval_s), self.flush_log_buffer)
        rospy.Timer(rospy.Duration(self.trait_update_interval_s), self.flush_trait_updates)

        # Asyncio setup (Prompt 20)
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop.run_until_complete(self._async_init())

        rospy.loginfo(f"{self.node_name}: Feedback Loop Node initialized.")

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
                "personality": {"adaptability": {"value": 0.7, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}},
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

    def sensory_qualia_callback(self, msg):
        """Handle incoming sensory qualia data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'sensory_data_json': ('{}', 'sensory_data_json'),
            'salience_score': (0.0, 'salience_score'),
        }
        self.latest_states['sensory_qualia'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

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

    async def _async_process_feedback(self):
        """Process action outcomes and generate feedback (Prompt 1, 20)."""
        actuator = self.latest_states['actuator'] or {'action_type': 'none', 'parameters': '{}', 'target': 'none', 'confidence_score': 0.0}
        sensory_qualia = self.latest_states['sensory_qualia'] or {'sensory_data_json': '{}', 'salience_score': 0.0}
        emotion = self.latest_states['emotion'] or {'mood': 'neutral', 'sentiment_score': 0.0}
        motivation = self.latest_states['motivation'] or {'dominant_goal_id': 'none', 'overall_drive_level': 0.0}
        value_drift = self.latest_states['value_drift'] or {'drift_score': 0.0}
        world_model = self.latest_states['world_model'] or {'entities_json': '[]'}

        sensory_data = json.loads(sensory_qualia['sensory_data_json'] or '{}')
        entities = json.loads(world_model['entities_json'] or '[]')
        action_parameters = json.loads(actuator['parameters'] or '{}')

        # Simplified LLM input for feedback generation
        llm_input = {
            'action_type': actuator['action_type'],
            'action_parameters': action_parameters,
            'action_target': actuator['target'],
            'action_confidence': actuator['confidence_score'],
            'sensory_context': sensory_data.get('context', 'neutral'),
            'salience_score': sensory_qualia['salience_score'],
            'mood': emotion['mood'],
            'goal_id': motivation['dominant_goal_id'],
            'drift_score': value_drift['drift_score'],
            'entities': entities,
            'recent_feedback': [{'type': e['type'], 'target_node': e['target_node']} for e in list(self.feedback_history)[-5:]],  # Limit history
            'traits': {
                'adaptability': self.character_traits['personality'].get('adaptability', {}).get('value', 0.7),
                'empathy': self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8),
            }
        }

        # Query LLM for feedback
        try:
            async with self.http_session.post(self.llm_endpoint, json=llm_input, timeout=5.0) as response:
                if response.status == 200:
                    llm_output = await response.json()
                    feedback_type = llm_output.get('feedback_type', 'none')
                    target_node = llm_output.get('target_node', 'none')
                    adjustment = llm_output.get('adjustment', {})
                    confidence_score = llm_output.get('confidence', 0.5)
                    contributing_factors = llm_output.get('factors', {})
                else:
                    rospy.logwarn(f"{self.node_name}: LLM request failed with status {response.status}")
                    feedback_type, target_node, adjustment, confidence_score, contributing_factors = 'none', 'none', {}, 0.3, {}
        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            rospy.logerr(f"{self.node_name}: LLM request failed: {e}")
            feedback_type, target_node, adjustment, confidence_score, contributing_factors = 'none', 'none', {}, 0.3, {}

        # Adjust feedback based on traits and inputs
        adaptability = self.character_traits['personality'].get('adaptability', {}).get('value', 0.7)
        empathy = self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8)

        if actuator['action_type'] == 'speak' and sensory_data.get('user_reaction', 'neutral') == 'positive' and empathy > 0.7:
            feedback_type = 'positive_reinforcement' if feedback_type == 'none' else feedback_type
            target_node = 'cognitive_reasoning_node' if target_node == 'none' else target_node
            adjustment = {'action': 'increase_empathy', 'value': 0.05} if not adjustment else adjustment
            self._update_character_traits('personality', 'adaptability', min(1.0, adaptability + 0.05), 'positive_outcome')
        if actuator['action_type'] != 'none' and sensory_data.get('user_reaction', 'neutral') == 'negative':
            feedback_type = 'corrective' if feedback_type == 'none' else feedback_type
            target_node = 'cognitive_control_node' if target_node == 'none' else target_node
            adjustment = {'action': 'reassess_directive', 'priority': 0.7} if not adjustment else adjustment
        if value_drift['drift_score'] > 0.5:
            feedback_type = 'ethical_adjustment' if feedback_type == 'none' else feedback_type
            target_node = 'value_drift_monitor_node' if target_node == 'none' else target_node
            adjustment = {'action': 'reassess_values', 'priority': 0.8} if not adjustment else adjustment
        if motivation['dominant_goal_id'] == 'user_assistance' and adaptability > 0.6:
            adjustment['priority'] = adjustment.get('priority', 0.5) + 0.1
            confidence_score = min(1.0, confidence_score + 0.1)

        adjustment_json = json.dumps(adjustment)
        return feedback_type, target_node, adjustment_json, confidence_score, contributing_factors

    def process_feedback(self, event):
        """Periodic feedback processing (Prompt 1, 3, 4)."""
        timestamp = str(rospy.get_time())
        feedback_type, target_node, adjustment_json, confidence_score, contributing_factors = self.loop.run_until_complete(self._async_process_feedback())

        self.current_feedback = {
            'type': feedback_type,
            'target_node': target_node,
            'adjustment': adjustment_json
        }
        self.feedback_history.append({
            'type': feedback_type,
            'target_node': target_node,
            'adjustment': adjustment_json,
            'timestamp': timestamp
        })

        # Publish feedback as directive
        self.publish_directive(timestamp, feedback_type, target_node, adjustment_json, confidence_score, contributing_factors)

        # Publish feedback to action_feedback topic
        self.publish_feedback(timestamp, feedback_type, target_node, adjustment_json, confidence_score, contributing_factors)

        # Log state
        self.log_buffer.append((
            timestamp,
            feedback_type,
            target_node,
            adjustment_json,
            confidence_score,
            json.dumps(contributing_factors)
        ))

        rospy.loginfo(f"{self.node_name}: Feedback: {feedback_type}, Target: {target_node}, Confidence: {confidence_score}")

    def publish_directive(self, timestamp, feedback_type, target_node, adjustment_json, confidence_score, contributing_factors):
        """Publish feedback as a cognitive directive."""
        try:
            if isinstance(CognitiveDirective, type(String)):
                directive_data = {
                    'timestamp': timestamp,
                    'directive_type': feedback_type,
                    'target_node': target_node,
                    'command_payload': adjustment_json,
                    'confidence_score': confidence_score,
                    'contributing_factors': contributing_factors,
                }
                self.pub_directive.publish(json.dumps(directive_data))
            else:
                msg = CognitiveDirective()
                msg.timestamp = timestamp
                msg.directive_type = feedback_type
                msg.target_node = target_node
                msg.command_payload = adjustment_json
                msg.confidence_score = confidence_score
                msg.contributing_factors_json = json.dumps(contributing_factors)
                self.pub_directive.publish(msg)
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to publish directive: {e}")

    def publish_feedback(self, timestamp, feedback_type, target_node, adjustment_json, confidence_score, contributing_factors):
        """Publish feedback to action_feedback topic."""
        try:
            feedback_data = {
                'timestamp': timestamp,
                'feedback_type': feedback_type,
                'target_node': target_node,
                'adjustment': adjustment_json,
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
                INSERT INTO feedback_log (timestamp, feedback_type, target_node, adjustment, confidence_score, contributing_factors)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', entries)
            self.conn.commit()
            rospy.loginfo(f"{self.node_name}: Flushed {len(entries)} feedback log entries to DB.")
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
        node = FeedbackLoopNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")