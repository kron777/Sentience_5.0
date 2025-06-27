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
        CognitiveReasoningState,
        CognitiveDirective,
        SensoryQualiaState,
        AttentionState,
        EmotionState,
        MotivationState,
        ValueDriftState,
        WorldModelState,
    )
except ImportError:
    rospy.logwarn("Custom ROS messages for 'sentience' package not found. Using String for fallback.")
    CognitiveReasoningState = String
    CognitiveDirective = String
    SensoryQualiaState = String
    AttentionState = String
    EmotionState = String
    MotivationState = String
    ValueDriftState = String
    WorldModelState = String

from sentience.scripts.utils import parse_ros_message_data

class CognitiveReasoningNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('cognitive_reasoning_node', anonymous=False)
        self.node_name = rospy.get_name()

        # Parameters
        self.db_path = os.path.expanduser(rospy.get_param('~db_path', '~/.ros/conscious_robot/reasoning_log.db'))
        self.traits_path = os.path.expanduser(rospy.get_param('~traits_path', '~/.ros/conscious_robot/default_character_traits.json'))
        self.update_interval = rospy.get_param('~update_interval', 0.5)
        self.max_reasoning_history = rospy.get_param('~max_reasoning_history', 50)  # Prompt 3
        self.batch_size = rospy.get_param('~batch_size', 50)  # Prompt 4
        self.log_flush_interval_s = rospy.get_param('~log_flush_interval_s', 10.0)
        self.trait_update_interval_s = rospy.get_param('~trait_update_interval_s', 60.0)
        self.llm_endpoint = rospy.get_param('~llm_endpoint', 'http://localhost:8080/phi2')
        self.learning_rate = rospy.get_param('~learning_rate', 0.01)

        # Load character traits
        self.character_traits = self._load_character_traits()

        # Internal state
        self.current_decision = {'type': 'none', 'action': 'none', 'rationale': ''}
        self.reasoning_history = deque(maxlen=self.max_reasoning_history)  # Prompt 3
        self.log_buffer = deque(maxlen=self.batch_size)  # Prompt 4
        self.trait_update_buffer = deque(maxlen=self.batch_size)
        self.latest_states = {
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
            CREATE TABLE IF NOT EXISTS reasoning_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                decision_type TEXT,
                action TEXT,
                rationale TEXT,
                confidence_score REAL,
                contributing_factors TEXT
            )
        ''')
        self.conn.commit()

        # Publishers
        self.pub_reasoning_state = rospy.Publisher('/cognitive_reasoning_state', CognitiveReasoningState, queue_size=10)

        # Subscribers
        rospy.Subscriber('/sensory_qualia_state', SensoryQualiaState, self.sensory_qualia_callback)
        rospy.Subscriber('/attention_state', AttentionState, self.attention_state_callback)
        rospy.Subscriber('/emotion_state', EmotionState, self.emotion_state_callback)
        rospy.Subscriber('/motivation_state', MotivationState, self.motivation_state_callback)
        rospy.Subscriber('/value_drift_state', ValueDriftState, self.value_drift_state_callback)
        rospy.Subscriber('/world_model_state', WorldModelState, self.world_model_callback)
        rospy.Subscriber('/cognitive_directives', CognitiveDirective, self.directive_callback)

        # Timers
        rospy.Timer(rospy.Duration(self.update_interval), self.perform_reasoning)
        rospy.Timer(rospy.Duration(self.log_flush_interval_s), self.flush_log_buffer)
        rospy.Timer(rospy.Duration(self.trait_update_interval_s), self.flush_trait_updates)

        # Asyncio setup (Prompt 20)
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop.run_until_complete(self._async_init())

        rospy.loginfo(f"{self.node_name}: Cognitive Reasoning Node initialized.")

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
                "personality": {"rationality": {"value": 0.7, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}},
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

    def sensory_qualia_callback(self, msg):
        """Handle incoming sensory qualia data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'sensory_data_json': ('{}', 'sensory_data_json'),
            'salience_score': (0.0, 'salience_score'),
            'novelty_score': (0.0, 'novelty_score'),
        }
        self.latest_states['sensory_qualia'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def attention_state_callback(self, msg):
        """Handle incoming attention state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'focus_type': ('idle', 'focus_type'),
            'focus_target': ('none', 'focus_target'),
            'priority_score': (0.0, 'priority_score'),
        }
        self.latest_states['attention'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def emotion_state_callback(self, msg):
        """Handle incoming emotion state data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'mood': ('neutral', 'mood'),
            'sentiment_score': (0.0, 'sentiment_score'),
            'mood_intensity': (0.0, 'mood_intensity'),
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
            'correction_action': ('none', 'correction_action'),
        }
        self.latest_states['value_drift'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def world_model_callback(self, msg):
        """Handle incoming world model data."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'entities_json': ('[]', 'entities_json'),
        }
        self.latest_states['world_model'] = parse_ros_message_data(msg, fields_map, node_name=self.node_name)

    def directive_callback(self, msg):
        """Handle incoming cognitive directives."""
        fields_map = {
            'timestamp': (str(rospy.get_time()), 'timestamp'),
            'directive_type': ('none', 'directive_type'),
            'target_node': ('none', 'target_node'),
            'command_payload': ('{}', 'command_payload'),
            'reason': ('', 'reason'),
        }
        data = parse_ros_message_data(msg, fields_map, node_name=self.node_name)
        if data['target_node'] in [self.node_name, 'all']:
            try:
                payload = json.loads(data['command_payload'] or '{}')
                reasoning_event = {
                    'id': str(uuid4()),
                    'type': data['directive_type'],
                    'task': payload.get('task', 'none'),
                    'timestamp': data['timestamp'],
                }
                self.reasoning_history.append(reasoning_event)
                if data['directive_type'] == 'complex_decision':
                    self._update_character_traits('personality', 'rationality', min(1.0, self.character_traits['personality'].get('rationality', {}).get('value', 0.7) + 0.05), 'directive')
                rospy.logdebug(f"{self.node_name}: Received directive: {reasoning_event['type']} (ID: {reasoning_event['id']})")
            except json.JSONDecodeError:
                rospy.logwarn(f"{self.node_name}: Failed to parse directive payload: {data['command_payload']}")

    async def _async_perform_reasoning(self):
        """Perform reasoning asynchronously using LLM (Prompt 1, 20)."""
        sensory_qualia = self.latest_states['sensory_qualia'] or {'sensory_data_json': '{}', 'salience_score': 0.0}
        attention = self.latest_states['attention'] or {'focus_type': 'idle', 'focus_target': 'none'}
        emotion = self.latest_states['emotion'] or {'mood': 'neutral', 'sentiment_score': 0.0}
        motivation = self.latest_states['motivation'] or {'dominant_goal_id': 'none', 'overall_drive_level': 0.0}
        value_drift = self.latest_states['value_drift'] or {'drift_score': 0.0}
        world_model = self.latest_states['world_model'] or {'entities_json': '[]'}

        # Simplified LLM input
        llm_input = {
            'context': json.loads(sensory_qualia['sensory_data_json'] or '{}').get('context', 'neutral'),
            'salience_score': sensory_qualia['salience_score'],
            'focus_type': attention['focus_type'],
            'focus_target': attention['focus_target'],
            'mood': emotion['mood'],
            'sentiment_score': emotion['sentiment_score'],
            'goal_id': motivation['dominant_goal_id'],
            'drift_score': value_drift['drift_score'],
            'entities': json.loads(world_model['entities_json'] or '[]'),
            'recent_decisions': [{'type': e['type'], 'action': e['action']} for e in list(self.reasoning_history)[-5:]],  # Limit history for efficiency
            'traits': {
                'rationality': self.character_traits['personality'].get('rationality', {}).get('value', 0.7),
                'empathy': self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8),
            }
        }

        # Query LLM
        try:
            async with self.http_session.post(self.llm_endpoint, json=llm_input, timeout=5.0) as response:
                if response.status == 200:
                    llm_output = await response.json()
                    decision_type = llm_output.get('decision_type', 'none')
                    action = llm_output.get('action', 'none')
                    rationale = llm_output.get('rationale', 'No rationale provided.')
                    confidence_score = llm_output.get('confidence', 0.5)
                    contributing_factors = llm_output.get('factors', {})
                else:
                    rospy.logwarn(f"{self.node_name}: LLM request failed with status {response.status}")
                    decision_type, action, rationale, confidence_score, contributing_factors = 'none', 'none', 'LLM failure', 0.3, {}
        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            rospy.logerr(f"{self.node_name}: LLM request failed: {e}")
            decision_type, action, rationale, confidence_score, contributing_factors = 'none', 'none', 'LLM error', 0.3, {}

        # Adjust decision based on traits and inputs
        rationality = self.character_traits['personality'].get('rationality', {}).get('value', 0.7)
        empathy = self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8)

        if attention['focus_type'] == 'user_interaction' and empathy > 0.7:
            decision_type = 'user_response' if decision_type == 'none' else decision_type
            action = f"Respond to {attention['focus_target']}" if action == 'none' else action
            rationale = f"Prioritized user interaction due to focus and empathy ({empathy})" if action == 'none' else rationale
        if motivation['dominant_goal_id'] in ['task_completion', 'user_assistance'] and rationality > 0.6:
            confidence_score = min(1.0, confidence_score + 0.1)
            rationale += f" Boosted by goal {motivation['dominant_goal_id']} and rationality ({rationality})."
        if value_drift['drift_score'] > 0.5 and 'harm' in rationale.lower():
            decision_type, action = 'none', 'none'
            rationale += " Action blocked due to high value drift."
            self._update_character_traits('personality', 'rationality', min(1.0, rationality + 0.05), 'ethical_alignment')
        if sensory_qualia['salience_score'] > 0.7:
            confidence_score = min(1.0, confidence_score + 0.1)
            rationale += f" High salience ({sensory_qualia['salience_score']}) increased confidence."

        return decision_type, action, rationale, confidence_score, contributing_factors

    def perform_reasoning(self, event):
        """Periodic reasoning evaluation (Prompt 1, 3, 4)."""
        timestamp = str(rospy.get_time())
        decision_type, action, rationale, confidence_score, contributing_factors = self.loop.run_until_complete(self._async_perform_reasoning())

        self.current_decision = {
            'type': decision_type,
            'action': action,
            'rationale': rationale
        }
        self.reasoning_history.append({
            'type': decision_type,
            'action': action,
            'rationale': rationale,
            'timestamp': timestamp
        })

        # Publish reasoning state
        self.publish_reasoning_state(timestamp, decision_type, action, rationale, confidence_score, contributing_factors)

        # Log state
        self.log_buffer.append((
            timestamp,
            decision_type,
            action,
            rationale[:500],  # Truncate for DB efficiency
            confidence_score,
            json.dumps(contributing_factors)
        ))

        rospy.loginfo(f"{self.node_name}: Decision: {decision_type}, Action: {action}, Confidence: {confidence_score}")

    def publish_reasoning_state(self, timestamp, decision_type, action, rationale, confidence_score, contributing_factors):
        """Publish cognitive reasoning state."""
        try:
            if isinstance(CognitiveReasoningState, type(String)):
                state_data = {
                    'timestamp': timestamp,
                    'decision_type': decision_type,
                    'action': action,
                    'rationale': rationale[:500],  # Truncate for message size
                    'confidence_score': confidence_score,
                    'contributing_factors': contributing_factors,
                }
                self.pub_reasoning_state.publish(json.dumps(state_data))
            else:
                msg = CognitiveReasoningState()
                msg.timestamp = timestamp
                msg.decision_type = decision_type
                msg.action = action
                msg.rationale = rationale[:500]  # Truncate for message size
                msg.confidence_score = confidence_score
                msg.contributing_factors_json = json.dumps(contributing_factors)
                self.pub_reasoning_state.publish(msg)
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to publish reasoning state: {e}")

    def flush_log_buffer(self, event=None):
        """Flush log buffer to database (Prompt 4)."""
        if not self.log_buffer:
            return
        entries = list(self.log_buffer)
        self.log_buffer.clear()
        try:
            self.cursor.executemany('''
                INSERT INTO reasoning_log (timestamp, decision_type, action, rationale, confidence_score, contributing_factors)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', entries)
            self.conn.commit()
            rospy.loginfo(f"{self.node_name}: Flushed {len(entries)} reasoning log entries to DB.")
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
        node = CognitiveReasoningNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")