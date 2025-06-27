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
        EmotionState,
        MotivationState,
        ValueDriftState,
    )
except ImportError:
    rospy.logwarn("Custom ROS messages for 'sentience' package not found. Using String for fallback.")
    CognitiveDirective = String
    EmotionState = String
    MotivationState = String
    ValueDriftState = String

from sentience.scripts.utils import parse_ros_message_data

class SystemIntegrationNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('system_integration_node', anonymous=False)
        self.node_name = rospy.get_name()

        # Parameters
        self.db_path = os.path.expanduser(rospy.get_param('~db_path', '~/.ros/conscious_robot/system_log.db'))
        self.traits_path = os.path.expanduser(rospy.get_param('~traits_path', '~/.ros/conscious_robot/default_character_traits.json'))
        self.update_interval = rospy.get_param('~update_interval', 1.0)
        self.max_event_history = rospy.get_param('~max_event_history', 50)  # Prompt 3
        self.batch_size = rospy.get_param('~batch_size', 50)  # Prompt 4
        self.log_flush_interval_s = rospy.get_param('~log_flush_interval_s', 10.0)
        self.trait_update_interval_s = rospy.get_param('~trait_update_interval_s', 60.0)
        self.llm_endpoint = rospy.get_param('~llm_endpoint', 'http://localhost:8080/phi2')
        self.learning_rate = rospy.get_param('~learning_rate', 0.01)

        # Load character traits
        self.character_traits = self._load_character_traits()

        # Internal state
        self.node_status = {
            'sensory_qualia_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'world_model_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'cognitive_reasoning_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'cognitive_control_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'behavior_execution_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'attention_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'emotion_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'motivation_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
            'value_drift_monitor_node': {'status': 'unknown', 'last_updated': str(rospy.get_time())},
        }
        self.event_history = deque(maxlen=self.max_event_history)  # Prompt 3
        self.log_buffer = deque(maxlen=self.batch_size)  # Prompt 4
        self.trait_update_buffer = deque(maxlen=self.batch_size)
        self.latest_states = {
            'emotion': None,
            'motivation': None,
            'value_drift': None,
        }

        # Initialize SQLite database
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS system_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                system_status TEXT,
                directive_type TEXT,
                target_node TEXT,
                confidence_score REAL,
                contributing_factors TEXT
            )
        ''')
        self.conn.commit()

        # Publishers
        self.pub_directive = rospy.Publisher('/cognitive_directives', CognitiveDirective, queue_size=10)
        self.pub_system_health = rospy.Publisher('/system_health', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/sensory_qualia_node/status', String, lambda msg: self.status_callback('sensory_qualia_node', msg))
        rospy.Subscriber('/world_model_node/status', String, lambda msg: self.status_callback('world_model_node', msg))
        rospy.Subscriber('/cognitive_reasoning_node/status', String, lambda msg: self.status_callback('cognitive_reasoning_node', msg))
        rospy.Subscriber('/cognitive_control_node/status', String, lambda msg: self.status_callback('cognitive_control_node', msg))
        rospy.Subscriber('/behavior_execution_node/status', String, lambda msg: self.status_callback('behavior_execution_node', msg))
        rospy.Subscriber('/attention_node/status', String, lambda msg: self.status_callback('attention_node', msg))
        rospy.Subscriber('/emotion_node/status', String, lambda msg: self.status_callback('emotion_node', msg))
        rospy.Subscriber('/motivation_node/status', String, lambda msg: self.status_callback('motivation_node', msg))
        rospy.Subscriber('/value_drift_monitor_node/status', String, lambda msg: self.status_callback('value_drift_monitor_node', msg))
        rospy.Subscriber('/emotion_state', EmotionState, self.emotion_state_callback)
        rospy.Subscriber('/motivation_state', MotivationState, self.motivation_state_callback)
        rospy.Subscriber('/value_drift_state', ValueDriftState, self.value_drift_state_callback)

        # Timers
        rospy.Timer(rospy.Duration(self.update_interval), self.monitor_system)
        rospy.Timer(rospy.Duration(self.log_flush_interval_s), self.flush_log_buffer)
        rospy.Timer(rospy.Duration(self.trait_update_interval_s), self.flush_trait_updates)

        # Asyncio setup (Prompt 20)
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop.run_until_complete(self._async_init())

        rospy.loginfo(f"{self.node_name}: System Integration Node initialized.")

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
                "personality": {"reliability": {"value": 0.7, "weight": 1.0, "last_updated": "2025-06-26T20:00:00Z", "update_source": "default"}},
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

    def status_callback(self, node_name, msg):
        """Handle node status updates."""
        try:
            status_data = json.loads(msg.data or '{}')
            status = status_data.get('status', 'unknown')
            self.node_status[node_name]['status'] = status
            self.node_status[node_name]['last_updated'] = str(rospy.get_time())
            rospy.logdebug(f"{self.node_name}: Updated status for {node_name}: {status}")
        except json.JSONDecodeError:
            rospy.logwarn(f"{self.node_name}: Failed to parse status for {node_name}: {msg.data}")

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

    async def _async_monitor_system(self):
        """Monitor system health and generate directives (Prompt 1, 20)."""
        emotion = self.latest_states['emotion'] or {'mood': 'neutral', 'sentiment_score': 0.0}
        motivation = self.latest_states['motivation'] or {'dominant_goal_id': 'none', 'overall_drive_level': 0.0}
        value_drift = self.latest_states['value_drift'] or {'drift_score': 0.0}

        # Calculate system health
        healthy_nodes = sum(1 for status in self.node_status.values() if status['status'] == 'running')
        total_nodes = len(self.node_status)
        system_health = healthy_nodes / total_nodes if total_nodes > 0 else 0.0
        failed_nodes = [name for name, status in self.node_status.items() if status['status'] != 'running']

        # Simplified LLM input for system directive
        llm_input = {
            'system_health': system_health,
            'failed_nodes': failed_nodes,
            'mood': emotion['mood'],
            'goal_id': motivation['dominant_goal_id'],
            'drift_score': value_drift['drift_score'],
            'recent_events': [{'type': e['type'], 'target_node': e['target_node']} for e in list(self.event_history)[-5:]],  # Limit history
            'traits': {
                'reliability': self.character_traits['personality'].get('reliability', {}).get('value', 0.7),
                'empathy': self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8),
            }
        }

        # Query LLM
        try:
            async with self.http_session.post(self.llm_endpoint, json=llm_input, timeout=5.0) as response:
                if response.status == 200:
                    llm_output = await response.json()
                    directive_type = llm_output.get('directive_type', 'none')
                    target_node = llm_output.get('target_node', 'none')
                    command_payload = llm_output.get('command_payload', {})
                    confidence_score = llm_output.get('confidence', 0.5)
                    contributing_factors = llm_output.get('factors', {})
                else:
                    rospy.logwarn(f"{self.node_name}: LLM request failed with status {response.status}")
                    directive_type, target_node, command_payload, confidence_score, contributing_factors = 'none', 'none', {}, 0.3, {}
        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            rospy.logerr(f"{self.node_name}: LLM request failed: {e}")
            directive_type, target_node, command_payload, confidence_score, contributing_factors = 'none', 'none', {}, 0.3, {}

        # Adjust directive based on traits and inputs
        reliability = self.character_traits['personality'].get('reliability', {}).get('value', 0.7)
        empathy = self.character_traits['emotional_tendencies'].get('empathy', {}).get('value', 0.8)

        if failed_nodes and reliability > 0.6:
            directive_type = 'restart_node' if directive_type == 'none' else directive_type
            target_node = failed_nodes[0] if failed_nodes else 'none'
            command_payload = {'action': 'restart', 'priority': 0.9} if not command_payload else command_payload
            self._update_character_traits('personality', 'reliability', min(1.0, reliability + 0.05), 'node_failure')
        if system_health > 0.9 and reliability > 0.7:
            self._update_character_traits('personality', 'reliability', min(1.0, reliability + 0.03), 'stable_system')
        if value_drift['drift_score'] > 0.5:
            directive_type = 'ethical_check' if directive_type == 'none' else directive_type
            target_node = 'value_drift_monitor_node' if target_node == 'none' else target_node
            command_payload = {'action': 'reassess_values', 'priority': 0.8} if not command_payload else command_payload
        if emotion['mood'] == 'positive' and empathy > 0.7:
            command_payload['tone'] = 'friendly'

        command_payload_json = json.dumps(command_payload)
        system_status = {'health': system_health, 'failed_nodes': failed_nodes}
        return directive_type, target_node, command_payload_json, confidence_score, contributing_factors, system_status

    def monitor_system(self, event):
        """Periodic system monitoring (Prompt 1, 3, 4)."""
        timestamp = str(rospy.get_time())
        directive_type, target_node, command_payload_json, confidence_score, contributing_factors, system_status = self.loop.run_until_complete(self._async_monitor_system())

        self.event_history.append({
            'type': directive_type,
            'target_node': target_node,
            'command_payload': command_payload_json,
            'timestamp': timestamp
        })

        # Publish directive
        self.publish_directive(timestamp, directive_type, target_node, command_payload_json, confidence_score, contributing_factors)

        # Publish system health
        self.publish_system_health(timestamp, system_status, confidence_score)

        # Log state
        self.log_buffer.append((
            timestamp,
            json.dumps(system_status),
            directive_type,
            target_node,
            confidence_score,
            json.dumps(contributing_factors)
        ))

        rospy.loginfo(f"{self.node_name}: System Health: {system_status['health']}, Directive: {directive_type}, Target: {target_node}")

    def publish_directive(self, timestamp, directive_type, target_node, command_payload_json, confidence_score, contributing_factors):
        """Publish cognitive directive."""
        try:
            if isinstance(CognitiveDirective, type(String)):
                directive_data = {
                    'timestamp': timestamp,
                    'directive_type': directive_type,
                    'target_node': target_node,
                    'command_payload': command_payload_json,
                    'confidence_score': confidence_score,
                    'contributing_factors': contributing_factors,
                }
                self.pub_directive.publish(json.dumps(directive_data))
            else:
                msg = CognitiveDirective()
                msg.timestamp = timestamp
                msg.directive_type = directive_type
                msg.target_node = target_node
                msg.command_payload = command_payload_json
                msg.confidence_score = confidence_score
                msg.contributing_factors_json = json.dumps(contributing_factors)
                self.pub_directive.publish(msg)
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to publish directive: {e}")

    def publish_system_health(self, timestamp, system_status, confidence_score):
        """Publish system health status."""
        try:
            health_data = {
                'timestamp': timestamp,
                'system_health': system_status['health'],
                'failed_nodes': system_status['failed_nodes'],
                'confidence_score': confidence_score,
            }
            self.pub_system_health.publish(json.dumps(health_data))
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to publish system health: {e}")

    def flush_log_buffer(self, event=None):
        """Flush log buffer to database (Prompt 4)."""
        if not self.log_buffer:
            return
        entries = list(self.log_buffer)
        self.log_buffer.clear()
        try:
            self.cursor.executemany('''
                INSERT INTO system_log (timestamp, system_status, directive_type, target_node, confidence_score, contributing_factors)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', entries)
            self.conn.commit()
            rospy.loginfo(f"{self.node_name}: Flushed {len(entries)} system log entries to DB.")
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
        node = SystemIntegrationNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"{rospy.get_name()}: Unexpected error: {e}")