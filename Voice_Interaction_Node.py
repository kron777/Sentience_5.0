#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr
import pyttsx3
import asyncio
from llm_utils.phi2_client_async import AsyncPhi2Client
from utils.error_logger import ErrorLogger
import threading

class VoiceInteractionNode:
    def __init__(self):
        rospy.init_node('voice_interaction_node')
        self.namespace = rospy.get_param('~namespace', '/sentience')

        self.response_pub = rospy.Publisher(f'{self.namespace}/voice_response', String, queue_size=10)
        self.command_sub = rospy.Subscriber(f'{self.namespace}/voice_command', String, self.on_command)

        self.phi2 = AsyncPhi2Client()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.error_logger = ErrorLogger()

        self.recognizer = sr.Recognizer()
        self.tts_engine = pyttsx3.init()

        # Start microphone listening in separate thread
        self.listening_thread = threading.Thread(target=self.listen_loop)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        rospy.loginfo("Voice Interaction Node initialized.")

    def listen_loop(self):
        mic = sr.Microphone()
        with mic as source:
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Voice Interaction Node listening for speech...")
            while not rospy.is_shutdown():
                try:
                    audio = self.recognizer.listen(source, timeout=5)
                    command = self.recognizer.recognize_google(audio)
                    rospy.loginfo(f"Recognized voice command: {command}")
                    self.publish_command(command)
                except sr.WaitTimeoutError:
                    continue  # no speech detected within timeout
                except sr.UnknownValueError:
                    rospy.logwarn("Could not understand audio")
                except Exception as e:
                    self.error_logger.log(f"Error in listen_loop: {e}")

    def publish_command(self, command):
        pub = rospy.Publisher(f'{self.namespace}/voice_command', String, queue_size=10)
        pub.publish(command)

    def on_command(self, msg):
        command = msg.data
        rospy.loginfo(f"Processing voice command: {command}")
        try:
            response = self.loop.run_until_complete(self.generate_response(command))
            self.response_pub.publish(response)
            self.speak(response)
        except Exception as e:
            error_msg = f"Exception in generate_response: {e}"
            rospy.logerr(error_msg)
            self.error_logger.log(error_msg)

    async def generate_response(self, command):
        prompt = f"Human says: {command}\nRespond kindly and helpfully."
        return await self.phi2.query(prompt)

    def speak(self, text):
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.error_logger.log(f"TTS error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = VoiceInteractionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
