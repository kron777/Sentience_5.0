#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import asyncio
from llm_utils.phi2_client_async import AsyncPhi2Client
from utils.error_logger import ErrorLogger

class SocialCognitionNode:
    def __init__(self):
        rospy.init_node('social_cognition_node')

        # Get namespace param, default to /sentience
        namespace = rospy.get_param('~namespace', '/sentience')

        # Topics with namespace prefix
        self.prompt_sub = rospy.Subscriber(f'{namespace}/interaction_request', String, self.on_prompt)
        self.response_pub = rospy.Publisher(f'{namespace}/interaction_response', String, queue_size=10)

        self.phi2 = AsyncPhi2Client()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.error_logger = ErrorLogger()
        rospy.loginfo("Social Cognition Node initialized.")

    def on_prompt(self, msg):
        prompt = msg.data
        rospy.loginfo(f"Received prompt: {prompt}")
        try:
            response = self.loop.run_until_complete(self.handle_phi2(prompt))
            self.response_pub.publish(response)
            rospy.loginfo(f"Published response: {response}")
        except Exception as e:
            error_msg = f"Exception in handle_phi2: {e}"
            rospy.logerr(error_msg)
            self.error_logger.log(error_msg)

    async def handle_phi2(self, prompt):
        return await self.phi2.query(prompt)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SocialCognitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
