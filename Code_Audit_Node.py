#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import asyncio
from llm_utils.phi2_client_async import AsyncPhi2Client
from utils.error_logger import ErrorLogger

class CodeAuditNode:
    def __init__(self):
        rospy.init_node('code_audit_node')
        self.namespace = rospy.get_param('~namespace', '/sentience')

        self.directive_sub = rospy.Subscriber(f'{self.namespace}/self_correction_directive', String, self.on_directive)
        self.approval_pub = rospy.Publisher(f'{self.namespace}/code_audit_approval', Bool, queue_size=10)

        self.phi2 = AsyncPhi2Client()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.error_logger = ErrorLogger()

        rospy.loginfo("Code Audit Node initialized.")

    def on_directive(self, msg):
        directive = msg.data
        rospy.loginfo("Received self-correction directive for audit.")
        try:
            approved = self.loop.run_until_complete(self.audit_directive(directive))
            self.approval_pub.publish(approved)
            rospy.loginfo(f"Audit result: {'Approved' if approved else 'Rejected'}")
        except Exception as e:
            self.error_logger.log(f"Exception in audit_directive: {e}")
            self.approval_pub.publish(False)

    async def audit_directive(self, directive):
        prompt = (
            f"Audit the following self-correction directive for safety, stability, and security.\n"
            f"Return 'True' if safe to apply, otherwise 'False'.\n\nDirective:\n{directive}"
        )
        response = await self.phi2.query(prompt)
        result = response.strip().lower()
        return result == 'true'

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CodeAuditNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
