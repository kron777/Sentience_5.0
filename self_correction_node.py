#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import asyncio
from llm_utils.phi2_client_async import AsyncPhi2Client
from utils.error_logger import ErrorLogger

class SelfCorrectionNode:
    def __init__(self):
        rospy.init_node('self_correction_node')
        self.namespace = rospy.get_param('~namespace', '/sentience')

        self.directive_pub = rospy.Publisher(f'{self.namespace}/self_correction_directive', String, queue_size=10)
        self.audit_sub = rospy.Subscriber(f'{self.namespace}/code_audit_approval', Bool, self.on_audit_result)

        self.phi2 = AsyncPhi2Client()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.error_logger = ErrorLogger()
        self.pending_directive = None

        rospy.loginfo("Self Correction Node initialized.")

    def generate_directive(self, issue_description):
        prompt = f"Generate a safe code correction directive based on this issue:\n{issue_description}"
        return self.loop.run_until_complete(self.phi2.query(prompt))

    def request_audit(self, directive):
        rospy.loginfo("Requesting audit for directive...")
        self.directive_pub.publish(directive)

    def on_audit_result(self, msg):
        approved = msg.data
        if approved and self.pending_directive:
            rospy.loginfo("Directive approved by audit. Applying correction...")
            self.apply_correction(self.pending_directive)
            self.pending_directive = None
        else:
            rospy.logwarn("Directive rejected by audit.")
            self.pending_directive = None

    def apply_correction(self, directive):
        # Implement code application logic here (sandboxed or controlled)
        rospy.loginfo(f"Applying correction:\n{directive}")
        # Placeholder: Log for now
        # TODO: integrate with safe self-modifying mechanism

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SelfCorrectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
