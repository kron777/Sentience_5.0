#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import psutil
import time
import threading
from collections import deque
from utils.error_logger import ErrorLogger

class PerformanceMonitorNode:
    def __init__(self):
        rospy.init_node('performance_monitor_node')
        self.namespace = rospy.get_param('~namespace', '/sentience')

        # Publishers
        self.cpu_pub = rospy.Publisher(f'{self.namespace}/cpu_usage', Float32, queue_size=10)
        self.mem_pub = rospy.Publisher(f'{self.namespace}/memory_usage', Float32, queue_size=10)
        self.disk_pub = rospy.Publisher(f'{self.namespace}/disk_usage', Float32, queue_size=10)
        self.net_pub = rospy.Publisher(f'{self.namespace}/network_sent_bytes', Float32, queue_size=10)
        self.latency_pub = rospy.Publisher(f'{self.namespace}/ros_message_latency', Float32, queue_size=10)

        # For message latency calculation
        self.msg_timestamps = deque(maxlen=100)

        self.error_logger = ErrorLogger()
        self.rate = rospy.Rate(1)  # 1 Hz

        # Subscribe to a dummy topic to measure latency; replace with a real topic if needed
        test_topic = rospy.get_param('~test_topic', f'{self.namespace}/heartbeat')
        self.latency_sub = rospy.Subscriber(test_topic, String, self.message_received_callback)

        rospy.loginfo("Performance Monitor Node initialized.")

    def message_received_callback(self, msg):
        recv_time = time.time()
        # Assume msg.data contains a timestamp string (sent time)
        try:
            sent_time = float(msg.data)
            latency = recv_time - sent_time
            self.msg_timestamps.append(latency)
        except Exception as e:
            self.error_logger.log(f"Failed to parse message timestamp for latency: {e}")

    def calculate_avg_latency(self):
        if not self.msg_timestamps:
            return 0.0
        return sum(self.msg_timestamps) / len(self.msg_timestamps)

    def monitor_loop(self):
        net_io_prev = psutil.net_io_counters()
        while not rospy.is_shutdown():
            try:
                cpu_percent = psutil.cpu_percent(interval=None)
                mem_percent = psutil.virtual_memory().percent
                disk_percent = psutil.disk_usage('/').percent
                net_io_current = psutil.net_io_counters()
                net_sent = (net_io_current.bytes_sent - net_io_prev.bytes_sent) / 1024.0  # KB/s
                net_io_prev = net_io_current

                avg_latency = self.calculate_avg_latency()

                self.cpu_pub.publish(cpu_percent)
                self.mem_pub.publish(mem_percent)
                self.disk_pub.publish(disk_percent)
                self.net_pub.publish(net_sent)
                self.latency_pub.publish(avg_latency)

                rospy.loginfo(
                    f"CPU: {cpu_percent:.1f}%, MEM: {mem_percent:.1f}%, DISK: {disk_percent:.1f}%, NET Sent: {net_sent:.1f} KB/s, Latency: {avg_latency:.3f}s"
                )

                self.rate.sleep()
            except Exception as e:
                self.error_logger.log(f"Exception in monitor loop: {e}")

    def run(self):
        thread = threading.Thread(target=self.monitor_loop)
        thread.daemon = True
        thread.start()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PerformanceMonitorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
