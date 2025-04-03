#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from scipy.signal import butter, lfilter

class GenericFilterNode:
    def __init__(self):
        rospy.init_node('generic_filter_node')

        # Get ROS parameters
        self.input_topic = rospy.get_param('~input_topic', '/input_signal')
        self.output_topic = rospy.get_param('~output_topic', '/filtered_signal')
        self.filter_type = rospy.get_param('~filter_type', 'lowpass')
        self.cutoff_frequency = rospy.get_param('~cutoff_frequency', 0.0)
        self.cutoff_topic = rospy.get_param('~cutoff_topic', '/ekg_filter_cutoff')

        # Filter settings
        self.enabled = True
        self.update_filter()

        # Publishers and subscribers
        self.filtered_pub = rospy.Publisher(self.output_topic, Float32, queue_size=10)
        self.input_sub = rospy.Subscriber(self.input_topic, Float32, self.filter_callback)
        self.cutoff_sub = rospy.Subscriber(self.cutoff_topic, Float32, self.update_cutoff_frequency)

        self.prev_data = []

    def update_filter(self):
        if self.cutoff_frequency <= 0:
            self.enabled = False
            rospy.loginfo("Filter disabled because cutoff frequency is zero or negative.")
            return
        else:
            self.enabled = True

        if self.filter_type == 'lowpass':
            self.b, self.a = butter(N=2, Wn=self.cutoff_frequency, btype='low')
            rospy.loginfo(f"Lowpass filter enabled with cutoff: {self.cutoff_frequency}")
        else:
            rospy.logwarn(f"Unsupported filter type: {self.filter_type}")
            self.enabled = False

    def filter_callback(self, msg):
        if not self.enabled:
            self.filtered_pub.publish(msg.data)
            return

        self.prev_data.append(msg.data)
        if len(self.prev_data) > 20:
            self.prev_data.pop(0)

        filtered = lfilter(self.b, self.a, self.prev_data)[-1]
        self.filtered_pub.publish(filtered)

    def update_cutoff_frequency(self, msg):
        self.cutoff_frequency = msg.data
        self.update_filter()

if __name__ == "__main__":
    try:
        GenericFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
