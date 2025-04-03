#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from scipy.signal import butter, lfilter

class EKGFilterNode:
    def __init__(self):
        rospy.init_node('ekg_filter_node')

        # Filter settings
        self.cutoff = 0.0  # Default cutoff frequency (filter disabled)
        self.enabled = False  # Filter is initially disabled
        self.update_filter()

        self.filtered_pub = rospy.Publisher('/ekg_filtered', Float32, queue_size=10)
        self.cutoff_sub = rospy.Subscriber('/ekg_filter_cutoff', Float32, self.update_cutoff)  # New subscriber
        self.ekg_sub = rospy.Subscriber('/ekg', Float32, self.filter_callback)

        self.prev_data = []

    def update_filter(self):
        if self.cutoff > 0.0:
            self.enabled = True
            self.b, self.a = butter(N=2, Wn=self.cutoff, btype='low')
            rospy.loginfo(f"Filter enabled with cutoff: {self.cutoff}")
        else:
            self.enabled = False
            rospy.loginfo("Filter disabled")

    def update_cutoff(self, msg):
        self.cutoff = msg.data
        self.update_filter()

    def filter_callback(self, msg):
        if not self.enabled:
            self.filtered_pub.publish(msg.data)
            return

        self.prev_data.append(msg.data)
        if len(self.prev_data) > 20:
            self.prev_data.pop(0)

        filtered = lfilter(self.b, self.a, self.prev_data)[-1]
        self.filtered_pub.publish(filtered)

if __name__ == "__main__":
    try:
        EKGFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
