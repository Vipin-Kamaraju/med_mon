#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from scipy.signal import butter, lfilter

class EKGFilterNode:
    def __init__(self):
        rospy.init_node('ekg_filter_node')

        # Filter settings
        self.enabled = True
        self.b, self.a = butter(N=2, Wn=0.1, btype='low')

        self.filtered_pub = rospy.Publisher('/ekg_filtered', Float32, queue_size=10)
        self.ctrl_sub = rospy.Subscriber('/ekg_filter_enable', Float32, self.toggle_filter)
        self.ekg_sub = rospy.Subscriber('/ekg', Float32, self.filter_callback)

        self.prev_data = []

    def toggle_filter(self, msg):
        self.enabled = bool(msg.data)
        rospy.loginfo(f"Filtering {'enabled' if self.enabled else 'disabled'}")

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
    except rospy.ROSInterruptException:
        pass
