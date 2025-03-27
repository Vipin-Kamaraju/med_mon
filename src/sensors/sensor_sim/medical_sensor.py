#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import Int32, Float32

def publish_vitals():
    rospy.init_node('medical_sensor')
    hr_pub = rospy.Publisher('/heart_rate', Int32, queue_size=10)
    bp_pub = rospy.Publisher('/blood_pressure', Float32, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        heart_rate = random.randint(60, 120)  # Simulating heart rate
        blood_pressure = random.uniform(90.0, 140.0)  # Simulating BP
        hr_pub.publish(heart_rate)
        bp_pub.publish(blood_pressure)
        rospy.loginfo(f"Heart Rate: {heart_rate}, Blood Pressure: {blood_pressure}")
        rate.sleep()

if __name__ == "__main__":
    publish_vitals()
