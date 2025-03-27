#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import rospkg
import os

class EkgSim:
    def __init__(self):
        rospy.init_node('ekg_sensor')
        self.ekg_pub = rospy.Publisher('/ekg', Float32, queue_size=10)
        self.ekg_data = self.load_ekg_data()
        self.get_ekg_voltage = self.get_next_ekg_voltage(self.ekg_data)

        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            ekg_voltage = self.get_ekg_voltage()
            self.ekg_pub.publish(ekg_voltage)
            rospy.loginfo(f"ekg_voltage: {ekg_voltage}")
            rate.sleep()

    def load_ekg_data(self) -> list:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sensor_sim')
        ecg_filename = 'ecg-data.csv'
        file_path = os.path.join(package_path, 'data', ecg_filename)
        with open(file_path, 'r') as file:
            ekg_data = [float(line.strip()) for line in file.readlines()]
        return ekg_data

    def get_next_ekg_voltage(self, ekg_data):
        ekg_voltage_idx = 0

        def get_ekg_voltage():
            nonlocal ekg_voltage_idx
            v = ekg_data[ekg_voltage_idx]
            ekg_voltage_idx = (ekg_voltage_idx + 1) % len(ekg_data)
            return v
    
        return get_ekg_voltage

if __name__ == "__main__":
    EkgSim()