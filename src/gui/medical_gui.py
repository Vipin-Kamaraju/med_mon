import sys
from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal, QUrl
from PyQt5.QtWidgets import QApplication
from PyQt5.QtQml import QQmlApplicationEngine
import rospy
from std_msgs.msg import Int32, Float32

class MedicalGUI(QObject):
    heartRateChanged = pyqtSignal(str)
    bloodPressureChanged = pyqtSignal(str)
    ekgChanged = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        rospy.init_node('medical_gui', anonymous=True)

        rospy.Subscriber('/heart_rate', Int32, self.update_heart_rate)
        rospy.Subscriber('/blood_pressure', Float32, self.update_blood_pressure)
        rospy.Subscriber('/ekg_filtered', Float32, self.update_ekg)

        self.filter_control_pub = rospy.Publisher('/ekg_filter_enable', Float32, queue_size=10)

    @pyqtSlot(Int32)
    def update_heart_rate(self, msg):
        self.heartRateChanged.emit(f"Heart Rate: {msg.data} bpm")

    @pyqtSlot(Float32)
    def update_blood_pressure(self, msg):
        self.bloodPressureChanged.emit(f"Blood Pressure: {msg.data:.1f} mmHg")

    @pyqtSlot(Float32)
    def update_ekg(self, msg):
        print(f"Emitting EKG data: {msg.data}")  # Debugging statement
        self.ekgChanged.emit(float(msg.data))

    @pyqtSlot(bool)
    def toggle_filter(self, enabled):
        # Called from QML checkbox
        rospy.loginfo(f"GUI filter toggle: {'enabled' if enabled else 'disabled'}")
        self.filter_control_pub.publish(1.0 if enabled else 0.0)

if __name__ == "__main__":
    try:
        rospy.init_node('medical_gui', anonymous=True)

        app = QApplication(sys.argv)
        engine = QQmlApplicationEngine()

        gui = MedicalGUI()
        context = engine.rootContext()
        context.setContextProperty("gui", gui)

        engine.load(QUrl('file:///med_mon/src/gui/medical_gui.qml'))

        if not engine.rootObjects():
            sys.exit(-1)
        sys.exit(app.exec_())
    except Exception as e:
        rospy.logerr(f"Exception in main: {e}")
        sys.exit(-1)