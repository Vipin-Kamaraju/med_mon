import sys
import threading
from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal, QUrl, QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtQml import QQmlApplicationEngine
import rospy
from std_msgs.msg import Int32, Float32

class MedicalGUI(QObject):
    heartRateChanged = pyqtSignal(int)
    bloodPressureChanged = pyqtSignal(float)
    ekgChanged = pyqtSignal(float)

    def __init__(self):
        super().__init__()

        rospy.Subscriber('/heart_rate', Int32, self.update_heart_rate)
        rospy.Subscriber('/blood_pressure', Float32, self.update_blood_pressure)
        rospy.Subscriber('/ekg_filtered', Float32, self.update_ekg)

        self.filter_control_pub = rospy.Publisher('/ekg_filter_enable', Float32, queue_size=10)

    @pyqtSlot(Int32)
    def update_heart_rate(self, msg):
        print(f"[Python] Heart Rate Callback: {msg.data}")
        self.heartRateChanged.emit(msg.data)

    @pyqtSlot(Float32)
    def update_blood_pressure(self, msg):
        print(f"[Python] Blood Pressure Callback: {msg.data}")
        self.bloodPressureChanged.emit(msg.data)

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

        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)
        
        if not engine.rootObjects():
            sys.exit(-1)
        
        threading.Thread(target=rospy.spin, daemon=True).start()
        sys.exit(app.exec_())
    except Exception as e:
        rospy.logerr(f"Exception in main: {e}")
        sys.exit(-1)
