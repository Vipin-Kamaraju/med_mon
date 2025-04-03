import sys
# import threading
from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal, QUrl, QTimer
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

        rospy.loginfo("[GUI INIT] Setting up ROS subscribers...")

        rospy.Subscriber('/heart_rate', Int32, self.update_heart_rate)
        rospy.loginfo("[ROS] Subscriber for /heart_rate created.")
        rospy.Subscriber('/blood_pressure', Float32, self.update_blood_pressure)
        rospy.loginfo("[ROS] Subscriber for /blood_pressure created.")
        # rospy.Subscriber('/ekg', Float32, self.update_ekg)
        rospy.Subscriber('/ekg_filtered', Float32, self.update_ekg)

        self.cutoff_topic = rospy.get_param('~cutoff_topic', '/ekg_filter_cutoff')  # New parameter
        self.cutoff_pub = rospy.Publisher(self.cutoff_topic, Float32, queue_size=10)

        rospy.loginfo("[GUI INIT] Subscribers set.")


    @pyqtSlot(str)
    def log_message(self, message):
        """Logs messages from QML to ROS logs."""
        rospy.loginfo(f"[QML] {message}")


    @pyqtSlot(Int32)
    def update_heart_rate(self, msg):
        # rospy.loginfo(f"[Python] Heart Rate Callback triggered with value: {msg.data}")
        # self.heartRateChanged.connect(lambda value: rospy.loginfo(f"[Debug] Heart Rate Signal Emitted: {value}"))
        self.heartRateChanged.emit(str(msg.data))

    @pyqtSlot(Float32)
    def update_blood_pressure(self, msg):
        # rospy.loginfo(f"[Python] Blood Pressure Callback triggered with value: {msg.data}")
        # self.bloodPressureChanged.connect(lambda value: rospy.loginfo(f"[Debug] Blood Pressure Signal Emitted: {value}"))
        blood_pressure = int(round(msg.data))  # Round to nearest integer
        self.bloodPressureChanged.emit(str(blood_pressure))

    @pyqtSlot(Float32)
    def update_ekg(self, msg):
        # rospy.loginfo(f"[Python] EKG Callback triggered with value: {msg.data}")
        # self.ekgChanged.connect(lambda value: rospy.loginfo(f"[Debug] EKG Signal Emitted: {value}"))
        self.ekgChanged.emit(float(msg.data))

    @pyqtSlot(bool)
    def toggle_filter(self, enabled):
        # Called from QML checkbox
        rospy.loginfo(f"GUI filter toggle: {'enabled' if enabled else 'disabled'}")
        self.filter_control_pub.publish(1.0 if enabled else 0.0)
        # self.filter_control_pub.publish(0.0)

    @pyqtSlot(float)
    def update_cutoff(self, cutoff):
        # Called from QML slider
        rospy.loginfo(f"GUI filter cutoff: {cutoff}")
        self.cutoff_pub.publish(float(cutoff))

if __name__ == "__main__":
    try:
        # rospy.init_node('medical_gui', anonymous=True, log_level=rospy.DEBUG)
        # rospy.loginfo("[ROS] Node initialized successfully.")

        app = QApplication(sys.argv)
        engine = QQmlApplicationEngine()

        gui = MedicalGUI()
        context = engine.rootContext()
        context.setContextProperty("gui", gui)
        context.setContextProperty("logger", gui.log_message)  # Expose the logging function
        rospy.loginfo("[QML] Context property 'gui' and 'logger' set.")

        engine.load(QUrl('file:///med_mon/src/gui/medical_gui.qml'))
        rospy.loginfo("[QML] QML file loaded.")

        if not engine.rootObjects():
            rospy.logerr("[QML] No root objects found. Exiting.")
            sys.exit(-1)

        # Add a timer to keep the event loop alive and log periodically
        # timer = QTimer()
        # timer.timeout.connect(lambda: rospy.loginfo("[Debug] Timer running."))
        # timer.start(1000)

        # rospy.loginfo("[ROS] Starting rospy.spin() in a separate thread.")
        # threading.Thread(target=rospy.spin, daemon=True).start()

        QTimer.singleShot(2000, lambda: gui.heartRateChanged.emit("123"))

        rospy.loginfo("[GUI] Starting PyQt application event loop.")
        sys.exit(app.exec_())
    except Exception as e:
        rospy.logerr(f"[Exception] {e}")
        sys.exit(-1)
