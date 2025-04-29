import sys
import os
import json
import rospy
import threading
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QComboBox, QLabel, QHBoxLayout
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor, QPalette
from std_msgs.msg import Float64MultiArray

class MotionPublisher(QWidget):
    def __init__(self, pub, motion_dir):
        super().__init__()
        self.pub = pub
        self.motion_dir = motion_dir
        self.motion_files = {}
        self.current_motion = None
        self.total_duration = 0
        self.time_left = 0

        self.init_ui()
        self.list_motions()  # Auto-load motions on start

    def init_ui(self):
        self.setWindowTitle("Motion Publisher")
        self.setGeometry(100, 100, 400, 180)

        main_layout = QVBoxLayout()

        # Dropdown selection menu and motion info
        dropdown_layout = QHBoxLayout()

        self.label = QLabel("Select Motion:")
        dropdown_layout.addWidget(self.label)

        self.dropdown = QComboBox()
        self.dropdown.addItem("Please select a motion")  # Default text
        self.dropdown.currentIndexChanged.connect(self.update_motion_info)
        dropdown_layout.addWidget(self.dropdown)

        self.motion_length_label = QLabel("Length: -")
        dropdown_layout.addWidget(self.motion_length_label)

        self.motion_duration_label = QLabel("Duration: - sec")
        dropdown_layout.addWidget(self.motion_duration_label)

        main_layout.addLayout(dropdown_layout)

        # Status Label for publishing
        self.status_label = QLabel("Select a motion to publish")
        self.status_label.setStyleSheet("color: green;")  # Default green
        main_layout.addWidget(self.status_label)

        # Publish Button
        self.pub_button = QPushButton("Publish")
        self.pub_button.clicked.connect(self.publish_motion)
        main_layout.addWidget(self.pub_button)

        self.setLayout(main_layout)

    def load_motion_files(self) -> dict:
        """Load and parse motion files from the directory."""
        motion_files = {}
        if not os.path.exists(self.motion_dir):
            rospy.logwarn("Motion directory not found!")
            return motion_files

        for f in os.listdir(self.motion_dir):
            with open(os.path.join(self.motion_dir, f), "r") as file:
                raw_data = json.load(file)
                motion_files[f] = raw_data[0]  # Assuming motion data is in raw_data[0]

        return dict(sorted(motion_files.items()))

    def list_motions(self):
        """Load motion files and populate the dropdown."""
        self.motion_files = self.load_motion_files()

        self.dropdown.clear()
        self.dropdown.addItem("Please select a motion")  # Reset first item
        self.dropdown.addItems(self.motion_files.keys())

        self.update_motion_info()  # Reset motion info

    def update_motion_info(self):
        """Update motion length and duration when selection changes."""
        selected_motion = self.dropdown.currentText()
        if selected_motion in self.motion_files:
            length = len(self.motion_files[selected_motion])
            duration = length / 50  # Convert to seconds (50 Hz rate)

            self.motion_length_label.setText(f"Length: {length}")
            self.motion_duration_label.setText(f"Duration: {duration:.2f} sec")
        else:
            self.motion_length_label.setText("Length: -")
            self.motion_duration_label.setText("Duration: - sec")

    def publish_motion(self):
        """Publish the selected motion."""
        selected_motion = self.dropdown.currentText()
        if selected_motion not in self.motion_files:
            rospy.logwarn("No valid motion selected!")
            return

        motion_data = self.motion_files[selected_motion]
        self.current_motion = selected_motion
        self.total_duration = len(motion_data) / 50  # Convert to seconds
        self.curr_time = 0

        rospy.loginfo(f"Publishing motion: {selected_motion}")
        self.status_label.setText(f"Publishing motion: {selected_motion} ({self.curr_time:.2f}/{self.total_duration:.2f} sec)")
        self.status_label.setStyleSheet("color: red;")  # Turn red while publishing

        # Start the publishing loop on a separate thread
        threading.Thread(target=self.publish_motion_ros, args=(motion_data,), daemon=True).start()

        # Start a QTimer to update time left
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.timer.start(20)  # 20ms update rate for smoother countdown

    def update_time(self):
        """Update the countdown timer while publishing."""
        if self.curr_time < self.total_duration:
            self.curr_time += 0.02  # Update every 20ms
            self.status_label.setText(f"Publishing motion: {self.current_motion} ({self.curr_time:.2f}/{self.total_duration:.2f} sec)")
        else:
            self.timer.stop()
            self.status_label.setText("Select a motion to publish")
            self.status_label.setStyleSheet("color: green;")  # Turn green when done

    def publish_motion_ros(self, motion: list):
        """Publish motion data to the ROS topic."""
        cur_item = 0
        rate = rospy.Rate(50)  # 50 Hz
        rospy.loginfo("[INFO] Publishing motion...")

        while cur_item < len(motion) and not rospy.is_shutdown():
            msg = Float64MultiArray(data=motion[cur_item])
            self.pub.publish(msg)
            cur_item += 1
            rate.sleep()

        # Stop motion
        msg = Float64MultiArray(data=[0.0] * 16)
        self.pub.publish(msg)
        rospy.loginfo("[INFO] Finished publishing!")
        self.time_left = 0  # Ensure countdown reaches 0 in UI

def main():
    """Main function to start ROS node and GUI."""
    rospy.init_node("reference_motion_gui", anonymous=True)
    pub = rospy.Publisher("reference_motion", Float64MultiArray, queue_size=1)

    cur_dir = os.path.dirname(os.path.realpath(__file__))
    motion_dir = os.path.join(cur_dir, "../motion_data")

    app = QApplication(sys.argv)
    window = MotionPublisher(pub, motion_dir)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
