import rospy
import numpy as np
from geometry_msgs.msg import Pose
from franka_msgs.msg import FrankaState, WrenchStamped
from sensor_msgs.msg import JointState
from datetime import datetime
import os
import threading

class PoseRecorder:
    def __init__(self):
        rospy.init_node('pose_recorder', anonymous=True)
        
        # Hard-coded parameters (adjust as you like)
        self.output_dir = "/home/hisham246/uwaterloo/panda_ws/src/franka_passive_ds_impedance_controller/robot_demos"
        self.output_filename = "free_space"
        self.buffer_size = 10000  # Number of samples to buffer
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo(f"Created output directory: {self.output_dir}")
        
        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(
            self.output_dir, 
            f"{self.output_filename}_{timestamp}.csv"
        )
        
        # Data buffer (thread-safe)
        self.data_buffer = []
        self.buffer_lock = threading.Lock()
        self.recording = True
        
        # Statistics
        self.sample_count = 0
        self.start_time = None

        # Latest Franka state fields we care about
        self.latest_O_dP_EE_d = None  # 6D desired twist
        self.latest_O_dP_EE_c = None  # 6D commanded twist
        self.latest_external_forces = None  # External forces on the EE
        
        # Initialize CSV file with header
        self._initialize_file()
        
        # Subscribe to FrankaState for O_dP_EE_d / O_dP_EE_c
        self.state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',  # adjust topic name if needed
            FrankaState,
            self.state_callback,
            queue_size=1000,
            tcp_nodelay=True
        )
        
        # Subscribe to external forces (F_ext) topic (wrench: force and torque)
        self.force_sub = rospy.Subscriber(
            '/franka_state_controller/F_ext',  # External forces topic
            WrenchStamped,
            self.force_callback,
            queue_size=1000,
            tcp_nodelay=True
        )
        
        # Start background thread for writing data
        self.write_thread = threading.Thread(target=self._write_loop)
        self.write_thread.daemon = True
        self.write_thread.start()
        
    def _initialize_file(self):
        """Initialize CSV file with header."""
        with open(self.filepath, 'w') as f:
            # Time
            header = "ros_time_sec,ros_time_nsec,"
            # O_dP_EE_d (desired EE twist)
            header += "O_dP_EE_d_vx,O_dP_EE_d_vy,O_dP_EE_d_vz,"
            header += "O_dP_EE_d_wx,O_dP_EE_d_wy,O_dP_EE_d_wz,"
            # O_dP_EE_c (commanded EE twist)
            header += "O_dP_EE_c_vx,O_dP_EE_c_vy,O_dP_EE_c_vz,"
            header += "O_dP_EE_c_wx,O_dP_EE_c_wy,O_dP_EE_c_wz,"
            # External forces (F_ext) - Force and Torque (Wrench)
            header += "F_ext_x,F_ext_y,F_ext_z,F_ext_wx,F_ext_wy,F_ext_wz\n"
            f.write(header)

    def state_callback(self, msg):
        """Callback for FrankaState messages, store latest twists."""
        # These are float64[6] arrays in FrankaState
        self.latest_O_dP_EE_d = np.array(msg.O_dP_EE_d, dtype=float)
        self.latest_O_dP_EE_c = np.array(msg.O_dP_EE_c, dtype=float)

    def force_callback(self, msg):
        """Callback for external forces (wrench) messages, store the latest external forces."""
        # Extract force and torque components from the wrench message
        self.latest_external_forces = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        ], dtype=float)

    def pose_callback(self, msg):
        """High-frequency callback for pose messages (primary trigger for logging)."""
        if not self.recording:
            return
        
        # Get system time for accurate timestamping
        now = rospy.Time.now()
        
        if self.start_time is None:
            self.start_time = now

        # Use latest FrankaState if available, otherwise NaNs
        if self.latest_O_dP_EE_d is not None:
            O_dP_EE_d = self.latest_O_dP_EE_d
        else:
            O_dP_EE_d = np.full(6, np.nan)

        if self.latest_O_dP_EE_c is not None:
            O_dP_EE_c = self.latest_O_dP_EE_c
        else:
            O_dP_EE_c = np.full(6, np.nan)
        
        # Use latest external forces if available, otherwise NaNs
        if self.latest_external_forces is not None:
            F_ext = self.latest_external_forces
        else:
            F_ext = np.full(6, np.nan)

        # Extract data row
        data_row = [
            float(now.secs),
            float(now.nsecs),
            # desired twist
            float(O_dP_EE_d[0]),
            float(O_dP_EE_d[1]),
            float(O_dP_EE_d[2]),
            float(O_dP_EE_d[3]),
            float(O_dP_EE_d[4]),
            float(O_dP_EE_d[5]),
            # commanded twist
            float(O_dP_EE_c[0]),
            float(O_dP_EE_c[1]),
            float(O_dP_EE_c[2]),
            float(O_dP_EE_c[3]),
            float(O_dP_EE_c[4]),
            float(O_dP_EE_c[5]),
            # External forces (wrench)
            float(F_ext[0]),
            float(F_ext[1]),
            float(F_ext[2]),
            float(F_ext[3]),
            float(F_ext[4]),
            float(F_ext[5]),
        ]
        
        # Add to buffer (thread-safe)
        with self.buffer_lock:
            self.data_buffer.append(data_row)
            self.sample_count += 1
            
            # If buffer is full, write immediately
            if len(self.data_buffer) >= self.buffer_size:
                self._flush_buffer()

    def _flush_buffer(self):
        """Write buffer contents to file (must be called with lock held)."""
        if not self.data_buffer:
            return
        
        # Convert to numpy array for efficient writing
        data_array = np.array(self.data_buffer, dtype=float)
        
        # Append to CSV
        with open(self.filepath, 'a') as f:
            np.savetxt(f, data_array, delimiter=',', fmt='%.9f')
        
        # Clear buffer
        self.data_buffer.clear()
    
    def _write_loop(self):
        """Background thread for periodic buffer flushing."""
        rate = rospy.Rate(10)  # Flush buffer 10 times per second
        
        while not rospy.is_shutdown() and self.recording:
            with self.buffer_lock:
                if self.data_buffer:
                    self._flush_buffer()
            rate.sleep()
    
    def get_statistics(self):
        """Get recording statistics."""
        if self.start_time is None:
            return None
        
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        avg_freq = self.sample_count / elapsed if elapsed > 0 else 0
        
        return {
            'sample_count': self.sample_count,
            'elapsed_time': elapsed,
            'average_frequency': avg_freq,
            'filepath': self.filepath
        }
    
    def stop_recording(self):
        """Stop recording and flush remaining data."""
        rospy.loginfo("Stopping recording...")
        self.recording = False
        
        # Final flush
        with self.buffer_lock:
            self._flush_buffer()
    
    def run(self):
        """Main run loop."""
        rospy.on_shutdown(self.stop_recording)
        rospy.spin()


if __name__ == '__main__':
    try:
        recorder = PoseRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass