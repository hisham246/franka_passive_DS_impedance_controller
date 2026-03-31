import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from datetime import datetime
import os
import threading

class PoseRecorder:
    def __init__(self):
        rospy.init_node('record_demos', anonymous=True)
        
        # Hard-coded parameters
        self.output_dir = "/home/hisham246/uwaterloo/panda_ws/src/franka_passive_ds_impedance_controller/robot_demos"
        self.output_filename = "contact"
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

        # Latest Franka state field we care about
        self.latest_O_T_EE = None  # 4x4 EE Pose (flattened to 16 elements)
        
        # Initialize CSV file with header
        self._initialize_file()
        
        # Subscribe to FrankaState for O_T_EE
        self.state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.state_callback,
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
            # O_T_EE (16 elements of the 4x4 transform matrix, column-major)
            # Elements 12, 13, 14 represent the x, y, z translation.
            header += ",".join([f"O_T_EE_{i}" for i in range(16)]) + "\n"
            f.write(header)

    def state_callback(self, msg):
        """Callback for FrankaState messages, store latest pose and trigger logging."""
        # O_T_EE is a float64[16] array in column-major order
        self.latest_O_T_EE = np.array(msg.O_T_EE, dtype=float)
        
        # Trigger recording immediately so we actually log data!
        self.record_data()

    def record_data(self):
        """High-frequency logging method called by state_callback."""
        if not self.recording:
            return
        
        now = rospy.Time.now()
        
        if self.start_time is None:
            self.start_time = now

        # Use latest pose if available, otherwise NaNs
        if self.latest_O_T_EE is not None:
            O_T_EE = self.latest_O_T_EE
        else:
            O_T_EE = np.full(16, np.nan)

        # Extract data row
        data_row = [float(now.secs), float(now.nsecs)]
        data_row.extend(O_T_EE.tolist())
        
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