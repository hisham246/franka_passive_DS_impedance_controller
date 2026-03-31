#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import os
import threading
from datetime import datetime
from geometry_msgs.msg import Twist, WrenchStamped
from franka_msgs.msg import FrankaState
from scipy.ndimage import gaussian_filter1d

class PlaybackAndRecord:
    def __init__(self, playback_csv_filepath):
        rospy.init_node('playback_and_record_node', anonymous=True)
        
        # --- Playback Setup ---
        self.twist_pub = rospy.Publisher('/passiveDS/desired_twist', Twist, queue_size=10)
        self.playback_csv_filepath = playback_csv_filepath
        self._process_playback_data()

        # --- Recording Setup ---
        self.output_dir = "/home/hisham246/uwaterloo/panda_ws/src/franka_passive_ds_impedance_controller/robot_demos"
        self.output_filename = "disturbance_execution_tracking"
        self.buffer_size = 5000
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.record_filepath = os.path.join(self.output_dir, f"{self.output_filename}_{timestamp}.csv")
        
        self.data_buffer = []
        self.buffer_lock = threading.Lock()
        self.recording = False  # Will be set to True when playback actually starts
        self.start_time = None

        self.latest_O_T_EE = None
        self.latest_O_T_EE_d = None
        self.latest_F_ext = None
        
        self._initialize_record_file()
        
        # Subscribers for recording
        self.state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.state_callback,
            queue_size=1000,
            tcp_nodelay=True
        )
        self.force_sub = rospy.Subscriber(
            '/franka_state_controller/F_ext',
            WrenchStamped,
            self.force_callback,
            queue_size=1000,
            tcp_nodelay=True
        )
        
        # Background write thread
        self.write_thread = threading.Thread(target=self._write_loop)
        self.write_thread.daemon = True
        self.write_thread.start()

    # ==========================================
    # PLAYBACK LOGIC
    # ==========================================
    def _process_playback_data(self):
        rospy.loginfo(f"Loading trajectory from {self.playback_csv_filepath}...")
        df = pd.read_csv(self.playback_csv_filepath)
            
        time_sec = df['ros_time_sec'].values + df['ros_time_nsec'].values * 1e-9
        x, y, z = df['O_T_EE_12'].values, df['O_T_EE_13'].values, df['O_T_EE_14'].values
        
        dt = np.gradient(time_sec)
        dt[dt == 0] = 1e-6 
        
        vx_raw, vy_raw, vz_raw = np.gradient(x) / dt, np.gradient(y) / dt, np.gradient(z) / dt
        
        smoothing_sigma = 10 
        self.vx = gaussian_filter1d(vx_raw, smoothing_sigma)
        self.vy = gaussian_filter1d(vy_raw, smoothing_sigma)
        self.vz = gaussian_filter1d(vz_raw, smoothing_sigma)
        
        self.avg_dt = np.mean(dt)
        rospy.loginfo(f"Data processed. Average recording dt: {self.avg_dt:.4f}s")

    def play(self):
        if not hasattr(self, 'vx'):
            return
            
        velocity_multiplier = 2.0 
        time_stretch = 2.0 

        base_playback_rate = 100
        actual_playback_rate = base_playback_rate / time_stretch
        rate = rospy.Rate(actual_playback_rate)
        
        recorded_freq = 1.0 / self.avg_dt
        step = max(1, int(recorded_freq / base_playback_rate))
        total_duration = (len(self.vx) / recorded_freq) * time_stretch
        
        rospy.loginfo(f"Starting playback and recording in 3 seconds. Expected duration: {total_duration:.2f} seconds.")
        rospy.sleep(3)
        
        # Enable recording just as the robot starts moving
        self.recording = True
        twist_msg = Twist()
        
        for i in range(0, len(self.vx), step):
            if rospy.is_shutdown():
                break
                
            twist_msg.linear.x = self.vx[i] * velocity_multiplier
            twist_msg.linear.y = self.vy[i] * velocity_multiplier
            twist_msg.linear.z = self.vz[i] * velocity_multiplier
            self.twist_pub.publish(twist_msg)
            rate.sleep()
            
        # Stop robot
        twist_msg.linear.x = twist_msg.linear.y = twist_msg.linear.z = 0.0
        self.twist_pub.publish(twist_msg)
        
        # Stop recording and flush buffer
        self.recording = False
        with self.buffer_lock:
            self._flush_buffer()
            
        rospy.loginfo(f"Playback complete. Execution data saved to:\n{self.record_filepath}")

    # ==========================================
    # RECORDING LOGIC
    # ==========================================
    def _initialize_record_file(self):
        with open(self.record_filepath, 'w') as f:
            header = "ros_time_sec,ros_time_nsec,"
            header += ",".join([f"O_T_EE_{i}" for i in range(16)]) + ","
            header += ",".join([f"O_T_EE_d_{i}" for i in range(16)]) + ","
            header += "F_ext_x,F_ext_y,F_ext_z,F_ext_wx,F_ext_wy,F_ext_wz\n"
            f.write(header)

    def state_callback(self, msg):
        self.latest_O_T_EE = np.array(msg.O_T_EE, dtype=float)
        self.latest_O_T_EE_d = np.array(msg.O_T_EE_d, dtype=float)
        self.record_data()

    def force_callback(self, msg):
        self.latest_F_ext = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        ], dtype=float)

    def record_data(self):
        if not self.recording:
            return
        
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now

        O_T_EE = self.latest_O_T_EE if self.latest_O_T_EE is not None else np.full(16, np.nan)
        O_T_EE_d = self.latest_O_T_EE_d if self.latest_O_T_EE_d is not None else np.full(16, np.nan)
        F_ext = self.latest_F_ext if self.latest_F_ext is not None else np.full(6, np.nan)

        data_row = [float(now.secs), float(now.nsecs)]
        data_row.extend(O_T_EE.tolist())
        data_row.extend(O_T_EE_d.tolist())
        data_row.extend(F_ext.tolist())
        
        with self.buffer_lock:
            self.data_buffer.append(data_row)
            if len(self.data_buffer) >= self.buffer_size:
                self._flush_buffer()

    def _flush_buffer(self):
        if not self.data_buffer:
            return
        data_array = np.array(self.data_buffer, dtype=float)
        with open(self.record_filepath, 'a') as f:
            np.savetxt(f, data_array, delimiter=',', fmt='%.9f')
        self.data_buffer.clear()
    
    def _write_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.recording:
                with self.buffer_lock:
                    if self.data_buffer:
                        self._flush_buffer()
            rate.sleep()


if __name__ == '__main__':
    CSV_FILE = "/home/hisham246/uwaterloo/panda_ws/src/franka_passive_ds_impedance_controller/robot_demos/free_space/free_space.csv"
    
    try:
        executor = PlaybackAndRecord(CSV_FILE)
        executor.play()
    except rospy.ROSInterruptException:
        pass