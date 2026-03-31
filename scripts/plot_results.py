import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

# Global font size settings for matplotlib
plt.rcParams.update({
    'font.size': 24,
    'axes.titlesize': 26,
    'axes.labelsize': 24,
    'xtick.labelsize': 24,
    'ytick.labelsize': 24,
    'legend.fontsize': 16,
    'figure.titlesize': 28
})

def generate_desired_trajectory(demo_csv_path, start_pos_actual):
    df = pd.read_csv(demo_csv_path)
    time_sec = df['ros_time_sec'].values + df['ros_time_nsec'].values * 1e-9
    
    x = df['O_T_EE_12'].values
    y = df['O_T_EE_13'].values
    z = df['O_T_EE_14'].values
    
    dt = np.gradient(time_sec)
    dt[dt == 0] = 1e-6
    
    vx_raw = np.gradient(x) / dt
    vy_raw = np.gradient(y) / dt
    vz_raw = np.gradient(z) / dt
    
    smoothing_sigma = 10
    vx = gaussian_filter1d(vx_raw, smoothing_sigma)
    vy = gaussian_filter1d(vy_raw, smoothing_sigma)
    vz = gaussian_filter1d(vz_raw, smoothing_sigma)
    
    avg_dt = np.mean(dt)
    
    velocity_multiplier = 2.0
    time_stretch = 2.0
    base_playback_rate = 100
    actual_playback_rate = base_playback_rate / time_stretch # 50 Hz
    playback_dt = 1.0 / actual_playback_rate
    
    recorded_freq = 1.0 / avg_dt
    step = max(1, int(recorded_freq / base_playback_rate))
    
    indices = np.arange(0, len(vx), step)
    
    cmd_vx = vx[indices] * velocity_multiplier
    cmd_vy = vy[indices] * velocity_multiplier
    cmd_vz = vz[indices] * velocity_multiplier
    
    cmd_time = np.arange(len(indices)) * playback_dt
    
    cmd_pos_x = start_pos_actual[0] + np.cumsum(cmd_vx) * playback_dt
    cmd_pos_y = start_pos_actual[1] + np.cumsum(cmd_vy) * playback_dt
    cmd_pos_z = start_pos_actual[2] + np.cumsum(cmd_vz) * playback_dt
    
    df_cmd = pd.DataFrame({
        'time': cmd_time,
        'vd_x': cmd_vx, 'vd_y': cmd_vy, 'vd_z': cmd_vz,
        'pd_x': cmd_pos_x, 'pd_y': cmd_pos_y, 'pd_z': cmd_pos_z
    })
    
    return df_cmd

def load_and_prep(file1, file2):
    df1 = pd.read_csv(file1)
    df2 = pd.read_csv(file2)
    
    for df in [df1, df2]:
        df['time'] = (df['ros_time_sec'] - df['ros_time_sec'].iloc[0]) + (df['ros_time_nsec'] - df['ros_time_nsec'].iloc[0]) * 1e-9
        
        dt = df['time'].diff()
        dt[dt == 0] = 1e-6 
        
        for axis, col in zip(['x', 'y', 'z'], ['12', '13', '14']):
            df[f'v_{axis}'] = df[f'O_T_EE_{col}'].diff() / dt
            df[f'v_{axis}'].fillna(0, inplace=True)
            df[f'v_{axis}'] = df[f'v_{axis}'].rolling(window=10, min_periods=1).mean()
            
    return df1, df2

def calculate_rmse(actual, desired):
    return np.sqrt(np.mean((actual - desired)**2))

def analyze_and_print_metrics(scenario_name, df_actual, df_cmd, tuning_name):
    metrics = {}
    
    for axis, col in zip(['x', 'y', 'z'], ['12', '13', '14']):
        pd_interp = np.interp(df_actual['time'], df_cmd['time'], df_cmd[f'pd_{axis}'])
        vd_interp = np.interp(df_actual['time'], df_cmd['time'], df_cmd[f'vd_{axis}'])
        
        metrics[f'pos_rmse_{axis}'] = calculate_rmse(df_actual[f'O_T_EE_{col}'], pd_interp)
        metrics[f'vel_rmse_{axis}'] = calculate_rmse(df_actual[f'v_{axis}'], vd_interp)
        
        df_actual[f'p_err_{axis}'] = df_actual[f'O_T_EE_{col}'] - pd_interp
        df_actual[f'v_err_{axis}'] = df_actual[f'v_{axis}'] - vd_interp

    df_actual['pos_err_mag'] = np.sqrt(df_actual['p_err_x']**2 + df_actual['p_err_y']**2 + df_actual['p_err_z']**2)
    df_actual['vel_err_mag'] = np.sqrt(df_actual['v_err_x']**2 + df_actual['v_err_y']**2 + df_actual['v_err_z']**2)
    df_actual['f_ext_mag'] = np.sqrt(df_actual['F_ext_x']**2 + df_actual['F_ext_y']**2 + df_actual['F_ext_z']**2)
    
    print(f"  [{tuning_name} Tuning]")
    print(f"    Position RMSE (m)     -> X: {metrics['pos_rmse_x']:.4f} | Y: {metrics['pos_rmse_y']:.4f} | Z: {metrics['pos_rmse_z']:.4f}")
    print(f"    Overall 3D Pos Error  -> {df_actual['pos_err_mag'].mean():.4f} m (Mean) | {df_actual['pos_err_mag'].max():.4f} m (Max)")
    print(f"    Velocity RMSE (m/s)   -> X: {metrics['vel_rmse_x']:.4f} | Y: {metrics['vel_rmse_y']:.4f} | Z: {metrics['vel_rmse_z']:.4f}")
    print(f"    Overall 3D Vel Error  -> {df_actual['vel_err_mag'].mean():.4f} m/s (Mean) | {df_actual['vel_err_mag'].max():.4f} m/s (Max)")
    print(f"    Ext. Force Mag (N)    -> {df_actual['f_ext_mag'].mean():.4f} N (Mean) | {df_actual['f_ext_mag'].max():.4f} N (Max)\n")

def main():
    scenarios = [
        {
            'name': 'Free_Space',
            'demo_file': 'robot_demos/free_space/free_space.csv',
            'file_iso': 'robot_demos/free_space/free_space_execution_tracking_150_150_150.csv',
            'file_aniso': 'robot_demos/free_space/free_space_execution_tracking_250_50_50.csv'
        },
        {
            'name': 'Contact',
            'demo_file': 'robot_demos/contact/contact.csv',
            'file_iso': 'robot_demos/contact/contact_execution_tracking_1_150_150_150.csv',
            'file_aniso': 'robot_demos/contact/contact_execution_tracking_2_250_50_50.csv'
        },
        {
            'name': 'Disturbance',
            'demo_file': 'robot_demos/free_space/free_space.csv', 
            'file_iso': 'robot_demos/disturbance/disturbance_execution_tracking_2_150_150_150.csv',
            'file_aniso': 'robot_demos/disturbance/disturbance_execution_tracking_250_50_50.csv'
        }
    ]

    for scenario in scenarios:
        print(f"======================================================")
        print(f"Processing {scenario['name']} Analysis...")
        print(f"======================================================")
        
        df_iso, df_aniso = load_and_prep(scenario['file_iso'], scenario['file_aniso'])
        
        start_pos = [df_iso['O_T_EE_12'].iloc[0], df_iso['O_T_EE_13'].iloc[0], df_iso['O_T_EE_14'].iloc[0]]
        df_cmd = generate_desired_trajectory(scenario['demo_file'], start_pos)
        
        analyze_and_print_metrics(scenario['name'], df_iso, df_cmd, "Isotropic")
        analyze_and_print_metrics(scenario['name'], df_aniso, df_cmd, "Anisotropic")
        
        # ---------------------------------------------------------
        # 1. TRACKING PLOTS 
        # ---------------------------------------------------------
        fig_track, axs_track = plt.subplots(2, 3, figsize=(20, 11))
        fig_track.suptitle(f"{scenario['name'].replace('_', ' ')} Tracking Analysis: Isotropic vs Anisotropic", fontweight='bold')
        
        axes = ['x', 'y', 'z']
        pos_cols = ['12', '13', '14'] 
        
        for i, (ax_name, p_col) in enumerate(zip(axes, pos_cols)):
            axs_track[0, i].plot(df_cmd['time'], df_cmd[f'pd_{ax_name}'], 'k--', linewidth=2.5, label='Desired')
            axs_track[0, i].plot(df_iso['time'], df_iso[f'O_T_EE_{p_col}'], 'b-', linewidth=2, alpha=0.7, label='Actual (Iso)')
            axs_track[0, i].plot(df_aniso['time'], df_aniso[f'O_T_EE_{p_col}'], 'r-', linewidth=2, alpha=0.7, label='Actual (Aniso)')
            axs_track[0, i].set_title(f'Position ({ax_name.upper()})')
            axs_track[0, i].set_xlabel('Time (s)')
            axs_track[0, i].set_ylabel('Position (m)')
            axs_track[0, i].legend(loc='best')
            axs_track[0, i].grid(True)
            
            axs_track[1, i].plot(df_cmd['time'], df_cmd[f'vd_{ax_name}'], 'k--', linewidth=2.5, label='Desired')
            axs_track[1, i].plot(df_iso['time'], df_iso[f'v_{ax_name}'], 'b-', linewidth=2, alpha=0.7, label='Actual (Iso)')
            axs_track[1, i].plot(df_aniso['time'], df_aniso[f'v_{ax_name}'], 'r-', linewidth=2, alpha=0.7, label='Actual (Aniso)')
            axs_track[1, i].set_title(f'Velocity ({ax_name.upper()})')
            axs_track[1, i].set_xlabel('Time (s)')
            axs_track[1, i].set_ylabel('Velocity (m/s)')
            axs_track[1, i].legend(loc='best')
            axs_track[1, i].grid(True)
            
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(f"{scenario['name']}_Tracking.pdf", format='pdf')
        plt.close()
        print(f"--> Saved {scenario['name']}_Tracking.pdf")
        
        # ---------------------------------------------------------
        # 2. FORCE MAGNITUDE PLOT
        # ---------------------------------------------------------
        fig_force, ax_force = plt.subplots(figsize=(10, 6))
        # Optional: uncomment to add a title back to the force plot
        # fig_force.suptitle(f"{scenario['name'].replace('_', ' ')} Interaction Force Magnitude", fontweight='bold')
        
        ax_force.plot(df_iso['time'], df_iso['f_ext_mag'], 'b-', linewidth=3, alpha=0.8, label='Isotropic')
        ax_force.plot(df_aniso['time'], df_aniso['f_ext_mag'], 'r-', linewidth=3, alpha=0.8, label='Anisotropic')
        ax_force.set_xlabel('Time (s)')
        ax_force.set_ylabel('Force Magnitude (N)')
        ax_force.legend()
        ax_force.grid(True)
        
        plt.tight_layout()
        plt.savefig(f"{scenario['name']}_Force.pdf", format='pdf')
        plt.close()
        print(f"--> Saved {scenario['name']}_Force.pdf\n")

if __name__ == '__main__':
    main()