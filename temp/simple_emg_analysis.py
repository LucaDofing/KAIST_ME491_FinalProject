import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os
import argparse

def process_emg(emg_signal, fs=1000):
    """Process EMG signal with bandpass filter, rectification, and envelope detection"""
    # Bandpass filter (20-450Hz typical for EMG)
    nyquist = 0.5 * fs
    low = 20 / nyquist
    high = 450 / nyquist
    b, a = signal.butter(4, [low, high], btype='band')
    emg_bandpass = signal.filtfilt(b, a, emg_signal)
    
    # Rectification
    emg_rectified = np.abs(emg_bandpass)
    
    # Envelope detection with low-pass filter
    cutoff = 6 / nyquist  # 6 Hz low-pass for envelope
    b, a = signal.butter(4, cutoff, btype='low')
    emg_envelope = signal.filtfilt(b, a, emg_rectified)
    
    return emg_envelope

def simple_emg_controller(df, threshold=0.02, gain=2.0, start_idx=0, window_size=5000):
    """Simple EMG controller that applies torque directly based on EMG threshold"""
    # Select window of data
    end_idx = min(start_idx + window_size, len(df))
    data = df.iloc[start_idx:end_idx].copy()
    
    # Process EMG signals
    emg_R1_filtered = process_emg(data['emg_R1'].values)
    emg_L1_filtered = process_emg(data['emg_L1'].values)
    
    # Calculate hip velocity
    hip_vel_RH = np.zeros(len(data))
    hip_vel_LH = np.zeros(len(data))
    
    for i in range(1, len(data)):
        hip_vel_RH[i] = (data['thighDeg_RH'].iloc[i] - data['thighDeg_RH'].iloc[i-1]) * 100.0
        hip_vel_LH[i] = (data['thighDeg_LH'].iloc[i] - data['thighDeg_LH'].iloc[i-1]) * 100.0
    
    # Apply simple threshold-based control
    # Only apply torque when EMG exceeds threshold AND hip is in flexion (positive velocity)
    right_torque = np.zeros(len(data))
    left_torque = np.zeros(len(data))
    
    for i in range(len(data)):
        # Right leg: Apply torque when EMG exceeds threshold AND hip is flexing
        if emg_R1_filtered[i] > threshold and hip_vel_RH[i] > 0:
            right_torque[i] = emg_R1_filtered[i] * gain
        
        # Left leg: Apply torque when EMG exceeds threshold AND hip is flexing
        if emg_L1_filtered[i] > threshold and hip_vel_LH[i] > 0:
            left_torque[i] = emg_L1_filtered[i] * gain
    
    # Create results dictionary
    results = {
        'emg_R1_filtered': emg_R1_filtered,
        'emg_L1_filtered': emg_L1_filtered,
        'right_torque': right_torque,
        'left_torque': left_torque,
        'hip_vel_RH': hip_vel_RH,
        'hip_vel_LH': hip_vel_LH,
        'right_active': (emg_R1_filtered > threshold) & (hip_vel_RH > 0),
        'left_active': (emg_L1_filtered > threshold) & (hip_vel_LH > 0)
    }
    
    return data, results

def plot_results(data, results, threshold):
    """Plot the results of the simple EMG controller"""
    fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
    
    # Plot hip angles
    axes[0].plot(data['thighDeg_RH'], 'b-', label='Right Hip Angle')
    axes[0].plot(data['thighDeg_LH'], 'r-', label='Left Hip Angle')
    axes[0].set_ylabel('Hip Angle (deg)')
    axes[0].legend()
    axes[0].set_title('Hip Angles')
    
    # Plot hip velocities
    axes[1].plot(results['hip_vel_RH'], 'b-', label='Right Hip Velocity')
    axes[1].plot(results['hip_vel_LH'], 'r-', label='Left Hip Velocity')
    axes[1].set_ylabel('Velocity (deg/s)')
    axes[1].legend()
    axes[1].set_title('Hip Velocities')
    
    # Plot raw and filtered EMG
    axes[2].plot(data['emg_R1'], 'b-', alpha=0.3, label='Raw Right EMG')
    axes[2].plot(data['emg_L1'], 'r-', alpha=0.3, label='Raw Left EMG')
    axes[2].plot(results['emg_R1_filtered'], 'b-', label='Filtered Right EMG')
    axes[2].plot(results['emg_L1_filtered'], 'r-', label='Filtered Left EMG')
    axes[2].axhline(y=threshold, color='k', linestyle='--', label=f'Threshold ({threshold})')
    axes[2].set_ylabel('EMG Amplitude')
    axes[2].legend()
    axes[2].set_title('EMG Signals')
    
    # Plot controller activation
    axes[3].plot(results['right_active'].astype(int), 'b-', label='Right Leg Active')
    axes[3].plot(results['left_active'].astype(int), 'r-', label='Left Leg Active')
    axes[3].set_ylabel('Active State\n(0=Off, 1=On)')
    axes[3].legend()
    axes[3].set_title('Controller Activation')
    
    # Plot output torques
    axes[4].plot(results['right_torque'], 'b-', label='Right Assist Torque')
    axes[4].plot(results['left_torque'], 'r-', label='Left Assist Torque')
    axes[4].set_ylabel('Torque (Nm)')
    axes[4].set_xlabel('Samples')
    axes[4].legend()
    axes[4].set_title('Assistive Torque')
    
    plt.tight_layout()
    return fig

def main():
    parser = argparse.ArgumentParser(description='Simple EMG-based controller simulation')
    parser.add_argument('--recording', type=str, help='Recording file to use (from data/raw folder)')
    parser.add_argument('--start', type=int, default=0, help='Start index for analysis')
    parser.add_argument('--window', type=int, default=5000, help='Window size for analysis')
    parser.add_argument('--threshold', type=float, default=0.02, help='EMG activation threshold')
    parser.add_argument('--gain', type=float, default=2.0, help='Torque gain')
    args = parser.parse_args()
    
    # Path to data directory
    data_dir = '../data/raw'
    
    # Default to recording 10 if none specified
    recording_file = args.recording if args.recording else 'converted_data10.csv'
    file_path = os.path.join(data_dir, recording_file)
    
    # Load data
    print(f"Loading data from {file_path}...")
    df = pd.read_csv(file_path)
    
    # Run simple EMG controller
    print("Analyzing EMG data...")
    data, results = simple_emg_controller(
        df, 
        threshold=args.threshold,
        gain=args.gain,
        start_idx=args.start,
        window_size=args.window
    )
    
    # Calculate statistics
    right_active_percent = np.mean(results['right_active']) * 100
    left_active_percent = np.mean(results['left_active']) * 100
    mean_right_torque = np.mean(results['right_torque'])
    max_right_torque = np.max(results['right_torque'])
    mean_left_torque = np.mean(results['left_torque'])
    max_left_torque = np.max(results['left_torque'])
    
    print("\nController Statistics:")
    print(f"Right Leg - Active: {right_active_percent:.1f}%, Mean Torque: {mean_right_torque:.4f} Nm, Max Torque: {max_right_torque:.4f} Nm")
    print(f"Left Leg - Active: {left_active_percent:.1f}%, Mean Torque: {mean_left_torque:.4f} Nm, Max Torque: {max_left_torque:.4f} Nm")
    
    # Plot results
    print("Plotting results...")
    fig = plot_results(data, results, args.threshold)
    
    # Save figure
    output_file = f"simple_emg_controller_{os.path.splitext(recording_file)[0]}_t{args.threshold}_g{args.gain}.png"
    fig.savefig(output_file)
    print(f"Plot saved as {output_file}")
    
    plt.show()

if __name__ == "__main__":
    main() 