import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os
import argparse

class RealtimeEMGController:
    """Simulates the real-time EMG controller with proper timing"""
    
    def __init__(self, threshold=0.01, gain=2.0):
        # EMG Filtering parameters
        self.FILTER_ORDER = 4
        
        # Butterworth filter coefficients (4th order bandpass 20-450Hz)
        self.fs = 1000  # Assume 1kHz sampling frequency
        nyquist = 0.5 * self.fs
        low = 20 / nyquist
        high = 450 / nyquist
        self.bandpass_b, self.bandpass_a = signal.butter(4, [low, high], btype='band')
        
        # Low-pass filter coefficients for envelope detection (6Hz cutoff)
        cutoff = 6 / nyquist
        self.lowpass_b, self.lowpass_a = signal.butter(4, cutoff, btype='low')
        
        # Filter state variables
        self.emg_R1_bp_states = np.zeros(8)
        self.emg_L1_bp_states = np.zeros(8)
        self.emg_R1_lp_states = np.zeros(8)
        self.emg_L1_lp_states = np.zeros(8)
        
        # Controller parameters
        self.threshold = threshold
        self.gain = gain
        
        # State variables
        self.prev_angle_RH = 0.0
        self.prev_angle_LH = 0.0
        self.first_call = True
    
    def process_emg_sample(self, raw_emg, bp_states, lp_states):
        """Process a single EMG sample through the full processing pipeline"""
        # Step 1: Apply bandpass filter
        filtered = signal.lfilter(self.bandpass_b, self.bandpass_a, [raw_emg])[0]
        
        # Step 2: Rectify the signal
        rectified = abs(filtered)
        
        # Step 3: Apply lowpass filter for envelope
        envelope = signal.lfilter(self.lowpass_b, self.lowpass_a, [rectified])[0]
        
        return envelope
    
    def run_controller(self, df, start_idx=0, window_size=5000):
        """Run the controller on a window of data with proper timing"""
        # Select window of data
        end_idx = min(start_idx + window_size, len(df))
        data = df.iloc[start_idx:end_idx].copy()
        
        # Initialize arrays for results
        n_samples = len(data)
        emg_R1_filtered = np.zeros(n_samples)
        emg_L1_filtered = np.zeros(n_samples)
        hip_vel_RH = np.zeros(n_samples)
        hip_vel_LH = np.zeros(n_samples)
        right_swing_phase = np.zeros(n_samples, dtype=bool)
        left_swing_phase = np.zeros(n_samples, dtype=bool)
        right_torque = np.zeros(n_samples)
        left_torque = np.zeros(n_samples)
        
        # Reset controller state
        self.first_call = True
        
        # Process each sample in sequence to simulate real-time operation
        for i in range(n_samples):
            # 1. Get current sample data
            emg_R1_raw = data['emg_R1'].iloc[i]
            emg_L1_raw = data['emg_L1'].iloc[i]
            angle_RH = data['thighDeg_RH'].iloc[i]
            angle_LH = data['thighDeg_LH'].iloc[i]
            
            # 2. Process EMG signals
            emg_R1_filtered[i] = self.process_emg_sample(emg_R1_raw, self.emg_R1_bp_states, self.emg_R1_lp_states)
            emg_L1_filtered[i] = self.process_emg_sample(emg_L1_raw, self.emg_L1_bp_states, self.emg_L1_lp_states)
            
            # 3. Calculate hip velocity
            if self.first_call:
                hip_vel_RH[i] = 0.0
                hip_vel_LH[i] = 0.0
                self.prev_angle_RH = angle_RH
                self.prev_angle_LH = angle_LH
                self.first_call = False
            else:
                hip_vel_RH[i] = (angle_RH - self.prev_angle_RH) * 100.0  # Scale to get deg/sec
                hip_vel_LH[i] = (angle_LH - self.prev_angle_LH) * 100.0
                self.prev_angle_RH = angle_RH
                self.prev_angle_LH = angle_LH
            
            # 4. Detect gait phases
            right_swing_phase[i] = (emg_R1_filtered[i] > self.threshold) and (hip_vel_RH[i] > 0.5)
            left_swing_phase[i] = (emg_L1_filtered[i] > self.threshold) and (hip_vel_LH[i] > 0.5)
            
            # 5. Apply torque based on detected gait phase
            if right_swing_phase[i]:
                right_torque[i] = emg_R1_filtered[i] * self.gain
            
            if left_swing_phase[i]:
                left_torque[i] = emg_L1_filtered[i] * self.gain
        
        # Prepare results
        results = {
            'emg_R1_filtered': emg_R1_filtered,
            'emg_L1_filtered': emg_L1_filtered,
            'hip_vel_RH': hip_vel_RH,
            'hip_vel_LH': hip_vel_LH,
            'right_swing_phase': right_swing_phase,
            'left_swing_phase': left_swing_phase,
            'right_torque': right_torque,
            'left_torque': left_torque
        }
        
        return data, results

def plot_realtime_simulation(data, results, threshold):
    """Plot the results of the real-time EMG controller simulation"""
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
    
    # Plot EMG signals
    axes[2].plot(data['emg_R1'], 'b-', alpha=0.3, label='Raw Right EMG')
    axes[2].plot(data['emg_L1'], 'r-', alpha=0.3, label='Raw Left EMG')
    axes[2].plot(results['emg_R1_filtered'], 'b-', label='Filtered Right EMG')
    axes[2].plot(results['emg_L1_filtered'], 'r-', label='Filtered Left EMG')
    axes[2].axhline(y=threshold, color='k', linestyle='--', label=f'Threshold ({threshold})')
    axes[2].set_ylabel('EMG Amplitude')
    axes[2].legend()
    axes[2].set_title('EMG Signals')
    
    # Plot gait phase detection
    axes[3].plot(results['right_swing_phase'].astype(int), 'b-', label='Right Swing Phase')
    axes[3].plot(results['left_swing_phase'].astype(int), 'r-', label='Left Swing Phase')
    axes[3].set_ylabel('Gait Phase\n(0=Stance, 1=Swing)')
    axes[3].legend()
    axes[3].set_title('Gait Phase Detection')
    
    # Plot assistive torque
    axes[4].plot(results['right_torque'], 'b-', label='Right Assistive Torque')
    axes[4].plot(results['left_torque'], 'r-', label='Left Assistive Torque')
    axes[4].set_ylabel('Torque (Nm)')
    axes[4].set_xlabel('Samples')
    axes[4].legend()
    axes[4].set_title('Assistive Torque')
    
    plt.tight_layout()
    return fig

def main():
    parser = argparse.ArgumentParser(description='Real-time EMG controller simulation')
    parser.add_argument('--recording', type=str, help='Recording file to use (from data/raw folder)')
    parser.add_argument('--start', type=int, default=0, help='Start index for simulation')
    parser.add_argument('--window', type=int, default=5000, help='Window size for simulation')
    parser.add_argument('--threshold', type=float, default=0.01, help='EMG activation threshold')
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
    
    # Create controller
    controller = RealtimeEMGController(threshold=args.threshold, gain=args.gain)
    
    # Run simulation
    print("Running real-time EMG controller simulation...")
    data, results = controller.run_controller(df, start_idx=args.start, window_size=args.window)
    
    # Calculate statistics
    right_swing_percent = np.mean(results['right_swing_phase']) * 100
    left_swing_percent = np.mean(results['left_swing_phase']) * 100
    mean_right_torque = np.mean(results['right_torque'])
    max_right_torque = np.max(results['right_torque'])
    mean_left_torque = np.mean(results['left_torque'])
    max_left_torque = np.max(results['left_torque'])
    
    print("\nSimulation Results:")
    print(f"Right Leg - Swing: {right_swing_percent:.1f}%, Mean Torque: {mean_right_torque:.4f} Nm, Max Torque: {max_right_torque:.4f} Nm")
    print(f"Left Leg - Swing: {left_swing_percent:.1f}%, Mean Torque: {mean_left_torque:.4f} Nm, Max Torque: {max_left_torque:.4f} Nm")
    
    # Plot results
    print("Plotting results...")
    fig = plot_realtime_simulation(data, results, args.threshold)
    
    # Save figure
    output_file = f"realtime_simulation_{os.path.splitext(recording_file)[0]}_t{args.threshold}_g{args.gain}.png"
    fig.savefig(output_file)
    print(f"Plot saved as {output_file}")
    
    plt.show()

if __name__ == "__main__":
    main() 