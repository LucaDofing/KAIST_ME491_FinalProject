import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os
import argparse

class EMGController:
    def __init__(self):
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
        
        # Filter state variables for real-time filtering
        # Get correct filter state sizes
        self.z_bp = signal.lfilter_zi(self.bandpass_b, self.bandpass_a)
        self.z_lp = signal.lfilter_zi(self.lowpass_b, self.lowpass_a)
        
        self.emg_R1_bp_states = np.zeros_like(self.z_bp)
        self.emg_L1_bp_states = np.zeros_like(self.z_bp)
        self.emg_R1_lp_states = np.zeros_like(self.z_lp)
        self.emg_L1_lp_states = np.zeros_like(self.z_lp)
        
        # Normalization parameters
        self.emg_R1_max = 1.0  # Will be updated during calibration
        self.emg_L1_max = 1.0  # Will be updated during calibration
        
        # Direct mapping gain - scales the normalized EMG to torque
        self.direct_torque_gain = 2.0
        
        # Minimum activation threshold to avoid noise
        self.min_activation_threshold = 0.05
        
    def calibrate(self, emg_R1_data, emg_L1_data):
        """Calibrate the controller with sample EMG data to set normalization values"""
        # Process the calibration data through the same filters
        filtered_R1 = np.zeros_like(emg_R1_data)
        filtered_L1 = np.zeros_like(emg_L1_data)
        
        # Reset filter states
        bp_states_R = np.zeros_like(self.z_bp)
        lp_states_R = np.zeros_like(self.z_lp)
        bp_states_L = np.zeros_like(self.z_bp)
        lp_states_L = np.zeros_like(self.z_lp)
        
        # Process each sample
        for i in range(len(emg_R1_data)):
            filtered_R1[i], bp_states_R, lp_states_R = self.process_emg_sample_with_states(
                emg_R1_data[i], bp_states_R, lp_states_R)
            filtered_L1[i], bp_states_L, lp_states_L = self.process_emg_sample_with_states(
                emg_L1_data[i], bp_states_L, lp_states_L)
        
        # Set max values for normalization (with a buffer to avoid clipping)
        self.emg_R1_max = np.percentile(filtered_R1, 95) * 1.2
        self.emg_L1_max = np.percentile(filtered_L1, 95) * 1.2
        
        print(f"Calibration complete: Right max = {self.emg_R1_max:.4f}, Left max = {self.emg_L1_max:.4f}")
    
    def process_emg_sample_with_states(self, raw_emg, bp_states, lp_states):
        """Process a single EMG sample and return updated filter states"""
        # Step 1: Apply bandpass filter to remove noise and DC offset
        filtered, bp_states = signal.lfilter(self.bandpass_b, self.bandpass_a, [raw_emg], zi=bp_states)
        
        # Step 2: Rectify the signal (take absolute value)
        rectified = abs(filtered[0])
        
        # Step 3: Apply lowpass filter to get the envelope
        envelope, lp_states = signal.lfilter(self.lowpass_b, self.lowpass_a, [rectified], zi=lp_states)
        
        return envelope[0], bp_states, lp_states
    
    def process_emg_sample(self, raw_emg, is_right_side=True):
        """Process a single EMG sample in real-time and return normalized envelope"""
        if is_right_side:
            envelope, self.emg_R1_bp_states, self.emg_R1_lp_states = self.process_emg_sample_with_states(
                raw_emg, self.emg_R1_bp_states, self.emg_R1_lp_states)
            # Normalize by max value
            normalized = envelope / self.emg_R1_max if self.emg_R1_max > 0 else envelope
        else:
            envelope, self.emg_L1_bp_states, self.emg_L1_lp_states = self.process_emg_sample_with_states(
                raw_emg, self.emg_L1_bp_states, self.emg_L1_lp_states)
            # Normalize by max value
            normalized = envelope / self.emg_L1_max if self.emg_L1_max > 0 else envelope
        
        # Clip to range [0, 1]
        normalized = np.clip(normalized, 0, 1)
        
        return normalized, envelope
    
    def compute_torque(self, normalized_emg):
        """Convert normalized EMG to torque using direct mapping"""
        # Apply minimum threshold to avoid noise-induced movements
        if normalized_emg < self.min_activation_threshold:
            return 0.0
        
        # Direct mapping from EMG to torque
        torque = normalized_emg * self.direct_torque_gain
        
        return torque
    
    def simulate_controller(self, df):
        """Simulate the controller on recorded data"""
        # Create arrays to store results
        n_samples = len(df)
        emg_R1_raw = np.array(df['emg_R1'])
        emg_L1_raw = np.array(df['emg_L1'])
        
        # Calibrate using the first portion of data
        calibration_window = min(5000, n_samples // 4)  # Use first 25% or 5000 samples
        self.calibrate(emg_R1_raw[:calibration_window], emg_L1_raw[:calibration_window])
        
        # Arrays for results
        emg_R1_filtered = np.zeros(n_samples)
        emg_L1_filtered = np.zeros(n_samples)
        emg_R1_normalized = np.zeros(n_samples)
        emg_L1_normalized = np.zeros(n_samples)
        right_torque = np.zeros(n_samples)
        left_torque = np.zeros(n_samples)
        
        # Reset filter states
        self.emg_R1_bp_states = np.zeros_like(self.z_bp)
        self.emg_R1_lp_states = np.zeros_like(self.z_lp)
        self.emg_L1_bp_states = np.zeros_like(self.z_bp)
        self.emg_L1_lp_states = np.zeros_like(self.z_lp)
        
        # Process each sample in real-time fashion
        for i in range(n_samples):
            # Get raw EMG data
            emg_R1_raw_sample = df['emg_R1'].iloc[i]
            emg_L1_raw_sample = df['emg_L1'].iloc[i]
            
            # Process EMG signals
            emg_R1_normalized[i], emg_R1_filtered[i] = self.process_emg_sample(emg_R1_raw_sample, is_right_side=True)
            emg_L1_normalized[i], emg_L1_filtered[i] = self.process_emg_sample(emg_L1_raw_sample, is_right_side=False)
            
            # Compute torque directly from normalized EMG
            right_torque[i] = self.compute_torque(emg_R1_normalized[i])
            left_torque[i] = self.compute_torque(emg_L1_normalized[i])
        
        return {
            'emg_R1_filtered': emg_R1_filtered,
            'emg_L1_filtered': emg_L1_filtered,
            'emg_R1_normalized': emg_R1_normalized,
            'emg_L1_normalized': emg_L1_normalized,
            'right_torque': right_torque,
            'left_torque': left_torque
        }

def plot_simulation_results(df, results, start_idx=0, window_size=2000):
    """Plot simulation results for a specific window of data"""
    end_idx = min(start_idx + window_size, len(df))
    time_slice = slice(start_idx, end_idx)
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 15), sharex=True)
    
    # Plot hip angles
    axes[0].plot(df['thighDeg_RH'].iloc[time_slice], 'b-', label='Right Hip Angle')
    axes[0].plot(df['thighDeg_LH'].iloc[time_slice], 'r-', label='Left Hip Angle')
    axes[0].set_ylabel('Hip Angle (deg)')
    axes[0].legend()
    axes[0].set_title('Hip Angles')
    
    # Plot raw EMG
    axes[1].plot(df['emg_R1'].iloc[time_slice], 'b-', alpha=0.5, label='Raw Right EMG')
    axes[1].plot(df['emg_L1'].iloc[time_slice], 'r-', alpha=0.5, label='Raw Left EMG')
    axes[1].set_ylabel('Raw EMG')
    axes[1].legend()
    axes[1].set_title('Raw EMG Signals')
    
    # Plot normalized EMG
    axes[2].plot(results['emg_R1_normalized'][time_slice], 'b-', label='Normalized Right EMG')
    axes[2].plot(results['emg_L1_normalized'][time_slice], 'r-', label='Normalized Left EMG')
    axes[2].axhline(y=0.05, color='k', linestyle='--', label='Min Activation Threshold')
    axes[2].set_ylabel('Normalized EMG')
    axes[2].set_ylim(0, 1.1)
    axes[2].legend()
    axes[2].set_title('Normalized EMG Signals')
    
    # Plot output torques
    axes[3].plot(results['right_torque'][time_slice], 'b-', label='Right Assist Torque')
    axes[3].plot(results['left_torque'][time_slice], 'r-', label='Left Assist Torque')
    axes[3].set_ylabel('Torque (Nm)')
    axes[3].set_xlabel('Samples')
    axes[3].legend()
    axes[3].set_title('Real-time Assistive Torque')
    
    plt.tight_layout()
    return fig

def list_available_recordings():
    """List all available recordings in the data/raw folder"""
    recordings = []
    data_dir = '../data/raw'
    for file in os.listdir(data_dir):
        if file.startswith('converted_data') and file.endswith('.csv'):
            recordings.append(file)
        elif file.startswith('SUIT_LOGGED_DATA-') and file.endswith('.csv'):
            recordings.append(file)
    return sorted(recordings)

def main():
    parser = argparse.ArgumentParser(description='Simulate real-time EMG-based controller using recorded data')
    parser.add_argument('--recording', type=str, help='Recording file to use (from data/raw folder)')
    parser.add_argument('--list', action='store_true', help='List available recordings')
    parser.add_argument('--start', type=int, default=0, help='Start index for plotting')
    parser.add_argument('--window', type=int, default=2000, help='Window size for plotting')
    parser.add_argument('--gain', type=float, default=2.0, help='Direct torque gain')
    parser.add_argument('--threshold', type=float, default=0.05, help='Minimum activation threshold')
    args = parser.parse_args()
    
    # Path to data directory
    data_dir = '../data/raw'
    
    if args.list:
        recordings = list_available_recordings()
        print("Available recordings:")
        for i, rec in enumerate(recordings):
            print(f"{i+1}. {rec}")
        return
    
    # Default to recording 10 if none specified
    recording_file = args.recording if args.recording else 'converted_data10.csv'
    
    # Load data
    file_path = os.path.join(data_dir, recording_file)
    print(f"Loading data from {file_path}...")
    df = pd.read_csv(file_path)
    
    # Create controller
    controller = EMGController()
    controller.direct_torque_gain = args.gain
    controller.min_activation_threshold = args.threshold
    
    # Simulate controller
    print("Simulating real-time controller...")
    results = controller.simulate_controller(df)
    
    # Plot results
    print("Plotting results...")
    fig = plot_simulation_results(df, results, args.start, args.window)
    
    # Calculate statistics
    mean_right_torque = np.mean(results['right_torque'])
    max_right_torque = np.max(results['right_torque'])
    mean_left_torque = np.mean(results['left_torque'])
    max_left_torque = np.max(results['left_torque'])
    
    print("\nController Statistics:")
    print(f"Right Leg - Mean Torque: {mean_right_torque:.4f} Nm, Max Torque: {max_right_torque:.4f} Nm")
    print(f"Left Leg - Mean Torque: {mean_left_torque:.4f} Nm, Max Torque: {max_left_torque:.4f} Nm")
    
    # Save figure
    output_file = f"realtime_controller_{os.path.splitext(recording_file)[0]}.png"
    fig.savefig(output_file)
    print(f"Plot saved as {output_file}")
    
    plt.show()

if __name__ == "__main__":
    main() 