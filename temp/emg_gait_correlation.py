import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# Load data
df = pd.read_csv('data/raw/converted_data10.csv')

# Apply EMG signal processing
def process_emg(emg_signal, fs=1000):
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

# Process EMG signals
df['emg_R1_processed'] = process_emg(df['emg_R1'].values)
df['emg_L1_processed'] = process_emg(df['emg_L1'].values)

# Select a section with clear gait cycles
start_idx = 1000
end_idx = 5000
data = df.iloc[start_idx:end_idx].copy()

# Plot thigh angles and processed EMG signals
plt.figure(figsize=(12, 10))

# Plot right leg data
plt.subplot(2, 1, 1)
plt.plot(data['thighDeg_RH'], 'b-', label='Right Hip Angle')
plt.ylabel('Hip Angle (deg)')
plt.twinx()
plt.plot(data['emg_R1_processed'], 'r-', label='Right Quad EMG')
plt.ylabel('EMG Amplitude')
plt.title('Right Leg: Hip Angle and Quad Activation')
# Add two legends
lines, labels = plt.gca().get_legend_handles_labels()
plt.legend(lines, labels, loc='upper right')

# Plot left leg data
plt.subplot(2, 1, 2)
plt.plot(data['thighDeg_LH'], 'b-', label='Left Hip Angle')
plt.ylabel('Hip Angle (deg)')
plt.xlabel('Samples')
plt.twinx()
plt.plot(data['emg_L1_processed'], 'r-', label='Left Quad EMG')
plt.ylabel('EMG Amplitude')
plt.title('Left Leg: Hip Angle and Quad Activation')
# Add two legends
lines, labels = plt.gca().get_legend_handles_labels()
plt.legend(lines, labels, loc='upper right')

plt.tight_layout()
plt.savefig('emg_angle_correlation.png')
print('Plot saved as emg_angle_correlation.png')

# Find gait events based on hip angle
def find_gait_events(hip_angle):
    # Find peaks and valleys in hip angle to detect flexion and extension events
    peaks, _ = signal.find_peaks(hip_angle, height=2, distance=100)  # Flexion
    valleys, _ = signal.find_peaks(-hip_angle, height=-(-2), distance=100)  # Extension
    
    return peaks, valleys

# Find gait events
right_flexion, right_extension = find_gait_events(data['thighDeg_RH'].values)
left_flexion, left_extension = find_gait_events(data['thighDeg_LH'].values)

# Plot gait cycle analysis
plt.figure(figsize=(12, 10))

# Right leg with gait events
plt.subplot(2, 1, 1)
plt.plot(data['thighDeg_RH'], 'b-', label='Right Hip Angle')
plt.plot(right_flexion, data['thighDeg_RH'].iloc[right_flexion], 'go', label='Flexion')
plt.plot(right_extension, data['thighDeg_RH'].iloc[right_extension], 'ro', label='Extension')
plt.plot(data['emg_R1_processed'], 'k-', label='Right Quad EMG')
plt.ylabel('Angle (deg) / EMG')
plt.title('Right Leg Gait Events and Quad Activation')
plt.legend()

# Left leg with gait events
plt.subplot(2, 1, 2)
plt.plot(data['thighDeg_LH'], 'b-', label='Left Hip Angle')
plt.plot(left_flexion, data['thighDeg_LH'].iloc[left_flexion], 'go', label='Flexion')
plt.plot(left_extension, data['thighDeg_LH'].iloc[left_extension], 'ro', label='Extension')
plt.plot(data['emg_L1_processed'], 'k-', label='Left Quad EMG')
plt.ylabel('Angle (deg) / EMG')
plt.xlabel('Samples')
plt.title('Left Leg Gait Events and Quad Activation')
plt.legend()

plt.tight_layout()
plt.savefig('gait_cycle_analysis.png')
print('Plot saved as gait_cycle_analysis.png') 