import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# Load data
df = pd.read_csv('data/raw/converted_data10.csv')

# Take a sample of the data for clarity
sample_start = 1000
sample_length = 2000
emg_right = df['emg_R1'][sample_start:sample_start+sample_length].values
emg_left = df['emg_L1'][sample_start:sample_start+sample_length].values

# Plot original signals
plt.figure(figsize=(15,15))
plt.subplot(4,2,1)
plt.plot(emg_right)
plt.title('Raw Right EMG')
plt.ylabel('Amplitude')

plt.subplot(4,2,2)
plt.plot(emg_left)
plt.title('Raw Left EMG')

# 1. Bandpass filter (20-450Hz typical for EMG)
fs = 1000  # Assume 1kHz sampling frequency
nyquist = 0.5 * fs
low = 20 / nyquist
high = 450 / nyquist
b, a = signal.butter(4, [low, high], btype='band')
emg_right_bandpass = signal.filtfilt(b, a, emg_right)
emg_left_bandpass = signal.filtfilt(b, a, emg_left)

plt.subplot(4,2,3)
plt.plot(emg_right_bandpass)
plt.title('Bandpass Filtered Right EMG')
plt.ylabel('Amplitude')

plt.subplot(4,2,4)
plt.plot(emg_left_bandpass)
plt.title('Bandpass Filtered Left EMG')

# 2. Rectification
emg_right_rectified = np.abs(emg_right_bandpass)
emg_left_rectified = np.abs(emg_left_bandpass)

plt.subplot(4,2,5)
plt.plot(emg_right_rectified)
plt.title('Rectified Right EMG')
plt.ylabel('Amplitude')

plt.subplot(4,2,6)
plt.plot(emg_left_rectified)
plt.title('Rectified Left EMG')

# 3. Envelope detection with low-pass filter
cutoff = 6 / nyquist  # 6 Hz low-pass for envelope
b, a = signal.butter(4, cutoff, btype='low')
emg_right_envelope = signal.filtfilt(b, a, emg_right_rectified)
emg_left_envelope = signal.filtfilt(b, a, emg_left_rectified)

plt.subplot(4,2,7)
plt.plot(emg_right_envelope)
plt.title('Envelope of Right EMG')
plt.xlabel('Samples')
plt.ylabel('Amplitude')

plt.subplot(4,2,8)
plt.plot(emg_left_envelope)
plt.title('Envelope of Left EMG')
plt.xlabel('Samples')

plt.tight_layout()
plt.savefig('emg_filtering_demo.png')
print('Plot saved as emg_filtering_demo.png') 