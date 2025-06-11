import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- Load CSV ---
csv_path = "data/raw/FSR/converted_data3.csv"  # <- Replace with your actual file path
df = pd.read_csv(csv_path)

# Set your sampling rate (Hz)
fs = 1000  # Change this to your actual sampling rate

# Create a time array
time = np.arange(len(df)) / fs

# --- Plot Settings ---
plt.figure(figsize=(12, 6))

# --- FSR Data Plot ---
plt.subplot(2, 1, 1)
plt.plot(time, df['fsr_R1'], label='FSR Right 1')
plt.plot(time, df['fsr_R2'], label='FSR Right 2')
# plt.plot(time, df['fsr_L1'], label='FSR Left 1')
# plt.plot(time, df['fsr_L2'], label='FSR Left 2')
plt.title('FSR Sensor Readings')
plt.xlabel('Time')
plt.ylabel('FSR Value')
plt.legend()
plt.grid(True)
plt.locator_params(axis='x', nbins=40)  # Increase number of x-ticks

plt.subplot(2, 1, 2)
# plt.plot(df['fsr_R1'], label='FSR Right 1')
# plt.plot(df['fsr_R2'], label='FSR Right 2')
plt.plot(time, df['fsr_L1'], label='FSR Left 1')
plt.plot(time, df['fsr_L2'], label='FSR Left 2')
plt.title('FSR Sensor Readings')
plt.xlabel('Time')
plt.ylabel('FSR Value')
plt.legend()
plt.grid(True)
plt.locator_params(axis='x', nbins=40)  # Increase number of x-ticks

# --- Hip Angle Plot ---
# plt.subplot(2, 1, 2)
# plt.plot(df['thighDeg_RH'], label='Right Hip Angle')
# plt.plot(df['thighDeg_LH'], label='Left Hip Angle')
# plt.title('Hip Angles')
# plt.xlabel('Sample')
# plt.ylabel('Angle (degrees)')
# plt.legend()
# plt.grid(True)

plt.tight_layout()
plt.show()
