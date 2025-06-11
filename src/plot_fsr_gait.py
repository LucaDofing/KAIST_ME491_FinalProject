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

FSR_FRONT_THRESHOLD = 2700  # adjust as needed
FSR_HEEL_THRESHOLD = 2700   # adjust as needed

# Get FSR data for left leg
fsr_front = df['fsr_L1']
fsr_heel = df['fsr_L2']

# Gait phase detection
gait_phase = np.zeros(len(df), dtype=int)
for i in range(len(df)):
    if fsr_front[i] < FSR_FRONT_THRESHOLD and fsr_heel[i] > FSR_HEEL_THRESHOLD:
        gait_phase[i] = 1  # INITIAL_CONTACT
    elif fsr_front[i] > FSR_FRONT_THRESHOLD and fsr_heel[i] > FSR_HEEL_THRESHOLD:
        gait_phase[i] = 2  # MID_STANCE
    elif fsr_front[i] > FSR_FRONT_THRESHOLD and fsr_heel[i] < FSR_HEEL_THRESHOLD:
        gait_phase[i] = 3  # TERMINAL_STANCE
    else:
        gait_phase[i] = 4  # SWING

# --- Plot Settings ---
plt.subplot(3, 1, 1)
plt.plot(time, fsr_front, label='FSR Left Front (L1)')
plt.plot(time, fsr_heel, label='FSR Left Heel (L2)')
plt.title('FSR Left Sensors')
plt.xlabel('Time (s)')
plt.ylabel('FSR Value')
plt.legend()
plt.grid(True)
plt.xticks(np.arange(0, time[-1]+1, 1))  # 1-second resolution

plt.subplot(3, 1, 2)
plt.plot(time, gait_phase, drawstyle='steps-post')
plt.yticks([1,2,3,4], ['IC', 'Mid Stance', 'Terminal Stance', 'Swing'])
plt.title('Detected Gait Phase (Left Leg)')
plt.xlabel('Time (s)')
plt.ylabel('Gait Phase')
plt.grid(True)
plt.xticks(np.arange(0, time[-1]+1, 1))  # 1-second resolution

plt.tight_layout()
plt.show()
