import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

# --- File list ---
fsr_folder = "data/raw/FSR"
file_pattern = os.path.join(fsr_folder, "converted_data_fsr_*.csv")
# file_list = sorted(glob.glob(file_pattern))
file_list = ["data/raw/FSR/fsr assisted 6.12/converted_fsr_612_2.csv"]

# Set your sampling rate (Hz)
fs = 1000  # Change this to your actual sampling rate

for idx, file in enumerate(file_list[:5]):
    df = pd.read_csv(file)
    time = np.arange(len(df)) / fs

    # Filter for first 30 seconds
    mask = time <= 30
    time_30 = time[mask]

    # FSR right leg
    fsr_r1 = df['fsr_R1'][mask]
    fsr_r2 = df['fsr_R2'][mask]
    # Thigh angles
    thigh_rh = df['thighDeg_RH'][mask]
    thigh_lh = df['thighDeg_LH'][mask]

    plt.figure(figsize=(12, 6))

    # FSR subplot
    plt.subplot(2, 1, 1)
    plt.plot(time_30, fsr_r1, label='FSR Right 1')
    plt.plot(time_30, fsr_r2, label='FSR Right 2')
    plt.title(f"FSR Right Leg - {os.path.basename(file)}")
    plt.xlabel('Time (s)')
    plt.ylabel('FSR Value')
    plt.legend()
    plt.grid(True)
    plt.xlim([0, min(30, time_30[-1])])

    # Thigh angle subplot
    plt.subplot(2, 1, 2)
    plt.plot(time_30, thigh_rh, label='Thigh RH')
    plt.plot(time_30, thigh_lh, label='Thigh LH')
    plt.title("Thigh Angles")
    plt.xlabel('Time (s)')
    plt.ylabel('Thigh Angle (deg)')
    plt.legend()
    plt.grid(True)
    plt.xlim([0, min(30, time_30[-1])])

    plt.tight_layout()
    plt.show()
