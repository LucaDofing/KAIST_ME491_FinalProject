import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = "converted5.csv"  # Replace with your actual file path
df = pd.read_csv(file_path)

# Create the figure and axes
fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle("Hip Angles, Torques, and EMG Signals", fontsize=16)

# Plot 1: Hip Angles
axs[0].plot(df['loopCnt'], df['thighDeg_RH'], label='Right Hip Angle')
axs[0].plot(df['loopCnt'], df['thighDeg_LH'], label='Left Hip Angle')
axs[0].set_ylabel('Angle (deg)')
axs[0].legend()
axs[0].grid(True)

# Plot 2: Torques
axs[1].plot(df['loopCnt'], df['u_RH'], label='Right Hip Torque')
axs[1].plot(df['loopCnt'], df['u_LH'], label='Left Hip Torque')
axs[1].set_ylabel('Torque')
axs[1].legend()
axs[1].grid(True)

# Plot 3: EMG
axs[2].plot(df['loopCnt'], df['emg_R1'], label='EMG Right')
axs[2].plot(df['loopCnt'], df['emg_L1'], label='EMG Left')
axs[2].set_xlabel('Loop Count')
axs[2].set_ylabel('EMG Signal')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
