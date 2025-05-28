# HAR_FinalProject/src/config.py

# --- General Configuration ---
# SAMPLING_RATE: How many data points per second for your EMG signal.
# This is CRITICAL. If your robot logs data every 5ms, Fs = 1/0.005 = 200 Hz.
# If you have a dedicated EMG DAQ, it might be 1000 Hz, 2000 Hz, etc.
SAMPLING_RATE = 1000  # Hz <<< --- !!! SET YOUR ACTUAL SAMPLING RATE HERE !!!

# --- EMG Filter Parameters ---
# These are common starting points, tune as needed based on your data.

# 1. High-pass filter to remove DC offset and low-frequency motion artifacts
HIGHPASS_FILTER_PARAMS = {
    "cutoff": 20,  # Hz (Commonly 20-50 Hz for EMG)
    "order": 4     # Butterworth filter order (e.g., 2 to 4)
}

# 2. Notch filter to remove powerline interference
NOTCH_FILTER_PARAMS = {
    "freq": 50.0,  # Hz (Use 60.0 Hz if you're in a 60Hz powerline region)
    "quality_factor": 30.0 # Controls the narrowness of the notch (e.g., 30-60)
}
# Optional: To apply notch filters for harmonics, you could extend this:
# NOTCH_FILTER_HARMONICS = {
#     "apply": False, # Set to True to apply harmonic notches
#     "multiples": [2, 3] # e.g., for 100Hz, 150Hz if fundamental is 50Hz
# }


# 3. Rectification (no parameters, just taking absolute value)

# 4. Low-pass filter for envelope extraction
LOWPASS_ENVELOPE_FILTER_PARAMS = {
    "cutoff": 6,   # Hz (Commonly 2-10 Hz for EMG envelope, lower for smoother)
    "order": 2     # Butterworth filter order (e.g., 2 to 4)
}

# --- Data File and Column Configuration (for main script execution) ---
# These can be overridden if the script is called with specific arguments in the future.
DEFAULT_DATA_FILEPATH = "data/raw/converted_data1.csv" 
DEFAULT_EMG_COLUMN_NAME = "emg_R1"
DEFAULT_TIME_COLUMN_NAME = "time" # or "loopCnt", or None if to be generated

# --- Simulation Parameters (if using simulated data) ---
SIMULATION_PARAMS = {
    "duration_seconds": 10,
    "use_if_file_not_found": True,
    "burst_starts_seconds": [1, 3.5, 6, 8],
    "burst_durations_seconds": [0.8, 1.0, 0.5, 0.7],
    "noise_levels": {
        "activation_modulation": 1.0, # Multiplier for noise modulated by activation
        "baseline_white_noise": 0.05,
        "motion_artifact_amplitude": 0.1,
        "powerline_interference_amplitude": 0.25
    }
}

# --- Plotting Configuration ---
PLOTTING_PARAMS = {
    "save_plots": True,
    "steps_plot_figsize": (14, 10),
    "comparison_plot_figsize": (14, 6)
}

if __name__ == '__main__':
    print("--- EMG Processing Configuration ---")
    print(f"SAMPLING_RATE: {SAMPLING_RATE} Hz")
    print("\nHigh-Pass Filter:")
    for key, value in HIGHPASS_FILTER_PARAMS.items():
        print(f"  {key}: {value}")
    print("\nNotch Filter:")
    for key, value in NOTCH_FILTER_PARAMS.items():
        print(f"  {key}: {value}")
    print("\nLow-Pass Envelope Filter:")
    for key, value in LOWPASS_ENVELOPE_FILTER_PARAMS.items():
        print(f"  {key}: {value}")
    print("\nDefault Data Source:")
    print(f"  Filepath: {DEFAULT_DATA_FILEPATH}")
    print(f"  EMG Column: {DEFAULT_EMG_COLUMN_NAME}")
    print(f"  Time Column: {DEFAULT_TIME_COLUMN_NAME}")