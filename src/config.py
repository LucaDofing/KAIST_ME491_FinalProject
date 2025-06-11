# HAR_FinalProject/src/config.py

# --- General Configuration ---
SAMPLING_RATE = 1000  # !!!!!!!!!!!!!!!!Check sampling rate!!!!!!!!!!!!!!!!!!

# --- EMG Filter Parameters ---
HIGHPASS_FILTER_PARAMS = {
    "cutoff": 20,
    "order": 4
}
NOTCH_FILTER_PARAMS = {
    "freq": 50.0,
    "quality_factor": 30.0
}
LOWPASS_ENVELOPE_FILTER_PARAMS = {
    "cutoff": 6,
    "order": 2
}

# --- Data File and Column Configuration ---
DEFAULT_DATA_FILEPATH = "data/raw/converted_data8.csv"
EMG_COLUMNS = {
    "RightLeg": "emg_R1",
    "LeftLeg": "emg_L1"
}
DEFAULT_TIME_COLUMN_NAME = "loopCnt"

# --- Plotting Configuration ---
PLOTTING_PARAMS = {
    "save_plots": True,
    "steps_plot_figsize": (14, 10),
    "comparison_plot_figsize": (14, 6)
}


if __name__ == '__main__':
    print("\nEMG Columns to Process:")
    for leg, col_name in EMG_COLUMNS.items():
        print(f"  {leg}: {col_name}")