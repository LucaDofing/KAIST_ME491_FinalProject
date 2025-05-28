# HAR_FinalProject/src/emg_processing.py

import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import os

import config

# --- Function to Load Data ---
def load_data_from_csv(filepath):
    try:
        df = pd.read_csv(filepath)
        print(f"Data loaded successfully from {filepath}")
        print(f"Columns found: {df.columns.tolist()}")
        return df
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return None

# --- EMG Processing Functions ---
def apply_highpass_filter(data, cutoff, fs, order):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff / nyquist_freq
    sos = signal.butter(order, normal_cutoff, btype='high', analog=False, output='sos')
    filtered_data = signal.sosfiltfilt(sos, data)
    return filtered_data

def apply_notch_filter(data, notch_freq, quality_factor, fs):
    b, a = signal.iirnotch(notch_freq, quality_factor, fs)
    filtered_data = signal.filtfilt(b, a, data)
    return filtered_data

def rectify_signal(data):
    return np.abs(data)

def apply_lowpass_filter_envelope(data, cutoff, fs, order):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff / nyquist_freq
    sos = signal.butter(order, normal_cutoff, btype='low', analog=False, output='sos')
    filtered_data = signal.sosfiltfilt(sos, data)
    return filtered_data

def process_single_emg_channel(raw_emg_data, fs,
                               hp_params=config.HIGHPASS_FILTER_PARAMS,
                               notch_params=config.NOTCH_FILTER_PARAMS,
                               lp_params=config.LOWPASS_ENVELOPE_FILTER_PARAMS):
    emg_hp = apply_highpass_filter(raw_emg_data, hp_params["cutoff"], fs, hp_params["order"])
    emg_notch = apply_notch_filter(emg_hp, notch_params["freq"], notch_params["quality_factor"], fs)
    emg_rectified = rectify_signal(emg_notch)
    emg_envelope = apply_lowpass_filter_envelope(emg_rectified, lp_params["cutoff"], fs, lp_params["order"])
    return raw_emg_data, emg_hp, emg_notch, emg_rectified, emg_envelope

# --- Plotting Functions ---
def plot_multi_leg_emg_processing_steps(time_axis, processed_signals_map, main_title_prefix="EMG Processing", save_path_prefix=None, figsize=config.PLOTTING_PARAMS["steps_plot_figsize"]):
    num_steps = 5
    leg_names = list(processed_signals_map.keys())
    if not leg_names:
        print("No signals to plot in plot_multi_leg_emg_processing_steps.")
        return

    leg_colors = {'RightLeg': 'blue', 'LeftLeg': 'green'} 
    fig, axs = plt.subplots(num_steps, 1, sharex=True, figsize=figsize)
    step_titles = [
        f"1. Raw EMG",
        f"2. High-Pass Filtered ({config.HIGHPASS_FILTER_PARAMS['cutoff']} Hz)",
        f"3. Notch Filtered ({config.NOTCH_FILTER_PARAMS['freq']} Hz)",
        "4. Rectified Signal",
        f"5. Smoothed Envelope (LP: {config.LOWPASS_ENVELOPE_FILTER_PARAMS['cutoff']} Hz)"
    ]

    for i in range(num_steps):
        axs[i].set_title(step_titles[i]); axs[i].grid(True); axs[i].set_ylabel("Amplitude")
        for leg_name in leg_names:
            signals = processed_signals_map.get(leg_name)
            if signals:
                color = leg_colors.get(leg_name, 'black')
                linewidth = 1.5 if i == 4 else 1.0
                axs[i].plot(time_axis, signals[i], label=f"{leg_name}", color=color, linewidth=linewidth)
        if i == 0: axs[i].legend(loc='upper right')

    axs[-1].set_xlabel("Time (s)")
    fig.suptitle(f"{main_title_prefix} Stages for {', '.join(leg_names)} (Fs={config.SAMPLING_RATE}Hz)", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    if config.PLOTTING_PARAMS["save_plots"] and save_path_prefix:
        final_save_path = f"{save_path_prefix}_all_steps_multi.png"
        save_dir = os.path.dirname(final_save_path)
        if save_dir and not os.path.exists(save_dir): os.makedirs(save_dir, exist_ok=True)
        plt.savefig(final_save_path); print(f"Multi-leg plot saved to {final_save_path}")
    plt.show()

def plot_multi_leg_raw_vs_envelope(time_axis, raw_signals_map, envelope_signals_map, main_title="EMG Envelopes", save_path_prefix=None, figsize=config.PLOTTING_PARAMS["comparison_plot_figsize"]):
    leg_names = list(raw_signals_map.keys())
    if not leg_names:
        print("No signals to plot in plot_multi_leg_raw_vs_envelope.")
        return

    leg_colors = {'RightLeg': 'blue', 'LeftLeg': 'green'}
    plt.figure(figsize=figsize)
    for leg_name in leg_names:
        raw_signal = raw_signals_map.get(leg_name)
        envelope_signal = envelope_signals_map.get(leg_name)
        color = leg_colors.get(leg_name, 'black')
        if raw_signal is not None: plt.plot(time_axis, raw_signal, label=f"Raw {leg_name}", alpha=0.4, color=color, linestyle=':')
        if envelope_signal is not None: plt.plot(time_axis, envelope_signal, label=f"Envelope {leg_name}", color=color, linewidth=2)
    
    plt.title(f"{main_title} for {', '.join(leg_names)} (LP: {config.LOWPASS_ENVELOPE_FILTER_PARAMS['cutoff']} Hz)")
    plt.xlabel("Time (s)"); plt.ylabel("Amplitude"); plt.legend(); plt.grid(True); plt.tight_layout()
    
    if config.PLOTTING_PARAMS["save_plots"] and save_path_prefix:
        final_save_path = f"{save_path_prefix}_raw_vs_envelope_multi.png"
        save_dir = os.path.dirname(final_save_path)
        if save_dir and not os.path.exists(save_dir): os.makedirs(save_dir, exist_ok=True)
        plt.savefig(final_save_path); print(f"Multi-leg comparison plot saved to {final_save_path}")
    plt.show()

# --- Main ---
if __name__ == "__main__":
    print(f"Current Working Directory: {os.getcwd()}")
    print("Starting EMG processing script for Final Project (using config.py)...")

    actual_data_filepath = config.DEFAULT_DATA_FILEPATH
    emg_columns_map = config.EMG_COLUMNS
    time_column_name = config.DEFAULT_TIME_COLUMN_NAME
    sampling_rate_hz = config.SAMPLING_RATE
    
    raw_data_df = load_data_from_csv(actual_data_filepath)
    
    processed_emg_data = {}
    raw_emg_signals_for_plot = {}
    envelope_emg_signals_for_plot = {}
    time_vector = None
    data_loaded_successfully = False

    if raw_data_df is None:
        print(f"Failed to load data from {actual_data_filepath}. Exiting.")
        exit()

    if time_column_name and time_column_name in raw_data_df.columns:
        if time_column_name == 'loopCnt':
             time_vector = (raw_data_df[time_column_name] - raw_data_df[time_column_name].iloc[0]) / sampling_rate_hz
        else:
            time_vector = raw_data_df[time_column_name].values
    elif not raw_data_df.empty:
        first_valid_emg_col = next((col for leg, col in emg_columns_map.items() if col and col in raw_data_df.columns), None)
        if first_valid_emg_col:
            time_vector = np.arange(len(raw_data_df[first_valid_emg_col])) / sampling_rate_hz
        else:
            print("Error: Cannot determine data length to create time vector. No valid EMG columns found and no time column specified or found.")
            exit()
    else:
        print("Error: Data file loaded but it is empty. Exiting.")
        exit()

    for leg_label, emg_col_name in emg_columns_map.items():
        if emg_col_name and emg_col_name in raw_data_df.columns:
            print(f"\nProcessing EMG for {leg_label} (column: {emg_col_name})...")
            raw_signal = raw_data_df[emg_col_name].values
            
            if len(raw_signal) != len(time_vector):
                print(f"Warning: Length mismatch for {emg_col_name} ({len(raw_signal)}) and time_vector ({len(time_vector)}). This might lead to plotting issues.")
            
            processed_signals = process_single_emg_channel(raw_signal, sampling_rate_hz)
            processed_emg_data[leg_label] = processed_signals
            raw_emg_signals_for_plot[leg_label] = processed_signals[0] # Raw
            envelope_emg_signals_for_plot[leg_label] = processed_signals[4] # Envelope
            data_loaded_successfully = True
        elif emg_col_name: 
            print(f"Warning: EMG column '{emg_col_name}' for {leg_label} not found in the data. Skipping this channel.")

    # --- Plotting data ---
    if not data_loaded_successfully:
        print("No EMG data was successfully processed. Cannot generate plots.")
    elif time_vector is None:
        print("Error: Time vector is not available. Cannot generate plots.")
    else:
        plot_multi_leg_emg_processing_steps(
            time_vector,
            processed_emg_data,
            main_title_prefix="EMG Processing",
            save_path_prefix=f"results/figures/multi_leg"
        )
        plot_multi_leg_raw_vs_envelope(
            time_vector,
            raw_emg_signals_for_plot,
            envelope_emg_signals_for_plot,
            main_title="EMG Raw vs. Envelope Comparison",
            save_path_prefix=f"results/figures/multi_leg"
        )

    print("\nEMG processing script finished.")