# HAR_FinalProject/src/emg_processing.py

import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import os

# Import configurations from config.py
import config # Assuming config.py is in the same directory (src/)

# --- Function to Load Data ---
def load_data_from_csv(filepath):
    """
    Loads data from a CSV file. Assumes the first row is the header.
    """
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

# --- EMG Processing Functions (using scipy.signal) ---
def apply_highpass_filter(data, cutoff, fs, order):
    """Applies a high-pass Butterworth filter using SOS for stability."""
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff / nyquist_freq
    sos = signal.butter(order, normal_cutoff, btype='high', analog=False, output='sos')
    filtered_data = signal.sosfiltfilt(sos, data)
    return filtered_data

def apply_notch_filter(data, notch_freq, quality_factor, fs):
    """Applies an IIR notch filter."""
    b, a = signal.iirnotch(notch_freq, quality_factor, fs)
    filtered_data = signal.filtfilt(b, a, data)
    return filtered_data

def rectify_signal(data):
    """Rectifies the signal (takes absolute value)."""
    return np.abs(data)

def apply_lowpass_filter_envelope(data, cutoff, fs, order):
    """Applies a low-pass Butterworth filter to get the envelope using SOS."""
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff / nyquist_freq
    sos = signal.butter(order, normal_cutoff, btype='low', analog=False, output='sos')
    filtered_data = signal.sosfiltfilt(sos, data)
    return filtered_data

def process_single_emg_channel(raw_emg_data, fs,
                               hp_params=config.HIGHPASS_FILTER_PARAMS,
                               notch_params=config.NOTCH_FILTER_PARAMS,
                               lp_params=config.LOWPASS_ENVELOPE_FILTER_PARAMS):
    """
    Applies the full EMG processing pipeline to a single EMG channel.
    Returns all intermediate signals and the final envelope.
    """
    # 1. High-pass filter
    emg_hp = apply_highpass_filter(raw_emg_data, hp_params["cutoff"], fs, hp_params["order"])
    
    # 2. Notch filter(s)
    emg_notch = apply_notch_filter(emg_hp, notch_params["freq"], notch_params["quality_factor"], fs)
    
    # Example for harmonic notches if configured:
    # if config.NOTCH_FILTER_HARMONICS.get("apply", False):
    #     for multiple in config.NOTCH_FILTER_HARMONICS.get("multiples", []):
    #         emg_notch = apply_notch_filter(emg_notch, notch_params["freq"] * multiple, notch_params["quality_factor"], fs)
            
    # 3. Rectification
    emg_rectified = rectify_signal(emg_notch)
    
    # 4. Low-pass filter for envelope
    emg_envelope = apply_lowpass_filter_envelope(emg_rectified, lp_params["cutoff"], fs, lp_params["order"])
    
    return raw_emg_data, emg_hp, emg_notch, emg_rectified, emg_envelope

# --- Plotting Functions ---
def plot_emg_processing_steps(time_axis, signals, titles, main_title="EMG Processing Stages", save_path=None, figsize=config.PLOTTING_PARAMS["steps_plot_figsize"]):
    """Plots multiple signals on separate subplots."""
    raw_emg, emg_hp, emg_notch, emg_rectified, emg_envelope = signals
    
    fig, axs = plt.subplots(5, 1, sharex=True, figsize=figsize)
    
    axs[0].plot(time_axis, raw_emg)
    axs[0].set_title(titles[0])
    axs[0].grid(True); axs[0].set_ylabel("Amplitude")

    axs[1].plot(time_axis, emg_hp)
    axs[1].set_title(titles[1])
    axs[1].grid(True); axs[1].set_ylabel("Amplitude")

    axs[2].plot(time_axis, emg_notch)
    axs[2].set_title(titles[2])
    axs[2].grid(True); axs[2].set_ylabel("Amplitude")

    axs[3].plot(time_axis, emg_rectified)
    axs[3].set_title(titles[3])
    axs[3].grid(True); axs[3].set_ylabel("Amplitude")

    axs[4].plot(time_axis, emg_envelope, color='red', linewidth=1.5)
    axs[4].set_title(titles[4])
    axs[4].grid(True); axs[4].set_ylabel("Amplitude (Envelope)")
    
    axs[-1].set_xlabel("Time (s)")
    fig.suptitle(main_title, fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    if config.PLOTTING_PARAMS["save_plots"] and save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")
    
    if config.PLOTTING_PARAMS["save_plots"] and save_path:
        plt.savefig(save_path)
        print(f"[DEBUG] Attempting to save plot to: {save_path}")
        print(f"Plot saved to {save_path}")
    plt.show()

def plot_raw_vs_envelope(time_axis, raw_signal, envelope_signal, channel_name="EMG", save_path=None, figsize=config.PLOTTING_PARAMS["comparison_plot_figsize"]):
    """Plots raw EMG signal against its final processed envelope."""
    plt.figure(figsize=figsize)
    plt.plot(time_axis, raw_signal, label=f"Raw {channel_name}", alpha=0.7)
    plt.plot(time_axis, envelope_signal, label=f"Processed Envelope ({config.LOWPASS_ENVELOPE_FILTER_PARAMS['cutoff']} Hz LP)", color='red', linewidth=2)
    plt.title(f"Raw vs. Processed EMG Envelope for {channel_name}")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    
    if config.PLOTTING_PARAMS["save_plots"] and save_path:
        plt.savefig(save_path)
        print(f"[DEBUG] Attempting to save plot to: {save_path}")
        print(f"Plot saved to {save_path}")
    plt.show()

# --- Main execution block ---
if __name__ == "__main__":
    print("Starting EMG processing script for Final Project (using config.py)...")

    # Use configurations from config.py
    actual_data_filepath = config.DEFAULT_DATA_FILEPATH
    emg_column_to_process = config.DEFAULT_EMG_COLUMN_NAME
    time_column_name = config.DEFAULT_TIME_COLUMN_NAME
    sampling_rate_hz = config.SAMPLING_RATE
    
    # --- Attempt to load actual data ---
    raw_data_df = load_data_from_csv(actual_data_filepath)
    
    emg_signal_to_process = None
    time_vector = None
    data_source_info = ""

    if raw_data_df is not None and emg_column_to_process in raw_data_df.columns:
        emg_signal_to_process = raw_data_df[emg_column_to_process].values
        if time_column_name and time_column_name in raw_data_df.columns:
            if time_column_name == 'loopCnt': # Special handling for loopCnt if needed
                 time_vector = (raw_data_df[time_column_name] - raw_data_df[time_column_name].iloc[0]) / sampling_rate_hz
            else: # Assumes 'time' column is already in seconds
                time_vector = raw_data_df[time_column_name].values
        else:
            time_vector = np.arange(len(emg_signal_to_process)) / sampling_rate_hz
        data_source_info = f"actual data from '{actual_data_filepath}', column '{emg_column_to_process}'"
        print(f"Successfully loaded EMG data for column: '{emg_column_to_process}'")

    elif config.SIMULATION_PARAMS["use_if_file_not_found"]:
        print(f"\n--- Actual data file or column not found. Using SIMULATED EMG data for demonstration. ---")
        sim_cfg = config.SIMULATION_PARAMS
        num_seconds = sim_cfg["duration_seconds"]
        num_samples = int(num_seconds * sampling_rate_hz)
        time_vector = np.linspace(0, num_seconds, num_samples, endpoint=False)
        
        activation = np.zeros(num_samples)
        for i, start_sec in enumerate(sim_cfg["burst_starts_seconds"]):
            start_sample = int(start_sec * sampling_rate_hz)
            end_sample = int(start_sample + sim_cfg["burst_durations_seconds"][i] * sampling_rate_hz)
            if end_sample < num_samples:
                pulse = np.sin(np.linspace(0, np.pi, end_sample - start_sample))**2
                activation[start_sample:end_sample] = pulse * (0.7 + np.random.rand() * 0.6)
        
        emg_clean_component = activation * np.random.normal(0, sim_cfg["noise_levels"]["activation_modulation"], num_samples)
        baseline_noise = np.random.normal(0, sim_cfg["noise_levels"]["baseline_white_noise"], num_samples)
        motion_artifact_drift = sim_cfg["noise_levels"]["motion_artifact_amplitude"] * (np.sin(2 * np.pi * 0.3 * time_vector) + 0.5 * np.sin(2 * np.pi * 0.8 * time_vector))
        powerline_interference = sim_cfg["noise_levels"]["powerline_interference_amplitude"] * np.sin(2 * np.pi * config.NOTCH_FILTER_PARAMS["freq"] * time_vector)
        
        emg_signal_to_process = emg_clean_component + baseline_noise + motion_artifact_drift + powerline_interference
        emg_column_to_process = "Simulated_EMG" # Update for plots if using simulated
        data_source_info = f"simulated EMG data (Fs={sampling_rate_hz}Hz)"
        print("Simulated EMG data generated.")
    else:
        print("Error: No data to process. Please check data filepath and column name in config.py or enable simulation fallback.")
        exit()

    # --- Process the selected EMG signal ---
    if emg_signal_to_process is not None:
        print(f"\nProcessing EMG signal from {data_source_info}...")
        
        raw_sig, hp_sig, notch_sig, rect_sig, env_sig = process_single_emg_channel(
            emg_signal_to_process,
            sampling_rate_hz
            # Filter parameters are now taken from config by default in process_single_emg_channel
        )
        
        # --- Plotting ---
        plot_titles = [
            f"1. Raw {emg_column_to_process}",
            f"2. High-Pass Filtered ({config.HIGHPASS_FILTER_PARAMS['cutoff']} Hz)",
            f"3. Notch Filtered ({config.NOTCH_FILTER_PARAMS['freq']} Hz)",
            "4. Rectified Signal",
            f"5. Smoothed Envelope (LP: {config.LOWPASS_ENVELOPE_FILTER_PARAMS['cutoff']} Hz)"
        ]
        plot_emg_processing_steps(
            time_vector,
            [raw_sig, hp_sig, notch_sig, rect_sig, env_sig],
            plot_titles,
            main_title=f"EMG Processing Stages for {emg_column_to_process} (Fs={sampling_rate_hz}Hz)",
            save_path=f"results/figures/{emg_column_to_process.replace(' ', '_')}_all_steps.png"
        )

        plot_raw_vs_envelope(
            time_vector,
            raw_sig,
            env_sig,
            channel_name=emg_column_to_process,
            save_path=f"results/figures/{emg_column_to_process.replace(' ', '_')}_raw_vs_envelope.png"
        )

        # --- (Optional) Save processed data ---
        # processed_df = pd.DataFrame({
        #     'time_s': time_vector,
        #     'raw_emg': raw_sig,
        #     f'{emg_column_to_process}_envelope': env_sig,
        # })
        # processed_output_filepath = f"../data/processed/{emg_column_to_process.replace(' ', '_')}_processed_data.csv"
        # processed_df.to_csv(processed_output_filepath, index=False)
        # print(f"\nProcessed data (including envelope) saved to: {processed_output_filepath}")

    print("\nEMG processing script finished.")