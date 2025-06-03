# HAR_FinalProject/src/MVIC_Estimator.py

import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt # Optional: for plotting the envelope to verify

# --- Step 1: Import your existing configurations and processing functions ---
# Assuming your config.py and emg_processing.py are in the same directory or accessible
import config as py_config # Rename to avoid clash with C 'config' if running in same mindset
from emg_processing import process_single_emg_channel # Your existing function

def get_sos_coefficients_and_print_c_array(filter_type_name, order, cutoff, fs, filter_btype='lowpass'):
    """
    Calculates SOS coefficients for a Butterworth filter and prints them in C array format.
    For Notch, use get_notch_coefficients_and_print_c_array.
    """
    sos_coeffs = signal.butter(order, cutoff, filter_btype, fs=fs, output='sos')
    print(f"--- {filter_type_name} (Order: {order}, Cutoff: {cutoff}Hz, Fs: {fs}Hz) ---")
    print(f"#define NUM_{filter_type_name.upper().replace('-', '_').replace(' ', '_')}_SOS_SECTIONS {len(sos_coeffs)}")
    print(f"const float {filter_type_name.upper().replace('-', '_').replace(' ', '_')}_SOS_COEFFS[NUM_{filter_type_name.upper().replace('-', '_').replace(' ', '_')}_SOS_SECTIONS][5] = {{")
    for i, section in enumerate(sos_coeffs):
        # Scipy SOS row: [b0, b1, b2, a0, a1, a2]
        # C implementation expects: {b0, b1, b2, a1_c, a2_c}
        # where y[n] = b0x + b1x1 + b2x2 - a1_c*y1 - a2_c*y2
        # If a0 from Scipy is 1 (section[3]), then:
        # a1_c = section[4] (Scipy's a1)
        # a2_c = section[5] (Scipy's a2)
        # This assumes the C formula is y = ... - a1_c*y1 - a2_c*y2
        # If the C formula is y = ... + a1_c*y1 + a2_c*y2, then signs of a1,a2 from scipy need to be flipped.
        # The C code we outlined uses y = ... - a1_C*y1 - a2_C*y2.
        # Scipy's 'a' coefficients in the denominator are for 1 + a1*z^-1 + a2*z^-2 ...
        # So, for y[n] = ... -a1_formula*y[n-1] -a2_formula*y[n-2],
        # a1_formula = scipy_a1, a2_formula = scipy_a2 from the SOS row's 5th and 6th elements.
        
        # Ensure signs are correct:
        # Scipy's a coefficients for sosfilt are such that den = [a0, a1, a2, ...]
        # y[n] = (b0x[n] + ... - a1y[n-1] - a2y[n-2] - ...)/a0
        # Our C struct will store a1 and a2 such that:
        # y[n] = b0x[n] + ... - stored_a1*y[n-1] - stored_a2*y[n-2]
        # So, stored_a1 = scipy_sos_output[4] and stored_a2 = scipy_sos_output[5] (assuming scipy_sos_output[3] is a0=1)

        b0, b1, b2, a0, a1_scipy, a2_scipy = section
        if abs(a0 - 1.0) > 1e-6:
            print(f"ERROR: a0 is not 1 for section {i+1}! Need to normalize or re-check C filter implementation.")
            # If a0 is not 1, all b's and a's need to be divided by a0 for the C implementation
            # b0_c, b1_c, b2_c = b0/a0, b1/a0, b2/a0
            # a1_c, a2_c = a1_scipy/a0, a2_scipy/a0
        
        # These are the coefficients for the formula: y[n] = b0x + b1x1 + b2x2 - a1_C*y1 - a2_C*y2
        # So, a1_C = a1_scipy, a2_C = a2_scipy from the sos array [b0,b1,b2,1,a1_scipy,a2_scipy]
        print(f"    {{ {b0:.8f}f, {b1:.8f}f, {b2:.8f}f, {a1_scipy:.8f}f, {a2_scipy:.8f}f }}, // Section {i+1}")
    print("};")
    print("-" * 40)
    return sos_coeffs

def get_notch_coefficients_and_print_c_array(filter_type_name, notch_freq, Q, fs):
    """
    Calculates Notch filter coefficients (b,a) and prints them in C SOS array format.
    """
    b_notch, a_notch = signal.iirnotch(notch_freq, Q, fs=fs)
    print(f"--- {filter_type_name} (Freq: {notch_freq}Hz, Q: {Q}, Fs: {fs}Hz) ---")
    if abs(a_notch[0] - 1.0) < 1e-6: # Check if a0 from a_notch is 1
        print(f"#define NUM_{filter_type_name.upper().replace('-', '_').replace(' ', '_')}_SOS_SECTIONS 1")
        print(f"const float {filter_type_name.upper().replace('-', '_').replace(' ', '_')}_SOS_COEFFS[NUM_{filter_type_name.upper().replace('-', '_').replace(' ', '_')}_SOS_SECTIONS][5] = {{")
        # b_notch = [b0, b1, b2]
        # a_notch = [a0, a1, a2] where a0=1
        # C implementation expects: {b0, b1, b2, a1_c, a2_c} for y[n] = ... - a1_c*y1 - a2_c*y2
        # So, a1_c = a_notch[1], a2_c = a_notch[2]
        print(f"    {{ {b_notch[0]:.8f}f, {b_notch[1]:.8f}f, {b_notch[2]:.8f}f, {a_notch[1]:.8f}f, {a_notch[2]:.8f}f }} // Section 1")
        print("};")
        # Return in SOS format for consistency if needed, though it's just one section
        sos_notch_manual = np.array([[b_notch[0], b_notch[1], b_notch[2], a_notch[1], a_notch[2]]])
    else:
        print(f"ERROR: Notch filter a0 coefficient is not 1 ({a_notch[0]}). Manual conversion or different C implementation needed.")
        print(f"b_notch: {b_notch}")
        print(f"a_notch: {a_notch}")
        sos_notch_manual = None
    print("-" * 40)
    return sos_notch_manual


if __name__ == "__main__":
    print("Starting MVIC Estimator and Coefficient Generator...\n")

    # --- Part 1: Generate Filter Coefficients ---
    # Use parameters from your py_config (which should mirror what you want in C)
    fs = py_config.SAMPLING_RATE

    # High-Pass Filter
    hpf_order = py_config.HIGHPASS_FILTER_PARAMS["order"]
    hpf_cutoff = py_config.HIGHPASS_FILTER_PARAMS["cutoff"]
    get_sos_coefficients_and_print_c_array("HPF", hpf_order, hpf_cutoff, fs, filter_btype='highpass')

    # Notch Filter
    notch_freq = py_config.NOTCH_FILTER_PARAMS["freq"]
    notch_q = py_config.NOTCH_FILTER_PARAMS["quality_factor"]
    get_notch_coefficients_and_print_c_array("NOTCH", notch_freq, notch_q, fs)
    
    # Low-Pass Filter (Envelope)
    lpf_order = py_config.LOWPASS_ENVELOPE_FILTER_PARAMS["order"]
    lpf_cutoff = py_config.LOWPASS_ENVELOPE_FILTER_PARAMS["cutoff"]
    get_sos_coefficients_and_print_c_array("LPF", lpf_order, lpf_cutoff, fs, filter_btype='lowpass')


    # --- Part 2: Estimate Max Envelope (MVIC Proxy) ---
    print("\n--- Max Envelope Estimation (MVIC Proxy) ---")
    
    # Path to your best recorded data file
    data_filepath = py_config.DEFAULT_DATA_FILEPATH # Use the path from your config
    # If your config has a different name, use that, e.g., "../data/raw/converted_data1.csv"
    
    print(f"Loading data from: {data_filepath}")
    raw_data_df = pd.read_csv(data_filepath)

    if raw_data_df is None:
        print("Could not load data file. Skipping MVIC estimation.")
    else:
        emg_cols_to_analyze = py_config.EMG_COLUMNS # e.g., {"RightLeg": "emg_R1", "LeftLeg": "emg_L1"}
        max_envelopes = {}

        for leg_label, col_name in emg_cols_to_analyze.items():
            if col_name and col_name in raw_data_df.columns:
                print(f"\nProcessing {leg_label} (column: {col_name}) for max envelope...")
                raw_emg_signal = raw_data_df[col_name].values

                # Use your existing EMG processing pipeline
                # process_single_emg_channel should return: raw, hp, notch, rect, envelope
                _, _, _, _, envelope = process_single_emg_channel(
                    raw_emg_signal,
                    fs=py_config.SAMPLING_RATE, # from your python config
                    hp_params=py_config.HIGHPASS_FILTER_PARAMS,
                    notch_params=py_config.NOTCH_FILTER_PARAMS,
                    lp_params=py_config.LOWPASS_ENVELOPE_FILTER_PARAMS
                )
                
                current_max_env = np.max(envelope)
                max_envelopes[leg_label] = current_max_env
                print(f"Max UNNORMALIZED envelope for {leg_label}: {current_max_env:.4f}")

                # Optional: Plot to verify
                # plt.figure(figsize=(10,4))
                # time_axis = np.arange(len(raw_emg_signal)) / fs
                # plt.plot(time_axis, raw_emg_signal, label=f"Raw {leg_label}", alpha=0.5)
                # plt.plot(time_axis, envelope, label=f"Envelope {leg_label}", color='red')
                # plt.title(f"EMG and Envelope for {leg_label} - Max Env: {current_max_env:.4f}")
                # plt.xlabel("Time (s)")
                # plt.ylabel("Amplitude")
                # plt.legend()
                # plt.grid(True)
                # plt.show()
            elif col_name:
                print(f"Column '{col_name}' for {leg_label} not found in CSV. Skipping.")

        print("\n--- Suggested MVIC_VALUES for C code (use these in MyEMG_Controller_Init): ---")
        for leg, max_val in max_envelopes.items():
            # Add a small buffer, or use as is. Consider if the max was an outlier.
            # A common practice is to take e.g., 95th percentile or average of top N peaks.
            # For simplicity, we'll use the raw max, but you might want to be more conservative.
            print(f"myEmgController.{leg_label.lower()}_leg.mvic_value = {max_val:.4f}f;")
        if not max_envelopes:
            print("No EMG columns were processed to estimate MVICs.")

    print("\nScript finished. Copy the printed SOS coefficients and MVIC estimates into your C code.")