# src/robot_control.py (Conceptual)
import numpy as np
import config # Assuming gains, thresholds are in config.py

def calculate_assistive_torques(norm_emg_env_R, norm_emg_env_L):
    torque_R = 0.0
    torque_L = 0.0

    # --- Right Leg Torque ---
    if norm_emg_env_R > config.CONTROLLER_PARAMS["EMG_THRESHOLD_R"]:
        active_emg_R = norm_emg_env_R - config.CONTROLLER_PARAMS["EMG_THRESHOLD_R"]
        # Optional: re-normalize active_emg_R from 0 to (1-threshold_R) to 0-1 range
        # if (1.0 - config.CONTROLLER_PARAMS["EMG_THRESHOLD_R"]) > 1e-3:
        #     active_emg_R = active_emg_R / (1.0 - config.CONTROLLER_PARAMS["EMG_THRESHOLD_R"])
        torque_R = config.CONTROLLER_PARAMS["GAIN_R"] * active_emg_R
    
    # --- Left Leg Torque (Symmetrical design) ---
    if norm_emg_env_L > config.CONTROLLER_PARAMS["EMG_THRESHOLD_L"]:
        active_emg_L = norm_emg_env_L - config.CONTROLLER_PARAMS["EMG_THRESHOLD_L"]
        # Optional: re-normalize active_emg_L
        # if (1.0 - config.CONTROLLER_PARAMS["EMG_THRESHOLD_L"]) > 1e-3:
        #     active_emg_L = active_emg_L / (1.0 - config.CONTROLLER_PARAMS["EMG_THRESHOLD_L"])
        torque_L = config.CONTROLLER_PARAMS["GAIN_L"] * active_emg_L
    
    # Apply saturation (safety limits)
    # Assuming GAIN_R and GAIN_L are positive and assist flexion (positive torque)
    # Adjust if assisting extension or if gains can be negative
    torque_R = np.clip(torque_R, 0, config.CONTROLLER_PARAMS["MAX_TORQUE_R"]) 
    torque_L = np.clip(torque_L, 0, config.CONTROLLER_PARAMS["MAX_TORQUE_L"])

    return torque_R, torque_L