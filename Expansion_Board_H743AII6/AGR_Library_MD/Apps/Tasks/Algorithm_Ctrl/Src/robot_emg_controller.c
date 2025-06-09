/*
 * robot_emg_controller.c
 * Real-time EMG-based controller implementation for the Angel Suit H10 exoskeleton
 */

// Include the core algorithm header first
#include "algorithm_ctrl.h"

// Then include our controller header
#include "../Inc/robot_emg_controller.h"

/* 
 * EMG Signal Processing Parameters
 */
// Filter order
#define FILTER_ORDER 4

// Butterworth filter coefficients (4th order bandpass 20-450Hz)
// Pre-calculated for 1kHz sampling rate
const float bandpass_b[FILTER_ORDER + 1] = {0.0971f, 0.0f, -0.1942f, 0.0f, 0.0971f};
const float bandpass_a[FILTER_ORDER + 1] = {1.0f, -2.4389f, 2.2948f, -0.9738f, 0.1576f};

// Low-pass filter coefficients for envelope detection (6Hz cutoff)
const float lowpass_b[FILTER_ORDER + 1] = {0.0007f, 0.0029f, 0.0044f, 0.0029f, 0.0007f};
const float lowpass_a[FILTER_ORDER + 1] = {1.0f, -3.0904f, 3.8008f, -2.0939f, 0.4326f};

// Filter state variables
float emg_R1_bp_states[FILTER_ORDER] = {0}; // Bandpass states for right EMG
float emg_L1_bp_states[FILTER_ORDER] = {0}; // Bandpass states for left EMG
float emg_R1_lp_states[FILTER_ORDER] = {0}; // Lowpass states for right EMG
float emg_L1_lp_states[FILTER_ORDER] = {0}; // Lowpass states for left EMG

// Normalization parameters (will be updated during calibration)
float emg_R1_max = 1.0f;
float emg_L1_max = 1.0f;

// Controller parameters
float min_activation_threshold = 0.05f;  // Minimum EMG level to activate assistance
float direct_torque_gain = 2.0f;         // Gain to convert normalized EMG to torque

// Variables for hip velocity calculation
float prev_angle_RH = 0.0f;
float prev_angle_LH = 0.0f;
int first_call = 1;  // Used as boolean

// Calibration variables
#define CALIBRATION_SAMPLES 1000
float calibration_buffer_R[CALIBRATION_SAMPLES];
float calibration_buffer_L[CALIBRATION_SAMPLES];
int calibration_index = 0;
int calibration_complete = 0;

/*
 * Implementation of bandpass filter for EMG signal
 */
float apply_filter(float new_sample, float* states, const float* b, const float* a) {
    // Direct Form II Transposed implementation - efficient for real-time processing
    float result = b[0] * new_sample + states[0];
    
    // Update states - shift values in the state array
    for (int i = 0; i < FILTER_ORDER-1; i++) {
        states[i] = b[i+1] * new_sample - a[i+1] * result + states[i+1];
    }
    states[FILTER_ORDER-1] = b[FILTER_ORDER] * new_sample - a[FILTER_ORDER] * result;
    
    return result;
}

/*
 * Process a single EMG sample through the full processing pipeline
 */
float process_emg_sample(float raw_emg, float* bp_states, float* lp_states) {
    // Step 1: Apply bandpass filter to remove noise and DC offset
    float filtered = apply_filter(raw_emg, bp_states, bandpass_b, bandpass_a);
    
    // Step 2: Rectify the signal (take absolute value)
    float rectified = (filtered < 0) ? -filtered : filtered;
    
    // Step 3: Apply lowpass filter to get the envelope
    float envelope = apply_filter(rectified, lp_states, lowpass_b, lowpass_a);
    
    return envelope;
}

/*
 * Normalize EMG envelope based on calibration
 */
float normalize_emg(float envelope, float max_value) {
    // Normalize by max value
    float normalized = (max_value > 0) ? (envelope / max_value) : envelope;
    
    // Clip to range [0, 1]
    if (normalized < 0) normalized = 0;
    if (normalized > 1) normalized = 1;
    
    return normalized;
}

/*
 * Convert normalized EMG to torque using direct mapping
 */
float compute_torque(float normalized_emg) {
    // Apply minimum threshold to avoid noise-induced movements
    if (normalized_emg < min_activation_threshold) {
        return 0.0f;
    }
    
    // Direct mapping from EMG to torque
    float torque = normalized_emg * direct_torque_gain;
    
    return torque;
}

/*
 * Update calibration buffers and calculate max values when complete
 */
void update_calibration(float emg_R1_filtered, float emg_L1_filtered) {
    if (calibration_complete) {
        return;  // Calibration already done
    }
    
    // Store filtered EMG values in calibration buffers
    calibration_buffer_R[calibration_index] = emg_R1_filtered;
    calibration_buffer_L[calibration_index] = emg_L1_filtered;
    calibration_index++;
    
    // Check if calibration is complete
    if (calibration_index >= CALIBRATION_SAMPLES) {
        // Calculate 95th percentile of the calibration data
        emg_R1_max = percentile(calibration_buffer_R, CALIBRATION_SAMPLES, 0.95f) * 1.2f;
        emg_L1_max = percentile(calibration_buffer_L, CALIBRATION_SAMPLES, 0.95f) * 1.2f;
        
        // Ensure we have non-zero values
        if (emg_R1_max < 0.001f) emg_R1_max = 0.001f;
        if (emg_L1_max < 0.001f) emg_L1_max = 0.001f;
        
        calibration_complete = 1;
        
        // Debug output using free variables
        free_var3 = emg_R1_max;
        free_var4 = emg_L1_max;
        free_var5 = 1.0f;  // Signal calibration complete
    }
}

/*
 * Calculate percentile of an array (simplified implementation)
 */
float percentile(float* data, int n, float p) {
    // Simple bubble sort (inefficient but works for small arrays)
    float temp;
    for (int i = 0; i < n-1; i++) {
        for (int j = 0; j < n-i-1; j++) {
            if (data[j] > data[j+1]) {
                temp = data[j];
                data[j] = data[j+1];
                data[j+1] = temp;
            }
        }
    }
    
    // Calculate the index corresponding to the percentile
    int idx = (int)(p * (n-1));
    return data[idx];
}

/*
 * Main control function to be called from algorithm_ctrl.c
 */
void emg_controller_update(void) {
    // Set the control mode to USER_DEFINED_CTRL for our EMG-based controller
    controlMode = USER_DEFINED_CTRL;
    
    // 1. Process EMG signals
    float emg_R1_filtered = process_emg_sample(EMG_R1_Rawsignal, emg_R1_bp_states, emg_R1_lp_states);
    float emg_L1_filtered = process_emg_sample(EMG_L1_Rawsignal, emg_L1_bp_states, emg_L1_lp_states);
    
    // Store raw filtered values for debugging
    free_var1 = emg_R1_filtered;
    free_var2 = emg_L1_filtered;
    
    // 2. Update calibration if not complete
    update_calibration(emg_R1_filtered, emg_L1_filtered);
    
    // 3. Calculate hip velocity
    float hip_vel_RH = 0.0f;
    float hip_vel_LH = 0.0f;
    
    if (first_call) {
        // Initialize on first call
        prev_angle_RH = robotDataObj_RH.thighTheta_act;
        prev_angle_LH = robotDataObj_LH.thighTheta_act;
        first_call = 0;
    } else {
        // Calculate velocity (deg/s) - multiply by 100 assuming 100Hz control frequency
        hip_vel_RH = (robotDataObj_RH.thighTheta_act - prev_angle_RH) * 100.0f;
        hip_vel_LH = (robotDataObj_LH.thighTheta_act - prev_angle_LH) * 100.0f;
        
        // Update previous values for next iteration
        prev_angle_RH = robotDataObj_RH.thighTheta_act;
        prev_angle_LH = robotDataObj_LH.thighTheta_act;
    }
    
    // 4. Normalize EMG signals
    float emg_R1_normalized = normalize_emg(emg_R1_filtered, emg_R1_max);
    float emg_L1_normalized = normalize_emg(emg_L1_filtered, emg_L1_max);
    
    // 5. Compute torque directly from normalized EMG (real-time approach)
    float right_torque = compute_torque(emg_R1_normalized);
    float left_torque = compute_torque(emg_L1_normalized);
    
    // 6. Apply torque to the robot
    UserDefinedCtrl_RH.control_input = right_torque;
    UserDefinedCtrl_LH.control_input = left_torque;
    
    // Reset other control inputs
    posCtrl_RH.control_input = 0.0f;
    posCtrl_LH.control_input = 0.0f;
    gravCompDataObj_RH.control_input = 0.0f;
    gravCompDataObj_LH.control_input = 0.0f;
    impedanceCtrl_RH.control_input = 0.0f;
    impedanceCtrl_LH.control_input = 0.0f;
    f_vector_input_RH = 0.0f;
    f_vector_input_LH = 0.0f;
    StepCurr_RH.control_input = 0.0f;
    StepCurr_LH.control_input = 0.0f;
} 