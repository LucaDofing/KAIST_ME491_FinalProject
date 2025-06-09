/*
 * robot_emg_controller.h
 * Header for real-time EMG-based controller implementation for the Angel Suit H10 exoskeleton
 */

#ifndef ROBOT_EMG_CONTROLLER_H
#define ROBOT_EMG_CONTROLLER_H

// Forward declarations of types from algorithm_ctrl.h
// This avoids including algorithm_ctrl.h directly
#ifdef __cplusplus
extern "C" {
#endif

// External variables from algorithm_ctrl.h that we need
extern float EMG_R1_Rawsignal;
extern float EMG_L1_Rawsignal;
extern float free_var1;
extern float free_var2;
extern float free_var3;
extern float free_var4;
extern float free_var5;
extern float f_vector_input_RH;
extern float f_vector_input_LH;

// Forward declare the types we need
typedef struct _RobotData_t RobotData_t;
typedef struct _UserDefinedCtrl UserDefinedCtrl;
typedef struct _GravComp GravComp;
typedef struct _ImpedanceCtrl ImpedanceCtrl;
typedef struct _StepCurr StepCurr;
typedef enum _ControlMode_t ControlMode;

// External variables that are structs
extern RobotData_t robotDataObj_RH;
extern RobotData_t robotDataObj_LH;
extern UserDefinedCtrl UserDefinedCtrl_RH;
extern UserDefinedCtrl UserDefinedCtrl_LH;
extern GravComp gravCompDataObj_RH;
extern GravComp gravCompDataObj_LH;
extern ImpedanceCtrl impedanceCtrl_RH;
extern ImpedanceCtrl impedanceCtrl_LH;
extern StepCurr StepCurr_RH;
extern StepCurr StepCurr_LH;
extern ControlMode controlMode;

// Define USER_DEFINED_CTRL if not already defined
#ifndef USER_DEFINED_CTRL
#define USER_DEFINED_CTRL 6
#endif

/*
 * Function Prototypes
 */

/**
 * @brief Main EMG controller update function
 * 
 * This function should be called from the algorithm_ctrl.c file in the USER_DEFINED_CTRL section.
 * It processes EMG signals, normalizes them, and applies appropriate torque to assist the user.
 */
void emg_controller_update(void);

/**
 * @brief Process a single EMG sample through the full processing pipeline
 * 
 * @param raw_emg Raw EMG signal value
 * @param bp_states Bandpass filter state array
 * @param lp_states Lowpass filter state array
 * @return Processed EMG envelope value
 */
float process_emg_sample(float raw_emg, float* bp_states, float* lp_states);

/**
 * @brief Normalize EMG envelope based on calibration
 * 
 * @param envelope EMG envelope value
 * @param max_value Maximum EMG value from calibration
 * @return Normalized EMG value in range [0,1]
 */
float normalize_emg(float envelope, float max_value);

/**
 * @brief Convert normalized EMG to torque using direct mapping
 * 
 * @param normalized_emg Normalized EMG value in range [0,1]
 * @return Torque value to apply to the joint
 */
float compute_torque(float normalized_emg);

/**
 * @brief Update calibration buffers and calculate max values when complete
 * 
 * @param emg_R1_filtered Filtered right EMG value
 * @param emg_L1_filtered Filtered left EMG value
 */
void update_calibration(float emg_R1_filtered, float emg_L1_filtered);

/**
 * @brief Calculate percentile of an array
 * 
 * @param data Array of data values
 * @param n Size of the array
 * @param p Percentile to calculate (0.0 to 1.0)
 * @return Value at the specified percentile
 */
float percentile(float* data, int n, float p);

/**
 * @brief Apply filter to a signal sample
 * 
 * @param new_sample New signal sample
 * @param states Filter state array
 * @param b Filter numerator coefficients
 * @param a Filter denominator coefficients
 * @return Filtered sample
 */
float apply_filter(float new_sample, float* states, const float* b, const float* a);

/*
 * Configuration Parameters
 * These can be modified to tune the controller behavior
 */
extern float min_activation_threshold;  // Minimum EMG level to activate assistance
extern float direct_torque_gain;        // Gain to convert normalized EMG to torque

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_EMG_CONTROLLER_H */ 