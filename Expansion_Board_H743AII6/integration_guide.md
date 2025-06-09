# EMG Controller Integration Guide

This guide explains how to integrate the real-time EMG-based controller with the Angel Suit H10 exoskeleton.

## Overview

The EMG controller provides direct mapping from muscle activity to assistive torque. It processes EMG signals in real-time, normalizes them based on calibration, and applies proportional torque to assist the user's movements.

## Files

- `robot_emg_controller.h` - Header file with function declarations
- `robot_emg_controller.c` - Implementation of the EMG controller

## Integration Steps

### 1. Add Files to Project

Copy the controller files to your project:

```
robot_emg_controller.h → Expansion_Board_H743AII6/AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Inc/
robot_emg_controller.c → Expansion_Board_H743AII6/AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/
```

### 2. Include Header in algorithm_ctrl.c

Add the following include at the top of `algorithm_ctrl.c`:

```c
#include "robot_emg_controller.h"
```

### 3. Modify the USER_DEFINED_CTRL Section

Find the USER_DEFINED_CTRL section in the `StateEnable_Run()` function in `algorithm_ctrl.c` and modify it as follows:

```c
else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control
    // Call the EMG controller update function
    emg_controller_update();
}
```

### 4. Set Control Mode to USER_DEFINED_CTRL

To activate the EMG controller, set the control mode to USER_DEFINED_CTRL (mode 6) using the control interface.

## Controller Parameters

You can adjust these parameters in `robot_emg_controller.c` to tune the controller:

- `min_activation_threshold` (default: 0.05) - Minimum normalized EMG level to activate assistance
- `direct_torque_gain` (default: 2.0) - Gain to convert normalized EMG to torque

## How It Works

1. **Signal Processing**: EMG signals are filtered using bandpass (20-450Hz) and lowpass (6Hz) filters to extract the muscle activation envelope.

2. **Calibration**: The controller automatically calibrates during the first 1000 samples (approximately 10 seconds at 100Hz) to determine the maximum EMG levels for normalization.

3. **Real-time Control**: After calibration, the controller:
   - Processes incoming EMG signals
   - Normalizes them based on calibration
   - Applies torque proportional to the normalized EMG signal

4. **Debugging**: The controller uses the `free_var` variables for debugging:
   - `free_var1`: Filtered right EMG
   - `free_var2`: Filtered left EMG
   - `free_var3`: Right EMG max value (after calibration)
   - `free_var4`: Left EMG max value (after calibration)
   - `free_var5`: Calibration complete flag (1.0 when calibration is done)

## Testing

1. Compile and upload the code to the exoskeleton.
2. Connect EMG sensors to the right and left quadriceps muscles.
3. Power on the system and set the control mode to USER_DEFINED_CTRL (mode 6).
4. Stand still for about 10 seconds to allow calibration.
5. Begin walking - the exoskeleton should provide assistance proportional to muscle activity.

## Troubleshooting

- **No assistance**: Check EMG signal quality and ensure calibration is complete (`free_var5` should be 1.0).
- **Too much/little assistance**: Adjust `direct_torque_gain` to increase/decrease assistance level.
- **Unwanted activation**: Increase `min_activation_threshold` to reduce sensitivity to small EMG signals.

## Advanced Customization

For more advanced control, you can modify:

- Filter coefficients in `robot_emg_controller.c` to adjust signal processing
- The `compute_torque()` function to implement non-linear mapping between EMG and torque
- Calibration parameters to change how normalization is performed