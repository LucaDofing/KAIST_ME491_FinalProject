# EMG Controller for Angel Suit H10 Exoskeleton

This folder contains the real-time EMG-based controller implementation for the Angel Suit H10 exoskeleton.

## Overview

The EMG controller provides direct mapping from muscle activity to assistive torque. It processes EMG signals in real-time, normalizes them based on calibration, and applies proportional torque to assist the user's movements.

## Files

- `AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Inc/robot_emg_controller.h` - Header file with function declarations
- `AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/robot_emg_controller.c` - Implementation of the EMG controller
- `AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/algorithm_ctrl.c.patch` - Patch file showing how to modify algorithm_ctrl.c

## Installation

1. Copy the controller files to your project:
   - `robot_emg_controller.h` → `Expansion_Board_H743AII6/AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Inc/`
   - `robot_emg_controller.c` → `Expansion_Board_H743AII6/AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/`

2. Modify `algorithm_ctrl.c` as shown in the patch file:
   - Add `#include "robot_emg_controller.h"` at the top
   - In the USER_DEFINED_CTRL section, call `emg_controller_update();`

## Usage

1. Set the control mode to USER_DEFINED_CTRL (mode 6) using the control interface.
2. The controller will automatically calibrate during the first 10 seconds.
3. After calibration, it will provide assistance proportional to EMG activity.

## Controller Parameters

You can adjust these parameters in `robot_emg_controller.c` to tune the controller:

- `min_activation_threshold` (default: 0.05) - Minimum normalized EMG level to activate assistance
- `direct_torque_gain` (default: 2.0) - Gain to convert normalized EMG to torque

## Debugging

The controller uses the `free_var` variables for debugging:
- `free_var1`: Filtered right EMG
- `free_var2`: Filtered left EMG
- `free_var3`: Right EMG max value (after calibration)
- `free_var4`: Left EMG max value (after calibration)
- `free_var5`: Calibration complete flag (1.0 when calibration is done)

## Troubleshooting

- **No assistance**: Check EMG signal quality and ensure calibration is complete (`free_var5` should be 1.0).
- **Too much/little assistance**: Adjust `direct_torque_gain` to increase/decrease assistance level.
- **Unwanted activation**: Increase `min_activation_threshold` to reduce sensitivity to small EMG signals. 