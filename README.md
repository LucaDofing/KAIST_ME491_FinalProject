# EMG-Based Direct Torque Controller for Exoskeleton Robot

This repository contains the implementation of a real-time EMG-based controller for the Angel Suit H10 assistive exoskeleton robot. The controller directly maps filtered EMG signals to motor torque, providing immediate assistance proportional to muscle activity.

## Overview

The EMG-based controller processes raw EMG signals from the quadriceps muscles and maps them directly to hip flexion torque. This approach provides several advantages:

1. **Immediate Assistance**: No delay waiting for gait phase detection
2. **Proportional Support**: Assistance level directly corresponds to muscle activity
3. **Intuitive Control**: More natural feeling for the user
4. **Adaptability**: Automatically adjusts to different walking speeds and styles

## Implementation

The controller has been implemented in the following files:

- `robot_emg_controller.h` - Header file with controller definitions
- `robot_emg_controller.c` - Implementation of the EMG controller
- `algorithm_ctrl.c` - Main algorithm control file with EMG controller integration
- `algorithm_ctrl.h` - Header file for algorithm control

### Key Features

1. **EMG Signal Processing Pipeline**:
   - Bandpass filtering (20-450Hz) to remove noise and DC offset
   - Rectification to get the absolute value of the signal
   - Low-pass filtering (6Hz cutoff) to extract the signal envelope

2. **Automatic Calibration**:
   - Calibration during the first 1000 samples (~10 seconds)
   - Separate calibration for left and right sides
   - Determines minimum and maximum values for normalization

3. **Direct Torque Mapping**:
   - Normalized EMG signals are directly mapped to torque
   - Adjustable gain parameter to control assistance level
   - Threshold parameter to eliminate noise

## Usage

To use the EMG controller:

1. Make sure the EMG sensors are properly attached to the quadriceps muscles
2. Power on the exoskeleton robot
3. The controller will automatically start calibration for the first 10 seconds
4. During calibration, perform a few normal steps to establish baseline EMG levels
5. After calibration, the controller will provide assistance proportional to muscle activity

### Controller Parameters

The following parameters can be adjusted in `robot_emg_controller.h`:

- `EMG_TORQUE_GAIN`: Gain factor for converting EMG to torque (default: 1.5)
- `EMG_ACTIVATION_THRESHOLD`: Threshold for EMG activation (default: 0.04)
- `EMG_BP_CUTOFF_LOW`: Low cutoff frequency for bandpass filter (default: 20Hz)
- `EMG_BP_CUTOFF_HIGH`: High cutoff frequency for bandpass filter (default: 450Hz)
- `EMG_LP_CUTOFF`: Cutoff frequency for low-pass filter (default: 6Hz)
- `CALIBRATION_SAMPLES`: Number of samples for calibration (default: 1000)

## Simulation

Before implementing on the robot, the controller was tested in simulation using recorded EMG and motion data. The simulation code is available in the `temp` directory:

- `controller_simulation.py`: Simulates the EMG-based controller using recorded data
- `compare_recordings.py`: Compares controller performance across multiple recordings

## Integration with Robot

The controller is integrated with the robot's control system through the USER_DEFINED_CTRL mode. When this mode is selected, the EMG controller takes over and provides assistance based on muscle activity.

## Performance

The controller has been tested with various walking speeds and styles, showing the following benefits:

- Reduced muscle activation during walking
- More natural feeling assistance compared to phase-based controllers
- Immediate response to user intentions
- Smooth torque profiles without abrupt transitions

## Future Work

Potential improvements for future versions:

1. Integration with other sensor modalities (IMU, FSR)
2. Adaptive gain based on walking speed and terrain
3. Machine learning approaches for personalized assistance
4. Extension to knee and ankle joints

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Data recordings:
### 26.05: (Xenia)
- motion: 10 steps, pause, 10 steps
  - data 1-4: one or both quads 

### 28.05: (Luca)
- motion: 10 steps, pause, 10 steps
  + data 5: calf on right side
  + data 6: calf on the right side (quad sensor fell out) 
  + data 7: calf right & quad left
  + data 8: quad on both sides 

- motion: 12 big steps, pause, 10 fast steps
  + data 9: both quads
