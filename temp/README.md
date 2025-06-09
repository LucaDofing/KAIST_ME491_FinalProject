# EMG-Based Controller Simulation

This folder contains scripts to simulate and evaluate the EMG-based controller for the Angel Suit H10 assistive robot.

## Controller Implementation

The controller has been implemented in the `algorithm_ctrl.c` file with the following features:

1. **EMG Signal Processing Pipeline:**
   - Bandpass filtering to remove noise and DC offset (20-450Hz)
   - Rectification to get the absolute value of the signal
   - Low-pass filtering to extract the signal envelope (6Hz cutoff)

2. **Gait Phase Detection:**
   - Using a combination of EMG activation and hip joint angle/velocity to detect swing phase initiation
   - Using hip velocity to detect transition back to stance phase

3. **Control Strategy:**
   - Assistive torque proportional to EMG activation level during swing phase
   - No assistance during stance phase to avoid interfering with natural movement
   - Separate control for left and right legs based on their respective EMG signals

## Simulation Scripts

### 1. Controller Simulation (`controller_simulation.py`)

Simulates the EMG-based controller using recorded data and visualizes the results.

**Usage:**
```bash
python controller_simulation.py [options]
```

**Options:**
- `--recording FILE`: Recording file to use from data/raw folder
- `--list`: List available recordings
- `--start INDEX`: Start index for plotting (default: 0)
- `--window SIZE`: Window size for plotting (default: 2000)
- `--gain VALUE`: EMG torque gain (default: 1.5)
- `--threshold VALUE`: EMG activation threshold (default: 0.04)

**Examples:**
```bash
# List available recordings
python controller_simulation.py --list

# Simulate with default settings (recording 10)
python controller_simulation.py

# Simulate with specific recording
python controller_simulation.py --recording converted_data11.csv

# Simulate with custom parameters
python controller_simulation.py --recording converted_data10.csv --gain 2.0 --threshold 0.03

# Simulate with specific window
python3 controller_simulation.py --start 5000 --window 3000

### 2. Compare Recordings (`compare_recordings.py`)

Compares controller performance across multiple recordings.

**Usage:**
```bash
python compare_recordings.py [options]
```

**Options:**
- `--recordings FILE1 FILE2 ...`: Recording files to compare from data/raw folder
- `--all`: Compare all recordings from 10-19
- `--gain VALUE`: EMG torque gain (default: 1.5)
- `--threshold VALUE`: EMG activation threshold (default: 0.04)

**Examples:**
```bash
# Compare default recordings (10 and 11)
python compare_recordings.py

# Compare specific recordings
python compare_recordings.py --recordings converted_data10.csv converted_data11.csv SUIT_LOGGED_DATA-12.csv

# Compare all recordings from 10-19
python compare_recordings.py --all

# Compare with custom parameters
python compare_recordings.py --all --gain 2.0 --threshold 0.03
```

## Output

The simulation scripts generate the following outputs:

1. **Visualizations:**
   - Hip angles and velocities
   - Raw and filtered EMG signals
   - Controller state (stance/swing)
   - Assistive torque

2. **Performance Metrics:**
   - Mean and max assistive torque
   - Percentage of time in swing phase
   - Mean and max EMG activation

## Notes

- The EMG signals were recorded from the quadriceps muscles, which are primarily active during the swing phase of gait.
- The controller is designed to assist hip flexion during the swing phase, which is when the quadriceps are most active.
- The controller parameters (gain and threshold) can be adjusted to optimize performance for different users or walking speeds. 