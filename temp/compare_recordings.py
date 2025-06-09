import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from controller_simulation import EMGController, list_available_recordings

def analyze_recording(file_path, controller):
    """Analyze a single recording and return performance metrics"""
    # Load data
    df = pd.read_csv(file_path)
    
    # Simulate controller
    results = controller.simulate_controller(df)
    
    # Calculate metrics
    metrics = {
        'mean_right_torque': np.mean(results['right_torque']),
        'max_right_torque': np.max(results['right_torque']),
        'mean_left_torque': np.mean(results['left_torque']),
        'max_left_torque': np.max(results['left_torque']),
        'right_swing_percent': np.mean(results['right_state']) * 100,
        'left_swing_percent': np.mean(results['left_state']) * 100,
        'mean_emg_r1': np.mean(results['emg_R1_filtered']),
        'mean_emg_l1': np.mean(results['emg_L1_filtered']),
        'max_emg_r1': np.max(results['emg_R1_filtered']),
        'max_emg_l1': np.max(results['emg_L1_filtered'])
    }
    
    return metrics, results

def plot_comparison(recordings, metrics):
    """Plot comparison of metrics across recordings"""
    # Extract recording IDs for x-axis labels
    recording_ids = [os.path.splitext(os.path.basename(r))[0].replace('converted_data', '').replace('SUIT_LOGGED_DATA-', '') for r in recordings]
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot mean torques
    axes[0, 0].bar(np.arange(len(recordings)) - 0.2, [m['mean_right_torque'] for m in metrics], width=0.4, label='Right')
    axes[0, 0].bar(np.arange(len(recordings)) + 0.2, [m['mean_left_torque'] for m in metrics], width=0.4, label='Left')
    axes[0, 0].set_title('Mean Assistive Torque')
    axes[0, 0].set_ylabel('Torque (Nm)')
    axes[0, 0].set_xticks(np.arange(len(recordings)))
    axes[0, 0].set_xticklabels(recording_ids)
    axes[0, 0].legend()
    
    # Plot max torques
    axes[0, 1].bar(np.arange(len(recordings)) - 0.2, [m['max_right_torque'] for m in metrics], width=0.4, label='Right')
    axes[0, 1].bar(np.arange(len(recordings)) + 0.2, [m['max_left_torque'] for m in metrics], width=0.4, label='Left')
    axes[0, 1].set_title('Max Assistive Torque')
    axes[0, 1].set_ylabel('Torque (Nm)')
    axes[0, 1].set_xticks(np.arange(len(recordings)))
    axes[0, 1].set_xticklabels(recording_ids)
    axes[0, 1].legend()
    
    # Plot swing percentages
    axes[1, 0].bar(np.arange(len(recordings)) - 0.2, [m['right_swing_percent'] for m in metrics], width=0.4, label='Right')
    axes[1, 0].bar(np.arange(len(recordings)) + 0.2, [m['left_swing_percent'] for m in metrics], width=0.4, label='Left')
    axes[1, 0].set_title('Swing Phase Percentage')
    axes[1, 0].set_ylabel('Percentage (%)')
    axes[1, 0].set_xticks(np.arange(len(recordings)))
    axes[1, 0].set_xticklabels(recording_ids)
    axes[1, 0].legend()
    
    # Plot mean EMG levels
    axes[1, 1].bar(np.arange(len(recordings)) - 0.2, [m['mean_emg_r1'] for m in metrics], width=0.4, label='Right')
    axes[1, 1].bar(np.arange(len(recordings)) + 0.2, [m['mean_emg_l1'] for m in metrics], width=0.4, label='Left')
    axes[1, 1].set_title('Mean EMG Activation')
    axes[1, 1].set_ylabel('EMG Amplitude')
    axes[1, 1].set_xticks(np.arange(len(recordings)))
    axes[1, 1].set_xticklabels(recording_ids)
    axes[1, 1].legend()
    
    plt.tight_layout()
    return fig

def main():
    parser = argparse.ArgumentParser(description='Compare controller performance across recordings')
    parser.add_argument('--recordings', nargs='+', type=str, help='Recording files to compare (from data/raw folder)')
    parser.add_argument('--gain', type=float, default=1.5, help='EMG torque gain')
    parser.add_argument('--threshold', type=float, default=0.04, help='EMG activation threshold')
    parser.add_argument('--all', action='store_true', help='Compare all recordings from 10-19')
    args = parser.parse_args()
    
    # Path to data directory
    data_dir = '../data/raw'
    
    # Create controller
    controller = EMGController()
    controller.emg_torque_gain = args.gain
    controller.emg_activation_threshold = args.threshold
    
    # Determine which recordings to analyze
    if args.all:
        recordings = []
        for i in range(10, 20):
            converted_file = os.path.join(data_dir, f'converted_data{i}.csv')
            suit_file = os.path.join(data_dir, f'SUIT_LOGGED_DATA-{i}.csv')
            
            if os.path.exists(converted_file):
                recordings.append(converted_file)
            elif os.path.exists(suit_file):
                recordings.append(suit_file)
    elif args.recordings:
        recordings = [os.path.join(data_dir, r) if not r.startswith(data_dir) else r for r in args.recordings]
    else:
        # Default to recordings 10-12
        recordings = [
            os.path.join(data_dir, 'converted_data10.csv'),
            os.path.join(data_dir, 'converted_data11.csv')
        ]
    
    print(f"Analyzing {len(recordings)} recordings: {[os.path.basename(r) for r in recordings]}")
    
    # Analyze each recording
    all_metrics = []
    all_results = []
    
    for recording in recordings:
        if not os.path.exists(recording):
            print(f"Warning: File {recording} not found, skipping")
            continue
            
        print(f"Processing {os.path.basename(recording)}...")
        metrics, results = analyze_recording(recording, controller)
        all_metrics.append(metrics)
        all_results.append(results)
        
        print(f"  Mean Right Torque: {metrics['mean_right_torque']:.4f} Nm")
        print(f"  Mean Left Torque: {metrics['mean_left_torque']:.4f} Nm")
        print(f"  Right Swing: {metrics['right_swing_percent']:.1f}%")
        print(f"  Left Swing: {metrics['left_swing_percent']:.1f}%")
    
    # Plot comparison
    if len(all_metrics) > 1:
        fig = plot_comparison(recordings, all_metrics)
        plt.savefig('controller_comparison.png')
        print("Comparison plot saved as controller_comparison.png")
        plt.show()

if __name__ == "__main__":
    main() 