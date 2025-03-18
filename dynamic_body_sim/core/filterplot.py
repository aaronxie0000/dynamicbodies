#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import argparse
import numpy as np


def load_data(filename):
    """Load force data from a JSON file."""
    with open(filename, 'r') as file:
        data = json.load(file)
    return data


def average_data(data, window_size, threshold):
    """
    Average data across a specified window size.
    
    Args:
        data: List of data points (dictionaries)
        window_size: Number of data points to average across, in both plot and data process
        threshold: Threshold for detecting significant changes, used in data printing
        
    Returns:
        List of averaged data points
    """
    if window_size <= 1:
        return data
    
    averaged_data = []
    data_length = len(data)
    
    all_keys = data[0].keys()
    force_keys = [key for key in all_keys if key != 'time']
    
    print(f"Threshold (y axis units): {threshold}")
    print("\nAveraged Data Summary:")
    print("======================")
    print(f"Showing periods with significant changes (>{threshold} in any component)")
    
    prev_avg_forces = None 
    significant_changes_count = 0
    
    for i in range(0, data_length, window_size):
        end_idx = min(i + window_size, data_length)
        chunk = data[i:end_idx]
        
        avg_point = {}
        
        avg_point['time'] = sum(entry['time'] for entry in chunk) / len(chunk)
        
        current_avg_forces = {}
        has_significant_change = False
        
        # Average each force component
        for key in force_keys:
            all_forces = [entry[key] for entry in chunk]
            all_forces_array = np.array(all_forces)

            avg_force = np.mean(all_forces_array, axis=0).tolist()            
            avg_point[key] = avg_force
            
            current_avg_forces[key] = avg_force
            
            if prev_avg_forces is not None:
                prev_force = prev_avg_forces.get(key, [0] * len(avg_force))
                for j, (curr, prev) in enumerate(zip(avg_force, prev_force)):
                    if abs(curr - prev) > threshold:
                        has_significant_change = True
                        break
        
        if has_significant_change or prev_avg_forces is None: 
            start_time = chunk[0]['time']
            end_time = chunk[-1]['time']
            print(f"\nTime period: {start_time:.4f}s - {end_time:.4f}s (Average: {avg_point['time']:.4f}s)")
            
            for key in force_keys:
                avg_force = current_avg_forces[key]
                
                if prev_avg_forces is not None and key in prev_avg_forces:
                    prev_force = prev_avg_forces[key]
                    changes = [curr - prev for curr, prev in zip(avg_force, prev_force)]
                    formatted_changes = []
                    for change in changes:
                        if abs(change) > threshold:
                            formatted_changes.append(f"*{change:.6f}*")
                        else:
                            formatted_changes.append(f"{change:.6f}")
                    
                    print(f"  {key.replace('_', ' ').title()}: {[f'{val:.6f}' for val in avg_force]}")
                else:
                    print(f"  {key.replace('_', ' ').title()}: {[f'{val:.6f}' for val in avg_force]}")
            
            significant_changes_count += 1
        
        averaged_data.append(avg_point)
        prev_avg_forces = current_avg_forces
    
    print(f"\nTotal periods with significant changes: {significant_changes_count}")
    print("======================")
    return averaged_data


def plot_forces(ax, data, times, title_prefix):
    """Plot force components in a subplot."""
    num_components = len(data[0])
    for i in range(num_components):
        force_component = [force[i] for force in data]
        ax.plot(times, force_component)
    ax.set_xlabel('Time (s)')
    ax.set_title(f'{title_prefix} ({num_components} components)')


def create_force_plots(data, window_size, threshold):
    """Create a grid of subplots for different force types."""
    times = [entry['time'] for entry in data]
    all_keys = data[0].keys()
    force_keys = [key for key in all_keys if key != 'time']

    num_plots = len(force_keys)
    cols = 3
    rows = (num_plots + cols - 1) // cols
    fig, axs = plt.subplots(rows, cols, figsize=(10, 2 * rows))


    fig.suptitle(f'Forces averaged over {window_size:.3f}s windows')

    if num_plots == 1:
        axs = [axs]

    for idx, key in enumerate(force_keys):
        if num_plots > 1:
            row, col = divmod(idx, cols)
            ax = axs[row, col] if rows > 1 else axs[col]
        else:
            ax = axs[0]
        data_points = [entry[key] for entry in data]
        plot_forces(ax, data_points, times, key.replace('_', ' ').title())

    # Hide any unused subplots
    if num_plots > 1:
        for idx in range(num_plots, rows * cols):
            row, col = divmod(idx, cols)
            if rows > 1:
                axs[row, col].axis('off')
            else:
                axs[col].axis('off')

    plt.tight_layout()
    return fig


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Plot force data from simulation')
    parser.add_argument('--body', choices=['pend', 'humanoid'], default='pend')
    parser.add_argument('--window', type=float, default=0.01, help='Time window (in seconds) for averaging data points')
    parser.add_argument('--threshold', type=float, default=10.0, help='Threshold for significant change detection')
    args = parser.parse_args()
    
    data_file = 'dynamic_body_sim/actuated_pendulum/data/pendulum_data.json' if args.body == 'pend' else 'dynamic_body_sim/humanoid_robot/data/kbot_data.json'
    print(f"Using data from {data_file}")
    
    data = load_data(data_file)
    
    if args.window > 0:
        # Calculate time step from the first two data points
        time_step = data[1]['time'] - data[0]['time']
        # Convert time window to number of data points
        window_size = int(args.window / time_step)
        window_size = max(1, window_size)  # Ensure at least 1 data point
        
        print(f"Time step: {time_step:.4f}s")
        print(f"Averaging data across {args.window}s ({window_size} data points)")
        data = average_data(data, window_size, args.threshold)
    
    fig = create_force_plots(data, args.window, args.threshold)
    plt.show()


if __name__ == "__main__":
    main()
