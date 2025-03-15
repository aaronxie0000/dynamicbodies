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


def average_data(data, window_size, threshold=10.0):
    """
    Average data across a specified window size.
    
    Args:
        data: List of data points (dictionaries)
        window_size: Number of data points to average across
        threshold: Threshold for detecting significant changes
        
    Returns:
        List of averaged data points
    """
    if window_size <= 1:
        return data
    
    averaged_data = []
    data_length = len(data)
    
    # Get all keys except 'time'
    all_keys = data[0].keys()
    force_keys = [key for key in all_keys if key != 'time']
    
    print("\nAveraged Data Summary:")
    print("======================")
    print(f"Showing periods with significant changes (>{threshold} in any component)")
    
    # Process data in chunks of window_size
    prev_avg_forces = None  # Store previous period's forces for comparison
    significant_changes_count = 0
    
    for i in range(0, data_length, window_size):
        end_idx = min(i + window_size, data_length)
        chunk = data[i:end_idx]
        
        # Create a new data point with averaged values
        avg_point = {}
        
        # Average the time values
        avg_point['time'] = sum(entry['time'] for entry in chunk) / len(chunk)
        
        # Collect all averaged forces for this period
        current_avg_forces = {}
        has_significant_change = False
        
        # Average each force component
        for key in force_keys:
            # Each force is a list of components
            # First, collect all force components for this key in the current chunk
            all_forces = [entry[key] for entry in chunk]
            
            # Convert to numpy array for easier averaging
            all_forces_array = np.array(all_forces)
            
            # Average across the chunk
            avg_force = np.mean(all_forces_array, axis=0).tolist()
            
            # Store in the averaged data point
            avg_point[key] = avg_force
            
            # Store for comparison
            current_avg_forces[key] = avg_force
            
            # Check for significant change if we have previous data
            if prev_avg_forces is not None:
                prev_force = prev_avg_forces.get(key, [0] * len(avg_force))
                # Check each component for significant change
                for j, (curr, prev) in enumerate(zip(avg_force, prev_force)):
                    if abs(curr - prev) > threshold:
                        has_significant_change = True
                        break
        
        # Print time period information if there's a significant change
        if has_significant_change or prev_avg_forces is None:  # Always print the first period
            start_time = chunk[0]['time']
            end_time = chunk[-1]['time']
            print(f"\nTime period: {start_time:.4f}s - {end_time:.4f}s (Average: {avg_point['time']:.4f}s)")
            
            # Print the averaged force components
            for key in force_keys:
                avg_force = current_avg_forces[key]
                
                # If we have previous data, calculate and show changes
                if prev_avg_forces is not None and key in prev_avg_forces:
                    prev_force = prev_avg_forces[key]
                    changes = [curr - prev for curr, prev in zip(avg_force, prev_force)]
                    # Highlight significant changes
                    formatted_changes = []
                    for change in changes:
                        if abs(change) > threshold:
                            formatted_changes.append(f"*{change:.6f}*")  # Highlight significant changes
                        else:
                            formatted_changes.append(f"{change:.6f}")
                    
                    print(f"  {key.replace('_', ' ').title()}: {[f'{val:.6f}' for val in avg_force]}")
                    # print(f"    Change: {formatted_changes}")
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
    ax.set_ylabel('Force')
    ax.set_title(f'{title_prefix} ({num_components} components)')


def create_force_plots(data):
    """Create a grid of subplots for different force types."""
    # Extract time
    times = [entry['time'] for entry in data]

    # Dynamically identify force-related keys
    all_keys = data[0].keys()
    force_keys = [key for key in all_keys if key != 'time']

    # Create a grid of subplots based on the number of force keys
    num_plots = len(force_keys)
    cols = 3
    rows = (num_plots + cols - 1) // cols  # Calculate rows needed for a 3-column layout
    fig, axs = plt.subplots(rows, cols, figsize=(10, 2 * rows))

    # Check if axs is a 1D array
    if num_plots == 1:
        axs = [axs]

    # Plot each force type in its own subplot
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

    # Adjust layout
    plt.tight_layout()
    return fig


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Plot force data from simulation')
    parser.add_argument('--body', choices=['pend', 'humanoid'], default='pend')
    parser.add_argument('--window', type=int, default=50, help='Window size for averaging data points, 100 is 1sec')
    parser.add_argument('--threshold', type=float, default=50.0, help='Threshold for significant change detection')
    args = parser.parse_args()
    
    # Determine which JSON file to use based on the argument
    data_file = 'dynamic_body_sim/actuated_pendulum/data/pendulum_data.json' if args.body == 'pend' else 'dynamic_body_sim/humanoid_robot/data/kbot_data.json'
    print(f"Using data from {data_file}")
    
    # Load the data
    data = load_data(data_file)
    
    # Average the data
    if args.window > 1:
        print(f"Averaging data across {args.window} data points")
        data = average_data(data, args.window, args.threshold)
    
    # Create and show the plots
    fig = create_force_plots(data)
    plt.show()


if __name__ == "__main__":
    main()
