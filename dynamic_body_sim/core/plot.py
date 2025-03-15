#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import argparse


def load_data(filename):
    """Load force data from a JSON file."""
    with open(filename, 'r') as file:
        data = json.load(file)
    return data


def plot_forces(ax, data, times, title_prefix):
    """Plot force components in a subplot."""
    num_components = len(data[0])
    for i in range(num_components):
        force_component = [force[i] for force in data]
        ax.plot(times, force_component)
    ax.set_xlabel('Time (s)')
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
    args = parser.parse_args()
    
    # Determine which JSON file to use based on the argument
    data_file = 'dynamic_body_sim/actuated_pendulum/data/pendulum_data.json' if args.body == 'pend' else 'dynamic_body_sim/humanoid_robot/data/kbot_data.json'
    print(f"Using data from {data_file}")
    
    # Load the data
    data = load_data(data_file)
    
    # Create and show the plots
    fig = create_force_plots(data)
    plt.show()


if __name__ == "__main__":
    main()
