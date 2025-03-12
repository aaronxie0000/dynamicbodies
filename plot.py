import json
import matplotlib.pyplot as plt

# Load the JSON data
with open('force_data.json', 'r') as file:
    data = json.load(file)

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

# Function to plot forces in a subplot
def plot_forces(ax, forces, title_prefix):
    num_components = len(forces[0])
    for i in range(num_components):
        force_component = [force[i] for force in forces]
        ax.plot(times, force_component)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Force')
    ax.set_title(f'{title_prefix} ({num_components} components)')

# Plot each force type in its own subplot
for idx, key in enumerate(force_keys):
    row, col = divmod(idx, cols)
    forces = [entry[key] for entry in data]
    plot_forces(axs[row, col], forces, key.replace('_', ' ').title())

# Hide any unused subplots
for idx in range(num_plots, rows * cols):
    row, col = divmod(idx, cols)
    axs[row, col].axis('off')

# Adjust layout
plt.tight_layout()
plt.show()
