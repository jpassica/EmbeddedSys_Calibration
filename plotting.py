import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read CSV file
filename = "raw_position_data.csv"
df = pd.read_csv(filename)

# Extract columns
timestamp = df["timestamp"]
filtered_x = df["Filtered_X"]
filtered_y = df["Filtered_Y"]
filtered_z = df["Filtered_Z"]

# Create a 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectory
ax.plot(filtered_x, filtered_y, filtered_z, marker="o", linestyle="-", label="Position Trajectory")

# Labels and title
ax.set_xlabel("Filtered X")
ax.set_ylabel("Filtered Y")
ax.set_zlabel("Filtered Z")
ax.set_title("3D Filtered Position Trajectory")

# Show legend
ax.legend()

# Show the plot
plt.show()
