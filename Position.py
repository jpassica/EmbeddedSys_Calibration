import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt


G = 9.81
ALPHA = 0.98
prev_theta = 0
prev_phi = 0
prev_time = 0
prev_velocity = [0,0,0]
prev_position = [0,0,0]
positions = [[0,0,0]]

# Remove Gravity Effect
def remove_gravity_effect(ax,ay,az,G):
    eps = 1e-7
    acc_roll = np.arctan(ay/(np.sqrt(ax**2+az**2)+eps))
    acc_pitch = np.arctan(ax/(np.sqrt(ay**2+az**2)+eps))

    # gyro_pitch = prev_theta + gx * dt
    # gyro_roll = prev_phi + gy * dt

    # theta = ALPHA * gyro_pitch + (1 - ALPHA) * acc_pitch
    # phi = ALPHA * gyro_roll + (1 - ALPHA) * acc_roll
    
    # gx_gravity = G*np.sin(theta)
    # gy_gravity= -G*np.sin(phi)
    # gz_gravity = G*np.cos(theta)*np.cos(phi)

    gx_gravity = G*np.sin(acc_pitch)
    gy_gravity= -G*np.sin(acc_roll)
    gz_gravity = G*np.cos(acc_pitch)*np.cos(acc_roll)

    ax_linear = ax - gx_gravity
    ay_linear = ay - gy_gravity
    az_linear = az - gz_gravity

    return ax_linear,ay_linear,az_linear

def integrate(acc,dt,prev_velocity,prev_position):
    velocity = [0,0,0]
    position = [0,0,0]
    for i in range(3):
        velocity[i] = prev_velocity[i] + acc[i] * dt
        position[i] = prev_position[i] + prev_velocity[i] * dt + 0.5 * acc[i] * dt**2
    return position,velocity

data = pd.read_csv('data.csv')
dt = 0.01
for i in range(0,len(data)):
    row = data.iloc[i]
    ax = row['aX'] * G
    ay = row['aY'] * G
    az = row['aZ'] * G
    ax_linear,ay_linear,az_linear = remove_gravity_effect(ax,ay,az,G)
    prev_position , prev_velocity = integrate([ax_linear,ay_linear,az_linear],dt,prev_velocity,prev_position)
    positions.append(prev_position)

# Convert to NumPy array for easier manipulation
positions = np.array(positions)

filename = "positions.csv"
with open(filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x','y','z'])  # Write header
    writer.writerows(positions)  # Write all data rows
