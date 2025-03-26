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
    # Compute roll (ϕ) and pitch (θ) from accelerometer data
    acc_roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))
    acc_pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2))

    print(f"roll = {acc_roll} , pitch = {acc_pitch}")

    # Compute gravity components
    gx = G * np.sin(acc_pitch)
    gy = -G * np.sin(acc_roll)
    gz = G * np.cos(acc_pitch) * np.cos(acc_roll)
    print(f"Gravity Components: gx={gx:.4f}, gy={gy:.4f}, gz={gz:.4f}")

    # Compute linear acceleration by subtracting gravity
    ax_linear = ax - (gx if ax>0 else -gx)
    ay_linear = ay - (gy if ay>0 else -gy)
    az_linear = az - (gz if az>0 else -gz)

    threshold = 0.5  
    ax_linear = 0 if abs(ax_linear) < threshold else ax_linear
    ay_linear = 0 if abs(ay_linear) < threshold else ay_linear
    az_linear = 0 if abs(az_linear) < threshold else az_linear
    return ax_linear, ay_linear, az_linear

# def remove_gravity_effect(ax,ay,az,G):
#     eps = 1e-7
#     acc_roll = np.arctan(ay/(np.sqrt(ax**2+az**2)+eps))
#     acc_pitch = np.arctan(ax/(np.sqrt(ay**2+az**2)+eps))

#     # Compute roll (ϕ) and pitch (θ) from accelerometer data
#     # acc_roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))  # Roll (ϕ)
#     # acc_pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2))  # Pitch (θ)
#     print(f"roll = {acc_roll} , pitch = {acc_pitch}")
#     # Compute gravity components
#     gx_gravity = np.abs(G * np.sin(acc_pitch))
#     gy_gravity = np.abs(-G * np.sin(acc_roll))
#     gz_gravity = np.abs(G * np.cos(acc_pitch) * np.cos(acc_roll))

#     print(f"Gravity Components: gx={gx_gravity:.4f}, gy={gy_gravity:.4f}, gz={gz_gravity:.4f}")

#     # Compute linear acceleration by subtracting gravity
#     ax_linear = ax - (gx_gravity if ax>0 else -gx_gravity)
#     ay_linear = ay - (gy_gravity if ay>0 else -gy_gravity)
#     az_linear = az - (gz_gravity if az>0 else -gz_gravity)
#     # ax_linear = ax + gx_gravity
#     # ay_linear = ay + gy_gravity
#     # az_linear = az + gz_gravity

#     # print(f"Linear Acceleration: ax={ax_linear:.4f}, ay={ay_linear:.4f}, az={az_linear:.4f}")
#     return ax_linear, ay_linear, az_linear

def integrate(acc,dt,prev_velocity,prev_position):
    velocity = [0,0,0]
    position = [0,0,0]
    print(f"dt = {dt}")
    for i in range(3):
        velocity[i] = prev_velocity[i] + acc[i] * dt
        position[i] = prev_position[i] + prev_velocity[i] * dt + 0.5 * acc[i] * dt**2
        # print(f"prev_position {i} = {prev_position[i]} , prev_velocity {i} = {prev_velocity[i]} , acc {i} = {acc[i]}")
    print(f"velocity = {velocity}")
    print(f"position = {position}")
    return position,velocity

# data = pd.read_csv('data.csv')
# dt = 0.01
# for i in range(0,len(data)):
#     row = data.iloc[i]
#     ax = row['aX'] * G
#     ay = row['aY'] * G
#     az = row['aZ'] * G
#     ax_linear,ay_linear,az_linear = remove_gravity_effect(ax,ay,az,G)
#     prev_position , prev_velocity = integrate([ax_linear,ay_linear,az_linear],dt,prev_velocity,prev_position)
#     positions.append(prev_position)

# # Convert to NumPy array for easier manipulation
# positions = np.array(positions)

# filename = "axxa.csv"
# with open(filename, 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(['x','y','z'])  # Write header
#     writer.writerows(positions)  # Write all data rows
