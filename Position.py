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
    eps = 1e-7
    acc_roll = np.arctan(ay/(np.sqrt(ax**2+az**2)+eps))
    acc_pitch = np.arctan(ax/(np.sqrt(ay**2+az**2)+eps))

    print(f"roll = {acc_roll} , pitch = {acc_pitch}")

    # Compute gravity components
    gx = np.abs(G * np.sin(acc_pitch))
    gy = np.abs(- G * np.sin(acc_roll))
    gz = np.abs(G * np.cos(acc_pitch) * np.cos(acc_roll))
    print(f"Gravity Components: gx={gx:.4f}, gy={gy:.4f}, gz={gz:.4f}")
    print(f"Acc Before Components: ax={ax:.4f}, ay={ay:.4f}, az={az:.4f}")    

    # Compute linear acceleration by subtracting gravity
    ax_linear = ax - (gx if ax>0 else -gx)
    ay_linear = ay - (gy if ay>0 else -gy)
    az_linear = az - (gz if az>0 else -gz)
    print(f"Acc After Components: ax_linear={ax_linear:.4f}, ay_linear={ay_linear:.4f}, az_linear={az_linear:.4f}")

    return ax_linear, ay_linear, az_linear

def integrate(acc,dt,prev_velocity,prev_position):
    velocity = [0,0,0]
    position = [0,0,0]
    print(f"dt = {dt}")
    for i in range(3):
        velocity[i] = prev_velocity[i] + acc[i] * dt
        position[i] = prev_position[i] + prev_velocity[i] * dt + 0.5 * acc[i] * dt**2
    print(f"velocity = {velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}")
    print(f"position = {position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}")

    return position,velocity