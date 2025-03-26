import numpy as np
import serial
import csv
import time
from Position import *
from kalman_filter import *

# Initialize sensor parameters
readings = np.zeros((10, 4))
G = 9.81
prev_timestamp = 0
prev_velocity = [0,0,0]
prev_position = [0,0,0]
positions = [[0,0,0]]
dt = 0.01
A = np.array([[0.801086, -0.115891, -0.018981],  
              [-0.115891, 0.871255, 0.010562],
              [-0.018981, 0.010562, 0.831012]])
b = np.array([[0.380675, 0.254083, 0.167651]]).T

# Connect to serial port
arduino = serial.Serial('COM7', 9600) 
time.sleep(2)  

filename = f"position_data.csv"
with open(filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(["timestamp","Filtered_X","Filtered_Y","Filtered_Z"])
    
    read_count = 0
    num_of_read = 10
    while read_count < 10000:
        if arduino.in_waiting > 0:
            sensor_x =sensor_y = sensor_z = timestamp = 0
            
            while True:
                if arduino.in_waiting <= 0:
                    continue
                data = arduino.readline().decode('utf-8').strip()
                values = data.split(',')
                if len(values) >= 8:
                    sensor_x = float(values[1]) * G
                    sensor_y = float(values[2]) * G
                    sensor_z = float(values[3]) * G 
                    timestamp = values[0]
                    print(f"timestamp = {timestamp}")
                    break
            
            # dt = (int(timestamp) - int(prev_timestamp)) / 1000
            prev_timestamp = timestamp
            print("-----------------------------------------------------")
            ax_linear, ay_linear, az_linear = remove_gravity_effect(sensor_x, sensor_y, sensor_z, G)
            acc = np.array([[ax_linear, ay_linear, az_linear]]).T
            acc = A @ (acc - b)
            
            # Apply Kalman filter
            KF_x, KF_P = kalman_filter(ax_linear, KF_x, KF_P)
            KF_y, KF_P = kalman_filter(ay_linear, KF_x, KF_P)
            KF_z, KF_P = kalman_filter(az_linear, KF_x, KF_P)
            
            prev_position, prev_velocity = integrate([KF_x[0][0],KF_y[0][0],KF_z[0][0]], dt, prev_velocity, prev_position)
            
            # Write filtered data to CSV
            row = [timestamp, prev_position[0], prev_position[1], prev_position[2]]
            np.set_printoptions(suppress=True, precision=6) 
            csv_writer.writerow(row)
            read_count += 1
            
            print(f"Filtered Data: {row}")
            print("-----------------------------------------------------")

    
    print(f"Data saved to {filename} ({read_count} reads)")
