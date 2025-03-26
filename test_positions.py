import serial
import time
import sys
import csv
import numpy as np

from Position import *

readings = np.zeros((10, 4))

# parameters for calculating position
G = 9.81
prev_timestamp = 0
prev_velocity = [0,0,0]
prev_position = [0,0,0]
positions = [[0,0,0]]
A = np.array([[0.801086, -0.115891, -0.018981],  # 'A^-1' matrix from Magneto
              [-0.115891, 0.871255, 0.010562],
              [-0.018981, 0.010562, 0.831012]])
# 'Combined bias (b)' vector from Magneto
b = np.array([[0.380675, 0.254083, 0.167651]]).T

# connect to serial port
arduino = serial.Serial('COM7', 9600) 
time.sleep(2)  

filename = f"position_data.csv"
with open(filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(["timestamp","ax","ay","az"])

    read_count = 0
    num_of_read=10
    while read_count < 10000:
        # Read data from the serial port
        if arduino.in_waiting > 0:
            sensor_x = 0
            sensor_y = 0
            sensor_z = 0
            i = 0
            while(i < 10):
                if(arduino.in_waiting <= 0):
                    continue
                data = arduino.readline().decode('utf-8').strip()
                values = data.split(',')
                if len(values) >= 8:
                    sensor_x+=float(values[1]) * G * 0.1
                    sensor_y+=float(values[2]) * G * 0.1
                    sensor_z+=float(values[3]) * G * 0.1
                    i+=1

            print("-----------------------------------------------")
            timestamp = values[0]
            
            dt = (int(timestamp) - int(prev_timestamp)) / 1000
            prev_timestamp = timestamp

            ax_linear, ay_linear, az_linear = remove_gravity_effect(sensor_x, sensor_y, sensor_z, G)
            acc = np.array([[ax_linear, ay_linear, az_linear]]).T
            acc = A @ (acc - b)
            print(acc)
            prev_position, prev_velocity = integrate(acc.T[0], dt, prev_velocity, prev_position)
            print("-------------------------------------")

            # # Write data to file
            row = [timestamp,sensor_x,sensor_y,sensor_z]
            np.set_printoptions(suppress=True, precision=6) 
            csv_writer.writerow(row)
            read_count += 1

            print(f"Received from Arduino: {row}")

    print(f"Data saved to {filename} ({read_count} reads)")