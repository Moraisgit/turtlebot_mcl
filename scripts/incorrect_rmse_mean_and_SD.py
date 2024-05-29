########################################################
#         THIS IMPLEMENTATION IS NOT CORRECT
#     USE THE rmse_mean_and_SD_SEM.py SCRIPT INSTEAD
########################################################

from math import sqrt
import os
import csv
import matplotlib.pyplot as plt
from collections import defaultdict

# Function to read the CSV data
def read_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]
    return data

# Function to write the mean RMSE data to a CSV file
def write_mean_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        fieldnames = ['Time (seconds)', 'RMSE (meters)']
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(data)

# Function to write the mean RMSE SD data to a CSV file
def write_SD_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        fieldnames = ['Time (seconds)', 'Standard Deviation']
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(data)

# Function to process each CSV file and collect RMSE values
def process_csv_files(source_dir):
    timestamp_rmse_dict = defaultdict(list)
    all_data = []

    for filename in os.listdir(source_dir):
        if filename.endswith('.csv') and 'rmse_data' in filename:
            source_file = os.path.join(source_dir, filename)
            data = read_csv(source_file)
            all_data.append((filename, data))
            
            for row in data:
                timestamp = int(row['Time (seconds)'])
                rmse_value = float(row['RMSE (meters)'])
                timestamp_rmse_dict[timestamp].append(rmse_value)
    
    return all_data, timestamp_rmse_dict

# Function to calculate mean RMSE for each timestamp
def calculate_mean_rmse(timestamp_rmse_dict):
    mean_rmse_data = []
    for timestamp in sorted(timestamp_rmse_dict.keys()):
        mean_rmse = sum(timestamp_rmse_dict[timestamp]) / len(timestamp_rmse_dict[timestamp])
        mean_rmse_data.append({'Time (seconds)': timestamp, 'RMSE (meters)': mean_rmse})

    return mean_rmse_data

# Function to calculate standard deviation
def calculate_SD_rmse(mean_rmse_data):
    SD_rmse_data = []
    total_rmse = sum(row['RMSE (meters)'] for row in mean_rmse_data)
    num_entries = len(mean_rmse_data)
    expected_mean_rmse = total_rmse / num_entries if num_entries else 0

    # Calculate the standard deviation for each timestamp
    for row in mean_rmse_data:
        rmse_value = row['RMSE (meters)']
        standard_deviation = sqrt((rmse_value - expected_mean_rmse) ** 2)
        SD_rmse_data.append({'Time (seconds)': row['Time (seconds)'], 'Standard Deviation': standard_deviation})

    return SD_rmse_data

# Function to plot the data
def plot_data(all_data, mean_rmse_data, mean_rmse_SD_data):
    # Plot mean rmse and rmse's
    plt.figure()
    
    for filename, data in all_data:
        times = [int(row['Time (seconds)']) for row in data]
        rmse_values = [float(row['RMSE (meters)']) for row in data]
        plt.plot(times, rmse_values, linewidth=0.5)

    mean_times = [row['Time (seconds)'] for row in mean_rmse_data]
    mean_rmse_values = [row['RMSE (meters)'] for row in mean_rmse_data]
    plt.plot(mean_times, mean_rmse_values, label='Mean RMSE', color='black', linewidth=2)

    plt.xlabel('Time (seconds)')
    plt.ylabel('RMSE (meters)')
    plt.title('RMSE over Time for multiple trials')
    plt.legend()
    plt.grid(False)

    # Plot mean rmse SD
    plt.figure()

    SD_mean_times = [row['Time (seconds)'] for row in mean_rmse_SD_data]
    mean_rmse_SD_values = [row['Standard Deviation'] for row in mean_rmse_SD_data]
    plt.plot(SD_mean_times, mean_rmse_SD_values, label='Mean RMSE SD', linewidth=2)

    plt.xlabel('Time (seconds)')
    plt.ylabel('Standard Deviation')
    plt.title('Standard Deviation of the mean RMSE')
    plt.grid(False)
    plt.show()

def main():
    # Define the source directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    #######################################################
    #           CHANGE FOR BUSCA OR MICROSIMULATOR
    #######################################################
    # source_dir = os.path.join(script_dir, '../csv/busca/corrected_time')
    # mean_csv_path = os.path.join(script_dir, '../csv/busca/corrected_time/rmse_mean_data.csv')
    # SD_csv_path = os.path.join(script_dir, '../csv/busca/corrected_time/rmse_mean_SD_data.csv')
    source_dir = os.path.join(script_dir, '../csv/micro_simulator/corrected_time')
    mean_csv_path = os.path.join(script_dir, '../csv/micro_simulator/corrected_time/rmse_mean_data.csv')
    SD_csv_path = os.path.join(script_dir, '../csv/micro_simulator/corrected_time/rmse_mean_SD_data.csv')

    # Process CSV files and collect RMSE values
    all_data, timestamp_rmse_dict = process_csv_files(source_dir)

    # Calculate mean RMSE for each timestamp
    mean_rmse_data = calculate_mean_rmse(timestamp_rmse_dict)

    mean_rmse_SD_data = calculate_SD_rmse(mean_rmse_data)

    # Write the mean RMSE data to the new CSV file
    write_mean_csv(mean_csv_path, mean_rmse_data)
    # Write the mean RMSE SD data to the new CSV file
    write_SD_csv(SD_csv_path, mean_rmse_SD_data)

    # Plot the data
    plot_data(all_data, mean_rmse_data, mean_rmse_SD_data)

if __name__ == '__main__':
    main()
