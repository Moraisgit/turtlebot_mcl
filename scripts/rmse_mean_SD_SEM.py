import math
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

# Function to write the mean RMSE SD data to a CSV file
def write_SEM_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        fieldnames = ['Time (seconds)', 'Standard Error Mean']
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

# Function to process CSV file and collect RMSE values
def process_csv_file(file_path):
    timestamp_rmse_dict = defaultdict(list)

    filename = os.path.basename(file_path)
    if filename.endswith('.csv') and 'rmse_data' in filename:
        data = read_csv(file_path)
        
        for row in data:
            timestamp = int(row['Time (seconds)'])
            rmse_value = float(row['RMSE (meters)'])
            timestamp_rmse_dict[timestamp].append(rmse_value)
    
    return timestamp_rmse_dict

# Function to add labels to CSV data
def convert_to_labels(only_this_timestamp_rmse_dict):
    only_this_csv_data = []
    for timestamp in sorted(only_this_timestamp_rmse_dict.keys()):
        rmse_value = only_this_timestamp_rmse_dict[timestamp]
        only_this_csv_data.append({'Time (seconds)': timestamp, 'RMSE (meters)': rmse_value})

    return only_this_csv_data

# Function to calculate mean RMSE for each timestamp
def calculate_mean_rmse(timestamp_rmse_dict):
    mean_rmse_data = []
    for timestamp in sorted(timestamp_rmse_dict.keys()):
        mean_rmse = sum(timestamp_rmse_dict[timestamp]) / len(timestamp_rmse_dict[timestamp])
        mean_rmse_data.append({'Time (seconds)': timestamp, 'RMSE (meters)': mean_rmse})

    return mean_rmse_data

# Function to calculate standard deviation
def calculate_SD_SEM_rmse(timestamp_rmse_dict, mean_rmse_data):
    SD_rmse_data = []
    SEM_rmse_data = []
    sum_squared_diff = 0
    max_SD = 0
    max_SEM = 0
    max_time_SD = 0
    max_time_SEM = 0
    aux1 = 0
    aux2 = 0
    #######################
    #   NUMBER OF SAMPLES
    #######################
    N = 10

    timestamp_mean_rmse_dict = {}
    for row in mean_rmse_data:
        timestamp = row['Time (seconds)']
        rmse_value = row['RMSE (meters)']
        if timestamp in timestamp_mean_rmse_dict:
            timestamp_mean_rmse_dict[timestamp].append(rmse_value)
        else:
            timestamp_mean_rmse_dict[timestamp] = [rmse_value]
    
    for timestamp in sorted(timestamp_rmse_dict.keys()):
        rmse_value_list = timestamp_rmse_dict[timestamp]
        mean_rmse_list = timestamp_mean_rmse_dict[timestamp]

        if isinstance(mean_rmse_list, list):
            mean_rmse_value = mean_rmse_list[0]

        for rmse_value in rmse_value_list:
            sum_squared_diff += (rmse_value - mean_rmse_value)**2

        aux = sum_squared_diff / (N-1)
        standard_deviation = math.sqrt(aux)
        standard_error_mean = standard_deviation / math.sqrt(N)

        # Keep track of the maximum values of SD and SEM
        aux1 = standard_deviation
        aux2 = standard_error_mean

        if aux1 > max_SD:
            max_SD = aux1
            max_time_SD = timestamp
        if aux2 > max_SEM:
            max_SEM = aux2
            max_time_SEM = timestamp

        sum_squared_diff = 0

        SD_rmse_data.append({'Time (seconds)': timestamp, 'Standard Deviation': standard_deviation})
        SEM_rmse_data.append({'Time (seconds)': timestamp, 'Standard Error Mean': standard_error_mean})

    return SD_rmse_data, SEM_rmse_data, max_SD, max_SEM, max_time_SD, max_time_SEM

# Function to plot the data
def plot_data(all_data, mean_rmse_data, mean_rmse_SD_data, mean_rmse_SEM_data):
    ############################
    # Plot mean rmse and rmse's
    ############################
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

    ####################
    # Plot mean rmse SD
    ####################
    plt.figure()

    SD_mean_times = [row['Time (seconds)'] for row in mean_rmse_SD_data]
    mean_rmse_SD_values = [row['Standard Deviation'] for row in mean_rmse_SD_data]
    plt.plot(SD_mean_times, mean_rmse_SD_values, label='Mean RMSE SD', linewidth=2)

    plt.xlabel('Time (seconds)')
    plt.ylabel('Standard Deviation')
    plt.title('Standard Deviation of RMSE')
    plt.grid(False)

    #####################
    # Plot mean rmse SEM
    #####################
    plt.figure()

    SEM_mean_times = [row['Time (seconds)'] for row in mean_rmse_SEM_data]
    mean_rmse_SEM_values = [row['Standard Error Mean'] for row in mean_rmse_SEM_data]
    plt.plot(SEM_mean_times, mean_rmse_SEM_values, label='Mean RMSE SEM', linewidth=2)

    plt.xlabel('Time (seconds)')
    plt.ylabel('Standard Error Mean')
    plt.title('Standard Error Mean of RMSE')
    plt.grid(False)

    #############################
    # Plot the mean rmse +/- SEM
    #############################
    plt.figure()

    #############################################################
    # THIS MAKES A SHADOW WITH FILLING
    # Calculate the upper and lower bounds
    upper_bounds = [mean_rmse_values[i] + mean_rmse_SEM_values[i] for i in range(len(mean_times))]
    lower_bounds = [mean_rmse_values[i] - mean_rmse_SEM_values[i] for i in range(len(mean_times))]

    # Plot the mean_rmse_values
    plt.plot(mean_times, mean_rmse_values, '-', label='Mean RMSE')

    # Fill between the upper and lower bounds
    plt.fill_between(mean_times, lower_bounds, upper_bounds, color='gray', alpha=0.7, label='SEM Error Bounds')
    #############################################################

    ############################################################
    # # THIS MAKES BOXES OF ERROR
    # # Determine indices for error bars at specified intervals
    # indices = range(0, len(mean_times), 3)

    # # Plot the mean_rmse_values
    # plt.plot(mean_times, mean_rmse_values, '-', label='Mean RMSE')

    # # Plot the error bars at specified intervals
    # plt.errorbar([mean_times[i] for i in indices], 
    #              [mean_rmse_values[i] for i in indices], 
    #              yerr=[mean_rmse_SEM_values[i] for i in indices], 
    #              fmt='o', ecolor='black', capsize=5, markersize=2, label='Error Bars')
    ############################################################

    # Add labels and title
    plt.xlabel('Time [seconds]')
    plt.ylabel('RMSE mean [meters]')
    plt.title('Micro-simultator data\nMean RMSE with SEM error bounds over Time')
    # plt.title('Real data\nMean RMSE with SEM error bounds over Time')
    plt.grid(False)
    plt.legend()

    # # Additional text annotations
    # plt.text(10, 1.6, r'$\sigma_{hit} = 60$', fontsize=12)
    # plt.text(10, 1.4, r'Maximum $SEM = 0.4 \, m$', fontsize=12)
    # plt.text(10, 1.2, r'at $time = 2 \, s$', fontsize=12)

    plt.show()

def main():
    # Define the source directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    #######################################################
    #              DEFINE THE NUMBER OF SIGMA
    #######################################################
    sigma = 60
    
    #######################################################
    #           CHANGE FOR BUSCA OR MICROSIMULATOR
    #######################################################
    # source_dir = os.path.join(script_dir, f'../csv/busca/corrected_time/sigma_{sigma}')
    # mean_csv_path = os.path.join(script_dir, f'../csv/busca/corrected_time/sigma_{sigma}/rmse_mean_data.csv')
    # SD_csv_path = os.path.join(script_dir, f'../csv/busca/corrected_time/sigma_{sigma}/rmse_mean_SD_data.csv')
    # SEM_csv_path = os.path.join(script_dir, f'../csv/busca/corrected_time/sigma_{sigma}/rmse_mean_SEM_data.csv')
    source_dir = os.path.join(script_dir, f'../csv/micro_simulator/corrected_time/sigma_{sigma}')
    mean_csv_path = os.path.join(script_dir, f'../csv/micro_simulator/corrected_time/sigma_{sigma}/rmse_mean_data.csv')
    SD_csv_path = os.path.join(script_dir, f'../csv/micro_simulator/corrected_time/sigma_{sigma}/rmse_mean_SD_data.csv')
    SEM_csv_path = os.path.join(script_dir, f'../csv/micro_simulator/corrected_time/sigma_{sigma}/rmse_mean_SEM_data.csv')

    # Process CSV files and collect RMSE values
    all_data, timestamp_rmse_dict = process_csv_files(source_dir)

    # only_this_csv_data = convert_to_labels(only_this_timestamp_rmse_dict)

    # Calculate mean RMSE for each timestamp
    mean_rmse_data = calculate_mean_rmse(timestamp_rmse_dict)

    mean_rmse_SD_data, mean_rmse_SEM_data, max_SD, max_SEM, max_time_SD, max_time_SEM = calculate_SD_SEM_rmse(timestamp_rmse_dict, mean_rmse_data)

    # Print maximum values of SD and SEM
    print("Maximum Standard Deviation (SD):", max_SD, "at Time:", max_time_SD)
    print("Maximum Standard Mean Error (SEM):", max_SEM, "at Time:", max_time_SEM)

    # Write the mean RMSE data to the new CSV file
    write_mean_csv(mean_csv_path, mean_rmse_data)
    # Write the mean RMSE SD data to the new CSV file
    write_SD_csv(SD_csv_path, mean_rmse_SD_data)
    # Write the mean RMSE SEM data to the new CSV file
    write_SEM_csv(SEM_csv_path, mean_rmse_SEM_data)

    # Plot the data
    plot_data(all_data, mean_rmse_data, mean_rmse_SD_data, mean_rmse_SEM_data)

if __name__ == '__main__':
    main()
