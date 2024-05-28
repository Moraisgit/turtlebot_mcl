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

def main():
    # Define the source directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    source_dir = os.path.join(script_dir, '../csv/micro_simulator/corrected_time')
    mean_csv_path = os.path.join(script_dir, '../csv/micro_simulator/corrected_time/rmse_mean_data.csv')

    # Dictionary to store RMSE values for each timestamp
    timestamp_rmse_dict = defaultdict(list)

    # List to store data for plotting
    all_data = []

    # Process each CSV file in the source directory
    for filename in os.listdir(source_dir):
        if filename.endswith('.csv'):
            source_file = os.path.join(source_dir, filename)
            data = read_csv(source_file)
            all_data.append((filename, data))
            
            # Collect RMSE values for each timestamp
            for row in data:
                timestamp = int(row['Time (seconds)'])
                rmse_value = float(row['RMSE (meters)'])
                timestamp_rmse_dict[timestamp].append(rmse_value)

    # Calculate the mean RMSE for each timestamp
    mean_rmse_data = []
    for timestamp in sorted(timestamp_rmse_dict.keys()):
        mean_rmse = sum(timestamp_rmse_dict[timestamp]) / len(timestamp_rmse_dict[timestamp])
        mean_rmse_data.append({'Time (seconds)': timestamp, 'RMSE (meters)': mean_rmse})

    # Write the mean RMSE data to the new CSV file
    write_mean_csv(mean_csv_path, mean_rmse_data)

    # Plotting
    plt.figure()

    for filename, data in all_data:
        times = [int(row['Time (seconds)']) for row in data]
        rmse_values = [float(row['RMSE (meters)']) for row in data]
        plt.plot(times, rmse_values, linewidth=0.5)

    # Plot the mean RMSE curve with label
    mean_times = [row['Time (seconds)'] for row in mean_rmse_data]
    mean_rmse_values = [row['RMSE (meters)'] for row in mean_rmse_data]
    plt.plot(mean_times, mean_rmse_values, label='Mean RMSE', color='black', linewidth=2)

    plt.xlabel('Time (seconds)')
    plt.ylabel('RMSE (meters)')
    plt.title('RMSE over Time for Multiple CSV Files')
    plt.legend()
    plt.grid(False)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
