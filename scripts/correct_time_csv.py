import os
import csv

# Function to read the CSV data
def read_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]
    return data

# Function to write the corrected CSV data
def write_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        fieldnames = data[0].keys()
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(data)

# Function to correct the timestamps, remove duplicates, and discard data after a given timestamp
def correct_timestamps(data, max_timestamp):
    corrected_data = []
    seen_timestamps = set()

    # Iterate through data in reverse order to keep the last occurrence of each timestamp
    for row in reversed(data):
        timestamp = int(float(row['Time (seconds)']))
        if timestamp > max_timestamp:
            continue  # Skip rows with timestamp greater than max_timestamp
        row['Time (seconds)'] = timestamp

        if timestamp not in seen_timestamps:
            seen_timestamps.add(timestamp)
            corrected_data.insert(0, row)  # Insert at the beginning to maintain order

    return corrected_data

def main():
    # Define the source and target directories
    script_dir = os.path.dirname(os.path.abspath(__file__))

    #######################################################
    #             DEFINE THE NUMBER OF SIGMA 
    #######################################################
    sigma = 60

    #######################################################
    #           CHANGE FOR BUSCA OR MICROSIMULATOR
    #######################################################
    # source_dir = os.path.join(script_dir, f'../csv/busca/sigma_{sigma}')
    # target_dir = os.path.join(script_dir, f'../csv/busca/corrected_time/sigma_{sigma}')
    source_dir = os.path.join(script_dir, f'../csv/micro_simulator/sigma_{sigma}')
    target_dir = os.path.join(script_dir, f'../csv/micro_simulator/corrected_time/sigma_{sigma}')

    # Ensure the target directory exists
    os.makedirs(target_dir, exist_ok=True)

    #######################################################
    #          TIME FROM WHICH WE CUT THE CSV
    #######################################################
    max_timestamp = 79

    # Process each CSV file in the source directory
    for filename in os.listdir(source_dir):
        if filename.endswith('.csv'):
            source_file = os.path.join(source_dir, filename)
            target_file = os.path.join(target_dir, filename)

            # Read, correct, and write the CSV data
            data = read_csv(source_file)
            corrected_data = correct_timestamps(data, max_timestamp)
            write_csv(target_file, corrected_data)
            print(f"Processed {filename} and saved to {target_file}")

if __name__ == '__main__':
    main()
