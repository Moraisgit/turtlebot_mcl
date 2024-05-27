import csv

# Function to read the CSV data
def read_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]
    return data

# Function to get min and max RMSE before and after a specified time
def get_min_max_rmse(data, split_time):
    rmse_before = [float(row['RMSE (meters)']) for row in data if float(row['Time (seconds)']) < split_time]
    rmse_after = [float(row['RMSE (meters)']) for row in data if float(row['Time (seconds)']) >= split_time]

    min_before = min(rmse_before) if rmse_before else None
    max_before = max(rmse_before) if rmse_before else None
    min_after = min(rmse_after) if rmse_after else None
    max_after = max(rmse_after) if rmse_after else None

    return min_before, max_before, min_after, max_after

# Function to check convergence
def check_convergence(data, threshold=0.05):
    for row in data:
        rmse = float(row['RMSE (meters)'])
        if rmse < threshold:
            return float(row['Time (seconds)']), "Converged"
    return None, "Did not converge"

def main():
    #######################################################
    # Read the CSV data - COMMENT THE ONE YOU ARE NOT USING
    #######################################################
    data = read_csv('../csv/micro_simulator/rmse_data.csv')
    # data = read_csv('../csv/busca/rmse_data.csv')

    ##########################
    # SPECIFY THE SPLIT TIME #
    ##########################
    split_time = 90

    # Get the min and max RMSE values before and after the split time
    min_before, max_before, min_after, max_after = get_min_max_rmse(data, split_time)

    # Print the results
    print(f"Before {split_time} seconds: Min RMSE = {min_before}  Max RMSE = {max_before}")
    print(f"After {split_time} seconds: Min RMSE = {min_after}  Max RMSE = {max_after}")

    # Check for convergence
    time_converged, convergence_message = check_convergence(data, 0.05)
    if time_converged is not None:
        print(f"Time of convergence: {time_converged} seconds")
    print(convergence_message)

if __name__ == '__main__':
    main()
