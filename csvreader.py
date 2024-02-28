import csv

# Define a function to process the data
def process_image_distances(csv_file):
    left_right_distances = []

    # Open the CSV file and read the data using DictReader
    with open(csv_file, newline='') as file:
        reader = csv.DictReader(file, delimiter='\t')  # Assuming tab delimiter
        for row in reader:
            # Extract data from each row
            left_distance = float(row['Left Distance (m)'])
            right_distance = float(row['Right Distance (m)'])
            front_distance = float(row['Front Distance (m)'])

            # Create a dictionary to store the left and right distances
            distances = {
                'left_distance': left_distance,
                'right_distance': right_distance,
                'front_distance': front_distance
            }

            # Append the distances to the list
            left_right_distances.append(distances)

    return left_right_distances
