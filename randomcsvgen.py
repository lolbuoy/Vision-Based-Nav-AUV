import csv
import random

# Define the filename for the CSV file
csv_filename = "generated_data3.csv"
j = 0
# Write the data directly to the CSV file
with open(csv_filename, 'w', newline='') as file:
    writer = csv.writer(file, delimiter='\t')  # Using tab delimiter
    writer.writerow(["Image", "Center Distance (m)", "Top Distance (m)", "Left Distance (m)", "Bottom Distance (m)", "Right Distance (m)", "Front Distance (m)"])

    # Generate and write 100 random data points in decreasing order within the range of 0.8 to 4.7
    for _ in range(10000):
        center = round(random.uniform(0.8, 4.7), 2)
        top = round(random.uniform(0.8, center), 2)
        left = round(random.uniform(0.8, center), 2)
        bottom = round(random.uniform(0.8, center), 2)
        right = round(random.uniform(0.8, center), 2)
        front = 6.00 - (j/1666.66666)
        j+=1
        data_row = ["/home/edhitha/Documents/images_1/processed_image_0.jpg", center, top, left, bottom, right,front]
        writer.writerow(data_row)

print(f"CSV file '{csv_filename}' has been generated successfully.")
