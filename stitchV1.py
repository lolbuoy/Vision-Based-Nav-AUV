'''BASIC STITCHING OF IMAGES FROM 2 FOLDERS'''

import os
import cv2

def stitch_images(input_folder1, input_folder2, output_folder):
    # Get list of files in each input folder
    files1 = sorted(os.listdir(input_folder1))
    files2 = sorted(os.listdir(input_folder2))

    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Loop through files in both folders
    for file1, file2 in zip(files1, files2):
        # Extract timestamps from filenames
        timestamp1 = file1.split('_')
        timestamp11 = int(timestamp1[2].split('.')[0])
        timestamp2 = file2.split('_')
        timestamp22 = int(timestamp2[2].split('.')[0])

        # Check if timestamps are similar
        if abs(timestamp11 - timestamp22) <= 1:  # Adjust threshold as needed
            # Read images
            image1 = cv2.imread(os.path.join(input_folder1, file1))
            image2 = cv2.imread(os.path.join(input_folder2, file2))

            # Resize images to target size
            image1 = cv2.resize(image1, (640, 480))  # Assuming aspect ratio is maintained
            image2 = cv2.resize(image2, (640, 480))

            # Concatenate images horizontally
            #stitched_image = cv2.hconcat([image1, image2])
            stitcher = cv2.Stitcher_create()
            (status, stitched_image) = stitcher.stitch([image1, image2])


            # Resize concatenated image to target size
            stitched_image = cv2.resize(stitched_image, (1200, 480))

            # Save stitched image to output folder
            cv2.imwrite(os.path.join(output_folder, f"{timestamp1}_{timestamp2}_stitched.jpg"), stitched_image)

            print(f"Stitched and saved: {timestamp1}_{timestamp2}_stitched.png")

# Example usage
input_folder1 = "I1"
input_folder2 = "I2"
output_folder = "stitch"

stitch_images(input_folder1, input_folder2, output_folder)
