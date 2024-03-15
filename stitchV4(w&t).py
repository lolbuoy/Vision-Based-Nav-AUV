'''ADVANCED STITCHING OF IMAGES FROM 
2 FOLDERS WITH WATCHDOG WITH DUPLICATE AVOIDANCE'''

import os
import cv2
import numpy as np
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import subprocess

class ImageStitcher:
    def __init__(self, input_folder1, input_folder2, output_folder):
        self.input_folder1 = input_folder1
        self.input_folder2 = input_folder2
        self.output_folder = output_folder
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.processed_pairs = set()

    def stitch_images(self):
        # Get list of files in each input folder
        files1 = sorted(os.listdir(self.input_folder1))
        files2 = sorted(os.listdir(self.input_folder2))

        # Loop through files in both folders
        for file1, file2 in zip(files1, files2):
            # Extract timestamps from filenames
            timestamp1 = file1.split('_')
            timestamp11 = int(timestamp1[2].split('.')[0])
            timestamp2 = file2.split('_')
            timestamp22 = int(timestamp2[2].split('.')[0])

            # Check if timestamps are similar and pair not processed yet
            if abs(timestamp11 - timestamp22) <= 1 and (file1, file2) not in self.processed_pairs:
                # Read images
                image1 = cv2.imread(os.path.join(self.input_folder1, file1))
                image2 = cv2.imread(os.path.join(self.input_folder2, file2))

                if image1 is None or image2 is None:
                    print("Error: Unable to read one or both images.")
                    continue

                # Resize images to target size
                image1 = cv2.resize(image1, (640, 480))  # Assuming aspect ratio is maintained
                image2 = cv2.resize(image2, (640, 480))
		
                stitcher = cv2.Stitcher_create()
                status, stitched_image = stitcher.stitch([image1, image2])

                if status != cv2.Stitcher_OK:
                    print("Error: Stitching failed.")
                    continue

                # Resize concatenated image to target size
                stitched_image = cv2.resize(stitched_image, (1200, 480))

                # Save stitched image
                output_filename = f"{timestamp1}_{timestamp2}_stitched.jpg"
                cv2.imwrite(os.path.join(self.output_folder, output_filename), stitched_image)
                print(f"Stitched and saved: {output_filename}")
                
                # Add the processed pair to the set
                self.processed_pairs.add((file1, file2))

class Watcher:
    def __init__(self, image_stitcher):
        self.image_stitcher = image_stitcher

    def run(self):
        event_handler = Handler(self.image_stitcher)
        observer = Observer()
        observer.schedule(event_handler, self.image_stitcher.input_folder1)
        observer.schedule(event_handler, self.image_stitcher.input_folder2)
        observer.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()

class Handler(FileSystemEventHandler):
    def __init__(self, image_stitcher):
        self.image_stitcher = image_stitcher

    def on_created(self, event):
        if event.is_directory:
            return
        print(f"Detected new file: {event.src_path}")
        self.image_stitcher.stitch_images()

# Call the bash script to clear the stitch folder
output_folder = "/home/vyana/Desktop/yolov8/stitch"
subprocess.call(['./singleclear.sh', output_folder])

# Example usage
if __name__ == "__main__":
    input_folder1 = "/home/vyana/Desktop/yolov8/camera3"
    input_folder2 = "/home/vyana/Desktop/yolov8/camera4"

    image_stitcher = ImageStitcher(input_folder1, input_folder2, output_folder)
    watcher = Watcher(image_stitcher)
    watcher.run()
