'''ADVANCED STITCHING OF IMAGES FROM 2 FOLDERS WITH WATCHDOG WITH DUPLICATE AVOIDANCE 
AND STITCHING PREVIOUSLY SAVED IMAGES'''

import os
import cv2
import numpy as np
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class ImageStitcher:
    def __init__(self, input_folder1, input_folder2, output_folder):
        self.input_folder1 = input_folder1
        self.input_folder2 = input_folder2
        self.output_folder = output_folder
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def stitch_existing_images(self):
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

            # Check if timestamps are similar
            if abs(timestamp11 - timestamp22) <= 1:  # Adjust threshold as needed
                # Read images
                image1 = cv2.imread(os.path.join(self.input_folder1, file1))
                image2 = cv2.imread(os.path.join(self.input_folder2, file2))

                # Resize images to target size
                image1 = cv2.resize(image1, (640, 480))  # Assuming aspect ratio is maintained
                image2 = cv2.resize(image2, (640, 480))

                # Find keypoints and descriptors
                kp1, des1 = self.orb.detectAndCompute(image1, None)
                kp2, des2 = self.orb.detectAndCompute(image2, None)

                # Match descriptors
                matches = self.matcher.match(des1, des2)

                # Sort matches by score
                matches = sorted(matches, key=lambda x: x.distance)

                # Draw top matches
                matched_img = cv2.drawMatches(image1, kp1, image2, kp2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                # Save matched image
                cv2.imwrite(os.path.join(self.output_folder, f"{timestamp1}_{timestamp2}_matched.jpg"), matched_img)

                print(f"Matched and saved: {timestamp1}_{timestamp2}_matched.jpg")

    def stitch_images(self):
        self.stitch_existing_images()  # Stitch existing images first

        # Now watch for new images
        event_handler = Handler(self)
        observer = Observer()
        observer.schedule(event_handler, self.input_folder1)
        observer.schedule(event_handler, self.input_folder2)
        observer.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            observer.stop()
            observer.join()

class Watcher:
    def __init__(self, image_stitcher):
        self.image_stitcher = image_stitcher

    def run(self):
        self.image_stitcher.stitch_images()

class Handler(FileSystemEventHandler):
    def __init__(self, image_stitcher):
        self.image_stitcher = image_stitcher

    def on_created(self, event):
        if event.is_directory:
            return
        print(f"Detected new file: {event.src_path}")
        self.image_stitcher.stitch_existing_images()

# Example usage
if __name__ == "__main__":
    input_folder1 = "I1"
    input_folder2 = "I2"
    output_folder = "stitch"

    image_stitcher = ImageStitcher(input_folder1, input_folder2, output_folder)
    watcher = Watcher(image_stitcher)
    watcher.run()
