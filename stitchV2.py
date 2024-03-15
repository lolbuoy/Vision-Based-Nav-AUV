'''BASIC STITCHING OF IMAGES FROM 2 FOLDERS WITH WATCHDOG'''

import os
import cv2
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class ImageStitcher:
    def __init__(self, input_folder1, input_folder2, output_folder):
        self.input_folder1 = input_folder1
        self.input_folder2 = input_folder2
        self.output_folder = output_folder
        self.stitcher = cv2.Stitcher_create()

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

            # Check if timestamps are similar
            if abs(timestamp11 - timestamp22) <= 1:  # Adjust threshold as needed
                # Read images
                image1 = cv2.imread(os.path.join(self.input_folder1, file1))
                image2 = cv2.imread(os.path.join(self.input_folder2, file2))

                # Check if images are valid
                if image1 is None or image2 is None:
                    print("Error: One or more images could not be read.")
                    continue

                # Resize images to target size
                image1 = cv2.resize(image1, (640, 480))  # Assuming aspect ratio is maintained
                image2 = cv2.resize(image2, (640, 480))

                # Stitch images
                (status, stitched_image) = self.stitcher.stitch([image1, image2])

                if status == cv2.Stitcher_OK:
                    # Resize stitched image to target size
                    stitched_image = cv2.resize(stitched_image, (1200, 480))

                    # Save stitched image to output folder
                    cv2.imwrite(os.path.join(self.output_folder, f"{timestamp1}_{timestamp2}_stitched.jpg"), stitched_image)

                    print(f"Stitched and saved: {timestamp1}_{timestamp2}_stitched.jpg")
                else:
                    print(f"Error stitching images: {status}")

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

# Example usage
if __name__ == "__main__":
    input_folder1 = "I1"
    input_folder2 = "I2"
    output_folder = "stitch"

    image_stitcher = ImageStitcher(input_folder1, input_folder2, output_folder)
    watcher = Watcher(image_stitcher)
    watcher.run()
