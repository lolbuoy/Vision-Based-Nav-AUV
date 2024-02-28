import cv2
import os
import numpy as np
import pickle

def undistort_image(img, calibration_file):
    """
    Applies lens distortion correction to an image.

    Args:
        img: The image to be undistorted.
        calibration_file: The path to the calibration data file (e.g., pickle file).

    Returns:
        The undistorted image.
    """

    if calibration_file:
        # Load calibration data if provided
        with open(calibration_file, 'rb') as f:
            camera_matrix, dist_coeffs = pickle.load(f)

        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0)
        undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, newcameramtx)
        return undistorted_img
    else:
        print("Skipping lens distortion correction (no calibration data provided)")
        return img


def remove_unusual_contours(image):
    """
    Removes unusual contours from the image.

    Args:
        image: The image from which unusual contours are to be removed.

    Returns:
        The image with removed unusual contours.
    """

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the grayscale image to obtain a binary image
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) 

    # Find contours in the binary image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find and remove unusual contours
    valid_contours = []
    for contour in contours:
        # Compute the area of the contour
        area = cv2.contourArea(contour)
        # Add contours with reasonable area
        if area > 100:  # Adjust this threshold as needed
            valid_contours.append(contour)

    # Create a mask for valid contours
    mask = np.zeros_like(gray)
    cv2.drawContours(mask, valid_contours, -1, (255), thickness=cv2.FILLED)

    # Apply the mask to the original image
    result = cv2.bitwise_and(image, image, mask=mask)

    return result


def stitch_and_postprocess_images(dir1, dir2, output_folder, calibration_file=None):
    """
    Stitches images from two directories, applying post-processing techniques.

    Args:
        dir1: The path to the first directory containing images.
        dir2: The path to the second directory containing images.
        output_folder: The path to the output folder for stitched images.
        calibration_file: The path to the calibration data file (e.g., pickle file) for lens distortion correction (optional).
    """

    # Get image paths and ensure consistent pairing
    image_paths_dir1 = [os.path.join(dir1, file) for file in os.listdir(dir1) if file.lower().endswith(('.png', '.jpg', '.jpeg'))]
    image_paths_dir2 = [os.path.join(dir2, file) for file in os.listdir(dir2) if file.lower().endswith(('.png', '.jpg', '.jpeg'))]
    image_paths_dir1.sort()
    image_paths_dir2.sort()

    stitcher = cv2.Stitcher_create()

    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for i in range(min(len(image_paths_dir1), len(image_paths_dir2))):
        img1 = cv2.imread(image_paths_dir1[i])
        img2 = cv2.imread(image_paths_dir2[i])

        # Resize images if necessary (adjust resize factor as needed)
        img1 = cv2.resize(img1, (0, 0), fx=0.5, fy=0.5)
        img2 = cv2.resize(img2, (0, 0), fx=0.5, fy=0.5)

        # Lens distortion correction (optional)
        img1 = undistort_image(img1, calibration_file)
        img2 = undistort_image(img2, calibration_file)

        # Perform stitching
        status, stitched_image = stitcher.stitch((img1, img2))

        if status == cv2.Stitcher_OK:
            # Remove unusual contours
            processed_image = remove_unusual_contours(stitched_image)

            # Save the processed image
            output_filename = os.path.join(output_folder, f'stitched_image_{i}.png')
            cv2.imwrite(output_filename, processed_image)
            print(f'Stitched image saved to {output_filename}')
        else:
            print(f"Stitching failed for images {image_paths_dir1[i]} and {image_paths_dir2[i]}")


# Example usage:
dir1 = '/home/lolbuoy/Desktop/yolov8/camera2'
dir2 = '/home/lolbuoy/Desktop/yolov8/camera1'
output_folder = 'output_images'
calibration_file = None  # Provide None if no calibration file

stitch_and_postprocess_images(dir1, dir2, output_folder, calibration_file)
