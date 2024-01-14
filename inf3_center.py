from ultralytics import YOLO

model = YOLO("/home/tanu/Desktop/me/predicts/all(train71)/weights/best.pt")

try:
    with open('/home/tanu/Desktop/yolov8/me/new1/log.txt', 'a') as log_file:
        log_file.write("Starting object detection process...\n")

        results = model.predict(source='/home/tanu/Desktop/yolov8/camera', project='/home/tanu/Desktop/yolov8/me/new', name='test1', exist_ok=True, save_crop=True,stream=True)

        for i, result in enumerate(results):
            log_file.write(f"Processing result {i}\n")
            for j, box in enumerate(result.boxes):
                log_file.write(f"Processing box {j}\n")
                class_name = result.names[box.cls[0].item()]
                coords = box.xyxy[0].tolist()
                coords = [round(x) for x in coords]
                center_x = (coords[0] + coords[2]) / 2
                center_y = (coords[1] + coords[3]) / 2

                # Log coordinates to the log file
                log_file.write(f"Object type: {class_name}, Center_X: {center_x}, Center_Y: {center_y}, Coordinates: {coords},\n")

except Exception as e:
    print(f"Error occurred: {e}")

