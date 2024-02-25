import cv2
import numpy as np


# Corrected intrinsic matrix (3x3)
intrinsic_matrix = np.array([[640, 0, 320], [0, 480, 240], [0, 0, 1]], dtype=np.float32)


# ...


# ...

def get_bbox(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return x, y, w, h
    else:
        return 0, 0, 0, 0

def pixel_to_world_coordinates(x, y):
    # Default depth for simplicity (replace with actual depth if available)
    depth = 1.0

    # Convert pixel coordinates to camera coordinates
    pixel_coordinates = np.array([[x, y, 1]], dtype=np.float32).T
    inverse_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)
    camera_coordinates = np.dot(inverse_intrinsic_matrix, pixel_coordinates)

    # Convert camera coordinates to world coordinates
    homogeneous_coordinates = np.append(camera_coordinates, 1)
    world_coordinates = homogeneous_coordinates * depth

    return world_coordinates[:-1]

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([160, 100, 20])
    upper2 = np.array([179, 255, 255])

    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)
    full_mask = cv2.bitwise_or(lower_mask, upper_mask)
    eroded_mask = cv2.erode(full_mask, None, iterations=3)

    x, y, w, h = get_bbox(eroded_mask)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Add label "CAR" to the image
    if x != 0 and y != 0:
        world_coordinates = pixel_to_world_coordinates(x + w / 2, y + h / 2)
        print("Real-world coordinates:", world_coordinates)

        # Project 3D axis onto 2D image plane
        axis_points = np.float32([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, -1]])
        image_points, _ = cv2.projectPoints(axis_points, np.zeros((3, 1)), np.zeros((3, 1)), intrinsic_matrix, None)

        # Draw 3D axis on the image
        origin = tuple(map(int, (x + w / 2, y + h / 2)))
        for point in image_points[1:]:
            end_point = tuple(map(int, point.reshape(2)))
            cv2.line(frame, origin, end_point, (0, 0, 255), 2)

        # Add label "CAR" over the bounding box
        label_position = (x + int(w / 2) - 20, y - 10)
        cv2.putText(frame, "Vehicle", label_position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()