
import cv2
from ultralytics import YOLO
# Assuming 'best.pt' is the model weights file located in the 'runs\detect\train\weights\' directory
model = YOLO(r"runs\detect\train\weights\best4.pt")

# Open camera stream
#camera = cv2.VideoCapture("http://192.168.1.2:8080/video")
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Unable to open camera stream.")
    exit()

while True:
    ret, frame = camera.read()
    if not ret:
        break

    boxes = model(frame, verbose=False)[0]
    for box in boxes.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = box
        x1, y1, x2, y2 = list(map(int, [x1, y1, x2, y2]))
        
        if score > 0.4:
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cx = x1 + (x2-x1)//2
            cx-=frame.shape[1]//2
            cx/=frame.shape[1]//2
            print(cx)


    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()