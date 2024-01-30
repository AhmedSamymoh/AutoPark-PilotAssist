#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import cv2
from ultralytics import YOLO

class YOLODetectorNode:
    def __init__(self):
        rospy.init_node('yolo_detector_node', anonymous=True)
        self.cx_pub = rospy.Publisher('/cx', Float32, queue_size=10)

        model_path = "/home/sami/Desktop/AutoParking-PilotAssistant/AutoPilot-ParkAssist-main/Version_3/YOLOmodel/runs/detect/train4/weights/best.pt"
        self.model = YOLO(model_path)
        self.camera = cv2.VideoCapture('https://192.168.43.243:8080/video')

#        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            rospy.logerr("Error: Unable to open camera stream.")
            rospy.signal_shutdown("Unable to open camera stream")

        self.rate = rospy.Rate(100)  # Adjust the frequency as needed

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.camera.read()
            if not ret:
                break

            # Detect objects using YOLO
            boxes = self.model(frame, verbose=False)[0]
            for box in boxes.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = box
                x1, y1, x2, y2 = list(map(int, [x1, y1, x2, y2]))

                if score > 0.7:
                    # Calculate cx value
                    cx = x1 + (x2 - x1) // 2
                    cx -= frame.shape[1] // 2
                    cx /= frame.shape[1] // 2
                    # Publish cx value
                    self.cx_pub.publish(cx)
                    # Draw rectangle around the object
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Display the frame with detection boxes
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.rate.sleep()

        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        yolo_detector_node = YOLODetectorNode()
        yolo_detector_node.run()
    except rospy.ROSInterruptException:
        pass
