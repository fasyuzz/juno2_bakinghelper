#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
import time

class IngredientDetector:
    def __init__(self):
        self.latest_frame = None
        self.detected = set()
        self.model = YOLO("/home/mustar/catkin_ws/src/juno2_baking_helper/src/models/best.pt")
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.loginfo("IngredientDetector initialized, waiting for user command...")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")

    def capture_and_detect(self):
        if self.latest_frame is not None:
            results = self.model.predict(source=self.latest_frame, show=False, conf=0.5)

            # draw bounding boxes
            frame_draw = self.latest_frame.copy()
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0].item())
                    label = result.names[cls_id]
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy
                    cv2.rectangle(frame_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame_draw, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    self.detected.add(label)

            # show segmentation window for 5 seconds
            rospy.loginfo("Showing segmentation window for 5 seconds...")
            start_time = time.time()
            while time.time() - start_time < 5:
                cv2.imshow("Ingredient Detection", frame_draw)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cv2.destroyWindow("Ingredient Detection")

            rospy.loginfo(f"Detected ingredients: {self.detected}")
        else:
            rospy.logwarn("No frame available yet.")

    def get_detected(self):
        return list(self.detected)

if __name__ == "__main__":
    rospy.init_node("ingredient_detector")
    detector = IngredientDetector()
    while not rospy.is_shutdown():
        user_input = input("Press ENTER to capture or 'q' to quit: ").strip().lower()
        if user_input == "q":
            break
        detector.capture_and_detect()


