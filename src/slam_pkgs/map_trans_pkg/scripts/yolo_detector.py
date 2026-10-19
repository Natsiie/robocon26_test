#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('bbox_topic', '/yolo_bboxes')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('show_window', True)

        self.model_path = self.get_parameter('model_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = self.get_parameter('device').value
        self.show_window = bool(self.get_parameter('show_window').value)

        if not self.model_path:
            self.get_logger().error(
                "Parameter 'model_path' is empty. Pass a YOLO weight path, e.g. "
                "-p model_path:=/path/to/v5.pt"
            )
            raise RuntimeError("model_path is required")

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        self.last_time = time.time()
        self.frame_count = 0

        self.img_sub = self.create_subscription(
            Image, self.image_topic, self.img_callback, 10)
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, self.bbox_topic, 10)

        self.get_logger().info(
            f"YOLO node started: model={self.model_path}, "
            f"image={self.image_topic}, bbox={self.bbox_topic}, "
            f"device={self.device}")

    def img_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(
            frame,
            conf=self.conf_thres,
            imgsz=self.imgsz,
            verbose=False,
            device=self.device,
        )

        # FPS statistics
        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_time
        if elapsed >= 1.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f">>> YOLO FPS: {fps:.2f}")
            self.frame_count = 0
            self.last_time = now

        bbox_msg = Float32MultiArray()
        all_data = []

        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    x_c, y_c, w, h = box.xywh[0]
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    label = self.model.names[cls]
                    all_data.extend([
                        float(x_c), float(y_c), float(w), float(h),
                        conf, float(cls)
                    ])

                    # Visualization
                    cv2.rectangle(
                        frame,
                        (int(x_c - w / 2), int(y_c - h / 2)),
                        (int(x_c + w / 2), int(y_c + h / 2)),
                        (0, 255, 0), 2)
                    cv2.putText(
                        frame, f"{label}",
                        (int(x_c - w / 2), int(y_c - h / 2 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        bbox_msg.data = all_data
        self.bbox_pub.publish(bbox_msg)

        if self.show_window:
            cv2.imshow("YOLO Result", frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
