import os
import threading
import queue
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

BASE_DIR = "./rec"
TOPIC = "/camera"
QUEUE_MAX = 500

class RGBSaver(Node):
    def __init__(self):
        super().__init__("rgb_saver")
        self.bridge = CvBridge()

        self.save_dir = os.path.join(BASE_DIR, "images")
        os.makedirs(self.save_dir, exist_ok=True)

        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self.stop_evt = threading.Event()
        self.writer = threading.Thread(target=self.writer_loop, daemon=True)
        self.writer.start()

        self.counter = 0

        self.create_subscription(
            Image, TOPIC, self.callback, qos_profile_sensor_data
        )

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            try:
                self.q.put_nowait(img)
            except queue.Full:
                pass
        except Exception as e:
            print("RGB error:", e)

    def writer_loop(self):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        while not self.stop_evt.is_set() or not self.q.empty():
            try:
                img = self.q.get(timeout=0.1)
            except queue.Empty:
                continue

            filename = os.path.join(
                self.save_dir,
                f"rgb_{self.counter:06d}.jpg"
            )
            cv2.imwrite(filename, img, params)
            self.counter += 1
            self.q.task_done()

    def destroy_node(self):
        self.stop_evt.set()
        self.writer.join()
        super().destroy_node()

def main():
    rclpy.init()
    node = RGBSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
