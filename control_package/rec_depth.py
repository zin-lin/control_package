import os
import threading
import queue
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

BASE_DIR = "./rec"
DEPTH_TOPIC = "a_a_v/ads_roll_e/perception"
ODOM_TOPIC = "a_a_v/ads_roll_e/odometry"
QUEUE_MAX = 300

class DepthSaver(Node):
    def __init__(self):
        super().__init__("depth_saver")
        self.bridge = CvBridge()

        self.save_dir = os.path.join(BASE_DIR, "depth")
        os.makedirs(self.save_dir, exist_ok=True)

        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self.stop_evt = threading.Event()
        self.writer = threading.Thread(target=self.writer_loop, daemon=True)
        self.writer.start()

        self.counter = 0

        # Latest odometry (updated by odom callback)
        self.latest_odom = None
        self.last_saved_odom = None

        # Subscribe to depth
        self.create_subscription(
            Image, DEPTH_TOPIC, self.depth_callback, qos_profile_sensor_data
        )

        # Subscribe to odometry
        self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, qos_profile_sensor_data
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def depth_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if msg.encoding == "16UC1":
                img = img.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                img = img.astype(np.float32)
            else:
                return

            # Get current odometry (skip if not available yet)
            odom = self.latest_odom
            if odom is None:
                return

            # Extract position
            odom_arr = np.array([
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ], dtype=np.float32)

            # Skip if position hasn't changed (duplicate frame)
            if self.last_saved_odom is not None and np.array_equal(odom_arr, self.last_saved_odom):
                return

            self.last_saved_odom = odom_arr.copy()

            try:
                self.q.put_nowait((img, odom_arr))
            except queue.Full:
                pass

        except Exception as e:
            print("Depth error:", e)

    def writer_loop(self):
        odom_list = []

        while not self.stop_evt.is_set() or not self.q.empty():
            try:
                img, odom_arr = self.q.get(timeout=0.1)
            except queue.Empty:
                continue

            # Save depth frame
            filename = os.path.join(
                self.save_dir,
                f"depth_{self.counter:06d}.npy"
            )
            np.save(filename, img)

            # Accumulate odometry
            odom_list.append(odom_arr)

            self.counter += 1
            self.q.task_done()

        # Save all odometry as single npz on shutdown
        if odom_list:
            odom_all = np.stack(odom_list, axis=0)  # [N, 3]
            odom_path = os.path.join(BASE_DIR, "odometry.npz")
            np.savez(odom_path,
                x=odom_all[:, 0],
                y=odom_all[:, 1],
                z=odom_all[:, 2],
            )
            print(f"Saved {len(odom_list)} odometry entries to {odom_path}")

    def destroy_node(self):
        self.stop_evt.set()
        self.writer.join()
        print(f"Saved {self.counter} depth frames to {self.save_dir}")
        super().destroy_node()

def main():
    rclpy.init()
    node = DepthSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
