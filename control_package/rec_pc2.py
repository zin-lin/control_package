import os
import threading
import queue
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# ===== CONFIG =====
BASE_DIR = "./rec"
TOPIC = "/a_a_v/ads_roll_e/laser_scan/points"
TARGET_N = 4200
VOXEL_SIZE = 0.05   # 5 cm, tune if needed
QUEUE_MAX = 100
# ==================

def voxel_downsample(xyz, voxel_size):
    coords = np.floor(xyz / voxel_size).astype(np.int32)
    _, unique_indices = np.unique(coords, axis=0, return_index=True)
    return xyz[unique_indices]

def fps(xyz, n):
    N = int(xyz.shape[0])

    if N == 0:
        return np.zeros((n, 3), dtype=np.float32)

    if N == n:
        return xyz

    if N < n:
        # Repeat points evenly to reach n without biasing only the first few points
        idx = np.arange(n) % N
        return xyz[idx]

    # FPS for N > n
    centroids = np.empty(n, dtype=np.int64)
    distances = np.full(N, 1e10, dtype=np.float32)

    # Choose a deterministic start: farthest from centroid of the cloud
    center = np.mean(xyz, axis=0)
    farthest = int(np.argmax(np.sum((xyz - center) ** 2, axis=1)))

    for i in range(n):
        centroids[i] = farthest
        centroid = xyz[farthest]
        dist = np.sum((xyz - centroid) ** 2, axis=1).astype(np.float32)
        distances = np.minimum(distances, dist)
        farthest = int(np.argmax(distances))

    return xyz[centroids]


class PC2Saver(Node):
    def __init__(self):
        super().__init__("pc2_saver")

        self.save_dir = os.path.join(BASE_DIR, "PC2")
        os.makedirs(self.save_dir, exist_ok=True)

        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self.stop_evt = threading.Event()
        self.writer = threading.Thread(target=self.writer_loop, daemon=True)
        self.writer.start()

        self.counter = 0

        self.create_subscription(
            PointCloud2, TOPIC, self.callback, qos_profile_sensor_data
        )

        print("Voxel size:", VOXEL_SIZE)
        print("Target N:", TARGET_N)

    def callback(self, msg):
        try:
            xyz = point_cloud2.read_points_numpy(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ).astype(np.float32)

            # --- Geometry preserving pipeline ---
            xyz = voxel_downsample(xyz, VOXEL_SIZE)
            xyz = fps(xyz, TARGET_N)

            try:
                self.q.put_nowait(xyz)
            except queue.Full:
                pass

        except Exception as e:
            print("PC2 error:", e)

    def writer_loop(self):
        while not self.stop_evt.is_set() or not self.q.empty():
            try:
                xyz = self.q.get(timeout=0.1)
            except queue.Empty:
                continue

            filename = os.path.join(
                self.save_dir,
                f"cloud_{self.counter:06d}.npz"
            )
            np.savez_compressed(filename, xyz=xyz)
            self.counter += 1
            self.q.task_done()

    def destroy_node(self):
        self.stop_evt.set()
        self.writer.join()
        super().destroy_node()

def main():
    rclpy.init()
    node = PC2Saver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
