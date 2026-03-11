#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2

class PointCloudToBEV(Node):
    def __init__(self):
        super().__init__("pointcloud_to_bev")

        # topics
        self.declare_parameter("points_topic", "a_a_v/ads_roll_e/perception/points")
        self.declare_parameter("frame_id", "base_link")

        # grid
        self.declare_parameter("grid_resolution", 0.055)   # m/cell
        self.declare_parameter("grid_width_m", 10)
        self.declare_parameter("grid_height_m", 10)

        # filters
        self.declare_parameter("max_range_m", 10.0)
        self.declare_parameter("min_range_m", 0.2)

        # ground estimation region (robot-centric: x forward, y left, z up)
        self.declare_parameter("ground_x_min", 0.5)
        self.declare_parameter("ground_x_max", 3.0)
        self.declare_parameter("ground_y_abs_max", 2.0)

        self.declare_parameter("ground_percentile", 5.0)   # estimate ground z
        self.declare_parameter("ground_band_m", 0.08)      # points within +/- band are ground
        self.declare_parameter("obstacle_z_margin", 0.10)  # keep points above ground+margin

        # publishing
        self.declare_parameter("publish_mono8", True)

        self.points_topic = self.get_parameter("points_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.res = float(self.get_parameter("grid_resolution").value)
        self.w_m = float(self.get_parameter("grid_width_m").value)
        self.h_m = float(self.get_parameter("grid_height_m").value)

        self.grid_w = int(round(self.w_m / self.res))
        self.grid_h = int(round(self.h_m / self.res))

        # robot at bottom center (y forward)
        self.robot_cx = self.grid_w // 2
        self.robot_cy = 0

        self.max_r = float(self.get_parameter("max_range_m").value)
        self.min_r = float(self.get_parameter("min_range_m").value)

        self.gx0 = float(self.get_parameter("ground_x_min").value)
        self.gx1 = float(self.get_parameter("ground_x_max").value)
        self.gyabs = float(self.get_parameter("ground_y_abs_max").value)

        self.gpct = float(self.get_parameter("ground_percentile").value)
        self.gband = float(self.get_parameter("ground_band_m").value)
        self.gmargin = float(self.get_parameter("obstacle_z_margin").value)

        self.publish_mono8 = bool(self.get_parameter("publish_mono8").value)

        self.sub = self.create_subscription(PointCloud2, self.points_topic, self.on_points, 10)
        self.pub_f32 = self.create_publisher(Image, "/bev/image_f32", 10)
        self.pub_m8  = self.create_publisher(Image, "/bev/image_mono8", 10)

        self.get_logger().info(
            f"Sub: {self.points_topic}\n"
            f"Grid: {self.grid_w}x{self.grid_h} @ {self.res}m\n"
            f"Pub: /bev/image_f32 (+ mono8 view={self.publish_mono8})"
        )

    def on_points(self, msg: PointCloud2):
        # Read xyz points (skip NaNs)
        pts = np.array(list(pc2.read_points_numpy(msg, field_names=("x","y","z"), skip_nans=True)), dtype=np.float32)
        if pts.size == 0:
            return

        x = pts[:,0]
        y = pts[:,1]
        z = pts[:,2]

        # range filter (front-facing only if you want: x>0)
        r = np.sqrt(x*x + y*y)
        keep = (r >= self.min_r) & (r <= self.max_r) & (x >= 0.0)
        x = x[keep]; y = y[keep]; z = z[keep]
        if x.size == 0:
            return

        # --- estimate ground z from a near region ---
        gmask = (x >= self.gx0) & (x <= self.gx1) & (np.abs(y) <= self.gyabs)
        if np.any(gmask):
            z_ground = np.percentile(z[gmask], self.gpct)
        else:
            # fallback if region empty
            z_ground = np.percentile(z, self.gpct)

        # remove ground band, keep obstacles above ground+margin
        obs = (z > (z_ground + self.gmargin))
        x = x[obs]; y = y[obs]; z = z[obs]
        # if no obstacles, still publish empty
        # (for predictive coding, empties are informative)
        # Create soft occupancy evidence map
        lo = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        if x.size > 0:
            # Convert to BEV coords: we want x_right, y_forward
            # LiDAR frame assumed: x forward, y left => right = -y, forward = x
            x_right = -y
            y_fwd   =  x

            gx = np.rint(self.robot_cx + x_right / self.res).astype(np.int32)
            gy = np.rint(self.robot_cy + y_fwd   / self.res).astype(np.int32)

            inside = (gx >= 0) & (gx < self.grid_w) & (gy >= 0) & (gy < self.grid_h)
            gx = gx[inside]; gy = gy[inside]

            # add evidence at occupied cells (no ray fan)
            lo[gy, gx] += 4.0  # lo_occ-style evidence

            # clip to avoid blowup
            np.clip(lo, -6.0, 6.0, out=lo)

        # convert to [0,1] probability
        p = 1.0 / (1.0 + np.exp(-lo))

        # publish 32FC1
        self.pub_f32.publish(self._to_image_32fc1(p, msg.header.stamp))

        if self.publish_mono8:
            # Make mono8 look like float: visualize log-odds (better contrast)
            lo_vis = np.flipud(lo)
            lo_norm = (lo_vis - (-6.0)) / (6.0 - (-6.0))
            lo_norm = np.clip(lo_norm, 0.0, 1.0)
            lo_norm = 1.0 - lo_norm  # occupied darker
            m8 = (lo_norm * 255.0).astype(np.uint8)
            self.pub_m8.publish(self._to_image_mono8(m8, msg.header.stamp))

    def _to_image_32fc1(self, p_occ: np.ndarray, stamp) -> Image:
        p = np.flipud(p_occ).astype(np.float32)
        imsg = Image()
        imsg.header.stamp = stamp
        imsg.header.frame_id = self.frame_id
        imsg.height, imsg.width = p.shape
        imsg.encoding = "32FC1"
        imsg.is_bigendian = False
        imsg.step = imsg.width * 4
        imsg.data = p.tobytes()
        return imsg

    def _to_image_mono8(self, img_u8: np.ndarray, stamp) -> Image:
        imsg = Image()
        imsg.header.stamp = stamp
        imsg.header.frame_id = self.frame_id
        imsg.height, imsg.width = img_u8.shape
        imsg.encoding = "mono8"
        imsg.is_bigendian = False
        imsg.step = imsg.width
        imsg.data = img_u8.tobytes()
        return imsg

def main():
    rclpy.init()
    node = PointCloudToBEV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
