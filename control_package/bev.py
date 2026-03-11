#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


def bresenham(x0, y0, x1, y1):
    """Integer grid ray tracing. Returns list of (x,y) cells along line including endpoint."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        cells.append((x1, y1))
    else:
        err = dy / 2
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        cells.append((x1, y1))
    return cells


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def dilate_binary_u8(img_u8: np.ndarray, iters: int = 2) -> np.ndarray:
    """
    Cheap 8-neighborhood dilation without OpenCV.
    img_u8: 0 or 255.
    """
    e = img_u8
    for _ in range(max(0, int(iters))):
        e2 = e.copy()
        # 4-neighbors
        e2[1:, :] = np.maximum(e2[1:, :], e[:-1, :])
        e2[:-1, :] = np.maximum(e2[:-1, :], e[1:, :])
        e2[:, 1:] = np.maximum(e2[:, 1:], e[:, :-1])
        e2[:, :-1] = np.maximum(e2[:, :-1], e[:, 1:])
        # diagonals
        e2[1:, 1:] = np.maximum(e2[1:, 1:], e[:-1, :-1])
        e2[1:, :-1] = np.maximum(e2[1:, :-1], e[:-1, 1:])
        e2[:-1, 1:] = np.maximum(e2[:-1, 1:], e[1:, :-1])
        e2[:-1, :-1] = np.maximum(e2[:-1, :-1], e[1:, 1:])
        e = e2
    return e


class LaserToBEV(Node):
    """
    LaserScan -> local egocentric BEV.

    Fixes included:
      - DO NOT create fake "circular walls" at max range:
        We only mark occupied endpoints (and edges) for *real hits*.
      - Optional cap on how far we "clear" free space for no-hit rays.
    """

    def __init__(self):
        super().__init__("laser_to_bev")

        # ---- Topics / frames ----
        self.declare_parameter("scan_topic", "/a_a_v/ads_roll_e/laser_scan")
        self.declare_parameter("frame_id", "base_link")

        # ---- Grid geometry ----
        self.declare_parameter("grid_resolution", 0.05)  # m/cell
        self.declare_parameter("grid_width_m", 10.0)     # left-right span
        self.declare_parameter("grid_height_m", 10.0)    # forward span

        # ---- FOV gating (explicit) ----
        self.declare_parameter("fov_min_angle", -math.pi / 2)
        self.declare_parameter("fov_max_angle", math.pi / 2)

        # ---- Range caps ----
        self.declare_parameter("max_range_cap", 10.0)
        self.declare_parameter("min_range_cap", 0.12)

        # ---- "No-hit" handling ----
        # If range is within this margin of range_max, treat it as NO HIT.
        self.declare_parameter("no_hit_margin_m", 0.15)
        # Optional: only clear free space up to this distance for NO-HIT rays.
        # Set <=0 to disable (clear all the way to r).
        self.declare_parameter("free_clear_cap_m", 0.0)

        # ---- Log-odds update strengths ----
        self.declare_parameter("lo_occ", 4.0)
        self.declare_parameter("lo_free", 1.0)
        self.declare_parameter("lo_min", -6.0)
        self.declare_parameter("lo_max", 6.0)

        # ---- View / edges rendering ----
        self.declare_parameter("publish_mono8_view", True)
        self.declare_parameter("publish_edges_mono8", True)
        self.declare_parameter("dilate_iters_view", 2)
        self.declare_parameter("dilate_iters_edges", 3)

        # ---- OccupancyGrid (optional) ----
        self.declare_parameter("publish_occupancy_grid", True)
        self.declare_parameter("occgrid_unknown_value", -1)
        self.declare_parameter("occgrid_free_value", 0)
        self.declare_parameter("occgrid_occ_value", 100)
        self.declare_parameter("occgrid_prob_occ_thresh", 0.65)
        self.declare_parameter("occgrid_prob_free_thresh", 0.35)

        # ---- Read params ----
        self.scan_topic = self.get_parameter("scan_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.res = float(self.get_parameter("grid_resolution").value)
        self.w_m = float(self.get_parameter("grid_width_m").value)
        self.h_m = float(self.get_parameter("grid_height_m").value)

        self.fov_min = float(self.get_parameter("fov_min_angle").value)
        self.fov_max = float(self.get_parameter("fov_max_angle").value)

        self.max_r = float(self.get_parameter("max_range_cap").value)
        self.min_r = float(self.get_parameter("min_range_cap").value)

        self.no_hit_margin = float(self.get_parameter("no_hit_margin_m").value)
        self.free_clear_cap = float(self.get_parameter("free_clear_cap_m").value)

        self.lo_occ = float(self.get_parameter("lo_occ").value)
        self.lo_free = float(self.get_parameter("lo_free").value)
        self.lo_min = float(self.get_parameter("lo_min").value)
        self.lo_max = float(self.get_parameter("lo_max").value)

        self.publish_m8 = bool(self.get_parameter("publish_mono8_view").value)
        self.publish_edges = bool(self.get_parameter("publish_edges_mono8").value)
        self.dilate_view = int(self.get_parameter("dilate_iters_view").value)
        self.dilate_edges = int(self.get_parameter("dilate_iters_edges").value)

        self.publish_og = bool(self.get_parameter("publish_occupancy_grid").value)
        self.og_unk = int(self.get_parameter("occgrid_unknown_value").value)
        self.og_free = int(self.get_parameter("occgrid_free_value").value)
        self.og_occ = int(self.get_parameter("occgrid_occ_value").value)
        self.p_occ_th = float(self.get_parameter("occgrid_prob_occ_thresh").value)
        self.p_free_th = float(self.get_parameter("occgrid_prob_free_thresh").value)

        # Grid dims
        self.grid_w = int(round(self.w_m / self.res))
        self.grid_h = int(round(self.h_m / self.res))

        # Robot at bottom center
        self.robot_cx = self.grid_w // 2
        self.robot_cy = 0

        # ---- ROS interfaces ----
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.pub_f32 = self.create_publisher(Image, "/bev/image_f32", 10)
        self.pub_m8 = self.create_publisher(Image, "/bev/image_mono8", 10)
        self.pub_edges_m8 = self.create_publisher(Image, "/bev/edges_mono8", 10)
        self.pub_grid = self.create_publisher(OccupancyGrid, "/bev/occupancy_grid", 10)

        self.get_logger().info(
            f"Subscribing: {self.scan_topic}\n"
            f"Publishing: /bev/image_f32 (32FC1)"
            + (", /bev/image_mono8 (mono8)" if self.publish_m8 else "")
            + (", /bev/edges_mono8 (mono8)" if self.publish_edges else "")
            + (", /bev/occupancy_grid" if self.publish_og else "")
            + "\n"
            f"Grid: {self.grid_w}x{self.grid_h} @ {self.res:.3f} m/cell (span {self.w_m}m x {self.h_m}m)\n"
            f"FOV gate: [{self.fov_min:.3f}, {self.fov_max:.3f}] rad\n"
            f"No-hit margin: {self.no_hit_margin:.3f}m, free_clear_cap_m: {self.free_clear_cap:.3f}m (<=0 disables)"
        )

    def on_scan(self, msg: LaserScan):
        lo = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        edges = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)

        angles = msg.angle_min + np.arange(len(msg.ranges), dtype=np.float32) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Valid if finite and inside LaserScan limits
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        # Cap to our max range (keeps grid consistent)
        ranges = np.clip(ranges, 0.0, self.max_r)

        # Effective "range_max" after our cap
        eff_rmax = min(float(msg.range_max), float(self.max_r))

        for i, (a, r) in enumerate(zip(angles, ranges)):
            if a < self.fov_min or a > self.fov_max:
                continue
            if not valid[i]:
                continue
            if r < self.min_r:
                continue

            # --- HIT GATING (prevents fake circular walls) ---
            # Treat ranges that are basically at max range as NO HIT.
            hit = (r < (eff_rmax - self.no_hit_margin))

            # Optionally cap how far we clear free space for NO-HIT rays
            r_used = r
            if (not hit) and (self.free_clear_cap > 0.0):
                r_used = min(r_used, self.free_clear_cap)

            # Recompute endpoint if we capped r_used (so free clearing ends sooner)
            # (For hit rays, r_used == r so this is unchanged.)
            # ROS scan frame: x forward, y left
            x_s = r_used * math.cos(a)
            y_s = r_used * math.sin(a)
            # BEV: x right, y forward (you already fixed left/right)
            x = -y_s
            y = x_s

            if y < 0.0:
                continue

            gx = int(round(self.robot_cx + x / self.res))
            gy = int(round(self.robot_cy + y / self.res))

            if gx < 0 or gx >= self.grid_w or gy < 0 or gy >= self.grid_h:
                continue

            cells = bresenham(self.robot_cx, self.robot_cy, gx, gy)

            # Free along ray: exclude endpoint because endpoint might be occupied or just "end of clearing"
            if len(cells) > 1:
                pass

            # Only for real hits, add occupied evidence + edge pixel at the TRUE hit location.
            if hit:
                # Compute true hit cell using original r (not r_used)
                x_s_hit = r * math.cos(a)
                y_s_hit = r * math.sin(a)
                x_hit = -y_s_hit
                y_hit = x_s_hit

                if y_hit >= 0.0:
                    gx_hit = int(round(self.robot_cx + x_hit / self.res))
                    gy_hit = int(round(self.robot_cy + y_hit / self.res))
                    if 0 <= gx_hit < self.grid_w and 0 <= gy_hit < self.grid_h:
                        lo[gy_hit, gx_hit] += self.lo_occ
                        edges[gy_hit, gx_hit] = 255

        np.clip(lo, self.lo_min, self.lo_max, out=lo)
        p_occ = sigmoid(lo).astype(np.float32)

        self.pub_f32.publish(self._to_image_32fc1(p_occ, msg.header.stamp))

        if self.publish_m8:
            p_vis = np.flipud(p_occ)

            img = np.full(p_vis.shape, 127, dtype=np.uint8)
            img[p_vis <= self.p_free_th] = 255
            img[p_vis >= self.p_occ_th] = 0

            occ = (img == 0).astype(np.uint8) * 255
            occ = dilate_binary_u8(occ, iters=self.dilate_view)
            img[occ == 255] = 0

            self.pub_m8.publish(self._to_image_mono8(img, msg.header.stamp))

        if self.publish_edges:
            e_vis = np.flipud(edges)
            if self.publish_edges:
                e_vis = np.flipud(edges)

                # Keep edges thin
                if self.dilate_edges > 0:
                    e_vis = dilate_binary_u8(e_vis, iters=min(self.dilate_edges, 1))

                self.pub_edges_m8.publish(self._to_image_mono8(e_vis, msg.header.stamp))

            self.pub_edges_m8.publish(self._to_image_mono8(e_vis, msg.header.stamp))

        if self.publish_og:
            self.pub_grid.publish(self._to_occupancy_grid(p_occ, msg.header.stamp))

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

    def _to_occupancy_grid(self, p_occ: np.ndarray, stamp) -> OccupancyGrid:
        og = OccupancyGrid()
        og.header = Header()
        og.header.stamp = stamp
        og.header.frame_id = self.frame_id

        og.info.resolution = float(self.res)
        og.info.width = self.grid_w
        og.info.height = self.grid_h

        og.info.origin = Pose()
        og.info.origin.position.x = -self.robot_cx * self.res
        og.info.origin.position.y = 0.0
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0

        data = np.full((self.grid_h, self.grid_w), self.og_unk, dtype=np.int8)
        data[p_occ >= self.p_occ_th] = self.og_occ
        data[p_occ <= self.p_free_th] = self.og_free

        og.data = data.flatten(order="C").tolist()
        return og


def main():
    rclpy.init()
    node = LaserToBEV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
