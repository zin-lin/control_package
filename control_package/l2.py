#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy



def is_finite(*xs):
    return all(math.isfinite(x) for x in xs)


def quat_to_rot_matrix(qx, qy, qz, qw):
    # normalize quaternion
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(n) or n < 1e-12:
        return None
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    # 3x3 rotation
    return (
        (1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)),
        (2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)),
        (2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)),
    )


def apply_tf_point(R, tx, ty, tz, x, y, z):
    (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = R
    xb = r00*x + r01*y + r02*z + tx
    yb = r10*x + r11*y + r12*z + ty
    zb = r20*x + r21*y + r22*z + tz
    return xb, yb, zb


class FullScanFrozenBase120(Node):
    """
    Each scan:
      - Capture frozen base pose: world_frame <- base_frame at first slice stamp
      - For each slice:
          * apply spinner transform base_frame <- lidar_frame at slice stamp
          * then apply frozen base pose world_frame <- base_frame (from scan start)
      - Publish 1 cloud after 120 slices
    This assembles wedges correctly while NOT deskewing base motion.
    """
    def __init__(self):
        super().__init__("l2_fullscan_frozen_base_120")

        # ---- Parameters ----
        self.slice_topic = self.declare_parameter("slice_topic", "a_a_v/ads_roll_e/L2/points").value
        self.out_topic = self.declare_parameter("out_topic", "a_a_v/ads_roll_e/L2/full_scan_points").value

        self.lidar_frame = self.declare_parameter("lidar_frame", "UTL2").value
        self.base_frame = self.declare_parameter("base_frame", "chassis").value
        self.world_frame = self.declare_parameter("world_frame", "odom").value

        self.slices_per_rev = int(self.declare_parameter("slices_per_rev", 120).value)

        # TF backoff settings (avoid starvation without using "latest")
        self.tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.01).value)
        self.tf_backoff_ms = int(self.declare_parameter("tf_backoff_ms", 2).value)      # try stamp, stamp-2ms, stamp-4ms...
        self.tf_backoff_tries = int(self.declare_parameter("tf_backoff_tries", 8).value)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Pub/Sub ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.pub = self.create_publisher(PointCloud2, self.out_topic, qos)
        self.sub = self.create_subscription(PointCloud2, self.slice_topic, self.on_slice, qos)

        # ---- Accumulation state ----
        self.slice_count = 0
        self.accum_xyz = []
        self.last_stamp = None

        # Frozen base pose for this revolution: world <- base at scan start
        self.frozen_world_base_R = None
        self.frozen_world_base_t = None
        self.scan_start_stamp = None

        # Output fields: XYZ float32
        self.xyz_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.get_logger().info(
            f"FullScanFrozenBase120 running. slice_topic={self.slice_topic} out={self.out_topic} "
            f"world={self.world_frame} base={self.base_frame} lidar={self.lidar_frame} slices_per_rev={self.slices_per_rev}"
        )
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.recv = 0
        self.tf_fail_wb = 0
        self.tf_fail_bl = 0

        self.create_timer(1.0, lambda: self.get_logger().info(
            f"alive recv={self.recv} tf_fail_wb={self.tf_fail_wb} tf_fail_bl={self.tf_fail_bl} slices={self.slice_count} pts={len(self.accum_xyz)}"
        ))

    def lookup_tf_with_backoff(self, target, source, stamp_time):
        """
        Try exact stamp, then back off a few ms at a time.
        This avoids 'latest TF' (which breaks geometry) but prevents starvation.
        """
        timeout = rclpy.duration.Duration(seconds=self.tf_timeout_sec)

        # Try multiple times: stamp, stamp-Δ, stamp-2Δ...
        backoff_ns = self.tf_backoff_ms * 1_000_000
        for i in range(self.tf_backoff_tries):
            t = rclpy.time.Time(nanoseconds=stamp_time.nanoseconds - i * backoff_ns)
            if t.nanoseconds < 0:
                break
            try:
                return self.tf_buffer.lookup_transform(target, source, t, timeout=timeout)
            except TransformException:
                continue
        return None

    def on_slice(self, msg: PointCloud2):
        self.recv += 1
        
        if not msg.header.frame_id:
            return

        # Use the message's own frame_id unless you want to force it
        src_frame = msg.header.frame_id

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.last_stamp = msg.header.stamp

        # Initialize scan: freeze world<-base at first slice time
        if self.slice_count == 0:
            self.scan_start_stamp = stamp

            tf_world_base = self.lookup_tf_with_backoff(self.world_frame, self.base_frame, stamp)
            if tf_world_base is None:
                # If we can't freeze base pose, skip starting this scan
                return

            t = tf_world_base.transform.translation
            q = tf_world_base.transform.rotation

            if not is_finite(t.x, t.y, t.z, q.x, q.y, q.z, q.w):
                return

            R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
            if R is None:
                return

            self.frozen_world_base_R = R
            self.frozen_world_base_t = (t.x, t.y, t.z)

        self.slice_count += 1
        # Need frozen pose available
        if self.frozen_world_base_R is None or self.frozen_world_base_t is None:
            return

        # Get instantaneous spinner transform base<-lidar at *slice time*
        tf_base_lidar = self.lookup_tf_with_backoff(self.base_frame, src_frame, stamp)
        if tf_base_lidar is None:
            return

        tb = tf_base_lidar.transform.translation
        qb = tf_base_lidar.transform.rotation

        if not is_finite(tb.x, tb.y, tb.z, qb.x, qb.y, qb.z, qb.w):
            return

        R_base_lidar = quat_to_rot_matrix(qb.x, qb.y, qb.z, qb.w)
        if R_base_lidar is None:
            return

        # Frozen world<-base
        R_world_base = self.frozen_world_base_R
        twx, twy, twz = self.frozen_world_base_t

        # base<-lidar translation
        tbx, tby, tbz = tb.x, tb.y, tb.z

        # Compose transform for points: world = (world<-base frozen) * (base<-lidar instantaneous) * p
        # We'll apply in two steps for clarity.

        # Read and transform points
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if not is_finite(x, y, z):
                continue

            # p_base = R_base_lidar * p_lidar + t_base_lidar
            xb, yb, zb = apply_tf_point(R_base_lidar, tbx, tby, tbz, float(x), float(y), float(z))
            if not is_finite(xb, yb, zb):
                continue

            # p_world = R_world_base * p_base + t_world_base(frozen)
            xw, yw, zw = apply_tf_point(R_world_base, twx, twy, twz, xb, yb, zb)
            if not is_finite(xw, yw, zw):
                continue

            self.accum_xyz.append((float(xw), float(yw), float(zw)))

        self.slice_count += 1

        # Publish when 120 slices accumulated
        if self.slice_count >= self.slices_per_rev:
            self.publish_and_reset()

    def publish_and_reset(self):
        if not self.accum_xyz or self.last_stamp is None:
            self.reset()
            return

        from std_msgs.msg import Header
        h = Header()
        h.frame_id = self.world_frame
        h.stamp = self.last_stamp  # single timestamp for whole scan

        cloud = pc2.create_cloud(h, self.xyz_fields, self.accum_xyz)
        self.pub.publish(cloud)

        self.get_logger().info(f"Published full scan: {len(self.accum_xyz)} points ({self.slices_per_rev} slices)")

        self.reset()

    def reset(self):
        self.slice_count = 0
        self.accum_xyz = []
        self.last_stamp = None
        self.scan_start_stamp = None
        self.frozen_world_base_R = None
        self.frozen_world_base_t = None


def main():
    rclpy.init()
    node = FullScanFrozenBase120()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
