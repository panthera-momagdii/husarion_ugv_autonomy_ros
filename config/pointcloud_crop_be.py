#!/usr/bin/env python3
# pointcloud_crop_be.py
#
# BEST_EFFORT pointcloud crop node — a drop-in replacement for
# pointcloud_crop_box::PointcloudCropBoxNode that subscribes to the
# input topic with BEST_EFFORT QoS.
#
# Why this exists: husarion's pointcloud_crop_box::PointcloudCropBoxNode
# is compiled with the default (RELIABLE) subscription QoS. The
# gz_bridge publisher is also RELIABLE, so the publisher blocks waiting
# for ACKs from every RELIABLE subscriber. When GLIM joins the network
# the extra DDS overhead pushes the bridge behind /clock — the lidar
# stamps grow stale by several seconds per minute, and GLIM eventually
# aborts with `IndexedSlidingWindow: index out of range`.
#
# With this node in its place the only RELIABLE consumer of
# /panther/ouster/points is removed; the bridge can keep up with sim
# time and GLIM stays synchronised. STVL subscribes to the output
# topic with BEST_EFFORT, so a BEST_EFFORT publisher here is QoS-compatible.
#
# Behaviour matches the C++ node it replaces:
#   * crops a base-frame bounding box (min_x..max_x, min_y..max_y,
#     min_z..max_z) from the pointcloud, keeping the COMPLEMENT
#     (`negative: true` semantics) so robot self-returns are removed
#   * preserves the full PointCloud2 layout (fields, point_step) so
#     STVL and pointcloud_to_laserscan see the same data as before
#   * transforms the box into the cloud's sensor frame via TF2 so we
#     don't have to reproject the points themselves (fast — same
#     trick the C++ node uses)
#
# Parameters (all overridable via the same names as
# pointcloud_crop_box.yaml — bringup_glim_stvl_launch.py reuses the
# same ReplaceString placeholders):
#   input_topic, output_topic, target_frame,
#   min_x, max_x, min_y, max_y, min_z, max_z,
#   negative (true = keep points OUTSIDE the box)

import struct

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


# rclpy's stock numpy-pc2 helper isn't part of sensor_msgs_py in jazzy's
# default install, so unpack inline. Only the x/y/z floats are touched;
# everything else (intensity, ring, timestamps, ...) is copied verbatim.
def _xyz_dtype(fields, point_step):
    offsets = {f.name: f.offset for f in fields if f.name in ("x", "y", "z")}
    if {"x", "y", "z"} - offsets.keys():
        raise ValueError("PointCloud2 missing x/y/z fields")
    return offsets


class PointcloudCropBE(Node):
    def __init__(self):
        super().__init__("pointcloud_crop_box")

        self.declare_parameter("input_topic", "/panther/ouster/points")
        self.declare_parameter("output_topic", "/panther/ouster/points_filtered")
        self.declare_parameter("target_frame", "panther/base_footprint")
        self.declare_parameter("min_x", -0.44)
        self.declare_parameter("max_x", 0.44)
        self.declare_parameter("min_y", -0.46)
        self.declare_parameter("max_y", 0.46)
        self.declare_parameter("min_z", 0.05)
        self.declare_parameter("max_z", 0.50)
        self.declare_parameter("negative", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.box_min = np.array(
            [
                self.get_parameter("min_x").value,
                self.get_parameter("min_y").value,
                self.get_parameter("min_z").value,
            ],
            dtype=np.float64,
        )
        self.box_max = np.array(
            [
                self.get_parameter("max_x").value,
                self.get_parameter("max_y").value,
                self.get_parameter("max_z").value,
            ],
            dtype=np.float64,
        )
        self.negative = bool(self.get_parameter("negative").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, be_qos)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.callback, be_qos
        )
        self.get_logger().info(
            f"crop {self.input_topic} -> {self.output_topic} box=[{self.box_min}, {self.box_max}] "
            f"target_frame={self.target_frame} negative={self.negative} (QoS=BEST_EFFORT)"
        )

    def callback(self, msg: PointCloud2):
        # Transform box corners from target_frame into the cloud's
        # sensor frame, then crop in sensor frame (no per-point TF).
        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return

        def _xform(x, y, z):
            p = PointStamped()
            p.header.frame_id = self.target_frame
            p.point.x, p.point.y, p.point.z = float(x), float(y), float(z)
            t = do_transform_point(p, tf).point
            return np.array([t.x, t.y, t.z])

        corners = np.array(
            [
                _xform(self.box_min[0], self.box_min[1], self.box_min[2]),
                _xform(self.box_max[0], self.box_min[1], self.box_min[2]),
                _xform(self.box_min[0], self.box_max[1], self.box_min[2]),
                _xform(self.box_min[0], self.box_min[1], self.box_max[2]),
                _xform(self.box_max[0], self.box_max[1], self.box_min[2]),
                _xform(self.box_max[0], self.box_min[1], self.box_max[2]),
                _xform(self.box_min[0], self.box_max[1], self.box_max[2]),
                _xform(self.box_max[0], self.box_max[1], self.box_max[2]),
            ]
        )
        b_min = corners.min(axis=0)
        b_max = corners.max(axis=0)

        try:
            offsets = _xyz_dtype(msg.fields, msg.point_step)
        except ValueError:
            return

        n = msg.width * msg.height
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

        # x, y, z are stored as little-endian float32 in standard PointCloud2 layouts.
        def _f32(off):
            sl = data[:, off : off + 4]
            return sl.view(np.float32).reshape(-1)

        x = _f32(offsets["x"])
        y = _f32(offsets["y"])
        z = _f32(offsets["z"])

        inside = (
            (x >= b_min[0])
            & (x <= b_max[0])
            & (y >= b_min[1])
            & (y <= b_max[1])
            & (z >= b_min[2])
            & (z <= b_max[2])
        )
        keep = ~inside if self.negative else inside
        kept = data[keep]

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.is_dense = False
        out.point_step = msg.point_step
        out.height = 1
        out.width = int(kept.shape[0])
        out.row_step = out.width * out.point_step
        out.data = kept.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointcloudCropBE()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
