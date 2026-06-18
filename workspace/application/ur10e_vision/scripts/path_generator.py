#!/usr/bin/env python3
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from sensor_msgs.msg import PointCloud2, RegionOfInterest
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

try:
    import open3d as o3d
    _HAVE_O3D = True
except Exception as exc:  # pragma: no cover
    o3d = None
    _HAVE_O3D = False
    _O3D_ERR = str(exc)


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def cloud_to_xyz_grid(msg: PointCloud2) -> np.ndarray:
    """Organized PointCloud2 -> (H, W, 3) float32 array (NaN where invalid)."""
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw.reshape(msg.height, msg.width, msg.point_step)

    def field(name):
        o = offs[name]
        return raw[:, :, o:o + 4].copy().view(np.float32).reshape(msg.height, msg.width)

    return np.stack([field("x"), field("y"), field("z")], axis=-1)


def mat_to_quat(R: np.ndarray):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")

        self.surface_topic = self.declare_parameter(
            "surface_cloud_topic", "/ur10e_vision/surface_cloud").value
        self.roi_topic = self.declare_parameter(
            "roi_info_topic", "/ur10e_vision/roi").value
        self.output_frame = self.declare_parameter("output_frame", "base_link").value
        self.camera_optical_frame = self.declare_parameter(
            "camera_optical_frame", "camera_color_optical_frame").value

        self.raster_direction = self.declare_parameter("raster_direction", "horizontal").value
        self.tool_width = float(self.declare_parameter("tool_width", 0.05).value)
        self.waypoint_spacing = float(self.declare_parameter("waypoint_spacing", 0.02).value)
        self.standoff = float(self.declare_parameter("standoff", 0.0).value)
        self.approach_sign = float(self.declare_parameter("approach_axis_sign", -1.0).value)

        self.normal_knn = int(self.declare_parameter("normal_knn", 30).value)
        self.normal_radius = float(self.declare_parameter("normal_radius", 0.03).value)

        self.enable_cad = bool(self.declare_parameter("enable_cad_fallback", True).value)
        self.cad_pkg = self.declare_parameter("cad_mesh_package", "ur10e_simulation_pkg").value
        self.cad_subdir = self.declare_parameter("cad_mesh_subdir", "meshes").value
        self.cad_files = list(self.declare_parameter(
            "cad_mesh_files", ["blade_1.stl", "blade_2.stl", "middle_blade.stl"]).value)
        self.icp_max_corr = float(self.declare_parameter("icp_max_correspondence", 0.05).value)
        self.icp_fitness_min = float(self.declare_parameter("icp_fitness_min", 0.3).value)

        # Back-fill interior holes (sparse/occluded cells bracketed by real
        # points) from the registered CAD blade. Needs enable_cad_fallback.
        self.cad_fill = bool(self.declare_parameter("cad_fill_interior", True).value)
        self.cad_fill_max_gap = float(self.declare_parameter("cad_fill_max_gap", 0.2).value)

        self.publish_wp_tf = bool(self.declare_parameter("publish_waypoint_tf", True).value)
        self.wp_tf_prefix = self.declare_parameter("waypoint_tf_prefix", "clean_wp_").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_bcast = StaticTransformBroadcaster(self)

        self.roi = None
        self.create_subscription(RegionOfInterest, self.roi_topic, self._roi_cb, latched_qos())
        self.create_subscription(PointCloud2, self.surface_topic, self._cloud_cb, latched_qos())

        self.pose_pub = self.create_publisher(PoseArray, "/ur10e_vision/cleaning_path", latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, "/ur10e_vision/path_markers", latched_qos())

        if not _HAVE_O3D:
            self.get_logger().error(
                f"Open3D import failed ({_O3D_ERR}). Install `python3-open3d` "
                "or run `pip install open3d` if you are outside the system Python environment. "
                "Normal estimation/CAD fallback will be unavailable.")
        self.get_logger().info(
            f"path_generator up. tool_width={self.tool_width} "
            f"spacing={self.waypoint_spacing} standoff={self.standoff} "
            f"raster={self.raster_direction} cad_fallback={self.enable_cad}")

    # ---- callbacks ---------------------------------------------------------
    def _roi_cb(self, msg: RegionOfInterest):
        self.roi = msg

    def _cloud_cb(self, msg: PointCloud2):
        if not _HAVE_O3D:
            self.get_logger().error("Cannot generate path without Open3D.")
            return
        try:
            self._generate(msg)
        except Exception as exc:  # keep the node alive on a bad frame
            self.get_logger().error(f"Path generation failed: {exc}")

    # ---- core --------------------------------------------------------------
    def _generate(self, msg: PointCloud2):
        xyz = cloud_to_xyz_grid(msg)          # (H, W, 3) in output_frame
        H, W, _ = xyz.shape
        valid = np.isfinite(xyz).all(axis=-1)
        n_valid = int(valid.sum())
        if n_valid < 10:
            self.get_logger().warn(f"Surface cloud has only {n_valid} valid points.")
            return

        # ROI pixel bounds (default to full frame).
        if self.roi is not None and self.roi.width > 0 and self.roi.height > 0:
            x0, y0 = self.roi.x_offset, self.roi.y_offset
            x1, y1 = x0 + self.roi.width, y0 + self.roi.height
        else:
            x0, y0, x1, y1 = 0, 0, W, H

        # metres-per-pixel from neighbouring valid samples.
        m_h = self._median_step(xyz, valid, axis=1)   # horizontal
        m_v = self._median_step(xyz, valid, axis=0)   # vertical
        along_px = max(1, int(round(self.waypoint_spacing / m_h))) if m_h > 0 else 4
        pitch_px = max(1, int(round(self.tool_width / m_v))) if m_v > 0 else 8

        # Camera origin in output_frame (to orient normals toward the camera).
        cam_origin = self._camera_origin(msg.header.stamp)

        # Open3D cloud of all valid surface points + normals.
        pts = xyz[valid].astype(np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.normal_radius, max_nn=self.normal_knn))
        if cam_origin is not None:
            pcd.orient_normals_towards_camera_location(cam_origin)
        else:
            pcd.orient_normals_consistent_tangent_plane(self.normal_knn)
        kdt = o3d.geometry.KDTreeFlann(pcd)
        normals = np.asarray(pcd.normals)

        # Optional CAD fallback (best-fit blade STL registered into the cloud).
        cad = self._register_cad(pcd) if self.enable_cad else None

        # ---- raster grid in ROI image plane (horizontal passes, step v) ----
        positions, wp_normals, inferred = [], [], []
        if self.raster_direction == "vertical":
            pass_iter = range(x0, x1, pitch_px)        # passes along columns
            along_iter = lambda: range(y0, y1, along_px)
            def pick(p, q):  # p = column (pass), q = row (along)
                return self._sample(xyz, valid, p, q, along_px, pitch_px)
        else:
            pass_iter = range(y0, y1, pitch_px)        # passes along rows
            along_iter = lambda: range(x0, x1, along_px)
            def pick(p, q):  # p = row (pass), q = col (along)
                return self._sample(xyz, valid, q, p, along_px, pitch_px)

        use_cad_fill = self.cad_fill and cad is not None
        serpentine = False
        for p in pass_iter:
            cols = list(along_iter())
            if serpentine:
                cols = cols[::-1]
            serpentine = not serpentine

            # Measured surface point per along-cell (None where the cloud has a
            # hole). Interior holes are back-filled from the registered CAD.
            raw = [pick(p, q) for q in cols]
            cells = self._cad_fill_pass(raw, cad) if use_cad_fill else \
                [(pt, None, False) if pt is not None else None for pt in raw]

            for rec in cells:
                if rec is None:
                    continue
                pt, cad_nrm, is_inferred = rec
                nrm = cad_nrm if cad_nrm is not None else \
                    self._normal_at(pt, kdt, normals, cad)
                if nrm is None:
                    continue
                positions.append(pt)
                wp_normals.append(nrm)
                inferred.append(is_inferred)

        if len(positions) < 2:
            self.get_logger().warn(
                f"Only {len(positions)} waypoints generated; check spacing/tool_width.")
            return

        positions = np.asarray(positions)
        wp_normals = np.asarray(wp_normals)
        inferred = np.asarray(inferred, dtype=bool)
        n_filled = int(inferred.sum())
        self._publish(positions, wp_normals, inferred, msg.header.stamp)
        self.get_logger().info(
            f"Generated {len(positions)} waypoints "
            f"({n_filled} CAD-filled in gaps; "
            f"pitch={pitch_px}px/{self.tool_width}m, along={along_px}px/{self.waypoint_spacing}m"
            f"{', CAD-assisted' if cad else ''}).")

    # ---- helpers -----------------------------------------------------------
    @staticmethod
    def _median_step(xyz, valid, axis):
        if axis == 1:
            d = np.linalg.norm(xyz[:, 1:, :] - xyz[:, :-1, :], axis=-1)
            m = valid[:, 1:] & valid[:, :-1]
        else:
            d = np.linalg.norm(xyz[1:, :, :] - xyz[:-1, :, :], axis=-1)
            m = valid[1:, :] & valid[:-1, :]
        d = d[m]
        d = d[np.isfinite(d) & (d > 0)]
        return float(np.median(d)) if d.size else 0.0

    @staticmethod
    def _sample(xyz, valid, u, v, win_u, win_v):
        """Nearest valid pixel to (u, v) within a small window -> 3D point."""
        H, W, _ = xyz.shape
        if 0 <= v < H and 0 <= u < W and valid[v, u]:
            return xyz[v, u].copy()
        ru = max(1, win_u // 2)
        rv = max(1, win_v // 2)
        best, best_d = None, 1e9
        for dv in range(-rv, rv + 1):
            for du in range(-ru, ru + 1):
                uu, vv = u + du, v + dv
                if 0 <= vv < H and 0 <= uu < W and valid[vv, uu]:
                    d = du * du + dv * dv
                    if d < best_d:
                        best_d, best = d, xyz[vv, uu].copy()
        return best

    def _cad_fill_pass(self, raw, cad):
        """Back-fill interior holes (None) in a raster pass from the registered CAD."""
        out = [(p, None, False) if p is not None else None for p in raw]
        valid_idx = [i for i, p in enumerate(raw) if p is not None]
        if cad is None or len(valid_idx) < 2:
            return out
        cad_pts, cad_nrm, kdt = cad["points"], cad["normals"], cad["kdt"]
        for a, b in zip(valid_idx[:-1], valid_idx[1:]):
            if b - a <= 1:
                continue                                   # adjacent, no hole
            pa, pb = raw[a], raw[b]
            if np.linalg.norm(pb - pa) > self.cad_fill_max_gap:
                continue                                   # too wide to bridge
            span = b - a
            for k in range(a + 1, b):
                est = (pa + (k - a) / span * (pb - pa)).astype(np.float64)
                kk, idx, _ = kdt.search_knn_vector_3d(est, 1)
                if kk < 1:
                    continue
                j = idx[0]
                n = cad_nrm[j]
                nn = np.linalg.norm(n)
                out[k] = (cad_pts[j].copy(), (n / nn) if nn > 1e-9 else None, True)
        return out

    def _camera_origin(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, self.camera_optical_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z], dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f"No TF {self.output_frame}<-{self.camera_optical_frame}: {exc}")
            return None

    def _normal_at(self, pt, kdt, normals, cad):
        k, idx, dist2 = kdt.search_knn_vector_3d(pt, self.normal_knn)
        sparse = (k < max(3, self.normal_knn // 3)) or \
                 (k > 0 and math.sqrt(dist2[0]) > 2.0 * self.normal_radius)
        if not sparse and k > 0:
            n = normals[list(idx)].mean(axis=0)
        elif cad is not None:
            ck, cidx, _ = cad["kdt"].search_knn_vector_3d(pt, 1)
            if ck < 1:
                return None
            n = cad["normals"][cidx[0]]
        elif k > 0:
            n = normals[list(idx)].mean(axis=0)   # best effort
        else:
            return None
        norm = np.linalg.norm(n)
        return (n / norm) if norm > 1e-9 else None

    def _register_cad(self, target_pcd):
        """Register the best-fit blade STL into the surface cloud via ICP."""
        try:
            share = get_package_share_directory(self.cad_pkg)
        except Exception as exc:
            self.get_logger().warn(f"CAD package '{self.cad_pkg}' not found: {exc}")
            return None
        voxel = max(self.tool_width, 0.01)
        tgt = target_pcd.voxel_down_sample(voxel)
        if len(tgt.points) < 10:
            return None
        tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
        tgt_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            tgt, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))

        best = None
        for fname in self.cad_files:
            path = os.path.join(share, self.cad_subdir, fname)
            if not os.path.isfile(path):
                continue
            try:
                mesh = o3d.io.read_triangle_mesh(path)
                mesh.compute_vertex_normals()
                src_full = mesh.sample_points_uniformly(number_of_points=20000)
                src = src_full.voxel_down_sample(voxel)
                src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
                src_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                    src, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
                ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                    src, tgt, src_fpfh, tgt_fpfh, True, voxel * 1.5,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
                    [o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel * 1.5)],
                    o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
                icp = o3d.pipelines.registration.registration_icp(
                    src, tgt, self.icp_max_corr, ransac.transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane())
                if best is None or icp.fitness > best[0]:
                    best = (icp.fitness, fname, icp.transformation, src_full)
            except Exception as exc:
                self.get_logger().warn(f"CAD ICP for {fname} failed: {exc}")

        if best is None or best[0] < self.icp_fitness_min:
            self.get_logger().warn(
                f"CAD fallback: no mesh reached fitness>={self.icp_fitness_min} "
                f"(best={best[0]:.2f} via {best[1]})." if best else
                "CAD fallback: no usable mesh.")
            return None

        fitness, fname, T, src_full = best
        src_full.transform(T)
        self.get_logger().info(f"CAD fallback: {fname} registered, fitness={fitness:.2f}.")
        return {"kdt": o3d.geometry.KDTreeFlann(src_full),
                "points": np.asarray(src_full.points),
                "normals": np.asarray(src_full.normals)}

    def _publish(self, positions, normals, inferred, stamp):
        pa = PoseArray()
        pa.header.frame_id = self.output_frame
        pa.header.stamp = stamp
        transforms = []
        for i, (pos, n) in enumerate(zip(positions, normals)):
            z_axis = self.approach_sign * n                      # into/out of surface
            z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-12)
            if i + 1 < len(positions):
                t = positions[i + 1] - pos
            else:
                t = pos - positions[i - 1]
            t = t - np.dot(t, z_axis) * z_axis                   # project onto tangent plane
            if np.linalg.norm(t) < 1e-6:
                t = np.cross(z_axis, [1.0, 0.0, 0.0])
                if np.linalg.norm(t) < 1e-6:
                    t = np.cross(z_axis, [0.0, 1.0, 0.0])
            x_axis = t / (np.linalg.norm(t) + 1e-12)
            y_axis = np.cross(z_axis, x_axis)
            R = np.column_stack([x_axis, y_axis, z_axis])
            qx, qy, qz, qw = mat_to_quat(R)

            wp = pos + self.standoff * n                         # offset along outward normal
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, wp)
            pose.orientation.x, pose.orientation.y = float(qx), float(qy)
            pose.orientation.z, pose.orientation.w = float(qz), float(qw)
            pa.poses.append(pose)

            if self.publish_wp_tf:
                ts = TransformStamped()
                ts.header.frame_id = self.output_frame
                ts.header.stamp = stamp
                ts.child_frame_id = f"{self.wp_tf_prefix}{i:03d}"
                ts.transform.translation.x = float(wp[0])
                ts.transform.translation.y = float(wp[1])
                ts.transform.translation.z = float(wp[2])
                ts.transform.rotation = pose.orientation
                transforms.append(ts)

        self.pose_pub.publish(pa)
        if transforms:
            self.static_bcast.sendTransform(transforms)
        self.marker_pub.publish(self._markers(pa, normals, inferred, stamp))

    def _markers(self, pa: PoseArray, normals, inferred, stamp):
        ma = MarkerArray()
        # path line
        line = Marker()
        line.header.frame_id = self.output_frame
        line.header.stamp = stamp
        line.ns = "cleaning_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.004
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.8, 1.0, 1.0
        line.points = [p.position for p in pa.poses]
        ma.markers.append(line)
        # normal arrows
        for i, (pose, n) in enumerate(zip(pa.poses, normals)):
            arr = Marker()
            arr.header.frame_id = self.output_frame
            arr.header.stamp = stamp
            arr.ns = "normals"
            arr.id = i + 1
            arr.type = Marker.ARROW
            arr.action = Marker.ADD
            arr.scale.x, arr.scale.y, arr.scale.z = 0.004, 0.008, 0.0
            if bool(inferred[i]):
                # CAD-inferred (gap-filled) waypoint -> yellow.
                arr.color.r, arr.color.g, arr.color.b, arr.color.a = 1.0, 0.85, 0.1, 0.9
            else:
                # Measured waypoint -> red.
                arr.color.r, arr.color.g, arr.color.b, arr.color.a = 1.0, 0.2, 0.2, 0.9
            start = pose.position
            end = type(start)()
            end.x = start.x + 0.03 * float(n[0])
            end.y = start.y + 0.03 * float(n[1])
            end.z = start.z + 0.03 * float(n[2])
            arr.points = [start, end]
            ma.markers.append(arr)
        # Highlight CAD-inferred (gap-filled) waypoint positions as spheres.
        sph = Marker()
        sph.header.frame_id = self.output_frame
        sph.header.stamp = stamp
        sph.ns = "cad_filled"
        sph.id = 0
        sph.type = Marker.SPHERE_LIST
        sph.action = Marker.ADD
        sph.scale.x = sph.scale.y = sph.scale.z = 0.01
        sph.color.r, sph.color.g, sph.color.b, sph.color.a = 1.0, 0.85, 0.1, 1.0
        for pose, is_inf in zip(pa.poses, inferred):
            if bool(is_inf):
                sph.points.append(pose.position)
        ma.markers.append(sph)
        return ma


def main():
    rclpy.init()
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
