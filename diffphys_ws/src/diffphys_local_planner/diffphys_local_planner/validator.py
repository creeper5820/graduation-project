import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import time
import json
import sys

from nav_msgs.msg import Odometry, Path


def path_to_segments(path_pts):
    segs = []
    for i in range(len(path_pts) - 1):
        segs.append((path_pts[i], path_pts[i + 1]))
    return segs


def point_segment_dist(p, a, b):
    ab = b - a
    ap = p - a
    t = np.dot(ap, ab) / (np.dot(ab, ab) + 1e-10)
    t = np.clip(t, 0.0, 1.0)
    proj = a + t * ab
    return float(np.linalg.norm(p - proj)), float(t)


def progress_on_path(pos, path_pts, segs):
    best_dist = float('inf')
    best_idx = 0
    best_t = 0.0
    for i, (a, b) in enumerate(segs):
        d, t = point_segment_dist(pos, a, b)
        if d < best_dist:
            best_dist = d
            best_idx = i
            best_t = t
    frac = (best_idx + best_t) / max(len(segs), 1)
    return best_dist, frac


class TrajectoryValidator(Node):
    def __init__(self, duration=60.0, goal=None):
        super().__init__('trajectory_validator')
        self.duration = duration
        self.goal = np.array(goal) if goal else None

        self.path_pts = None
        self.path_segs = None

        self.odom_positions = []
        self.odom_times = []
        self.odom_yaws = []

        path_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, 'drone_0_global_path', self.path_cb, path_qos)
        self.create_subscription(Odometry, 'drone_0_visual_slam/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.5, self.check_cb)
        self.start_time = time.time()
        self.get_logger().info(f'TrajectoryValidator started, collecting for {duration}s')

    def path_cb(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
               for p in msg.poses]
        self.path_pts = np.array(pts)
        self.path_segs = path_to_segments(self.path_pts)
        self.get_logger().info(f'=== Global path received: {len(pts)} waypoints ===')
        for i, p in enumerate(pts):
            self.get_logger().info(f'  wp[{i}]: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})')

    def odom_cb(self, msg):
        now = time.time()
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom_positions.append(pos)
        self.odom_times.append(now)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaws.append(np.arctan2(siny, cosy))
        if len(self.odom_positions) == 1:
            self.get_logger().info(f'First odom: ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})')
            if self.path_pts is None:
                self.get_logger().warn('No path received yet! Path comparison will be unavailable.')

    def check_cb(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.analyze()
            rclpy.shutdown()

    def analyze(self):
        if len(self.odom_positions) < 10:
            self.get_logger().warn('Not enough odom data')
            return

        odom = np.array(self.odom_positions)
        n = len(odom)
        dt = self.odom_times[-1] - self.odom_times[0]

        result = {
            'total_samples': n,
            'duration_s': round(dt, 1),
            'start_position': [round(v, 2) for v in odom[0]],
            'final_position': [round(v, 2) for v in odom[-1]],
            'flight_distance': round(float(np.sum(np.linalg.norm(np.diff(odom, axis=0), axis=1))), 2),
            'avg_speed': round(float(np.sum(np.linalg.norm(np.diff(odom, axis=0), axis=1))) / dt, 3),
        }

        if self.goal is not None:
            final_dist = float(np.linalg.norm(odom[-1] - self.goal))
            result['goal_position'] = [round(v, 2) for v in self.goal]
            result['goal_distance'] = round(final_dist, 2)
            result['goal_reached'] = final_dist < 1.0

        if self.path_pts is not None and self.path_segs is not None:
            cross_tracks = []
            progresses = []
            for p in odom:
                ct, prog = progress_on_path(p, self.path_pts, self.path_segs)
                cross_tracks.append(ct)
                progresses.append(prog)

            result['mean_cross_track_m'] = round(float(np.mean(cross_tracks)), 3)
            result['max_cross_track_m'] = round(float(np.max(cross_tracks)), 3)
            result['median_cross_track_m'] = round(float(np.median(cross_tracks)), 3)
            result['final_progress'] = round(float(progresses[-1]), 3)
            result['max_progress'] = round(float(np.max(progresses)), 3)

            path_len = float(np.sum(np.linalg.norm(np.diff(self.path_pts, axis=0), axis=1)))
            result['path_length_m'] = round(path_len, 2)

            if len(self.odom_yaws) > 1:
                path_dir = self.path_pts[-1] - self.path_pts[0]
                path_yaw = np.arctan2(path_dir[1], path_dir[0])
                avg_yaw = np.mean(self.odom_yaws[-10:])
                yaw_err = abs(avg_yaw - path_yaw)
                if yaw_err > np.pi:
                    yaw_err = 2 * np.pi - yaw_err
                result['path_heading_rad'] = round(float(path_yaw), 3)
                result['drone_heading_rad'] = round(float(avg_yaw), 3)
                result['heading_error_rad'] = round(float(yaw_err), 3)

            sample_positions = []
            step = max(1, n // 20)
            for i in range(0, n, step):
                ct, prog = cross_tracks[i], progresses[i]
                sample_positions.append({
                    't': round(self.odom_times[i] - self.odom_times[0], 1),
                    'pos': [round(v, 2) for v in odom[i]],
                    'cross_track_m': round(ct, 2),
                    'progress': round(prog, 2),
                })
            sample_positions.append({
                't': round(dt, 1),
                'pos': [round(v, 2) for v in odom[-1]],
                'cross_track_m': round(cross_tracks[-1], 2),
                'progress': round(progresses[-1], 2),
            })
            result['trajectory_samples'] = sample_positions

        self.get_logger().info('=== TRAJECTORY VALIDATION ===')
        for k, v in result.items():
            if k != 'trajectory_samples':
                self.get_logger().info(f'  {k}: {v}')

        if 'trajectory_samples' in result:
            self.get_logger().info('  --- Trajectory Samples ---')
            for s in result['trajectory_samples']:
                self.get_logger().info(
                    f"    t={s['t']:5.1f}s  pos=({s['pos'][0]:7.2f},{s['pos'][1]:7.2f},{s['pos'][2]:5.2f})  "
                    f"xtrack={s['cross_track_m']:5.2f}m  progress={s['progress']:.2f}")

        with open('/workspace/simulation/validation_result.json', 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info('Saved to /workspace/simulation/validation_result.json')


def main():
    duration = 60.0
    goal = [15.0, 0.0, 1.0]
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    if len(sys.argv) > 4:
        goal = [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
    rclpy.init()
    node = TrajectoryValidator(duration, goal)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
