import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import torch
import torch.nn.functional as F
import cv_bridge

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from quadrotor_msgs.msg import PositionCommand

from .model import Model


class DiffPhysLocalPlanner(Node):
    def __init__(self):
        super().__init__('diffphys_local_planner')

        self.declare_parameter('checkpoint', '')
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('max_vel', 2.0)

        ckpt = self.get_parameter('checkpoint').value
        self.drone_id = self.get_parameter('drone_id').value
        self.max_vel = self.get_parameter('max_vel').value

        self.device = torch.device('cpu')
        self.model = Model(7 + 3, 6).to(self.device)
        state_dict = torch.load(ckpt, map_location=self.device)
        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.h = None
        self.get_logger().info(f'Loaded checkpoint: {ckpt}')

        self.odom_pos = np.zeros(3)
        self.odom_vel = np.zeros(3)
        self.odom_orient = np.array([1.0, 0.0, 0.0, 0.0])
        self.have_odom = False

        self.goal_pos = None
        self.waypoints = []
        self.current_wp = 0
        self.wp_threshold = 1.5
        self.segment_len = 3.0

        self.bridge = cv_bridge.CvBridge()

        odom_topic = f'drone_{self.drone_id}_visual_slam/odom'
        depth_topic = f'drone_{self.drone_id}_pcl_render_node/depth'
        cmd_topic = f'drone_{self.drone_id}_planning/pos_cmd'

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_cb, 10)

        depth_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_cb, depth_qos)

        self.cmd_pub = self.create_publisher(PositionCommand, cmd_topic, 50)
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, f'drone_{self.drone_id}_global_path', path_qos)
        self.cmd_timer = self.create_timer(0.005, self.cmd_cb)
        self.last_yaw = 0.0

        self.get_logger().info('DiffPhysLocalPlanner ready (waypoint mode)')

    def goal_cb(self, msg):
        goal = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.get_logger().info(f'Goal received: {goal}')
        if self.have_odom:
            self._generate_waypoints(goal)
        else:
            self.goal_pos = goal
        self.h = None

    def _generate_waypoints(self, goal_pos):
        start = self.odom_pos.copy()
        total_vec = goal_pos - start
        total_dist = np.linalg.norm(total_vec)
        if total_dist < 0.1:
            return
        n_segs = max(1, int(np.ceil(total_dist / self.segment_len)))
        self.waypoints = []
        for i in range(1, n_segs + 1):
            t = i / n_segs
            wp = start + total_vec * t
            self.waypoints.append(wp)
        self.current_wp = 0

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'
        pts = [start] + [wp for wp in self.waypoints]
        for pt in pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = float(pt[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints, '
                               f'first={self.waypoints[0]}, last={self.waypoints[-1]}, '
                               f'path published to drone_{self.drone_id}_global_path')

    def odom_cb(self, msg):
        self.odom_pos = np.array([msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z])
        self.odom_vel = np.array([msg.twist.twist.linear.x,
                                   msg.twist.twist.linear.y,
                                   msg.twist.twist.linear.z])
        self.odom_orient = np.array([msg.pose.pose.orientation.w,
                                      msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z])
        if not self.have_odom and self.goal_pos is not None:
            self._generate_waypoints(self.goal_pos)
        self.have_odom = True

    def depth_cb(self, msg):
        try:
            if msg.encoding in ('32FC1', 'passthrough'):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding == 'mono16':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16').astype(np.float32) / 1000.0
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.latest_depth = depth
        except Exception as e:
            self.get_logger().warn(f'Depth convert failed: {e}')

    def quat_to_rot(self, q):
        w, x, y, z = q
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        return R

    def cmd_cb(self):
        if not self.have_odom:
            return

        if not self.waypoints:
            cmd = PositionCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'world'
            cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            cmd.trajectory_id = 0
            cmd.position.x = float(self.odom_pos[0])
            cmd.position.y = float(self.odom_pos[1])
            cmd.position.z = float(self.odom_pos[2])
            cmd.velocity.x = 0.0
            cmd.velocity.y = 0.0
            cmd.velocity.z = 0.0
            cmd.acceleration.x = 0.0
            cmd.acceleration.y = 0.0
            cmd.acceleration.z = 0.0
            cmd.yaw = self.last_yaw
            cmd.yaw_dot = 0.0
            self.cmd_pub.publish(cmd)
            return

        if self.current_wp >= len(self.waypoints):
            target_pos = self.waypoints[-1]
            vel_ref = np.zeros(3)
        else:
            wp = self.waypoints[self.current_wp]
            dist_to_wp = np.linalg.norm(wp - self.odom_pos)
            if dist_to_wp < self.wp_threshold and self.current_wp < len(self.waypoints) - 1:
                self.current_wp += 1
                self.get_logger().info(f'WP {self.current_wp}/{len(self.waypoints)}: {self.waypoints[self.current_wp]}')

            target_pos = self.waypoints[self.current_wp]
            target_dir = target_pos - self.odom_pos
            dist = np.linalg.norm(target_dir)
            if dist > 0.1:
                vel_ref = target_dir / dist * min(dist, self.max_vel)
            else:
                vel_ref = np.zeros(3)

        R = self.quat_to_rot(self.odom_orient)

        local_v = R.T @ self.odom_vel
        local_target_v = R.T @ vel_ref
        r_z = R[:, 2]
        margin = 0.2

        state = np.concatenate([local_v, local_target_v, r_z, [margin]])
        state_t = torch.from_numpy(state).float().unsqueeze(0).to(self.device)

        if not hasattr(self, 'latest_depth'):
            depth_t = torch.zeros(1, 1, 12, 16, device=self.device)
        else:
            d = self.latest_depth
            if d.ndim == 2:
                d = d[np.newaxis, np.newaxis, :, :]
            elif d.ndim == 3:
                d = d[np.newaxis, :, :]
            d = torch.from_numpy(d).float().to(self.device)
            if d.shape[-1] != 64 or d.shape[-2] != 48:
                d = F.interpolate(d, size=(48, 64), mode='bilinear', align_corners=False)
            d = 3.0 / d.clamp(0.3, 24) - 0.6
            d = F.max_pool2d(d, 4, 4)
            depth_t = d

        with torch.no_grad():
            act, _, self.h = self.model(depth_t, state_t, self.h)

        act_np = act[0].cpu().numpy()
        a_pred = R @ act_np[:3]
        v_pred = R @ act_np[3:6]

        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'world'
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = 0

        cmd.position.x = float(target_pos[0])
        cmd.position.y = float(target_pos[1])
        cmd.position.z = float(target_pos[2])

        cmd.velocity.x = float(vel_ref[0])
        cmd.velocity.y = float(vel_ref[1])
        cmd.velocity.z = float(vel_ref[2])

        cmd.acceleration.x = float(a_pred[0] - v_pred[0])
        cmd.acceleration.y = float(a_pred[1] - v_pred[1])
        cmd.acceleration.z = float(a_pred[2] - v_pred[2])

        goal_dir = self.waypoints[-1] - self.odom_pos if self.waypoints else np.zeros(3)
        dist_goal = np.linalg.norm(goal_dir)

        if dist_goal < 1.5:
            self.get_logger().info('Goal reached, hovering')
            self.waypoints = []

        if self.waypoints:
            yaw = np.arctan2(goal_dir[1], goal_dir[0])
            max_yaw_change = np.pi * 0.05
            dyaw = yaw - self.last_yaw
            if dyaw > np.pi:
                dyaw -= 2 * np.pi
            elif dyaw < -np.pi:
                dyaw += 2 * np.pi
            dyaw = np.clip(dyaw, -max_yaw_change, max_yaw_change)
            yaw = self.last_yaw + dyaw
        else:
            yaw = self.last_yaw

        cmd.yaw = float(yaw)
        cmd.yaw_dot = float((yaw - self.last_yaw) / 0.005)
        self.last_yaw = yaw

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = DiffPhysLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
