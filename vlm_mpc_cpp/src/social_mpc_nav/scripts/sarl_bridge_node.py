#!/usr/bin/env python3
"""
SARL Bridge Node for VISTA-MPC Integration

Loads a trained SARL (Socially Attentive RL) model and provides:
  1. Topic publisher (10 Hz): attention weights + V(s) for current state
  2. Service server: batch V(s) evaluation for MPC rollout terminal states

The attention weights indicate per-pedestrian importance for the MPC cost function.
The value function V(s) provides a learned social-awareness terminal cost.
"""

import sys
import os
import time
import math
import numpy as np
# Patch for NumPy >= 1.24 compatibility with transforms3d
if not hasattr(np, 'float'):
    np.float = float
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf_transformations import euler_from_quaternion

from social_mpc_nav.msg import SARLOutput
from social_mpc_nav.srv import EvaluateSARLBatch

# Attempt to import person_tracker messages; fall back to People2D
try:
    from person_tracker.msg import PersonInfoArray
    HAS_PERSON_TRACKER = True
except ImportError:
    HAS_PERSON_TRACKER = False

from social_mpc_nav.msg import People2D


# ---------------------------------------------------------------------------
# SARL model architecture (replicated from CrowdNav)
# ---------------------------------------------------------------------------

def mlp(input_dim, mlp_dims, last_relu=False):
    layers = []
    mlp_dims = [input_dim] + mlp_dims
    for i in range(len(mlp_dims) - 1):
        layers.append(nn.Linear(mlp_dims[i], mlp_dims[i + 1]))
        if i != len(mlp_dims) - 2 or last_relu:
            layers.append(nn.ReLU())
    return nn.Sequential(*layers)


class ValueNetwork(nn.Module):
    def __init__(self, input_dim, self_state_dim, mlp1_dims, mlp2_dims,
                 mlp3_dims, attention_dims, with_global_state,
                 cell_size, cell_num):
        super().__init__()
        self.self_state_dim = self_state_dim
        self.global_state_dim = mlp1_dims[-1]
        self.mlp1 = mlp(input_dim, mlp1_dims, last_relu=True)
        self.mlp2 = mlp(mlp1_dims[-1], mlp2_dims)
        self.with_global_state = with_global_state
        if with_global_state:
            self.attention = mlp(mlp1_dims[-1] * 2, attention_dims)
        else:
            self.attention = mlp(mlp1_dims[-1], attention_dims)
        self.cell_size = cell_size
        self.cell_num = cell_num
        mlp3_input_dim = mlp2_dims[-1] + self_state_dim
        self.mlp3 = mlp(mlp3_input_dim, mlp3_dims)
        self.attention_weights = None

    def forward(self, state):
        """
        Forward pass through SARL value network.

        :param state: tensor of shape (batch_size, num_humans, 13)
        :return: tensor of shape (batch_size, 1)
        """
        size = state.shape
        self_state = state[:, 0, :self.self_state_dim]
        mlp1_output = self.mlp1(state.view((-1, size[2])))
        mlp2_output = self.mlp2(mlp1_output)

        if self.with_global_state:
            global_state = torch.mean(
                mlp1_output.view(size[0], size[1], -1), 1, keepdim=True)
            global_state = global_state.expand(
                (size[0], size[1], self.global_state_dim)
            ).contiguous().view(-1, self.global_state_dim)
            attention_input = torch.cat([mlp1_output, global_state], dim=1)
        else:
            attention_input = mlp1_output

        scores = self.attention(attention_input).view(
            size[0], size[1], 1).squeeze(dim=2)

        # Masked softmax
        scores_exp = torch.exp(scores) * (scores != 0).float()
        weights = (scores_exp / torch.sum(
            scores_exp, dim=1, keepdim=True)).unsqueeze(2)
        self.attention_weights = weights[0, :, 0].data.cpu().numpy()

        features = mlp2_output.view(size[0], size[1], -1)
        weighted_feature = torch.sum(torch.mul(weights, features), dim=1)

        joint_state = torch.cat([self_state, weighted_feature], dim=1)
        value = self.mlp3(joint_state)
        return value


# ---------------------------------------------------------------------------
# Coordinate transform (from cadrl.py rotate())
# ---------------------------------------------------------------------------

def rotate_state(robot_px, robot_py, robot_vx, robot_vy, robot_radius,
                 robot_gx, robot_gy, robot_v_pref,
                 human_px, human_py, human_vx, human_vy, human_radius):
    """
    Transform world-frame joint state to agent-centric rotated coordinates.

    Returns 13D vector:
      [dg, v_pref, theta, radius, vx, vy, px1, py1, vx1, vy1, radius1, da, radius_sum]
    """
    dx = robot_gx - robot_px
    dy = robot_gy - robot_py
    rot = math.atan2(dy, dx)
    cos_rot = math.cos(rot)
    sin_rot = math.sin(rot)

    dg = math.sqrt(dx * dx + dy * dy)
    v_pref = robot_v_pref
    theta = 0.0  # holonomic convention

    # Robot velocity in rotated frame
    vx = robot_vx * cos_rot + robot_vy * sin_rot
    vy = robot_vy * cos_rot - robot_vx * sin_rot

    # Human position relative to robot, rotated
    rel_px = human_px - robot_px
    rel_py = human_py - robot_py
    px1 = rel_px * cos_rot + rel_py * sin_rot
    py1 = rel_py * cos_rot - rel_px * sin_rot

    # Human velocity in rotated frame
    vx1 = human_vx * cos_rot + human_vy * sin_rot
    vy1 = human_vy * cos_rot - human_vx * sin_rot

    da = math.sqrt(rel_px * rel_px + rel_py * rel_py)
    radius_sum = robot_radius + human_radius

    return [dg, v_pref, theta, robot_radius, vx, vy,
            px1, py1, vx1, vy1, human_radius, da, radius_sum]


def build_rotated_state_tensor(robot_state, human_states):
    """
    Build SARL input tensor from robot and human states.

    Args:
        robot_state: dict with keys px, py, vx, vy, radius, gx, gy, v_pref
        human_states: list of dicts with keys px, py, vx, vy, radius

    Returns:
        torch.Tensor of shape (1, num_humans, 13)
    """
    if not human_states:
        return None

    rows = []
    for h in human_states:
        row = rotate_state(
            robot_state['px'], robot_state['py'],
            robot_state['vx'], robot_state['vy'],
            robot_state['radius'],
            robot_state['gx'], robot_state['gy'],
            robot_state['v_pref'],
            h['px'], h['py'], h['vx'], h['vy'], h['radius']
        )
        rows.append(row)

    tensor = torch.Tensor([rows])  # shape: (1, num_humans, 13)
    return tensor


def build_batch_rotated_tensor(robot_states_list, human_states, goal_x, goal_y,
                               robot_radius, robot_v_pref, human_radius):
    """
    Build batched SARL input tensor for multiple robot states (terminal states).

    Args:
        robot_states_list: list of dicts with keys px, py, yaw, vx, vy
        human_states: list of dicts with keys px, py, vx, vy, radius
        goal_x, goal_y: navigation goal
        robot_radius, robot_v_pref: robot parameters
        human_radius: default human radius

    Returns:
        torch.Tensor of shape (batch_size, num_humans, 13)
    """
    if not human_states or not robot_states_list:
        return None

    batch = []
    for rs in robot_states_list:
        rows = []
        for h in human_states:
            row = rotate_state(
                rs['px'], rs['py'], rs['vx'], rs['vy'], robot_radius,
                goal_x, goal_y, robot_v_pref,
                h['px'], h['py'], h['vx'], h['vy'],
                h.get('radius', human_radius)
            )
            rows.append(row)
        batch.append(rows)

    return torch.Tensor(batch)  # shape: (batch_size, num_humans, 13)


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class SARLBridgeNode(Node):
    def __init__(self):
        super().__init__('sarl_bridge_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('robot_v_pref', 1.0)
        self.declare_parameter('human_default_radius', 0.3)
        self.declare_parameter('odom_topic',
                               '/task_generator_node/tiago_base/odom')
        self.declare_parameter('crowd_topic',
                               '/person_tracker/person_info')
        self.declare_parameter('output_topic', '/sarl/output')
        self.declare_parameter('service_name', '/sarl/evaluate_batch')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('marker_topic', '/sarl/attention_markers')
        self.declare_parameter('map_frame', 'map')

        model_path = self.get_parameter('model_path').value
        rate_hz = self.get_parameter('rate_hz').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.robot_v_pref = self.get_parameter('robot_v_pref').value
        self.human_radius = self.get_parameter('human_default_radius').value
        odom_topic = self.get_parameter('odom_topic').value
        crowd_topic = self.get_parameter('crowd_topic').value
        output_topic = self.get_parameter('output_topic').value
        service_name = self.get_parameter('service_name').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        marker_topic = self.get_parameter('marker_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        # Load SARL model
        self.model = self._load_model(model_path)
        self.get_logger().info(f'SARL model loaded from: {model_path}')

        # State storage
        self.latest_odom = None
        self.latest_people = []  # list of dicts: {name, px, py, vx, vy}

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic,
            self._on_odom, QoSProfile(depth=10))

        if HAS_PERSON_TRACKER:
            self.crowd_sub = self.create_subscription(
                PersonInfoArray, crowd_topic,
                self._on_person_info_array, QoSProfile(depth=10))
            self.get_logger().info(
                f'Subscribing to PersonInfoArray on {crowd_topic}')
        else:
            self.crowd_sub = self.create_subscription(
                People2D, crowd_topic,
                self._on_people2d, QoSProfile(depth=10))
            self.get_logger().warn(
                'person_tracker not found, using People2D messages')

        # Publishers
        self.sarl_pub = self.create_publisher(
            SARLOutput, output_topic, QoSProfile(depth=10))
        self.marker_pub = self.create_publisher(
            MarkerArray, marker_topic, QoSProfile(depth=10))

        # Service
        self.batch_srv = self.create_service(
            EvaluateSARLBatch, service_name, self._evaluate_batch_callback)

        # Timer for periodic inference
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'SARL bridge node started at {rate_hz} Hz')

    def _load_model(self, model_path):
        """Load the trained SARL ValueNetwork."""
        # Architecture from policy.config [sarl] section
        model = ValueNetwork(
            input_dim=13,
            self_state_dim=6,
            mlp1_dims=[150, 100],
            mlp2_dims=[100, 50],
            mlp3_dims=[150, 100, 100, 1],
            attention_dims=[100, 100, 1],
            with_global_state=True,
            cell_size=1,
            cell_num=4
        )

        if model_path and os.path.exists(model_path):
            model.load_state_dict(
                torch.load(model_path, map_location='cpu'))
            self.get_logger().info('Model weights loaded successfully')
        else:
            self.get_logger().error(
                f'Model file not found: {model_path}')

        model.eval()
        return model

    # --- Subscriber callbacks ---

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_odom = {
            'px': msg.pose.pose.position.x,
            'py': msg.pose.pose.position.y,
            'yaw': yaw,
            'vx': msg.twist.twist.linear.x * math.cos(yaw)
                  - msg.twist.twist.linear.y * math.sin(yaw),
            'vy': msg.twist.twist.linear.x * math.sin(yaw)
                  + msg.twist.twist.linear.y * math.cos(yaw),
        }

    def _on_person_info_array(self, msg):
        people = []
        for person in msg.persons:
            people.append({
                'name': str(person.person_id),
                'px': person.position.x,
                'py': person.position.y,
                'vx': person.velocity.x,
                'vy': person.velocity.y,
                'radius': self.human_radius,
            })
        self.latest_people = people

    def _on_people2d(self, msg):
        people = []
        for person in msg.people:
            people.append({
                'name': person.name,
                'px': float(person.x),
                'py': float(person.y),
                'vx': float(person.vx),
                'vy': float(person.vy),
                'radius': self.human_radius,
            })
        self.latest_people = people

    # --- Timer callback (topic publisher) ---

    def _timer_callback(self):
        if self.latest_odom is None or not self.latest_people:
            return

        odom = self.latest_odom
        robot_state = {
            'px': odom['px'], 'py': odom['py'],
            'vx': odom['vx'], 'vy': odom['vy'],
            'radius': self.robot_radius,
            'gx': self.goal_x, 'gy': self.goal_y,
            'v_pref': self.robot_v_pref,
        }

        t0 = time.monotonic()

        state_tensor = build_rotated_state_tensor(
            robot_state, self.latest_people)
        if state_tensor is None:
            return

        with torch.no_grad():
            value = self.model(state_tensor)

        elapsed_ms = (time.monotonic() - t0) * 1000.0

        # Build output message
        msg = SARLOutput()
        msg.stamp = self.get_clock().now().to_msg()
        msg.attention_weights = self.model.attention_weights.tolist()
        msg.person_names = [p['name'] for p in self.latest_people]
        msg.state_value = float(value.item())
        msg.robot_x = float(odom['px'])
        msg.robot_y = float(odom['py'])
        msg.robot_yaw = float(odom['yaw'])
        msg.inference_time_ms = float(elapsed_ms)
        msg.is_valid = True

        self.sarl_pub.publish(msg)

        # Publish RViz markers
        self._publish_attention_markers(
            self.latest_people, self.model.attention_weights,
            float(value.item()))

    # --- RViz visualization ---

    def _publish_attention_markers(self, people, attention_weights, state_value):
        """
        Publish per-person colored sphere markers + text labels for RViz.

        Color gradient: green (low attention) -> yellow -> red (high attention).
        Sphere size scales with attention weight.
        """
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, person in enumerate(people):
            attn = float(attention_weights[i]) if i < len(attention_weights) else 0.0

            # Sphere marker at person position
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = now
            sphere.ns = 'sarl_attention'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(person['px'])
            sphere.pose.position.y = float(person['py'])
            sphere.pose.position.z = 2.0  # above person's head
            sphere.pose.orientation.w = 1.0
            # Scale proportional to attention (min 0.2, max 0.8)
            scale = 0.2 + attn * 0.6
            sphere.scale.x = scale
            sphere.scale.y = scale
            sphere.scale.z = scale
            # Color: green->yellow->red based on attention
            sphere.color = self._attention_color(attn)
            sphere.lifetime.sec = 0
            sphere.lifetime.nanosec = 200000000  # 200ms
            marker_array.markers.append(sphere)

            # Text label above sphere
            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = 'sarl_attention_text'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(person['px'])
            text.pose.position.y = float(person['py'])
            text.pose.position.z = 2.5  # above sphere
            text.scale.z = 0.25  # text height
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{person.get('name', str(i))}: {attn:.2f}"
            text.lifetime.sec = 0
            text.lifetime.nanosec = 200000000
            marker_array.markers.append(text)

        # State value text at robot position
        if self.latest_odom is not None:
            value_text = Marker()
            value_text.header.frame_id = self.map_frame
            value_text.header.stamp = now
            value_text.ns = 'sarl_state_value'
            value_text.id = 0
            value_text.type = Marker.TEXT_VIEW_FACING
            value_text.action = Marker.ADD
            value_text.pose.position.x = float(self.latest_odom['px'])
            value_text.pose.position.y = float(self.latest_odom['py'])
            value_text.pose.position.z = 2.5
            value_text.scale.z = 0.3
            value_text.color.r = 0.3
            value_text.color.g = 0.8
            value_text.color.b = 1.0
            value_text.color.a = 1.0
            value_text.text = f"V(s)={state_value:.3f}"
            value_text.lifetime.sec = 0
            value_text.lifetime.nanosec = 200000000
            marker_array.markers.append(value_text)

        self.marker_pub.publish(marker_array)

    @staticmethod
    def _attention_color(attn):
        """Map attention weight [0,1] to green->yellow->red color."""
        color = ColorRGBA()
        color.a = 0.8
        if attn < 0.5:
            # Green to yellow
            color.r = float(attn * 2.0)
            color.g = 1.0
            color.b = 0.0
        else:
            # Yellow to red
            color.r = 1.0
            color.g = float(1.0 - (attn - 0.5) * 2.0)
            color.b = 0.0
        return color

    # --- Service callback (batch terminal state evaluation) ---

    def _evaluate_batch_callback(self, request, response):
        t0 = time.monotonic()

        num_rollouts = request.num_rollouts
        num_people = request.num_people

        if num_rollouts <= 0 or num_people <= 0:
            response.values = []
            response.inference_time_ms = 0.0
            response.success = False
            return response

        # Parse robot terminal states: 5 floats per rollout [x, y, yaw, vx, vy]
        robot_states_list = []
        for i in range(num_rollouts):
            base = i * 5
            x = request.robot_states[base]
            y = request.robot_states[base + 1]
            yaw = request.robot_states[base + 2]
            vx_body = request.robot_states[base + 3]
            vy_body = request.robot_states[base + 4]
            # Convert body-frame velocity to world-frame (holonomic)
            vx = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
            vy = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
            robot_states_list.append({
                'px': float(x), 'py': float(y),
                'vx': float(vx), 'vy': float(vy),
            })

        # Parse people states: 4 floats per person [x, y, vx, vy]
        human_states = []
        for i in range(num_people):
            base = i * 4
            human_states.append({
                'px': float(request.people_states[base]),
                'py': float(request.people_states[base + 1]),
                'vx': float(request.people_states[base + 2]),
                'vy': float(request.people_states[base + 3]),
                'radius': self.human_radius,
            })

        # Build batched tensor and run forward pass
        batch_tensor = build_batch_rotated_tensor(
            robot_states_list, human_states,
            request.goal_x, request.goal_y,
            self.robot_radius, self.robot_v_pref, self.human_radius
        )

        if batch_tensor is None:
            response.values = [0.0] * num_rollouts
            response.inference_time_ms = 0.0
            response.success = False
            return response

        with torch.no_grad():
            values = self.model(batch_tensor)  # (num_rollouts, 1)

        elapsed_ms = (time.monotonic() - t0) * 1000.0

        response.values = values.squeeze(1).tolist()
        response.inference_time_ms = float(elapsed_ms)
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SARLBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
