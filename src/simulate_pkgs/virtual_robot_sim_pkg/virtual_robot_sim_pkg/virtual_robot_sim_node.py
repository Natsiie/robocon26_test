#!/usr/bin/env python3
# virtual_robot_sim_pkg/virtual_robot_sim_pkg/virtual_robot_sim_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R


class VirtualRobotSimNode(Node):
    def __init__(self):
        super().__init__("virtual_robot_sim_node")

        # 声明参数
        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_topic", "/world_map"),
                ("cmd_topic", "/cmd_vel"),
                ("initialpose_topic", "/initialpose"),  # 添加初始位姿话题参数
                ("world_tf", "world"),
                ("robot_tf", "base_link"),
                ("init_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # x,y,z,roll,pitch,yaw
                ("init_orientation", [1.0, 0.0, 0.0, 0.0]),  # qw,qx,qy,qz
                ("update_rate", 100.0),  # Hz
                ("linear_velocity_scale", 1.0),
                ("angular_velocity_scale", 1.0),
                ("enable_6dof", False),
                ("use_quaternion_integration", False),
            ],
        )

        # 获取参数
        self.map_topic = self.get_parameter("map_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.initialpose_topic = self.get_parameter("initialpose_topic").value
        self.world_tf = self.get_parameter("world_tf").value
        self.robot_tf = self.get_parameter("robot_tf").value
        init_pose = self.get_parameter("init_pose").value
        init_orientation = self.get_parameter("init_orientation").value
        self.update_rate = self.get_parameter("update_rate").value
        self.linear_velocity_scale = self.get_parameter("linear_velocity_scale").value
        self.angular_velocity_scale = self.get_parameter("angular_velocity_scale").value
        self.enable_6dof = self.get_parameter("enable_6dof").value
        self.use_quaternion_integration = self.get_parameter(
            "use_quaternion_integration"
        ).value

        # 初始化机器人状态
        self.current_pose = {
            "x": init_pose[0],
            "y": init_pose[1],
            "z": init_pose[2],
            "roll": init_pose[3],
            "pitch": init_pose[4],
            "yaw": init_pose[5],
        }

        # 使用四元数表示姿态
        if len(init_orientation) >= 4:
            self.current_orientation = init_orientation  # [qw, qx, qy, qz]
        else:
            # 从欧拉角转换四元数
            self.current_orientation = self.euler_to_quaternion(
                self.current_pose["roll"],
                self.current_pose["pitch"],
                self.current_pose["yaw"],
            )

        self.current_cmd_vel = Twist()
        self.map_data = None
        self.map_info = None

        # TF相关变量
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_topic, self.cmd_vel_callback, 10
        )

        # 初始位姿订阅者
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )

        # 地图订阅者
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )

        # 创建机器人运动更新定时器
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_robot)

        self.get_logger().info(f"Virtual robot simulation node started")
        self.get_logger().info(f"Initial pose: {self.current_pose}")
        self.get_logger().info(f"Initial orientation: {self.current_orientation}")
        self.get_logger().info(f"6DOF enabled: {self.enable_6dof}")
        self.get_logger().info(f"Initialpose topic: {self.initialpose_topic}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]

    def quaternion_to_euler(self, q):
        """四元数转欧拉角"""
        w, x, y, z = q

        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def quaternion_multiply(self, q1, q2):
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]

    def quaternion_normalize(self, q):
        """四元数归一化"""
        norm = math.sqrt(sum(x * x for x in q))
        if norm == 0:
            return [1.0, 0.0, 0.0, 0.0]
        return [x / norm for x in q]

    def quaternion_rotate_vector(self, q, v):
        """使用四元数旋转向量"""
        # 四元数乘以向量
        w, x, y, z = q
        vx, vy, vz = v

        # q * v (虚数)
        qv_result = [
            -x * vx - y * vy - z * vz,
            w * vx + y * vz - z * vy,
            w * vy + z * vx - x * vz,
            w * vz + x * vy - y * vx,
        ]

        # (q * v) * q^(-1)
        return self.quaternion_multiply(qv_result, [w, -x, -y, -z])

    def map_callback(self, msg):
        """处理地图数据"""
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # self.get_logger().info(
        #     f"Received map: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}"
        # )

    def cmd_vel_callback(self, msg):
        """处理速度指令"""
        self.current_cmd_vel = msg

    def initialpose_callback(self, msg):
        """处理初始位姿设置"""
        try:
            # 提取位置信息
            pose = msg.pose.pose
            self.current_pose["x"] = pose.position.x
            self.current_pose["y"] = pose.position.y
            self.current_pose["z"] = pose.position.z

            # 提取四元数并转换为欧拉角
            q = [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ]

            # 归一化四元数
            q = self.quaternion_normalize(q)

            if self.enable_6dof:
                # 6DOF模式：使用完整的欧拉角
                roll, pitch, yaw = self.quaternion_to_euler(q)
                self.current_pose["roll"] = roll
                self.current_pose["pitch"] = pitch
                self.current_pose["yaw"] = yaw
            else:
                # 3DOF模式：只使用yaw角度
                roll, pitch, yaw = self.quaternion_to_euler(q)
                self.current_pose["yaw"] = yaw
                # 保持roll和pitch为0
                self.current_pose["roll"] = 0.0
                self.current_pose["pitch"] = 0.0

            # 更新四元数表示
            self.current_orientation = q

            self.get_logger().info(
                f"Initial pose updated via /initialpose: "
                f"x={self.current_pose['x']:.2f}, y={self.current_pose['y']:.2f}, "
                f"yaw={self.current_pose['yaw']:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing initialpose: {e}")

    def is_obstacle(self, x, y):
        """检查给定坐标是否为障碍物"""
        if self.map_info is None or self.map_data is None:
            return False

        # 将世界坐标转换为地图坐标
        map_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        # 检查坐标是否在地图范围内
        if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:
            return self.map_data[map_y, map_x] > 50
        else:
            return True

    def update_robot_6dof(self, dt):
        """6DOF机器人运动更新"""
        # 获取6DOF速度指令
        linear_vel_x = self.current_cmd_vel.linear.x * self.linear_velocity_scale
        linear_vel_y = self.current_cmd_vel.linear.y * self.linear_velocity_scale
        linear_vel_z = self.current_cmd_vel.linear.z * self.linear_velocity_scale

        angular_vel_roll = self.current_cmd_vel.angular.x * self.angular_velocity_scale
        angular_vel_pitch = self.current_cmd_vel.angular.y * self.angular_velocity_scale
        angular_vel_yaw = self.current_cmd_vel.angular.z * self.angular_velocity_scale

        # 方式1：使用四元数积分（推荐）
        if self.use_quaternion_integration:
            # 创建角速度四元数
            omega = [
                0,
                angular_vel_roll * dt,
                angular_vel_pitch * dt,
                angular_vel_yaw * dt,
            ]

            # 归一化
            omega_norm = math.sqrt(sum(x * x for x in omega))
            if omega_norm > 1e-8:
                omega = [x / omega_norm * math.sin(omega_norm / 2) for x in omega]
                omega[0] = math.cos(omega_norm / 2)
            else:
                omega = [1.0, 0.0, 0.0, 0.0]

            # 四元数积分：q_new = q * omega
            self.current_orientation = self.quaternion_multiply(
                self.current_orientation, omega
            )
            self.current_orientation = self.quaternion_normalize(
                self.current_orientation
            )

        else:
            # 方式2：使用欧拉角积分
            new_roll = self.current_pose["roll"] + angular_vel_roll * dt
            new_pitch = self.current_pose["pitch"] + angular_vel_pitch * dt
            new_yaw = self.current_pose["yaw"] + angular_vel_yaw * dt

            # 角度限制（可选）
            new_roll = math.atan2(math.sin(new_roll), math.cos(new_roll))
            new_pitch = math.atan2(math.sin(new_pitch), math.cos(new_pitch))
            new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

            # 更新欧拉角
            self.current_pose["roll"] = new_roll
            self.current_pose["pitch"] = new_pitch
            self.current_pose["yaw"] = new_yaw

            # 更新四元数
            self.current_orientation = self.euler_to_quaternion(
                new_roll, new_pitch, new_yaw
            )

        # 位置更新（考虑姿态影响）
        velocity_body = [linear_vel_x, linear_vel_y, linear_vel_z]
        velocity_world = self.quaternion_rotate_vector(
            self.current_orientation, velocity_body
        )

        # 积分位置
        self.current_pose["x"] += velocity_world[0] * dt
        self.current_pose["y"] += velocity_world[1] * dt
        self.current_pose["z"] += velocity_world[2] * dt

    def update_robot_3dof(self, dt):
        """3DOF机器人运动更新（平面运动）"""
        # 计算新位置
        linear_vel_x = self.current_cmd_vel.linear.x * self.linear_velocity_scale
        angular_vel_z = self.current_cmd_vel.angular.z * self.angular_velocity_scale

        # 计算新位姿
        new_yaw = self.current_pose["yaw"] + angular_vel_z * dt
        new_x = self.current_pose["x"] + linear_vel_x * math.cos(new_yaw) * dt
        new_y = self.current_pose["y"] + linear_vel_x * math.sin(new_yaw) * dt

        # 检查新位置是否在障碍物上
        if not self.is_obstacle(new_x, new_y):
            self.current_pose["x"] = new_x
            self.current_pose["y"] = new_y
            self.current_pose["yaw"] = new_yaw

        # 更新四元数（仅yaw）
        self.current_orientation = self.euler_to_quaternion(
            self.current_pose["roll"],
            self.current_pose["pitch"],
            self.current_pose["yaw"],
        )

    def update_robot(self):
        """更新机器人位置并发布TF"""
        dt = 1.0 / self.update_rate

        if self.enable_6dof:
            self.update_robot_6dof(dt)
        else:
            self.update_robot_3dof(dt)

        # 发布TF
        self.publish_tf()

    def publish_tf(self):
        """发布机器人base_link TF变换"""
        current_time = self.get_clock().now().to_msg()

        # 发布base_link TF
        t_base = TransformStamped()
        t_base.header.stamp = current_time
        t_base.header.frame_id = self.world_tf
        t_base.child_frame_id = self.robot_tf

        # 设置base_link的平移
        t_base.transform.translation.x = self.current_pose["x"]
        t_base.transform.translation.y = self.current_pose["y"]
        t_base.transform.translation.z = self.current_pose["z"]

        # 设置base_link的旋转（四元数）
        t_base.transform.rotation.w = self.current_orientation[0]
        t_base.transform.rotation.x = self.current_orientation[1]
        t_base.transform.rotation.y = self.current_orientation[2]
        t_base.transform.rotation.z = self.current_orientation[3]

        # 发布TF
        self.tf_broadcaster.sendTransform(t_base)

    def publish_lidar_tf(self, current_time):
        """发布激光雷达连杆的TF变换"""
        t_lidar = TransformStamped()
        t_lidar.header.stamp = current_time
        t_lidar.header.frame_id = self.robot_tf
        t_lidar.child_frame_id = "robot_lidar"

        # robot_lidar相对于base_link的固定变换
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.06
        t_lidar.transform.translation.z = 0.2

        # robot_lidar与base_link之间没有相对旋转（修复后）
        t_lidar.transform.rotation.w = 1.0
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = 0.0

        # 发布激光雷达 TF
        self.tf_broadcaster.sendTransform(t_lidar)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualRobotSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
