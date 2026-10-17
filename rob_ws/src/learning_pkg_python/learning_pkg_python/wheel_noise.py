#!/usr/bin/env python3

# 目的是生成高斯噪音,在线速度和转向加速度加入噪声标准差
import copy  
import math  
import random  

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class WheelNoise(Node):
    def __init__(self):
        super().__init__("wheel_noise")

        # 实验参数
        self.declare_parameter("sigma_v", 0.02)  # 线速度噪声标准差，m/s
        self.declare_parameter("sigma_w", 0.02)  # 角速度噪声标准差，rad/s
        self.declare_parameter("seed", 42)  # 随机种子

        # 读取参数。
        self.sigma_v = float(self.get_parameter("sigma_v").value)
        self.sigma_w = float(self.get_parameter("sigma_w").value)
        seed = int(self.get_parameter("seed").value)

        # 判断标准差是不是非负有限数
        for value in (self.sigma_v, self.sigma_w):
            if not math.isfinite(value) or value < 0:
                raise ValueError("噪声标准差必须是非负有限数值")

        # 随机数生成器。
        # 相同种子、相同输入消息顺序，会产生相同随机数序列。
        self.rng = random.Random(seed)

        # 发布带噪声的轮速观测。
        self.publisher = self.create_publisher(
            Odometry,
            "/wheel/odometry",
            10,
        )

        # 订阅原始轮式里程计。
        # 每收到一条消息，执行一次add_noise。
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.add_noise,
            qos_profile_sensor_data,
        )
        # 输出噪音日志数据
        self.get_logger().info(
            f"轮速加噪节点启动："
            f"sigma_v={self.sigma_v} m/s，"
            f"sigma_w={self.sigma_w} rad/s，seed={seed}"
        )

    def add_noise(self, msg):
        noisy_msg = copy.deepcopy(msg)

        # 从均值0、指定标准差的高斯分布中各抽取一个随机数。
        noise_v = self.rng.gauss(0.0, self.sigma_v)
        noise_w = self.rng.gauss(0.0, self.sigma_w)

        # 测量值 = 原始轮速 + 随机测量误差。
        noisy_msg.twist.twist.linear.x += noise_v
        noisy_msg.twist.twist.angular.z += noise_w

        # Twist协方差是6×6矩阵，按行展开为36个元素。
        # 变量顺序：vx、vy、vz、wx、wy、wz。
        covariance = [0.0] * 36

        # 先给未使用的速度分量设置很大的不确定性。
        # 之后EKF只选择vx和wz，不融合这些未使用分量。
        for index in (0, 7, 14, 21, 28, 35):
            covariance[index] = 1e6

        # 下标0：vx的方差。
        covariance[0] = self.sigma_v ** 2

        # 下标35：wz的方差。
        covariance[35] = self.sigma_w ** 2

        noisy_msg.twist.covariance = covariance

        # pose仍是复制来的原始里程计位置，不是带噪声速度积分结果。
        # 用较大的位置/姿态方差标记它不作为本节点的定位观测。
        pose_covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            pose_covariance[index] = 1e6

        noisy_msg.pose.covariance = pose_covariance

        # 发布处理后的消息。
        self.publisher.publish(noisy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelNoise()

    try:
        rclpy.spin(node)  # 持续接收原始里程计。
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()