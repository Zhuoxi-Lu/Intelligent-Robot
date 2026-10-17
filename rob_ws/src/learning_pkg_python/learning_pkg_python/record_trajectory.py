#!/usr/bin/env python3
# use to record the data from car/odom & ground_truth/odom

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry

def quaternation_to_yaw(q):
    # 四元数转弧度

    numerator = 2.0 * (q.w * q.z + q.x * q.y)
    denominator = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(numerator, denominator)  


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")  

        # create results
        self.declare_parameter("output_dir", "results")

        # 读取参数，并转换为路径对象
        output_dir = Path(
            self.get_parameter("output_dir").value
        ).expanduser()

        # use the computer time to name the document
        run_name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        self.run_dir = output_dir / run_name  # combine together
        self.run_dir.mkdir(parents=True, exist_ok=False)  # create catalog
        
        # 打开轮
        # self.odom_file = (self.run_dir / "wheel_odom.csv").open(
        #     "w", newline="", encoding="utf-8"
        # )
        
        # 打开带噪声轮
        self.odom_file = (self.run_dir / "wheel_odom.csv").open(
            "w", newline="", encoding="utf-8"
        )

        self.truth_file = (self.run_dir / "ground_truth.csv").open(
            "w", newline="", encoding="utf-8"
        )

        # init csvd
        self.odom_writer = csv.writer(self.odom_file)
        self.truth_writer = csv.writer(self.truth_file)

        columns = [
            "sim_time_s",       # 消息对应的仿真时间，秒。
            "frame_id",         # 参考坐标系。
            "child_frame_id",   # 被描述的坐标系。
            "x_m",              # x位置，米。
            "y_m",              # y位置，米。
            "z_m",              # z位置，米。
            "yaw_rad",          # 平面朝向，弧度。
            "vx_m_s",           # 消息中的车体x方向线速度，米/秒。
            "vy_m_s",           # 消息中的车体y方向线速度，米/秒。
            "wz_rad_s",         # 绕z轴角速度，弧度/秒。
        ]

        self.odom_writer.writerow(columns)  # write odom document
        self.truth_writer.writerow(columns)  # wirte real data document

        self.odom_count = 0  
        self.truth_count = 0 

        # # subscribe odom topic
        # 记录原始轮速，作为航位推算的输入
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     "/odom",
        #     self.record_odom,
        #     10,
        # )

        # 记录带噪声的轮速，作为航位推算的输入 
        self.odom_sub = self.create_subscription(
            Odometry,
            "/wheel/odometry",
            self.record_odom,
            10,
        )

        # subscribe real data
        self.truth_sub = self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.record_truth,
            10,
        )

        self.get_logger().info(f"数据保存目录：{self.run_dir.resolve()}")
        self.get_logger().info("开始记录。请随后启动autodrive，结束后按Ctrl+C。")

    def message_to_row(self, msg):
        """translate ROS message into row"""
    
        # 仿真时间由整数秒和纳秒组成。
        # 1纳秒=10^-9秒，因此乘以1e-9转换成秒。
        stamp = msg.header.stamp
        sim_time = stamp.sec + stamp.nanosec * 1e-9

        position = msg.pose.pose.position  # 取出位置。
        orientation = msg.pose.pose.orientation  # 取出姿态四元数。
        velocity = msg.twist.twist  # 取出速度。

        # 顺序必须与上面的CSV表头一致。
        return [
            sim_time,
            msg.header.frame_id,
            msg.child_frame_id,
            position.x,
            position.y,
            position.z,
            quaternation_to_yaw(orientation),
            velocity.linear.x,
            velocity.linear.y,
            velocity.angular.z,
        ]

    def record_odom(self, msg):
        """only record odom"""

        self.odom_writer.writerow(self.message_to_row(msg))  # 写入一行。
        self.odom_file.flush()  # 将缓冲数据写入文件。
        self.odom_count += 1  # 累加记录数量。

    def record_truth(self, msg):
        """record truth data"""

        self.truth_writer.writerow(self.message_to_row(msg))
        self.truth_file.flush()
        self.truth_count += 1

    def close_files(self):
        """结束记录并关闭文件。"""

        self.odom_file.close()
        self.truth_file.close()

        self.get_logger().info(
            f"记录结束：里程计{self.odom_count}条，真值{self.truth_count}条。"
        )


def main(args=None):
    rclpy.init(args=args)  
    node = TrajectoryRecorder()  

    try:
        rclpy.spin(node)  # 持续接收消息并执行对应回调。
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C或ROS关闭时，进入下面的清理流程。
    finally:
        node.close_files()  
        node.destroy_node()  

        if rclpy.ok():
            rclpy.shutdown()  


if __name__ == "__main__":
    main() 