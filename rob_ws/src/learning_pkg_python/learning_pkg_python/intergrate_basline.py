#!/usr/bin/env python3

import argparse  # 读取命令行参数。
import csv  
import math  
from pathlib import Path  


def main():
    # 创建命令行参数解析器。
    parser = argparse.ArgumentParser(
        description="根据轮式里程计速度计算无人工噪声的估计轨迹"
    )

    # 要求运行时指定输入文件。
    parser.add_argument(
        "--input",
        required=True,
        help="wheel_odom.csv的路径",
    )

    # 初始位置由已知实验设置提供，不从真值读取。
    # 从世界原点附近、朝向+x方向开始，因此默认都是0。
    parser.add_argument("--x0", type=float, default=0.0)
    parser.add_argument("--y0", type=float, default=0.0)
    parser.add_argument("--yaw0", type=float, default=0.0)

    args = parser.parse_args()  # 解析用户输入的参数。

    input_path = Path(args.input)  # 将输入路径转成Path对象。

    # 输出到原始记录所在目录
    output_path = input_path.parent / "estimated_baseline.csv"

    # 保存当前估计位置和朝向。
    x = args.x0
    y = args.y0
    yaw = args.yaw0

    # 保存上一条消息的时间与速度。
    previous_time = None
    previous_v = None
    previous_w = None

    start_time = None  # 用于计算从第一条记录开始经过了多少秒。
    row_count = 0  # 统计记录数量。
    has_motion = False  # 判断文件中是否包含非零运动指令对应的测量速度。

    # 同时打开输入文件和输出文件。
    with input_path.open(
        "r", newline="", encoding="utf-8-sig"
    ) as input_file, output_path.open(
        "x", newline="", encoding="utf-8"
    ) as output_file:

        
        reader = csv.DictReader(input_file) # 读取数据。
        writer = csv.writer(output_file)

        # 输出文件的表头。
        writer.writerow([
            "sim_time_s",
            "elapsed_s",
            "x_est_m",
            "y_est_m",
            "yaw_est_rad",
        ])

        # 逐行处理记录；第1行是表头，因此数据行号从2开始。
        for line_number, row in enumerate(reader, start=2):

            # 提取这一条消息的时间、前向线速度和转向角速度。
            try:
                current_time = float(row["sim_time_s"])
                current_v = float(row["vx_m_s"])
                current_w = float(row["wz_rad_s"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"CSV第{line_number}行缺少有效的时间或速度数据"
                ) from error

            # 时间和速度必须是有限数值，不能是nan或inf。
            if not all(math.isfinite(value) for value in (
                current_time, current_v, current_w
            )):
                raise ValueError(f"CSV第{line_number}行包含nan或inf")

            # 检查数据中有没有运动。
            if abs(current_v) > 1e-6 or abs(current_w) > 1e-6:
                has_motion = True

            if previous_time is None:
                # 第一条消息没有上一条时间，不能计算时间差。
                # 这里只记录已知初始位置。
                start_time = current_time

            else:
                # 使用消息的仿真时间计算时间差，不使用电脑时间。
                dt = current_time - previous_time

                # 实验中不要重置仿真时间，记录应严格按时间递增。
                if dt <= 0:
                    raise ValueError(
                        f"CSV第{line_number}行时间没有递增，"
                        "请检查是否在记录期间重置了仿真"
                    )

                # 假设上一条测得的速度在这段时间内保持不变。
                distance = previous_v * dt
                delta_yaw = previous_w * dt

                # 这段运动中间时刻的朝向。
                middle_yaw = yaw + delta_yaw / 2.0

                # 修正转弯时圆弧长度与两端直线位移之间的关系。
                # 转角接近0时，修正系数趋近1。
                half_turn = delta_yaw / 2.0

                if abs(half_turn) < 1e-8:
                    arc_factor = 1.0
                else:
                    arc_factor = math.sin(half_turn) / half_turn

                # 将位移分解到估计参考系的x和y方向。
                x += distance * arc_factor * math.cos(middle_yaw)
                y += distance * arc_factor * math.sin(middle_yaw)

                # 更新朝向。
                yaw += delta_yaw

                # 将朝向保持在-π到+π范围内。
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))

            # 保存当前时刻的估计位置。
            writer.writerow([
                current_time,
                current_time - start_time,
                x,
                y,
                yaw,
            ])

            # 保存这一条测量值，供下一段时间使用。
            previous_time = current_time
            previous_v = current_v
            previous_w = current_w
            row_count += 1

    print(f"处理完成，共{row_count}条记录")
    print(f"输出文件：{output_path.resolve()}")

    if row_count < 2:
        print("记录不足两条，无法形成轨迹。")

    if not has_motion:
        print("提示：未发现明显非零速度，请确认记录包含小车运动过程。")


if __name__ == "__main__":
    main()