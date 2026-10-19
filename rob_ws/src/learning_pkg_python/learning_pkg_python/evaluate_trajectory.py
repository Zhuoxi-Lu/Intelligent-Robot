#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse 
import csv  
import json  
from pathlib import Path  

import numpy as np 
import matplotlib
matplotlib.use("Agg") 
import matplotlib.pyplot as plt


def read_csv(path, columns):
    """按指定列名读取数值，拒绝不完整或时间混乱的数据。"""
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)  # 每行转换成“列名:数值”的字典。
        missing = set(columns) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path.name} 缺少列：{sorted(missing)}")

        records = []
        for line_number, row in enumerate(reader, start=2):
            try:
                records.append([float(row[name]) for name in columns])
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"{path.name} 第{line_number}行不完整或不是有效数字。"
                    "请使用原始CSV，不要粘贴截断的数据。"
                ) from error

    data = np.asarray(records, dtype=float)  # 转换成二维数值数组。
    if len(data) < 2:
        raise ValueError(f"{path.name} 至少需要两条记录")
    if not np.isfinite(data).all():  # 轨迹数值不应出现nan或inf。
        raise ValueError(f"{path.name} 含有nan或inf")
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError(f"{path.name} 时间戳必须严格递增，不能重复或回退")
    return data


def draw_figures(output, elapsed, estimate, truth, position_error, yaw_error_deg):
    """绘图只使用已经完成对齐的数据，不额外修改轨迹。"""
    plt.rcParams.update({"font.size": 11, "savefig.dpi": 220})

    # 第一张图：真实轨迹与估计轨迹。图中文字使用英文，避免中文字体缺失。
    fig, ax = plt.subplots(figsize=(7.2, 6.2), constrained_layout=True)
    ax.plot(truth[:, 0], truth[:, 1], color="#1b6a9b", linewidth=2.4,
            label="Gazebo ground truth")
    ax.plot(estimate[:, 1], estimate[:, 2], "--", color="#d56324",
            linewidth=1.6, label="Dead reckoning estimate")
    ax.scatter(truth[0, 0], truth[0, 1], color="black", s=40,
               label="Ground truth start", zorder=5)
    ax.scatter(truth[-1, 0], truth[-1, 1], marker="s", color="#1b6a9b",
               s=55, label="Ground truth end", zorder=5)
    ax.scatter(estimate[-1, 1], estimate[-1, 2], marker="X", color="#d56324",
               s=60, label="Estimate end", zorder=5)
    ax.set(xlabel="x (m)", ylabel="y (m)",
           title="Estimated and ground truth trajectories")
    ax.set_aspect("equal", adjustable="datalim")  # XY一米显示为相同长度。
    ax.grid(alpha=0.25)
    ax.legend(fontsize=9)
    for extension in ("png", "pdf"):
        fig.savefig(output / f"trajectory_comparison.{extension}")
    plt.close(fig)

    # 第二张图：位置误差和有符号的朝向误差，横轴是仿真时间。
    fig, axes = plt.subplots(2, 1, figsize=(9, 6.3), sharex=True,
                             constrained_layout=True)
    axes[0].plot(elapsed, position_error, color="#d56324", linewidth=1.5)
    axes[0].set(ylabel="Position error (m)", title="Localization errors")
    axes[0].set_ylim(bottom=0)
    axes[1].plot(elapsed, yaw_error_deg, color="#1b6a9b", linewidth=1.5)
    axes[1].axhline(0, color="gray", linewidth=0.8)
    axes[1].set(ylabel="Signed yaw error (deg)",
                xlabel="Elapsed simulation time from selected start (s)")
    for ax in axes:
        ax.grid(alpha=0.25)
    for extension in ("png", "pdf"):
        fig.savefig(output / f"error_curves.{extension}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="航位推算轨迹误差计算与绘图")
    parser.add_argument("--estimate", type=Path, required=True,
                        help="estimated_baseline.csv的路径")
    parser.add_argument("--truth", type=Path, required=True,
                        help="ground_truth.csv的路径")
    parser.add_argument("--output", type=Path, required=True,
                        help="结果保存目录；建议每个实验窗口使用独立目录")
    parser.add_argument("--start", type=float, default=None,
                        help="可选：开始仿真时间，单位秒，不是相对运行时间")
    parser.add_argument("--end", type=float, default=None,
                        help="可选：结束仿真时间，单位秒")
    args = parser.parse_args()

    # 估计数组各列：时间、x、y、yaw。
    estimate = read_csv(args.estimate, [
        "sim_time_s", "x_est_m", "y_est_m", "yaw_est_rad",
    ])
    # 真值数组各列也整理为：时间、x、y、yaw。
    truth = read_csv(args.truth, ["sim_time_s", "x_m", "y_m", "yaw_rad"])

    # 只在两份记录共同覆盖的时间内比较，不把末端数据向后外推。
    overlap_start = max(estimate[0, 0], truth[0, 0])
    overlap_end = min(estimate[-1, 0], truth[-1, 0])
    start = overlap_start if args.start is None else args.start
    end = overlap_end if args.end is None else args.end
    if not np.isfinite([start, end]).all() or start >= end:
        raise ValueError("评估时间窗口必须有限，且开始时间小于结束时间")
    if start < overlap_start - 1e-9 or end > overlap_end + 1e-9:
        raise ValueError(
            f"指定窗口超出共同数据范围：{overlap_start:.6f}～{overlap_end:.6f}秒"
        )

    # 选择窗口内的估计点；不重新把位置或朝向归零。
    selected = estimate[(estimate[:, 0] >= start) & (estimate[:, 0] <= end)]
    if len(selected) < 2:
        raise ValueError("所选窗口内不足两个估计点")
    time = selected[:, 0]  # 本次比较采用的仿真时间戳。

    # 将真值插值到估计时间，避免把不同时刻的数据直接相减。
    truth_x = np.interp(time, truth[:, 0], truth[:, 1])
    truth_y = np.interp(time, truth[:, 0], truth[:, 2])
    # yaw可能在+pi与-pi处跳变，先展开为连续角度再插值。
    truth_yaw = np.interp(time, truth[:, 0], np.unwrap(truth[:, 3]))
    aligned_truth = np.column_stack([truth_x, truth_y, truth_yaw])

    # 相同时间、相同平面坐标参考下的位置差。
    dx = selected[:, 1] - truth_x
    dy = selected[:, 2] - truth_y
    position_error = np.hypot(dx, dy)  # 等价于sqrt(dx**2 + dy**2)。

    # 角度误差限制在[-pi, pi]，例如179度与-179度相差2度而非358度。
    yaw_difference = selected[:, 3] - truth_yaw
    yaw_error = np.arctan2(np.sin(yaw_difference), np.cos(yaw_difference))
    yaw_error_deg = np.degrees(yaw_error)  # 图和表用度，便于阅读。
    elapsed = time - time[0]  # 仅将图的时间轴从0开始，不修改轨迹坐标。

    # RMSE：先平方、再平均、最后开方；不是简单的平均距离。
    metrics = {
        "sample_count": len(time),
        "start_sim_time_s": float(time[0]),
        "end_sim_time_s": float(time[-1]),
        "duration_s": float(time[-1] - time[0]),
        "position_rmse_m": float(np.sqrt(np.mean(position_error ** 2))),
        "position_mean_m": float(np.mean(position_error)),
        "position_max_m": float(np.max(position_error)),
        "position_final_m": float(position_error[-1]),
        "yaw_rmse_deg": float(np.sqrt(np.mean(yaw_error_deg ** 2))),
        "yaw_max_abs_deg": float(np.max(np.abs(yaw_error_deg))),
        "yaw_final_signed_deg": float(yaw_error_deg[-1]),
        "yaw_final_abs_deg": float(abs(yaw_error_deg[-1])),
    }

    output = args.output
    output.mkdir(parents=True, exist_ok=True)  # 创建输出目录，不修改原始CSV。
    # 避免重复运行时不小心覆盖之前的结果。
    if any(output.iterdir()):
        raise ValueError("输出目录非空，请为本次运行指定一个新的目录")

    # 保存逐时刻数据，老师或你自己可以独立核对误差计算。
    with (output / "aligned_errors.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "sim_time_s", "elapsed_s", "x_est_m", "y_est_m", "yaw_est_rad",
            "x_truth_m", "y_truth_m", "yaw_truth_rad", "dx_m", "dy_m",
            "position_error_m", "yaw_error_rad", "yaw_error_deg",
        ])
        writer.writerows(zip(
            time, elapsed, selected[:, 1], selected[:, 2], selected[:, 3],
            truth_x, truth_y, truth_yaw, dx, dy, position_error,
            yaw_error, yaw_error_deg,
        ))

    # 保存汇总、输入时间范围和坐标假设，避免只剩图片无法解释方法。
    summary = {
        "metrics": metrics,
        "estimate_file": args.estimate.name,
        "truth_file": args.truth.name,
        "estimate_time_range_s": estimate[[0, -1], 0].tolist(),
        "truth_time_range_s": truth[[0, -1], 0].tolist(),
        "requested_window_s": [args.start, args.end],
        "method": "按仿真时间插值配对；朝向误差归一化；RMSE对配对样本等权计算。",
        "coordinate_assumption": (
            "输入已经使用同一平面原点与朝向。本实验初始位姿已知为零，"
            "模型原点与base_link平面XY对应；未进行轨迹拟合或真值修正。"
        ),
    }
    (output / "metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with (output / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "value"])
        writer.writerows(metrics.items())

    draw_figures(output, elapsed, selected, aligned_truth, position_error, yaw_error_deg)

    # 显示关键结果。其余详细数据保存在输出目录。
    print(f"评估区间：{time[0]:.3f}～{time[-1]:.3f}秒，共{len(time)}个点")
    print(f"位置RMSE：{metrics['position_rmse_m']:.6f} m")
    print(f"最大位置误差：{metrics['position_max_m']:.6f} m")
    print(f"末端位置误差：{metrics['position_final_m']:.6f} m")
    print(f"朝向RMSE：{metrics['yaw_rmse_deg']:.6f}度")
    print(f"末端朝向误差绝对值：{metrics['yaw_final_abs_deg']:.6f}度")
    print(f"结果目录：{output.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit(f"运行失败：{error}")
