#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS bag文件四旋翼轨迹可视化脚本
从bag文件中读取/visualizer/quadpos话题数据并绘制三维轨迹时序图
"""

import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os

def save_data_to_files(quad_exists, quad_pos, quad_time, load_exists, load_pos, load_time, bag_path):
    """
    将无人机和负载位置数据保存到txt文件
    
    Args:
        quad_exists (bool): 无人机数据是否存在
        quad_pos (np.array): 无人机位置数据
        quad_time (np.array): 无人机时间戳
        load_exists (bool): 负载数据是否存在
        load_pos (np.array): 负载位置数据
        load_time (np.array): 负载时间戳
        bag_path (str): bag文件路径
    """
    # 生成输出文件名（基于bag文件名）
    bag_dir = os.path.dirname(bag_path)
    bag_name = os.path.splitext(os.path.basename(bag_path))[0]
    
    print("\n正在保存数据文件...")
    
    # 保存无人机数据
    if quad_exists:
        quad_file = os.path.join(bag_dir, f"{bag_name}_quadrotor_trajectory.txt")
        quad_relative_time = quad_time - quad_time[0]
        
        with open(quad_file, 'w', encoding='utf-8') as f:
            # 写入文件头
            f.write("# 无人机位置轨迹数据\n")
            f.write("# Quadrotor Position Trajectory Data\n")
            f.write("# Generated from ROS bag file: {}\n".format(os.path.basename(bag_path)))
            f.write("# Topic: /visualizer/quadPos\n")
            f.write("# Total data points: {}\n".format(len(quad_pos)))
            f.write("# Format: Time(s) X(m) Y(m) Z(m)\n")
            f.write("# ----------------------------------------\n")
            
            # 写入数据
            for i in range(len(quad_pos)):
                f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                    quad_relative_time[i], 
                    quad_pos[i, 0], 
                    quad_pos[i, 1], 
                    quad_pos[i, 2]
                ))
        
        print(f"无人机轨迹数据已保存至: {quad_file}")
        print(f"  数据点数: {len(quad_pos)}")
        print(f"  时间范围: 0.000 - {quad_relative_time[-1]:.3f} 秒")
    
    # 保存负载数据
    if load_exists:
        load_file = os.path.join(bag_dir, f"{bag_name}_load_trajectory.txt")
        load_relative_time = load_time - load_time[0]
        
        with open(load_file, 'w', encoding='utf-8') as f:
            # 写入文件头
            f.write("# 负载位置轨迹数据\n")
            f.write("# Load Position Trajectory Data\n")
            f.write("# Generated from ROS bag file: {}\n".format(os.path.basename(bag_path)))
            f.write("# Topic: /visualizer/loadPos\n")
            f.write("# Total data points: {}\n".format(len(load_pos)))
            f.write("# Format: Time(s) X(m) Y(m) Z(m)\n")
            f.write("# ----------------------------------------\n")
            
            # 写入数据
            for i in range(len(load_pos)):
                f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                    load_relative_time[i], 
                    load_pos[i, 0], 
                    load_pos[i, 1], 
                    load_pos[i, 2]
                ))
        
        print(f"负载轨迹数据已保存至: {load_file}")
        print(f"  数据点数: {len(load_pos)}")
        print(f"  时间范围: 0.000 - {load_relative_time[-1]:.3f} 秒")
    
    if not quad_exists and not load_exists:
        print("没有数据需要保存")
    else:
        print("数据保存完成！")

def plot_quadrotor_trajectory(bag_file_path, quad_topic="/visualizer/quadPos", load_topic="/visualizer/loadPos"):
    """
    从ROS bag文件中读取四旋翼和负载位置数据并绘制时序图
    
    Args:
        bag_file_path (str): bag文件路径
        quad_topic (str): 无人机话题名称，默认为"/visualizer/quadPos"
        load_topic (str): 负载话题名称，默认为"/visualizer/loadPos"
    """
    
    # 检查文件是否存在
    if not os.path.exists(bag_file_path):
        print(f"错误：找不到bag文件 {bag_file_path}")
        return
    
    # 存储位置数据
    quad_positions = []
    quad_timestamps = []
    load_positions = []
    load_timestamps = []
    
    try:
        # 打开bag文件
        with rosbag.Bag(bag_file_path, 'r') as bag:
            print(f"正在读取bag文件: {bag_file_path}")
            print(f"无人机话题: {quad_topic}")
            print(f"负载话题: {load_topic}")
            
            # 获取bag文件信息
            info = bag.get_type_and_topic_info()
            topics = info.topics
            
            # 检查话题是否存在
            missing_topics = []
            if quad_topic not in topics:
                missing_topics.append(quad_topic)
            if load_topic not in topics:
                missing_topics.append(load_topic)
                
            if missing_topics:
                print(f"警告：在bag文件中未找到以下话题: {missing_topics}")
                print("可用话题列表:")
                for topic in topics:
                    print(f"  - {topic}")
                return
            
            # 读取指定话题的消息
            quad_message_count = 0
            load_message_count = 0
            
            def extract_position(msg):
                """提取位置信息的辅助函数"""
                if hasattr(msg, 'position'):
                    return msg.position.x, msg.position.y, msg.position.z
                elif hasattr(msg, 'pose'):
                    return msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
                elif hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
                    return msg.x, msg.y, msg.z
                else:
                    return None
            
            for topic, msg, t in bag.read_messages(topics=[quad_topic, load_topic]):
                try:
                    pos = extract_position(msg)
                    if pos is None:
                        print(f"话题 {topic} 的消息类型: {type(msg)}")
                        print(f"可用字段: {dir(msg)}")
                        break
                    
                    x, y, z = pos
                    
                    if topic == quad_topic:
                        quad_positions.append([x, y, z])
                        quad_timestamps.append(t.to_sec())
                        quad_message_count += 1
                        
                        if quad_message_count % 100 == 0:
                            print(f"已读取 {quad_message_count} 条无人机消息...")
                    
                    elif topic == load_topic:
                        load_positions.append([x, y, z])
                        load_timestamps.append(t.to_sec())
                        load_message_count += 1
                        
                        if load_message_count % 100 == 0:
                            print(f"已读取 {load_message_count} 条负载消息...")
                        
                except Exception as e:
                    print(f"解析话题 {topic} 消息时出错: {e}")
                    print(f"消息类型: {type(msg)}")
                    break
            
            print(f"总共读取了 {quad_message_count} 条无人机位置消息")
            print(f"总共读取了 {load_message_count} 条负载位置消息")
    
    except Exception as e:
        print(f"读取bag文件时出错: {e}")
        return
    
    if not quad_positions and not load_positions:
        print("未找到有效的位置数据")
        return
    
    # 处理无人机数据
    quad_data_exists = len(quad_positions) > 0
    load_data_exists = len(load_positions) > 0
    
    if quad_data_exists:
        quad_positions = np.array(quad_positions)
        quad_timestamps = np.array(quad_timestamps)
        quad_relative_time = quad_timestamps - quad_timestamps[0]
        print(f"无人机数据点数: {len(quad_positions)}")
    
    if load_data_exists:
        load_positions = np.array(load_positions)
        load_timestamps = np.array(load_timestamps)
        load_relative_time = load_timestamps - load_timestamps[0]
        print(f"负载数据点数: {len(load_positions)}")
    
    if not quad_data_exists and not load_data_exists:
        print("未找到任何有效的位置数据")
        return
    
    # 创建图形和子图 - 科研制图规范
    plt.style.use('default')  # 使用默认风格确保一致性
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    # fig.suptitle('Quadrotor Position Time Series', fontsize=16, fontweight='bold', y=0.95)
    
    # 设置科研制图参数
    plt.rcParams.update({
        'font.size': 12,
        'axes.linewidth': 1.0,
        'grid.linewidth': 0.5,
        'lines.linewidth': 2.0,
        'patch.linewidth': 0.5,
        'xtick.major.width': 1.0,
        'ytick.major.width': 1.0,
        'xtick.minor.width': 0.5,
        'ytick.minor.width': 0.5
    })
    
    # 定义颜色方案 - 使用专业的颜色
    quad_colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # 无人机：蓝色、橙色、绿色
    load_colors = ['#d62728', '#9467bd', '#8c564b']  # 负载：红色、紫色、棕色
    
    # 1. X方向时序图
    ax1 = axes[0]
    
    # 绘制无人机X轴数据
    if quad_data_exists:
        ax1.plot(quad_relative_time, quad_positions[:, 0], color=quad_colors[0], 
                linewidth=2.0, label='Quadrotor X', linestyle='-')
        # 添加起始点标记
        ax1.scatter(quad_relative_time[0], quad_positions[0, 0], color='green', s=60, 
                   marker='o', zorder=5, edgecolors='white', linewidth=1)
        ax1.scatter(quad_relative_time[-1], quad_positions[-1, 0], color='red', s=60, 
                   marker='s', zorder=5, edgecolors='white', linewidth=1)
    
    # 绘制负载X轴数据
    if load_data_exists:
        ax1.plot(load_relative_time, load_positions[:, 0], color=load_colors[0], 
                linewidth=2.0, label='Load X', linestyle='--')
        # 添加起始点标记
        ax1.scatter(load_relative_time[0], load_positions[0, 0], color='green', s=60, 
                   marker='^', zorder=5, edgecolors='white', linewidth=1)
        ax1.scatter(load_relative_time[-1], load_positions[-1, 0], color='red', s=60, 
                   marker='D', zorder=5, edgecolors='white', linewidth=1)
    
    ax1.set_ylabel('X Position (m)', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax1.set_title('X-axis Trajectory', fontsize=14, fontweight='bold', pad=10)
    
    # 设置X轴范围和刻度
    all_x_data = []
    if quad_data_exists:
        all_x_data.extend(quad_positions[:, 0])
    if load_data_exists:
        all_x_data.extend(load_positions[:, 0])
    
    if all_x_data:
        x_min, x_max = min(all_x_data), max(all_x_data)
        x_range = x_max - x_min if x_max != x_min else 1.0
        ax1.set_ylim(x_min - 0.1*x_range, x_max + 0.1*x_range)
    
    ax1.legend(loc='upper right', frameon=True, fancybox=True, shadow=True)
    
    # 2. Y方向时序图
    ax2 = axes[1]
    
    # 绘制无人机Y轴数据
    if quad_data_exists:
        ax2.plot(quad_relative_time, quad_positions[:, 1], color=quad_colors[1], 
                linewidth=2.0, label='Quadrotor Y', linestyle='-')
        # 添加起始点标记
        ax2.scatter(quad_relative_time[0], quad_positions[0, 1], color='green', s=60, 
                   marker='o', zorder=5, edgecolors='white', linewidth=1)
        ax2.scatter(quad_relative_time[-1], quad_positions[-1, 1], color='red', s=60, 
                   marker='s', zorder=5, edgecolors='white', linewidth=1)
    
    # 绘制负载Y轴数据
    if load_data_exists:
        ax2.plot(load_relative_time, load_positions[:, 1], color=load_colors[1], 
                linewidth=2.0, label='Load Y', linestyle='--')
        # 添加起始点标记
        ax2.scatter(load_relative_time[0], load_positions[0, 1], color='green', s=60, 
                   marker='^', zorder=5, edgecolors='white', linewidth=1)
        ax2.scatter(load_relative_time[-1], load_positions[-1, 1], color='red', s=60, 
                   marker='D', zorder=5, edgecolors='white', linewidth=1)
    
    ax2.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax2.set_title('Y-axis Trajectory', fontsize=14, fontweight='bold', pad=10)
    
    # 设置Y轴范围和刻度
    all_y_data = []
    if quad_data_exists:
        all_y_data.extend(quad_positions[:, 1])
    if load_data_exists:
        all_y_data.extend(load_positions[:, 1])
    
    if all_y_data:
        y_min, y_max = min(all_y_data), max(all_y_data)
        y_range = y_max - y_min if y_max != y_min else 1.0
        ax2.set_ylim(y_min - 0.1*y_range, y_max + 0.1*y_range)
    
    ax2.legend(loc='upper right', frameon=True, fancybox=True, shadow=True)
    
    # 3. Z方向时序图（高度）
    ax3 = axes[2]
    
    # 绘制无人机Z轴数据
    if quad_data_exists:
        ax3.plot(quad_relative_time, quad_positions[:, 2], color=quad_colors[2], 
                linewidth=2.0, label='Quadrotor Z', linestyle='-')
        ax3.fill_between(quad_relative_time, quad_positions[:, 2], alpha=0.2, color=quad_colors[2])
        # 添加起始点标记
        ax3.scatter(quad_relative_time[0], quad_positions[0, 2], color='green', s=60, 
                   marker='o', zorder=5, edgecolors='white', linewidth=1)
        ax3.scatter(quad_relative_time[-1], quad_positions[-1, 2], color='red', s=60, 
                   marker='s', zorder=5, edgecolors='white', linewidth=1)
    
    # 绘制负载Z轴数据
    if load_data_exists:
        ax3.plot(load_relative_time, load_positions[:, 2], color=load_colors[2], 
                linewidth=2.0, label='Load Z', linestyle='--')
        ax3.fill_between(load_relative_time, load_positions[:, 2], alpha=0.2, color=load_colors[2])
        # 添加起始点标记
        ax3.scatter(load_relative_time[0], load_positions[0, 2], color='green', s=60, 
                   marker='^', zorder=5, edgecolors='white', linewidth=1)
        ax3.scatter(load_relative_time[-1], load_positions[-1, 2], color='red', s=60, 
                   marker='D', zorder=5, edgecolors='white', linewidth=1)
    
    ax3.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Z Position (m)', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax3.set_title('Z-axis Trajectory (Altitude)', fontsize=14, fontweight='bold', pad=10)
    
    # 设置Z轴范围和刻度
    all_z_data = []
    if quad_data_exists:
        all_z_data.extend(quad_positions[:, 2])
    if load_data_exists:
        all_z_data.extend(load_positions[:, 2])
    
    if all_z_data:
        z_min, z_max = min(all_z_data), max(all_z_data)
        z_range = z_max - z_min if z_max != z_min else 1.0
        ax3.set_ylim(z_min - 0.1*z_range, z_max + 0.1*z_range)
    
    ax3.legend(loc='upper right', frameon=True, fancybox=True, shadow=True)
    
    # 统一设置所有子图的时间轴范围
    all_times = []
    if quad_data_exists:
        all_times.extend(quad_relative_time)
    if load_data_exists:
        all_times.extend(load_relative_time)
    
    if all_times:
        time_max = max(all_times)
        for ax in axes:
            ax.set_xlim(0, time_max * 1.02)
            ax.tick_params(axis='both', which='major', labelsize=10)
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.spines['left'].set_linewidth(1.0)
            ax.spines['bottom'].set_linewidth(1.0)
    
    plt.tight_layout()
    
    # 保存数据到txt文件
    save_data_to_files(quad_data_exists, quad_positions, quad_timestamps, 
                      load_data_exists, load_positions, load_timestamps, bag_file_path)
    
    # 显示统计信息
    print("\n轨迹统计信息:")
    
    if quad_data_exists:
        print(f"\n无人机数据:")
        print(f"  飞行时间: {quad_relative_time[-1]:.2f} 秒")
        print(f"  航点数量: {len(quad_positions)}")
        print(f"  X范围: [{quad_positions[:, 0].min():.2f}, {quad_positions[:, 0].max():.2f}] m")
        print(f"  Y范围: [{quad_positions[:, 1].min():.2f}, {quad_positions[:, 1].max():.2f}] m") 
        print(f"  Z范围: [{quad_positions[:, 2].min():.2f}, {quad_positions[:, 2].max():.2f}] m")
        
        # 计算无人机飞行距离
        quad_distances = np.sqrt(np.sum(np.diff(quad_positions, axis=0)**2, axis=1))
        quad_total_distance = np.sum(quad_distances)
        print(f"  飞行距离: {quad_total_distance:.2f} m")
    
    if load_data_exists:
        print(f"\n负载数据:")
        print(f"  运动时间: {load_relative_time[-1]:.2f} 秒")
        print(f"  航点数量: {len(load_positions)}")
        print(f"  X范围: [{load_positions[:, 0].min():.2f}, {load_positions[:, 0].max():.2f}] m")
        print(f"  Y范围: [{load_positions[:, 1].min():.2f}, {load_positions[:, 1].max():.2f}] m") 
        print(f"  Z范围: [{load_positions[:, 2].min():.2f}, {load_positions[:, 2].max():.2f}] m")
        
        # 计算负载运动距离
        load_distances = np.sqrt(np.sum(np.diff(load_positions, axis=0)**2, axis=1))
        load_total_distance = np.sum(load_distances)
        print(f"  运动距离: {load_total_distance:.2f} m")
    
    plt.show()

def main():
    """主函数"""
    # bag文件路径
    bag_file_path = "/bag_plot/bag/initial.bag"
    
    # 如果文件路径是相对路径，尝试在当前目录和上级目录中查找
    if not os.path.exists(bag_file_path):
        # 尝试相对于当前脚本的路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        alternative_paths = [
            os.path.join(script_dir, "initial.bag"),
            os.path.join(script_dir, "bag", "initial.bag"),
            os.path.join(script_dir, "bag_plot", "initial.bag"),
            "initial.bag"
        ]
        
        for path in alternative_paths:
            if os.path.exists(path):
                bag_file_path = path
                break
        else:
            print(f"错误：在以下位置都找不到bag文件:")
            print(f"  - {bag_file_path}")
            for path in alternative_paths:
                print(f"  - {path}")
            return
    
    # 绘制轨迹
    plot_quadrotor_trajectory(bag_file_path)

if __name__ == "__main__":
    main()
