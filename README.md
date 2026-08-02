# Server Clean Robot Simulation

基于 ROS Noetic + Gazebo 的服务型清扫机器人仿真项目。

本项目实现了一个具备区域导航、路径点巡航、地图加载、`move_base` 导航控制以及任务触发机制的清扫机器人仿真系统。机器人可在 Gazebo 环境中运行，通过 ROS 话题接收启动信号，并按照预设区域顺序完成导航清扫任务。

---

## 1. 项目功能概述

当前项目已经实现以下功能：

- Gazebo 仿真环境启动；
- ROS 机器人模型加载；
- 激光雷达 `/scan` 数据发布；
- 里程计 `/odom` 数据发布；
- `move_base` 导航框架接入；
- DWA 局部路径规划；
- 全局路径规划；
- `/cmd_vel` 速度控制输出；
- 自动清扫主程序 `navigation_demo`；
- 基于 `/auto_slam` 话题的任务触发；
- 基于 `/apriltag_id` 话题的区域顺序选择；
- 地图文件 `map.yaml` 与 `map.pgm` 接入；
- 静态地图 `map_server` 加载测试；
- AMCL 定位启动文件规划；
- 导航栈总启动文件规划。

---

## 2. 运行环境

### 系统环境

- Ubuntu 20.04
- ROS Noetic
- Gazebo
- Catkin
- C++

### 主要 ROS 依赖

项目运行依赖以下 ROS 包：

```bash
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-dwa-local-planner
sudo apt install ros-noetic-tf
sudo apt install ros-noetic-robot-state-publisher
sudo apt install ros-noetic-joint-state-publisher
sudo apt install ros-noetic-gazebo-ros
sudo apt install ros-noetic-gazebo-ros-control

# Changelog

本文件用于记录 `server_clean_robot_simulation` 项目的主要开发进度、功能新增、问题修复和后续计划。

---

## v0.2.1 - 2026-08-02

### Added

- 新增项目运行说明文档内容；
- 新增静态地图文件接入说明；
- 新增 `map.yaml` 与 `map.pgm` 地图资源记录；
- 新增 `map_server` 地图加载测试流程；
- 新增 ROS Master 启动依赖说明；
- 新增完整仿真运行流程说明：
  - 启动 Gazebo；
  - 启动地图服务；
  - 启动 `move_base`；
  - 启动 `navigation_demo`；
  - 发布 `/auto_slam` 触发信号；
- 新增 AMCL 定位方案说明；
- 新增 TF 链路检查方法：
  - `map -> odom`
  - `odom -> base_footprint`
  - `base_footprint -> laser_link`
- 新增常用调试命令记录；
- 新增 Gitee 提交流程说明。

### Changed

- 调整地图文件说明，明确 `map.yaml` 中 `image` 字段建议使用相对路径：
  - 推荐：`image: map.pgm`
  - 避免：`image: /home/xxx/.../map.pgm`
- 调整仿真运行顺序，明确 `rosrun map_server` 前需要先启动 `roscore`；
- 调整导航启动说明，将基础模式和完整地图导航模式分开描述；
- 调整 `move_base` 使用说明，明确其需要 `/scan`、`/odom`、`/tf` 支持；
- 调整 `/auto_slam` 触发说明，明确主程序不读取键盘输入，而是监听 ROS 话题；
- 调整 AprilTag 模拟说明，明确 `/apriltag_id` 可用于测试区域顺序选择；
- 调整项目文档结构，使其更适合提交到 Gitee 进行版本管理。

### Fixed

- 修复 `map_server` 启动时报错无法连接 ROS Master 的问题定位：
  - 报错信息：
    ```text
    Failed to contact master at [localhost:11311]
    Connection refused
    ```
  - 原因：
    ```text
    roscore 未启动
    ```
  - 解决方式：
    ```bash
    roscore
    ```
- 修复对 `rosrun` 与 `roslaunch` 启动行为的说明：
  - `rosrun` 不会自动启动 ROS Master；
  - `roslaunch` 通常可自动启动 ROS Master；
- 修复对 `navigation_demo` 等待触发状态的说明；
- 修复对手动输入字符无法触发导航的误解；
- 修复地图文件路径跨用户不可用的问题说明；
- 修复 ROS 包路径与 launch 文件路径排查流程说明。

### Verified

已验证以下功能和现象：

- `navigation_demo` 可正常启动；
- `navigation_demo` 可成功连接 `move_base` action server；
- `/cmd_vel` publisher 创建成功；
- `/cmd_vel` 存在订阅者；
- Region1、Region2、Region3、Region4 路径点均成功初始化；
- 主程序可进入等待触发状态：
  ```text
  Navigation node started, waiting for trigger signal...


