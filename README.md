# ROS 2 Urban UGV Project

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216792254-87258637-fd16-405b-b3c5-17b48007a776.png"/>
</p>

---

## Description

A ROS 2-based urban unmanned ground vehicle (UGV) built on an aluminum chassis. The platform is derived from a Yahboom robot kit, originally designed with ROS Melodic and Ubuntu 18.04. This project modernizes the stack using **Ubuntu 20.04** (JetPack 5.x), **Jetson Orin Nano (upgraded to Super)**, and **ROS 2 Humble Hawksbill**.

The robot will support full 2D SLAM using LIDAR and vision-based 3D mapping. A modular ROS 2 package structure is used, and a **Qt-based ground control station** is planned. The system will initially be developed in **Python**, with performance-critical components later optimized in **C++**. A possible future addition is a secondary MCU for separating deterministic control from AI tasks.

---

## Components

### 🧠 Controller — Jetson Orin Nano (4GB Super Upgrade)

A powerful embedded platform with a 128-core Ampere GPU and quad-core Cortex-A78AE CPU. Ideal for AI, SLAM, and edge computing.

🔗 [NVIDIA Jetson Orin Nano](https://developer.nvidia.com/embedded/jetson-orin)

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216793317-191b04b0-cc6a-494a-a0e5-7243bb77e929.png"/>
</p>

---

### 🔦 LIDAR — Slamtec RPLIDAR A1

Performs 360° scanning of the environment using a laser rangefinder, used for SLAM, obstacle detection, and navigation.

🔗 [Slamtec RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216794038-e1df33cd-0daa-4bec-a96f-c886bc5ca6cc.png"/>
</p>

---

### 🎥 3D Camera — Orbbec Astra Pro

Provides RGB-D data for scene understanding and 3D mapping.

🔗 [Orbbec Astra](https://orbbec3d.com/index/Product/info.html?cate=38&id=36)

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216794140-11ecd091-677f-453e-a90a-2af1877be800.png"/>
</p>

---

## ⚙️ Kinematics

The robot uses a 4-wheel mecanum configuration for omnidirectional movement. Kinematic transformations are handled in the `motion_node`.

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216794807-49920923-b720-4ad9-a612-b6342776a7b4.png"/>
</p>

🔗 [Research reference (IJCA)](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)

---

## 📋 Tasks

- [x] Upgrade Jetson Nano to Orin Nano (Super)
- [x] Install ROS 2 Humble
- [x] Basic SLAM using `slam_toolbox` and `rplidar_ros`
- [ ] Clean up and refactor package layout
- [ ] Finalize URDF and TF tree
- [ ] Add 3D mapping with RTAB-Map
- [ ] Build ground control station using Qt
- [ ] Optimize critical path with C++

💡 *Kit source:* [Yahboom Rosmaster X3](https://category.yahboom.net/collections/ros-robotics/products/rosmaster-x3?variant=39664834248788)
