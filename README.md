# ROS 2 Urban UGV Project

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216792254-87258637-fd16-405b-b3c5-17b48007a776.png"/>
</p>

---

## Description

A ROS 2-based urban unmanned ground vehicle (UGV) built on an aluminum chassis. The platform is derived from a Yahboom robot kit, originally designed with ROS Melodic and Ubuntu 18.04. This project modernizes the stack using **Ubuntu 22.04** (JetPack 6.2), **Jetson Orin Nano (upgraded to Super)**, and **ROS 2 Humble Hawksbill**.

System Overview and Roadmap
The robot will support 2D SLAM using LIDAR and vision-based 3D mapping for robust indoor and urban navigation. A modular ROS 2 package structure is used to keep components maintainable and scalable across multiple ground robot platforms.

The system is being developed in Python, with plans to optimize performance-critical modules in C++. A secondary MCU may be introduced to separate real-time deterministic control (e.g. motor commands, safety checks) from higher-level AI and perception tasks.

A web-based ground control station is planned using Flask to expose a REST API. This interface will allow remote operation, status monitoring, and visualization through a browser-based frontend. This approach improves portability across devices compared to a native desktop application like Qt.

Machine Learning and Semantic Mapping
The robot will incorporate machine learning for detecting and tracking objects, people, and features in the environment. These semantic detections will inform the navigation stack and may be used to:

Avoid dynamic obstacles (e.g. pedestrians, pets)

Recognize terrain or scene types (e.g. sidewalk, doorway)

Improve localization with visual landmarks

In parallel, semantic features will be stored in a GeoJSON-like format, enabling:

Persistent storage of object detections with spatial data

Integration into a semantic map

Inter-robot data sharing and contextual mission planning

Future integration with PostGIS is planned for outdoor robots, enabling advanced spatial reasoning, geofencing, and spatial queries from the ground control station.

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
