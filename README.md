# ROS2 UGV Project
<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216792254-87258637-fd16-405b-b3c5-17b48007a776.png"/>
</p>

---

## Description

An aluminum chassis unmanned groud vehicle (UGM) using ROS2.  The chassis and components are purchased as an existing kit from [yahboom](https://category.yahboom.net/).  The kit originally comes loaded with ROS Melodic and Ubuntu 18.04.  The OS will be updated and [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) utilized.  A ground control station will be created with [QT Creator](https://www.qt.io/product/development-tools).  All code will be written with Python and later optimized with C/C++.  I may add a secondary controller and seperate deterministic and AI code.

---
## Components
### Controller
Jetson Nano 4GB: A small, powerful board with a 128-core Maxwell GPU, perfect for AI and computer vision applications. [nvidia](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)

<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216793317-191b04b0-cc6a-494a-a0e5-7243bb77e929.png"/>
</p>

### Lidar
The Slamtec RPLIDAR A1 rotates 360 degrees using a laser to scan the environment.  This will be used for mapping, obstacle detection, and path planning. [Slamtec](https://www.slamtec.com/en/Lidar/A1)
<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216794038-e1df33cd-0daa-4bec-a96f-c886bc5ca6cc.png"
</p>


### 3D Camera
The Orbbec 3D camera provides three-dimensional data for the robot [Orbbec](https://orbbec3d.com/index/Product/info.html?cate=38&id=36)
<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216794140-11ecd091-677f-453e-a90a-2af1877be800.png"
</p>

---
## Kinematics
<p align="center">
<img src="https://user-images.githubusercontent.com/30501420/216794807-49920923-b720-4ad9-a612-b6342776a7b4.png"
</p>

[research reference](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)

---
## Tasks


* [ ] Update Nano OS
* [ ] Install ROS2
* [ ] URDF model and kinematics
* [ ] QT GUI ground station
* [ ] Optimize time dependent code with C/C++

 *purchase this kit ->* [yahboom](https://category.yahboom.net/collections/ros-robotics/products/rosmaster-x3?variant=39664834248788)

