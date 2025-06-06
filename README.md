# **🏁 MaPPI 🏁**  

<details>
    <summary>Table of Contents</summary>
    <ol>
        <li>
        <a href="#disclaimer">Disclaimer</a>
        </li>
        <li><a href="#installation">Installation</a>
        </li>
        <li>
        <a href="#quick-start">Quick Start</a>
        </li>
        <li>
        <a href="#approach">Approach</a>
        </li>
        <li>
        <a href="#package-structure">Package Structure</a>
        </li>
    </ol>
</details>

MaPPI is a lightweight **Model Predictive Path Integral (MPPI)** controller for autonomous navigation. It efficiently computes kinematically feasible trajectories by sampling control inputs and evaluating them based on multiple cost functions (critics). This controller is designed for **real-time performance**, **obstacle avoidance** and **smooth motion planning**, making it ideal for **embedded systems and resource-constrained platforms**.  

### **Distinctive Features**  
✅ **Kinematic Bicycle Model** available for vehicle-alike robots _(long. velocity + steering commands)_   
✅ **Stand alone _C++_ library** - MaPPI is implemented as a stand-alone _C++_ library in order to be used outside the ROS framework without changes, making it more portable.   
✅ **ROS Noetic compatibility** - _C++_ plugin that integrates with `move_base` ___nav_core::BaseLocalPlanner___

---

<div align="center">
  <img src="docs/ona_born_mappi.gif" alt="Mappi at born"  width="800"/>
  <small>
  <p> <a href="https://www.vaivelogistics.com/">ONA</a> prototype navigating through El Born - Barcelona using MaPPI. </p>
  </small>
</div>
<br/>

<div align="center">
  <img src="docs/demo_gz.gif" alt="Mappi at Gazebo"  width="800"/>
  <small>
  <p> Maneuverability of MaPPI at simulation. </p>
  </small>
</div>

## **Disclaimer**
_This project is based on the [nav2](https://docs.nav2.org/) MPPI controller, originally developed by [Alex](https://github.com/artofnothingness) in [mppic](https://github.com/artofnothingness/mppic) and adapted by [Steve Macenski](https://github.com/SteveMacenski) for [nav2](https://docs.nav2.org/). If you plan to use `MaPPI` please make sure to give some love to [nav2](https://github.com/ros-navigation/navigation2) and [mppic](https://github.com/artofnothingness/mppic) projects, which greatly influenced this work._

_`MaPPI` has been developed with the sole purpose of being able to use MPPI within ROS Noetic with different motion models than the ones offered in nav2. If you plan to use MPPI in ROS2 please use the official nav2 controller, which is a much better implementation (now optimized using Eigen)._
## **Installation**  

### **Dependencies**  
<details open>
    <summary>MaPPI C++14 library:</summary>
    <ol>
        <li>
        <a href="https://github.com/xtensor-stack/xtensor">xtensor</a>
        </li>
        <li>
        <a href="https://github.com/xtensor-stack/xtl.git">xtl</a>
        </li>
        <li>
        <a href="http://wiki.ros.org/costmap_2d">costmap_2d</a><i> (To-Do: erase this dependency, make it a templated library)</i>
        </li>
    </ol>
</details>

<details>
    <summary>ROS (Noetic) wrapper:</summary>
    <ol>
        <li>
        <a href="./mappi/">mappi</a>
        <li>
        <a href="http://wiki.ros.org/sensor_msgs">sensor_msgs</a>
        </li>
        <li>
        <a href="http://wiki.ros.org/geometry_msgs">geometry_msgs</a>
        </li>
        </li>
        <li><a href="http://wiki.ros.org/nav_msgs">nav_msgs</a>
        </li>
        <li>
        <a href="http://wiki.ros.org/std_msgs">std_msgs</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/tf2">tf2</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/costmap_2d">costmap_2d</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/dynamic_reconfigure">dynamic_reconfigure</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/nav_core">nav_core</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/std_srvs">std_srvs</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/navfn">navfn</a><i> (optional) </i>
        </li>
    </ol>
</details>

### **Build Instructions**  
Build dependencies
```sh
git clone https://github.com/xtensor-stack/xtl.git # install x template library (xtl)
cd xtl
cmake -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install
```
```sh
git clone https://github.com/xtensor-stack/xtensor.git # install xtensor library
cd xtensor
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install
```

Clone the repository and build the package:  
```bash
cd ~/catkin_ws/src
git clone https://gitlab.iri.upc.edu/mobile_robotics/botnet_project/vaive/mappi.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make 
```

---

## **Quick Start**  

### 1. MaPPI as Local Planner
Make sure your `move_base` configuration includes:
```xml
<param name="base_global_planner" value="mappi/MPPIPlannerROS"/>
```
This allows `move_base` to call the _MPPIPlannerROS_ plugin, which wraps around the stand-alone `MaPPI` library.

### **2. Tuning Configuration Parameters**  
Modify [`config/mappi_local_planner.yaml`](config/mappi_local_planner.yaml) to adjust the controller behavior. Remember to add this to your `move_base` launch:
```xml
<rosparam file="$(find mappi)/config/mappi_local_planner.yaml" command="load"/>
```
---

## Approach
To-Do

## **Package Structure**  
```
mappi/
│── cfg/                    # Dynamic Reconfigure definition
│── include/                # Header files for ROS functionality
│── mappi/                  # MaPPI stand-alone C++ library
│── src/                    # nav_core::BaseLocalPlanner definition
│── CMakeLists.txt          # Build system configuration
│── mappi_plugin.xml        # nav_core pluginlib definition
│── package.xml             # ROS package metadata
└── README.md               # This file
```

---