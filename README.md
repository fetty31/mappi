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
        <a href="#quick-start-ros1">Quick Start ROS1</a>
        </li>
        <li>
        <a href="#quick-start-ros2">Quick Start ROS2</a>
        </li>
        <li>
        <a href="#package-structure">Package Structure</a>
        </li>
    </ol>
</details>

MaPPI is a lightweight implementation of the **Model Predictive Path Integral (MPPI)** controller for autonomous navigation. It generates kinematically feasible trajectories by sampling control inputs and evaluating them against multiple cost functions (critics). Designed for **real-time operation**, **obstacle avoidance** and **smooth motion planning**, MaPPI is well-suited for **embedded systems and resource-limited platforms**.  

By now you'll be probably asking yourself: **What Makes MaPPI Different from the Nav2 MPPI Controller?** 

### **Distinctive Features**  
✅ **Kinematic Bicycle Model** – unlike the official Nav2 MPPI controller, MaPPI supports vehicle-like robots through a kinematic bicycle model (longitudinal velocity + steering inputs), not just differential/ackermann drive kinematics.  
✅ **Standalone _C++_ library** – MaPPI is implemented as a portable, standalone _C++_ library, enabling use outside the ROS ecosystem without modification.  
✅ **ROS Noetic support** – while Nav2 targets ROS 2, MaPPI also provides a _C++_ plugin for ROS 1 (`move_base`) via ___nav_core::BaseLocalPlanner___, ensuring compatibility with existing ROS 1 workflows.  

### **Other Features**
✅ **Model prediction is contained in _MotionModel_ class** – unlike the Nav2 MPPI controller, the trajectory generation is computed propagating the Motion Model ODE, which ensures direct compliance with your robot motion.
 

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

<details>
    <summary>ROS2 (Humble) wrapper:</summary>
    <ol>
        <li>
        <a href="./mappi/">mappi</a>
        <li>
        <a href="https://index.ros.org/p/sensor_msgs/">sensor_msgs</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/geometry_msgs/">geometry_msgs</a>
        </li>
        </li>
        <li><a href="https://index.ros.org/p/nav_msgs/">nav_msgs</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/std_msgs/">std_msgs</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/tf2/">tf2</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/visualization_msgs/">visualization_msgs</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/nav2_core/">nav2_core</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/nav2_common/">nav2_common</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/nav2_util/">nav2_util</a>
        </li>
        <li>
        <a href="https://index.ros.org/p/nav2_costmap_2d/">nav2_costmap_2d</a>
        </li>
    </ol>
</details>

### **Build Instructions**  
Install dependencies

Check the [official installation page](https://xtensor.readthedocs.io/en/latest/installation.html) or build from sources:

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
cd ~/your_workspace/src
git clone https://github.com/fetty31/mappi.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make # or colcon build for ROS2
```

---

## **Quick Start ROS1**  

### 1. MaPPI as Local Planner
Make sure your `move_base` configuration includes:
```xml
<param name="base_local_planner" value="mappi/MPPIPlannerROS"/>
```
This allows `move_base` to call the _MPPIPlannerROS_ plugin, which wraps around the stand-alone `MaPPI` library.

### **2. Tuning Configuration Parameters**  
Modify [`config/mappi_local_planner.yaml`](config/mappi_local_planner.yaml) to adjust the controller behavior. Remember to add this to your `move_base` launch:
```xml
<rosparam file="$(find mappi)/config/mappi_local_planner.yaml" command="load"/>
```
---

## **Quick Start ROS2**  

### MaPPI as Nav2 Controller
Make sure to add the plugin `mappi::MPPIcROS` in the nav2 config file (see `nav2_custom.yaml`):
```
controller_plugins: ["FollowPath"]
FollowPath:
      plugin: "mappi::MPPIcROS"
      MotionModel: "BicycleKin"
      GeneralSettings:
        num_iterations: 1
        batch_size: 1000
        time_steps: 100
        num_retry: 4
      ...
```

---

## **Package Structure**  
```
mappi/
│── cfg/                    # Dynamic Reconfigure definition (ROS1 only)
│── config/                 # Tuning config files
│── include/                # Header files for ROS functionality
│── mappi/                  # MaPPI stand-alone C++ library
│── src/                    # nav_core::BaseLocalPlanner definition
│── CMakeLists.txt          # Build system configuration
│── mappi_plugin.xml        # nav/nav2 pluginlib definition
│── package.xml             # ROS package metadata
└── README.md               # This file
```

---