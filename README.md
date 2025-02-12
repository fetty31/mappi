# **Nano MPPIc 🏁**  

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

<br>

Nano MPPIc is a lightweight **Model Predictive Path Integral (MPPI)** controller for autonomous navigation. It efficiently computes kinematically feasible trajectories by sampling control inputs and evaluating them based on multiple cost functions (critics). This controller is designed for **real-time performance**, **obstacle avoidance**, and **smooth motion planning**, making it ideal for **embedded systems and resource-constrained platforms**.  

### **Distinctive Features**  
✅ **Kinematic Bicycle Model** available for vehicle-alike robots _(long. velocity + steering commands)_   
✅ **Stand alone _C++_ library** - Nano MPPIc is implemented as a stand-alone _C++_ library in order to be used outside the ROS framework without changes, making it more portable.   
✅ **ROS Noetic compatibility** - _C++_ plugin that integrates with `move_base` ___nav_core::BaseLocalPlanner___

---

## **Disclaimer**
_This project is based on the [nav2](https://docs.nav2.org/) MPPI controller, developed by [Alex](https://github.com/artofnothingness) in [mppic](https://github.com/artofnothingness/mppic). If you plan to use `Nano MPPIc` please make sure to give some love to [nav2](https://github.com/ros-navigation/navigation2) and [mppic](https://github.com/artofnothingness/mppic) projects, which greatly influenced this work._

## **Installation**  

### **Dependencies**  
<details open>
    <summary>Nano-MPPIc C++14 library:</summary>
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
        <a href="./nano_mppic/">nano_mppic</a>
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
mkdir build & cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install
```

Clone the repository and build the package:  
```bash
cd ~/catkin_ws/src
git clone https://gitlab.iri.upc.edu/mobile_robotics/botnet_project/vaive/nano_mppic.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make 
```

---

## **Quick Start**  

### 1. Nano MPPIc as Local Planner
Make sure your `move_base` configuration includes:
```xml
<param name="base_global_planner" value="nano_mppic/MPPIPlannerROS"/>
```
This allows `move_base` to call the _MPPIPlannerROS_ plugin, which wraps around the `Nano MPPIc` library.

### **2. Tuning Configuration Parameters**  
Modify [`config/nano_mppic_local_planner.yaml`](config/nano_mppic_local_planner.yaml) to adjust the controller behavior. Remember to add this to the `move_base` launch:
```xml
<rosparam file="$(find nano_mppic)/config/nano_mppic_local_planner.yaml" command="load"/>
```
---

## Approach
To-Do

## **Package Structure**  
```
nano_mppic/
│── cfg/                    # Dynamic Reconfigure definition
│── include/                # Header files for ROS functionality
│── nano_mppic/             # Nano MPPIc stand-alone C++ library
│── src/                    # nav_core::BaseLocalPlanner definition
│── CMakeLists.txt          # Build system configuration
│── nano_mppic_plugin.xml   # nav_core pluginlib definition
│── package.xml             # ROS package metadata
└── README.md               # This file
```

---