# Go2-EDU-Mycobot-320-M5-Gazebo-Simulation-with-Navigation-and-MoveIt
This repository provides a comprehensive Gazebo simulation environment for the Unitree Go2 EDU quadruped robot and the Mycobot 320 M5 robotic arm. The simulation integrates ROS Navigation Stack for Go2 EDU's autonomous navigation and MoveIt! for Mycobot 320 M5's motion planning and control.

# Go2 EDU & Mycobot 320 M5: Gazebo Simulation with Navigation and MoveIt

This repository provides a comprehensive Gazebo simulation environment for the **Unitree Go2 EDU quadruped robot** and the **Mycobot 320 M5 robotic arm**. The simulation integrates **ROS Navigation Stack** for Go2 EDU's autonomous navigation and **MoveIt!** for Mycobot 320 M5's motion planning and control. This setup is ideal for researchers and developers working on mobile manipulation, collaborative robotics, or multi-robot systems.

---

## Table of Contents
1. [Dependencies](#1-dependencies)
2. [File Structure](#2-file-structure)
3. [Execution Flow](#3-execution-flow)
4. [Detailed Workflow](#4-detailed-workflow)
5. [Usage](#5-usage)
6. [Contributing](#6-contributing)
7. [License](#7-license)

---

## 1. Dependencies

Ensure the following dependencies are installed:

### ROS Packages
- **ROS Noetic**: The primary ROS distribution used in this project.
- **MoveIt!**: For motion planning and control of the Mycobot 320 M5.
  ```bash
  sudo apt-get install ros-noetic-moveit
  ```
- **Gazebo**: For simulation.
  ```bash
  sudo apt-get install ros-noetic-gazebo-ros-pkgs
  ```
- **Navigation Stack**: For Go2 EDU's autonomous navigation.
  ```bash
  sudo apt-get install ros-noetic-navigation
  ```

### Additional Dependencies
- **Unitree ROS Packages**: For Go2 EDU control and simulation.
  ```bash
  sudo apt-get install ros-noetic-unitree-ros
  ```

---

## 2. File Structure

The repository is organized as follows:

```
go2_mycobot_sim_2.0/
├── laser_filter/               # Laser filter configurations for Go2 EDU
├── config/                     # Configuration files for navigation and simulation
├── launch/                     # Launch files for Gazebo and ROS nodes
├── CMakeLists.txt              # CMake configuration for the package
├── package.xml                 # Package manifest
├── mycobot_sim_3.0/            # Mycobot 320 M5 simulation package
│   ├── gazebo_camera_sim/      # Camera simulation for Mycobot
│   ├── mycobot_description/    # URDF and XACRO files for Mycobot
│   ├── mycobot_moveit_config/  # MoveIt! configuration for Mycobot
│   │   ├── config/             # MoveIt! configuration files
│   │   ├── launch/             # Launch files for MoveIt!
│   │   ├── setup_assistant/    # MoveIt! setup assistant files
│   │   ├── CMakeLists.txt      # CMake configuration for MoveIt!
│   │   └── package.xml         # Package manifest for MoveIt!
│   ├── mycobot_moveit_cpp_ctrl/ # C++ control scripts for Mycobot
│   │   ├── src/                # Source files for C++ control
│   │   ├── CMakeLists.txt      # CMake configuration for C++ control
│   │   └── package.xml         # Package manifest for C++ control
│   ├── mycobot_moveit_py_ctrl/ # Python control scripts for Mycobot
│   │   ├── mycobot_moveit_py_image_sub/ # Image processing for Mycobot
│   │   ├── mycobot_moveit_py_launch/    # Launch files for Python control
│   │   └── README              # Documentation for Python control
├── command/                    # Command scripts for simulation
├── unitree_sim/                # Unitree Go2 EDU simulation package
│   ├── unitree_guide/          # Go2 EDU control scripts
│   ├── unitree_legged_msgs/    # Custom messages for Go2 EDU
│   ├── unitree_ros/            # ROS integration for Go2 EDU
│   ├── README                  # Documentation for Go2 EDU simulation
│   └── LICENSE                 # License file for Go2 EDU simulation
└── README.md                   # Main README file
```

---

## 3. Execution Flow

To run the simulation, follow these steps:

1. **Launch Gazebo Simulation**:
   ```bash
   roslaunch mycobot_moveit_config demo_gazebo.launch
   ```

2. **Switch to Superuser** (if required):
   ```bash
   sudo su
   ```

3. **Source the Workspace**:
   ```bash
   source /home/u20/catkin_ws/devel/setup.bash
   ```

4. **Start Go2 EDU Control**:
   ```bash
   rosrun unitree_guide junior_ctrl
   ```

5. **Launch Navigation Stack**:
   ```bash
   roslaunch unitree_move_base rvizMoveBase.launch
   ```

6. **Run Mycobot Motion Script**:
   ```bash
   rosrun mycobot_moveit_py_ctrl mycobot_moveit_py_search.py
   ```

---

## 4. Detailed Workflow

### Gazebo Simulation
- The `demo_gazebo.launch` file initializes the Gazebo environment, loading the Go2 EDU and Mycobot 320 M5 models. It also sets up the necessary ROS nodes for simulation.

### Go2 EDU Navigation
- The `unitree_move_base` package provides the navigation stack for Go2 EDU, including AMCL for localization and move_base for path planning. The `rvizMoveBase.launch` file visualizes the navigation process in RViz.

### Mycobot 320 M5 Control
- The `mycobot_moveit_config` package contains the MoveIt! configuration for the Mycobot 320 M5. The `mycobot_moveit_py_search.py` script controls the robotic arm's motion, utilizing MoveIt! for trajectory planning and execution.

---

## 5. Usage

### Go2 EDU Navigation
- The **Navigation Stack** is used for autonomous path planning and obstacle avoidance. It includes AMCL for localization and move_base for global and local planning.

### Mycobot 320 M5 Control
- **MoveIt!** is used for motion planning, inverse kinematics, and collision detection. The `mycobot_moveit_py_search.py` script controls the robotic arm's motion.

### Gazebo Simulation
- The simulation environment integrates both Go2 EDU and Mycobot 320 M5, allowing for realistic co-simulation scenarios.

---

## 6. Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Submit a pull request with a detailed description of your changes.

---

## 7. License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

For any questions or issues, please open an issue on the repository. Happy coding! 🚀
