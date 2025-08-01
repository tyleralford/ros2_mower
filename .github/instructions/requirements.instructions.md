---
applyTo: "**"
---
---

# **PRD: Robotic Reel Mower URDF & Control Package**

## **1. Project Overview**

This document outlines the requirements for creating a comprehensive ROS 2 package for a custom-built robotic reel lawnmower. The primary objective is to develop a precise and robust digital twin of the mower, encapsulated in a URDF (Unified Robot Description Format) file. This package will serve as the foundational layer for both high-level autonomous navigation using ROS 2 Nav2 and precise real-time teleoperation. The project includes full integration with `ros2_control` for a seamless transition between a high-fidelity Gazebo simulation and the physical hardware, which uses O-Drive controllers for motor control.

## **2. Guiding Principles & Development Philosophy**

This project will be executed following a set of core principles to ensure efficiency, clarity, and maintainability. All development work must adhere to the following directives:

*   **Minimalist Implementation:** Always provide the most straightforward and minimalist solution possible. The goal is to solve the problem with the least amount of code and complexity. Avoid premature optimization or over-engineering.
*   **Clarity and Readability:** Prioritize simple, readable, and maintainable code over complex, "clever," or obscure solutions. All work must be precise, to the point, and free from unnecessary filler.
*   **Adherence to Best Practices:** All architectural patterns and solutions must align with widely accepted ROS 2 and industry best practices. Avoid experimental or unproven approaches.
*   **Minimize Code Churn:** When adding a new feature or making a modification, alter the absolute minimum amount of existing code required to implement the change successfully.
*   **Meaningful Version Control:** Commits must be atomic. Commit messages will be short, concise, and descriptive, following conventional standards.
*   **Data-Driven Decisions:** When information is version-dependent, time-sensitive, or requires specific external data (e.g., library documentation, API details), prioritize using tools to find the current, factual answer over relying on general knowledge.

## **3. Core Requirements**

| ID | Requirement | Description |
| :-- | :--- | :--- |
| **REQ-01** | **Unified Robot Model** | A single, master URDF file shall define the robot's physical structure, sensor layout, and control interfaces for both simulation and hardware. |
| **REQ-02** | **Dual-Mode Operation** | The package must support two primary modes of operation, selectable via a launch-file argument: **Simulation** and **Hardware**. |
| **REQ-03** | **Simulation Fidelity** | The simulation must accurately model the robot's kinematics and dynamics, and provide realistic sensor data, including noise and drift, to facilitate robust algorithm development. |
| **REQ-04** | **Hardware Abstraction** | The control system shall use `ros2_control` to abstract the hardware specifics, allowing high-level nodes to control the robot with identical interfaces in both simulation and hardware modes. |
| **REQ-05** | **Differential Drive Control** | The drive system must be controlled via a standard `diff_drive_controller`, accepting `geometry_msgs/Twist` commands on a `/cmd_vel` topic. |
| **REQ-06** | **Independent Reel Control** | The cutting reel motor must be controlled independently of the drive system, allowing for its RPM to be set via a dedicated ROS 2 interface. |

## **4. Core Features**

### **4.1. URDF Model**
A detailed URDF will be created, containing:
*   **Links:** Geometrically and physically accurate representations of all robot components (chassis, wheels, reel, etc.).
*   **Joints:** Correctly defined joints connecting all links, with appropriate types (continuous, fixed) and axes of rotation.
*   **Physical Properties:** Mass and inertial properties for each link to ensure realistic physics simulation. Initial inertial tensors will be calculated based on primitive shapes (box, cylinder).
*   **Sensors:** Dedicated links for the IMU and GPS sensors, placed and oriented accurately relative to the robot's `base_link`.

### **4.2. Control System**
*   **`ros2_control` Integration:** The URDF will contain the `<ros2_control>` tag to define the hardware interfaces.
*   **Differential Drive:** The `ros2_controllers/diff_drive_controller` will be used for the two drive wheels. It will manage the conversion of `/cmd_vel` messages into individual wheel velocity commands and publish odometry based on wheel encoder feedback.
*   **Reel Control:** A `joint_trajectory_controller` will be configured to manage the reel motor, accepting velocity (RPM) commands.

### **4.3. Gazebo Simulation Environment**
*   **Physics:** A standard rigid body simulation will be used.
*   **Control Interface:** The `gazebo_ros2_control` plugin will be used to simulate the `ros2_control` hardware interfaces, allowing the same controllers to be used as on the real hardware.
*   **IMU Simulation:** A Gazebo IMU plugin will be configured to provide `sensor_msgs/Imu` data with realistic noise and drift models.
*   **GPS Simulation:** The simulation will generate two distinct topics to mimic the real hardware:
    *   A `gazebo_ros_gps_sensor` plugin will publish `sensor_msgs/NavSatFix` messages on `/gps/fix`.
    *   A secondary mechanism (e.g., ground truth pose processed by a noise-adding node) will publish a heading message on `/gps/heading`.

### **4.4. Hardware Abstraction & Launch**
*   **O-Drive Driver:** The `Factor-Robotics/odrive_ros2_control` hardware interface will be used to communicate with the physical O-Drive controllers.
*   **Device Identification:** The system will use O-Drive serial numbers to ensure the correct motors are mapped to the correct controllers, supported by `udev` rules for robust device naming.
*   **Unified Launch System:** A top-level launch file (`mower.launch.py`) will be created with a `use_sim_time` argument.
    *   `use_sim_time:=true`: Launches Gazebo and the simulation-specific configurations.
    *   `use_sim_time:=false`: Launches the O-Drive hardware interface for physical operation.

## **5. Core Components (Detailed Breakdown)**

### **5.1. Robot Physical Structure (Links & Joints)**

The origin (`base_link`) is defined at ground level, centered between the two drive wheels.
**Coordinate System:** +X is forward, +Y is left, +Z is up.

| Link Name | Description | Mass (kg) | Dimensions / Position | Joint to Parent |
| :--- | :--- | :--- | :--- | :--- |
| `base_link` | The central reference frame of the robot. | - | Origin (0, 0, 0) | - |
| `chassis` | The main body of the robot. | 2.27 | L:19cm, W:44cm, H:22cm. Sits on axle. | Fixed to `base_link` |
| `counterweight` | Rear counterweight for balance. | 9.07 | L:21cm, W:13cm, H:13cm. Attached to rear. | Fixed to `chassis` |
| `right_wheel` | Right drive wheel with hub motor. | 1.81 | Ø19.5cm, W:3.6cm. At (0, -23cm, 9.75cm) | Continuous, revolute |
| `left_wheel` | Left drive wheel with hub motor. | 1.81 | Ø19.5cm, W:3.6cm. At (0, 23cm, 9.75cm) | Continuous, revolute |
| `front_roller` | Front caster roller. | 0.45 | Ø8.89cm, W:33cm. At (48cm, 0, 4.445cm) | Continuous, revolute |
| `reel` | The cutting reel assembly. | 11.34 | Ø17.5cm, W:47cm. At (30cm, 0, 8.75cm) | Continuous, revolute |
| `reel_motor` | The motor driving the reel. | (est. 1kg) | At (21.8cm, 23.5cm, 17.55cm) | Fixed to `chassis` |

### **5.2. Sensor Suite**

| Sensor | Mount Link | Position Relative to `base_link` (x, y, z) | Orientation Notes |
| :--- | :--- | :--- | :--- |
| **IMU** | `imu_link` | (7.75cm, 8.75cm, 16.9cm) | Aligned with ROS standard (+X Fwd, +Y Left, +Z Up) |
| **GPS** | `gps_link` | (0cm, 21.415cm, 25.3cm) | Heading baseline is along the robot's +Y axis. |

### **5.3. `ros2_control` Configuration**

*   **URDF `<ros2_control>` Tag:**
    *   **Hardware Plugin:** `odrive_ros2_control/OdriveHardwareInterface`
    *   **Parameters:**
        *   Right Wheel O-Drive: `serial_number: 206334624633`
        *   Left Wheel O-Drive: `serial_number: 206334624633`
        *   Reel Motor O-Drive: `serial_number: 355E37703033`
*   **Joint Interfaces:**
    *   `right_wheel_joint`: Command `velocity`, State `velocity`.
    *   `left_wheel_joint`: Command `velocity`, State `velocity`.
    *   `reel_joint`: Command `velocity`, State `velocity`.
*   **Controller YAML File (`controllers.yaml`):**
    *   **`diff_drive_controller`:**
        *   `left_wheel_name`: `left_wheel_joint`
        *   `right_wheel_name`: `right_wheel_joint`
        *   `wheel_separation`: 0.46 m
        *   `wheel_radius`: 0.0975 m
    *   **`joint_trajectory_controller` (for reel):**
        *   `joint`: `reel_joint`
        *   Interface: `velocity`

### **5.4. O-Drive Hardware Configuration**

*   **Prerequisite:** All O-Drive controllers must be pre-configured using the native O-Drive tools.
*   **Drive Motors (M0 & M1 on SN: ...4633):** Encoder CPR must be set to **90**.
*   **Reel Motor (M0 on SN: ...3033):** Encoder CPR must be set to **42**.
*   **Reel Transmission:** A `<transmission>` tag will be added to the URDF for the `reel_joint` to define the **20:76** belt reduction.

## **6. User/Application Flow**

1.  **Launch:** The developer/user executes the main launch file, choosing the mode:
    *   Simulation: `ros2 launch ros2_mower mower.launch.py use_sim_time:=true`
    *   Hardware: `ros2 launch ros2_mower mower.launch.py use_sim_time:=false`
2.  **Initialization:** The `robot_state_publisher` will publish the `/tf` tree. The `controller_manager` will load and start the configured controllers. All relevant sensor and control topics will become active.
3.  **Control:** An external node (teleop, Nav2) can now control the robot:
    *   **Movement:** Publish a `geometry_msgs/Twist` message to `/cmd_vel`. The `diff_drive_controller` handles the rest.
    *   **Mowing:** Publish a `trajectory_msgs/JointTrajectory` message to the reel controller's topic to command a specific RPM.
4.  **Feedback:** The system will provide feedback via:
    *   `/odom`: Published by the `diff_drive_controller`.
    *   `/tf`: Published by `robot_state_publisher` and `/odom`.
    *   `/gps/fix`, `/gps/heading`, `/imu/data`: Published by the sensor interfaces (real or simulated).

## **7. Tech Stack**

| Category | Technology | Version/Notes |
| :--- | :--- | :--- |
| **Framework** | ROS 2 | **Jazzy Jalisco** |
| **Simulation** | Gazebo | Fortress |
| **Control** | `ros2_control`, `ros2_controllers` | |
| **Hardware Interface** | `Factor-Robotics/odrive_ros2_control` | |
| **Physical Hardware** | O-Drive 3.6 | |
| **Languages** | Python, XML (URDF/Launch), YAML | |

## **8. Implementation Plan**

The implementation will proceed in phases, adhering strictly to the Guiding Principles. Each task is designed to be a small, logical step with a clear deliverable and minimal complexity.

| Phase | Task Breakdown | Acceptance Criteria / Deliverable |
| :--- | :--- | :--- |
| **1. Foundation** | 1. Create Git repository and ROS 2 package `ros2_mower`. <br>2. Develop `urdf/mower.urdf.xacro` with visual links and joints. <br>3. Create `launch/view_robot.launch.py` to view the model in RViz2. | The robot model loads correctly in RViz2 and can be visually inspected. |
| **2. Simulation Physics** | 1. Add `<collision>` and `<inertial>` tags to the URDF. <br>2. Create a basic Gazebo world file. <br>3. Create `launch/gazebo.launch.py` to spawn the robot. | The robot model spawns in Gazebo and rests stably on the ground plane. |
| **3. Simulation Control** | 1. Add the `gazebo_ros2_control` plugin to the URDF. <br>2. Create `config/mower_controllers.yaml` with controller definitions. <br>3. Update `launch/gazebo.launch.py` to load the controllers. | The robot moves as commanded via `/cmd_vel` within the Gazebo simulation. |
| **4. Simulation Sensors** | 1. Add Gazebo plugins for IMU and GPS to the URDF. <br>2. Implement a simple node to publish a noisy `/gps/heading`. <br>3. Add the sensor nodes to the Gazebo launch file. | The topics `/imu/data`, `/gps/fix`, and `/gps/heading` are all published with valid, noisy data. |
| **5. Hardware Prep** | 1. Configure O-Drive motor and encoder parameters using `odrivetool`. <br>2. Create and test `udev` rules on the robot computer for persistent device naming. | O-Drives are fully calibrated. `udev` symlinks are created reliably. |
| **6. Hardware Integration** | 1. Conditionally add the `odrive_ros2_control` interface to the URDF. <br>2. Add the reel motor `<transmission>` tag. <br>3. Create a `launch/hardware.launch.py` to start the hardware nodes. | With the robot on blocks, the physical wheels spin correctly in response to `/cmd_vel` commands. |
| **7. Finalization** | 1. Create a unified `launch/mower.launch.py` with a `use_sim_time` argument. <br>2. Test both simulation and hardware modes from the unified launch file. <br>3. Write a `README.md` with setup and usage instructions. | The project is fully functional from a single launch command for both modes. The repository is well-documented. |
