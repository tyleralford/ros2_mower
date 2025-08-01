---

# **Implementation Plan: `ros2_mower` Package**

This document provides a sequential, step-by-step plan for the development of the `ros2_mower` package. Each task builds directly on the previous ones, ensuring a logical progression with no complexity jumps. Intermediate testing is integrated at the end of each phase to validate functionality before proceeding.

## **Phase 1: Project Foundation & Visual Model**

**Goal:** Establish the project's version-controlled structure and create a visually accurate, non-physical robot model.

### **T01: Project & Repository Setup**
*   **Depends on:** None
*   **Context:** Create the foundational Git repository on GitHub and the local ROS 2 package structure. This ensures all work is version-controlled from the start, adhering to the principle of having a clean, managed codebase.
*   **Subtasks:**
    -   [ ] Use the GitHub CLI (`gh`) to create a new public repository named `ros2_mower`.
    -   [ ] Clone the new repository to your local development machine: `git clone <repository_url>`.
    -   [ ] Inside the repository, create a ROS 2 package: `ros2 pkg create --build-type ament_cmake ros2_mower`.
    -   [ ] Create the required directories within the package: `ros2_mower/urdf`, `ros2_mower/launch`, `ros2_mower/config`, `ros2_mower/worlds`.
    -   [ ] Add a `.gitignore` file to the repository root, populated with standard ROS 2, Python, and OS-specific ignores.
    -   [ ] Make the first commit and push to the `main` branch: `git commit -m "feat: Initial project structure"` and `git push -u origin main`.

### **T02: Create the Core URDF with Xacro**
*   **Depends on:** T01
*   **Context:** Develop the robot's structural model using the `xacro` templating language. This allows us to define dimensions and properties as variables for easy modification, promoting maintainability and clarity.
*   **Subtasks:**
    -   [ ] Create a new file: `ros2_mower/urdf/mower.urdf.xacro`.
    -   [ ] Define xacro properties for all key dimensions (track width, wheel diameter, chassis size, etc.) and masses as specified in the PRD.
    -   [ ] Define the `base_link` as the root link.
    -   [ ] Create the `chassis`, `counterweight`, `right_wheel`, `left_wheel`, `front_roller`, and `reel` links. Each link must have a `<visual>` tag with geometry matching the PRD.
    -   [ ] Define all the necessary `joint` tags (`fixed` and `continuous`) to connect the links correctly based on the PRD's spatial relationships.
    -   [ ] Add the `reel_motor` link and its fixed joint to the chassis.

### **T03: Create Visualization Launch File**
*   **Depends on:** T02
*   **Context:** Create a launch file to view the robot model in RViz2. This provides immediate visual feedback to validate the URDF structure, ensuring we are building on a correct foundation.
*   **Subtasks:**
    -   [ ] Create a new file: `ros2_mower/launch/view_robot.launch.py`.
    -   [ ] In this launch file, add nodes for `robot_state_publisher` (to process the URDF) and `joint_state_publisher_gui` (to allow for manual joint animation).
    -   [ ] Add a node to start `rviz2` with a basic configuration that displays the RobotModel and TF frames.
    -   [ ] Ensure the launch file correctly finds and passes the `mower.urdf.xacro` file to the `robot_state_publisher`.

### **T04: Intermediate Test - Visual Validation**
*   **Depends on:** T03
*   **Context:** Build the package and run the visualization launch file to confirm the robot model is geometrically and kinematically correct before adding physics or control.
*   **Subtasks:**
    -   [ ] Build the package: `colcon build --symlink-install`.
    -   [ ] Source the workspace: `source install/setup.bash`.
    -   [ ] Run the launch file: `ros2 launch ros2_mower view_robot.launch.py`.
    -   [ ] **Validation:**
        -   [ ] Confirm the robot model appears in RViz2 without errors in the terminal.
        -   [ ] Confirm all links are present and correctly positioned relative to each other.
        -   [ ] Use the `joint_state_publisher_gui` sliders to move the wheels and reel joints and verify they rotate around the correct axes.
    -   [ ] Commit all work from Phase 1: `git commit -m "feat: Add visual URDF and RViz launcher"`.

## **Phase 2: Simulation Integration**

**Goal:** Make the robot a "physical" object within the Gazebo simulator, capable of interacting with a world.

### **T05: Add Physics and Collision Properties**
*   **Depends on:** T02
*   **Context:** Add the physical properties to the URDF that Gazebo needs to simulate the robot correctly. This step transitions the model from a purely visual object to a physical one.
*   **Subtasks:**
    -   [ ] In `ros2_mower/urdf/mower.urdf.xacro`, add a `<collision>` tag to every link. The geometry should match the `<visual>` tags.
    -   [ ] Add an `<inertial>` tag to every link. Use the `<mass>` value from the PRD. For the `<inertia>` tensor, use the formulas for simple primitive shapes (box, cylinder), which can be calculated or left for Gazebo to auto-calculate.

### **T06: Create Basic Gazebo World and Launch File**
*   **Depends on:** T05
*   **Context:** Create a simple world for the robot to exist in and a launch file to start Gazebo and spawn the robot model.
*   **Subtasks:**
    -   [ ] Create a simple Gazebo world file (`ros2_mower/worlds/empty.world`) with a ground plane and basic lighting.
    -   [ ] Create a new launch file: `ros2_mower/launch/gazebo.launch.py`.
    -   [ ] In this file, add the logic to start the Gazebo server (`gzserver`) with your world file and the Gazebo client (`gzclient`).
    -   [ ] Add a node for `spawn_entity.py` to spawn the `ros2_mower` model into the simulation.
    -   [ ] Include the `robot_state_publisher` from T03.

### **T07: Intermediate Test - Physics Validation**
*   **Depends on:** T06
*   **Context:** Build and launch the Gazebo simulation to ensure the robot's physical model is stable and interacts correctly with the world.
*   **Subtasks:**
    -   [ ] Build and source the workspace.
    -   [ ] Run the launch file: `ros2 launch ros2_mower gazebo.launch.py`.
    -   [ ] **Validation:**
        -   [ ] Confirm Gazebo opens and the mower model is spawned into the world.
        -   [ ] Confirm the mower falls to the ground plane and comes to rest stably without jittering, exploding, or falling through the floor.
    -   [ ] Commit all work from Phase 2: `git commit -m "feat: Add Gazebo simulation with physics"`.

## **Phase 3: Simulation Control System**

**Goal:** Implement the `ros2_control` stack to enable control of the robot's joints within Gazebo.

### **T08: Add Gazebo `ros2_control` Plugin to URDF**
*   **Depends on:** T05
*   **Context:** Integrate `ros2_control` into the URDF for the simulation environment. This is the essential bridge that allows ROS controllers to interface with Gazebo's physics engine.
*   **Subtasks:**
    -   [ ] In `ros2_mower/urdf/mower.urdf.xacro`, add a `<ros2_control>` tag for a `GazeboSystem` plugin.
    -   [ ] Inside this tag, define the `joint` interfaces for `left_wheel_joint`, `right_wheel_joint`, and `reel_joint`.
    -   [ ] For each joint, specify the available command and state interfaces. For the wheels and reel, this will be `velocity`.

### **T09: Create Controller Configuration File**
*   **Depends on:** T08
*   **Context:** Create the YAML file that defines which controllers to use and how they are configured. This separates controller configuration from the robot model, following the principle of separation of concerns.
*   **Subtasks:**
    -   [ ] Create a new file: `ros2_mower/config/mower_controllers.yaml`.
    -   [ ] In this file, define the `controller_manager`.
    -   [ ] Define the `diff_drive_controller`, setting its `type`, wheel joint names, `wheel_separation`, and `wheel_radius` as specified in the PRD.
    -   [ ] Define the `joint_trajectory_controller` for the reel, setting its `type`, `joint` name, and specifying `velocity` as the command interface.

### **T10: Update Gazebo Launch File to Load Controllers**
*   **Depends on:** T09
*   **Context:** Modify the simulation launch file to load the `ros2_control` system and start the controllers, making the robot controllable.
*   **Subtasks:**
    -   [ ] In `ros2_mower/launch/gazebo.launch.py`, add a node for the `spawner` from the `controller_manager` package.
    -   [ ] Create two `spawner` executions: one to load the `diff_drive_controller` and another for the `joint_trajectory_controller`.

### **T11: Intermediate Test - Simulation Control Validation**
*   **Depends on:** T10
*   **Context:** Test the full simulation control stack by sending commands from the terminal. This validates that the entire chain from ROS topic to joint movement is working.
*   **Subtasks:**
    -   [ ] Build and launch `ros2_mower/launch/gazebo.launch.py`.
    -   [ ] In a new terminal, check the active controllers: `ros2 control list_controllers`. Verify they are `active`.
    -   [ ] In a new terminal, use `ros2 topic pub` to send a `geometry_msgs/Twist` message to `/diff_drive_controller/cmd_vel_unstamped`.
    -   [ ] **Validation:**
        -   [ ] Confirm the robot moves forward, backward, and turns in Gazebo in response to the commands.
        -   [ ] Echo the `/diff_drive_controller/odom` topic and confirm it is publishing plausible odometry data.
    -   [ ] Commit all work from Phase 3: `git commit -m "feat: Implement ros2_control for simulation"`.

## **Phase 4: Simulation Sensors**

**Goal:** Add the virtual IMU and GPS sensors to the simulated robot to enable development of localization and navigation algorithms.

### **T12: Add Sensor Links and Gazebo Plugins to URDF**
*   **Depends on:** T05
*   **Context:** Add the physical sensor links to the URDF and attach the appropriate Gazebo plugins to generate simulated data.
*   **Subtasks:**
    -   [ ] In `ros2_mower/urdf/mower.urdf.xacro`, create the `imu_link` and `gps_link` with their respective fixed joints, using the precise locations from the PRD.
    -   [ ] Attach the `gazebo_ros_imu_sensor` plugin to the `imu_link`. Configure it to publish to `/imu/data` and add realistic noise parameters.
    -   [ ] Attach the `gazebo_ros_gps_sensor` plugin to the `gps_link`. Configure it to publish to `/gps/fix`.
    -   [ ] Attach the `gazebo_ros_p3d` plugin to the `base_link` to get ground truth pose data for the heading calculation. Configure it to publish on a topic like `/ground_truth/pose`.

### **T13: Create GPS Heading Publisher Node**
*   **Depends on:** T12
*   **Context:** Create a minimal Python node that subscribes to the ground truth pose, extracts the heading, adds noise, and publishes it on a separate topic, as required by the PRD.
*   **Subtasks:**
    -   [ ] Create a new directory `ros2_mower/scripts`.
    -   [ ] Create a new file `ros2_mower/scripts/gps_heading_node.py`.
    -   [ ] In this node, create a subscriber to `/ground_truth/pose`.
    -   [ ] Create a publisher for a `sensor_msgs/Imu` or `geometry_msgs/QuaternionStamped` on the `/gps/heading` topic.
    -   [ ] Implement the logic to convert the pose to a heading, add noise, and publish it.
    -   [ ] Update the `package.xml` (add dependency) and `CMakeLists.txt` (`install` directive) to handle the new script.
    -   [ ] Add the new node to the `ros2_mower/launch/gazebo.launch.py` file.

### **T14: Intermediate Test - Sensor Validation**
*   **Depends on:** T13
*   **Context:** Verify that all sensor topics are being published with realistic data.
*   **Subtasks:**
    -   [ ] Build and launch `ros2_mower/launch/gazebo.launch.py`.
    -   [ ] In new terminals, `ros2 topic echo` each of the following topics: `/imu/data`, `/gps/fix`, and `/gps/heading`.
    -   [ ] **Validation:**
        -   [ ] Confirm all three topics are publishing data at the expected rate.
        -   [ ] Confirm the data format is correct for each topic.
        -   [ ] Move the robot in the simulation and observe that the sensor data changes plausibly.
    -   [ ] Commit all work from Phase 4: `git commit -m "feat: Add simulated IMU and GPS sensors"`.

## **Phase 5: Physical Hardware Preparation**

**Goal:** Configure the physical hardware and the robot's operating system. This phase is independent of the simulation work but is a prerequisite for Phase 6.

### **T15: Configure O-Drive Controllers**
*   **Depends on:** None
*   **Context:** Connect to each O-Drive controller and configure the motor and encoder parameters required for operation. This is done outside of ROS using O-Drive's native tools.
*   **Subtasks:**
    -   [ ] Connect to the first O-Drive (SN: ...4633) via USB.
    -   [ ] Using `odrivetool`, perform motor and encoder calibration for both the right wheel motor (M0) and left wheel motor (M1).
    -   [ ] Set the encoder `cpr` to **90** for both axes and save the configuration.
    -   [ ] Connect to the second O-Drive (SN: ...3033) via USB.
    -   [ ] Perform motor and encoder calibration for the reel motor (M0).
    -   [ ] Set the encoder `cpr` to **42** and save the configuration.
    -   [ ] **Validation:** Manually command the motors in `odrivetool` to confirm they spin correctly.

### **T16: Set Up `udev` Rules on Robot Computer**
*   **Depends on:** T15
*   **Context:** Create a `udev` rule to give the O-Drives persistent device names. This is a best practice that prevents the device path from changing on reboot.
*   **Subtasks:**
    -   [ ] On the robot's onboard computer, create a new file: `/etc/udev/rules.d/99-odrive.rules`.
    -   [ ] Add two rules to this file, one for each O-Drive, that use the `ATTRS{serial}` property to create symlinks (e.g., `/dev/odrive_drive` and `/dev/odrive_reel`).
    -   [ ] Reload the `udev` rules (`sudo udevadm control --reload-rules && sudo udevadm trigger`).
    -   [ ] **Validation:** Plug in both O-Drives and confirm that the symbolic links appear correctly in the `/dev/` directory.

## **Phase 6: Hardware Control Integration**

**Goal:** Connect the ROS 2 control system to the physical robot hardware, reusing the controllers from the simulation.

### **T17: Update URDF for Hardware Interface**
*   **Depends on:** T08
*   **Context:** Modify the URDF's `<ros2_control>` tag to select the physical hardware interface when not in simulation mode. This is the core of the "digital twin" concept.
*   **Subtasks:**
    -   [ ] In `ros2_mower/urdf/mower.urdf.xacro`, add a xacro argument `use_sim_time` defaulting to `false`.
    -   [ ] Use `<xacro:if>` and `<xacro:unless>` blocks. The `GazeboSystem` plugin should only be included `if use_sim_time` is true.
    -   [ ] In the `unless` block, define the `odrive_ros2_control/OdriveHardwareInterface` as the plugin.
    -   [ ] Pass the O-Drive serial numbers as parameters to the plugin.
    -   [ ] Add the `<transmission>` tag for the `reel_joint` to define the 20:76 gear reduction.

### **T18: Create Hardware Launch File**
*   **Depends on:** T10, T16, T17
*   **Context:** Create a launch file dedicated to starting the robot's hardware control system.
*   **Subtasks:**
    -   [ ] Create a new file: `ros2_mower/launch/hardware.launch.py`.
    -   [ ] This file should be similar to `gazebo.launch.py` but without any Gazebo nodes.
    -   [ ] It must load the `robot_state_publisher`, `controller_manager`, and spawn the controllers from `ros2_mower/config/mower_controllers.yaml`.
    -   [ ] Ensure it passes `use_sim_time:=false` to the URDF via the `robot_description` parameter.

### **T19: Intermediate Test - Hardware Control Validation**
*   **Depends on:** T18
*   **Context:** Test the control system on the physical robot. **Safety is paramount.**
*   **Subtasks:**
    -   [ ] **SAFETY:** Place the robot on blocks so the wheels can spin freely off the ground.
    -   [ ] Build and source the workspace on the robot computer.
    -   [ ] Run the hardware launch file: `ros2 launch ros2_mower hardware.launch.py`.
    -   [ ] Check `ros2 control list_controllers` to ensure controllers are active.
    -   [ ] Use `ros2 topic pub` to send a small `/diff_drive_controller/cmd_vel_unstamped` command.
    -   [ ] **Validation:**
        -   [ ] Confirm the physical wheels spin in the correct direction and respond to commands.
        -   [ ] Confirm the `/diff_drive_controller/odom` topic is being published based on real encoder feedback.
    -   [ ] Commit all work from Phases 5 & 6: `git commit -m "feat: Implement hardware integration and control"`.

## **Phase 7: Finalization and Unification**

**Goal:** Combine the simulation and hardware launch files into a single, user-friendly entrypoint and finalize documentation.

### **T20: Create Unified Top-Level Launch File**
*   **Depends on:** T11, T19
*   **Context:** Create the final, master launch file that can start either the simulation or the hardware based on a single argument, fulfilling a core project requirement.
*   **Subtasks:**
    -   [ ] Create a new file: `ros2_mower/launch/mower.launch.py`.
    -   [ ] Add a `DeclareLaunchArgument` for `use_sim_time`.
    -   [ ] Use `OpaqueFunction` or `actions.IncludeLaunchDescription` with conditional logic based on the `use_sim_time` argument to include either `gazebo.launch.py` or `hardware.launch.py`.
    -   [ ] Refactor the common nodes (like `robot_state_publisher`) into a separate launch file (`robot.launch.py`) that can be included by both the Gazebo and hardware launch files to avoid code duplication.

### **T21: Final System Test**
*   **Depends on:** T20
*   **Context:** Perform a final end-to-end test of the unified launch file in both modes to ensure the project is complete and robust.
*   **Subtasks:**
    -   [ ] **Test Case 1 (Sim):**
        -   [ ] Run `ros2 launch ros2_mower mower.launch.py use_sim_time:=true`.
        -   [ ] Confirm the simulation starts and the robot is controllable.
    -   [ ] **Test Case 2 (Hardware):**
        -   [ ] Run `ros2 launch ros2_mower mower.launch.py use_sim_time:=false` on the robot computer.
        -   [ ] Confirm the hardware initializes and the robot is controllable.
    -   [ ] **Validation:** The exact same ROS 2 command from a high-level node (e.g., `teleop_twist_keyboard`) successfully and correctly controls the robot's movement in both simulation and the real world.

### **T22: Finalize Documentation**
*   **Depends on:** T21
*   **Context:** A project is not complete until it is documented. This ensures maintainability and usability for future developers (including yourself).
*   **Subtasks:**
    -   [ ] Create a `README.md` file in the repository root.
    -   [ ] Document the project's purpose.
    -   [ ] Provide clear, concise instructions for installation, building, and running both the simulation and hardware modes using the unified launch file.
    -   [ ] Commit the final work: `git commit -m "docs: Add README and finalize project"`.
