---
title: Chapter 3 - Defining the Humanoid
sidebar_label: Chapter 3 - Defining the Humanoid
description: Understanding URDF for robot description and kinematics
---

# Chapter 3: Defining the Humanoid - Unified Robot Description Format (URDF)

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the structure and components of URDF (Unified Robot Description Format)
- Define links and joints for a bipedal humanoid structure
- Understand kinematic relationships between robot components
- Use Xacro to simplify complex URDF definitions
- Visualize URDF models in ROS tools

## Prerequisites

Before starting this chapter, you should:
- Complete Chapter 1 - The Core of ROS 2
- Complete Chapter 2 - Programming Embodiment
- Have basic understanding of 3D geometry and robotics kinematics
- Have ROS 2 Humble Hawksbill installed on your system

## Introduction

The Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and kinematic relationships. In this chapter, we'll explore how to define a humanoid robot using URDF, focusing on the structure and kinematics of a bipedal system.

## 1. URDF Structure and Components

A URDF file describes a robot as a collection of rigid bodies (links) connected by joints. The basic structure includes:

- **Links**: Rigid parts of the robot (e.g., torso, limbs)
- **Joints**: Connections between links that allow relative motion
- **Visual**: How the link appears in visualization
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.3 -0.1" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## 2. Links and Joints Concepts

### Links

Links represent rigid bodies in the robot. Each link has:

- A name for identification
- Visual properties (shape, color, material)
- Collision properties (for physics simulation)
- Inertial properties (mass, center of mass, inertia tensor)

### Joints

Joints define how links connect and move relative to each other. Joint types include:

- **fixed**: No movement allowed (0 DOF)
- **revolute**: Rotational movement around an axis (1 DOF)
- **continuous**: Continuous rotation around an axis (1 DOF)
- **prismatic**: Linear movement along an axis (1 DOF)
- **floating**: 6 DOF movement (not commonly used in URDF)
- **planar**: Movement in a plane (3 DOF)

### Joint Definition Example

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.0 0.2 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## 3. Simplified Bipedal URDF Example

Here's a simplified URDF for a basic bipedal humanoid structure:

```xml
<?xml version="1.0"?>
<robot name="simple_biped">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0.0 0.0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="40" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0.0 0.0 -0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Leg (similar to left leg) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="1"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0.0 0.0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="40" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0.0 0.0 -0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="-0.15 0.0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>

  <!-- Right Arm (similar to left arm) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 0.0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0.0 0.0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 -0.175" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>
</robot>
```

## 4. Kinematics for Humanoid Robots

Kinematics in robotics deals with the motion of robot parts without considering the forces that cause the motion. For humanoid robots, understanding kinematics is crucial for:

- **Forward Kinematics**: Calculating the position of end-effectors (hands, feet) based on joint angles
- **Inverse Kinematics**: Calculating required joint angles to achieve a desired end-effector position

The kinematic chain of a humanoid robot typically includes:
- Torso as the base
- Arms with shoulder, elbow, and wrist joints
- Legs with hip, knee, and ankle joints
- Head with neck joints

## 5. Xacro: Simplifying URDF

Xacro (XML Macros) is an XML macro language that allows you to define reusable components, properties, and mathematical expressions in URDF. This makes complex robot definitions more manageable.

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_biped_xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="torso_width" value="0.3" />
  <xacro:property name="torso_depth" value="0.2" />

  <!-- Macro for a simple link -->
  <xacro:macro name="simple_link" params="name mass x y z">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${x} ${y} ${z}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="${x} ${y} ${z}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Using the macro -->
  <xacro:simple_link name="torso" mass="10.0" x="0.3" y="0.2" z="0.6"/>

</robot>
```

## 6. URDF Visualization in ROS

ROS provides tools to visualize URDF models:

### Using RViz:
```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2
# In RViz, add RobotModel display and set Robot Description to your robot's parameter
```

### Using Robot State Publisher:
```bash
# Publish robot description to TF
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat your_robot.urdf)'
```

### Using Gazebo for Simulation:
```bash
# Launch Gazebo with your robot model
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot
```

## 7. Practical Exercise: URDF Creation

### Exercise 1: URDF Model Creation

1. Create a new file called `simple_biped.urdf` with the bipedal URDF example above
2. Validate the URDF file for XML syntax: `xmllint --noout simple_biped.urdf`
3. Check the kinematic structure: `check_urdf simple_biped.urdf`
4. Visualize the robot in RViz using robot_state_publisher

### Exercise 2: Xacro Conversion

1. Convert your URDF file to Xacro format to make it more maintainable
2. Add parameters for robot dimensions that can be easily modified
3. Create macros for repeated components (like legs and arms)

### Exercise 3: Customization

1. Modify the bipedal model to change proportions
2. Add additional joints or links (e.g., wrist joints, finger links)
3. Adjust inertial properties for more realistic simulation

## Summary

In this chapter, we've explored the Unified Robot Description Format (URDF) and its role in defining humanoid robots:

- **URDF Structure**: Links, joints, and their properties
- **Bipedal Design**: How to structure a two-legged humanoid robot
- **Kinematics**: Understanding motion relationships in humanoid robots
- **Xacro**: Simplifying complex URDF definitions with macros
- **Visualization**: Tools for viewing and validating URDF models

URDF is fundamental to robotics in ROS, as it provides the necessary information for simulation, visualization, motion planning, and control of robotic systems.

## Next Steps

With all three chapters complete, you now have a comprehensive understanding of ROS 2 fundamentals, AI agent integration, and robot description. You can now combine these concepts to create complete robotic systems that bridge AI algorithms with physical robot control.