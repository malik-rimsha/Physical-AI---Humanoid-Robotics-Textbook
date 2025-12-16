---
title: Chapter 2 - Programming Embodiment
sidebar_label: Chapter 2 - Programming Embodiment
description: Creating Python AI Agents to control robots via ROS 2
---

# Chapter 2: Programming Embodiment - Python Agents to ROS Control

## Learning Objectives

After completing this chapter, you will be able to:
- Create an rclpy-based agent that can publish joint commands to control a robot
- Understand the structure of an AI agent for robot control
- Implement topic publishing for joint command messages
- Bridge AI decision-making to physical robot control via ROS 2

## Prerequisites

Before starting this chapter, you should:
- Complete Chapter 1 - The Core of ROS 2
- Have basic Python programming knowledge
- Understand ROS 2 concepts (nodes, topics, services)
- Have ROS 2 Humble Hawksbill installed on your system

## Introduction

In this chapter, we'll bridge the gap between AI concepts and physical robot control. We'll explore how to create Python agents that can publish joint commands to control a robot via ROS 2. This represents the "embodiment" of AI - taking abstract decisions and making them manifest in the physical world through robotic systems.

## 1. Understanding Joint Commands in ROS 2

Joint commands in ROS 2 are messages sent to robot controllers to specify desired positions, velocities, or efforts for specific joints. These commands are typically published to topics that are subscribed to by the robot's hardware interface or simulation.

The most common message type for joint commands is `JointTrajectory` from the `trajectory_msgs` package, which allows specifying a sequence of joint positions over time.

## 2. Creating an rclpy Agent Structure

An rclpy agent for robot control should be structured to:
- Initialize as a ROS 2 node
- Maintain internal state representing the AI's decision-making process
- Publish joint commands based on its decision-making
- Potentially subscribe to sensor feedback for closed-loop control

Here's a basic structure for an AI agent:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math

class RobotControlAgent(Node):
    def __init__(self):
        super().__init__('robot_control_agent')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states (feedback)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal state for the AI agent
        self.current_joint_states = None
        self.time_step = 0

        self.get_logger().info('Robot Control Agent initialized')

    def joint_state_callback(self, msg):
        """Callback to receive joint state feedback"""
        self.current_joint_states = msg
        self.get_logger().info(f'Received joint states: {msg.name}')

    def control_loop(self):
        """Main control loop where AI decisions are translated to joint commands"""
        # This is where the AI decision-making happens
        # For now, we'll implement a simple oscillating pattern
        self.publish_joint_commands()

    def publish_joint_commands(self):
        """Publish joint trajectory commands to control the robot"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']  # Replace with actual joint names
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Calculate desired joint positions based on AI decision-making
        # This is a simple example - in practice, this would come from more complex AI logic
        time_in_cycle = self.time_step * 0.1  # 0.1s timer period

        # Create oscillating joint positions
        joint_positions = [
            math.sin(time_in_cycle),           # Joint 1: sine wave
            math.cos(time_in_cycle) * 0.5,     # Joint 2: cosine wave, smaller amplitude
            math.sin(time_in_cycle * 2) * 0.3  # Joint 3: double frequency
        ]

        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)  # Start with zero velocities
        point.accelerations = [0.0] * len(joint_positions)  # Start with zero accelerations

        # Set a short time from start (0.1 seconds)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds

        trajectory_msg.points = [point]

        # Publish the trajectory
        self.joint_cmd_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published joint commands: {joint_positions}')

        # Increment time step for next iteration
        self.time_step += 1

def main(args=None):
    rclpy.init(args=args)

    agent = RobotControlAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Advanced Agent with Decision-Making

Let's create a more sophisticated agent that makes decisions based on sensor input:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import random

class IntelligentRobotAgent(Node):
    def __init__(self):
        super().__init__('intelligent_robot_agent')

        # Publishers
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.laser_scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz control loop

        # Internal state
        self.current_joint_states = None
        self.laser_scan_data = None
        self.agent_state = "EXPLORING"  # EXPLORING, AVOIDING, IDLE
        self.last_decision_time = self.get_clock().now()

        self.get_logger().info('Intelligent Robot Agent initialized')

    def joint_state_callback(self, msg):
        """Callback to receive joint state feedback"""
        self.current_joint_states = msg

    def laser_scan_callback(self, msg):
        """Callback to receive laser scan data for obstacle detection"""
        self.laser_scan_data = msg

    def make_decision(self):
        """AI decision-making function based on sensor data"""
        if self.laser_scan_data is None:
            return "EXPLORING"

        # Check for obstacles in front (assuming front is center of scan)
        min_distance = min(self.laser_scan_data.ranges)

        if min_distance < 0.5:  # Obstacle within 0.5m
            return "AVOIDING"
        else:
            return "EXPLORING"

    def generate_joint_commands(self, agent_state):
        """Generate appropriate joint commands based on agent state"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()

        if agent_state == "EXPLORING":
            # Gentle exploratory movement
            joint_positions = [
                math.sin(self.get_clock().now().nanoseconds * 1e-9) * 0.2,
                math.cos(self.get_clock().now().nanoseconds * 1e-9) * 0.15,
                0.0
            ]
        elif agent_state == "AVOIDING":
            # Evasive movement pattern
            joint_positions = [
                math.sin(self.get_clock().now().nanoseconds * 1e-9 + math.pi) * 0.3,
                math.cos(self.get_clock().now().nanoseconds * 1e-9 + math.pi/2) * 0.25,
                math.sin(self.get_clock().now().nanoseconds * 1e-9 * 2) * 0.2
            ]
        else:
            # Default position
            joint_positions = [0.0, 0.0, 0.0]

        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        point.accelerations = [0.0] * len(joint_positions)

        # Set execution time
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 0.05 seconds

        trajectory_msg.points = [point]
        return trajectory_msg

    def control_loop(self):
        """Main control loop with AI decision-making"""
        # Make decision based on sensor data
        new_state = self.make_decision()

        # Update agent state
        if new_state != self.agent_state:
            self.get_logger().info(f'Agent state changed: {self.agent_state} -> {new_state}')
            self.agent_state = new_state

        # Generate and publish joint commands
        trajectory_msg = self.generate_joint_commands(self.agent_state)
        self.joint_cmd_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    agent = IntelligentRobotAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Joint Command Message Structure

The JointTrajectory message is the standard way to send joint commands in ROS 2. It contains:

- `joint_names`: Array of joint names to control
- `points`: Array of trajectory points, each containing:
  - `positions`: Desired joint positions
  - `velocities`: Desired joint velocities
  - `accelerations`: Desired joint accelerations
  - `time_from_start`: When to execute this point relative to the trajectory start

## 5. Practical Exercise: AI Agent Implementation

### Exercise 1: Basic Agent

1. Create a new Python file called `basic_agent.py` with the basic agent code
2. Make sure your ROS 2 environment is sourced
3. Run the agent: `python3 basic_agent.py`
4. Observe the joint command publications in another terminal: `ros2 topic echo /joint_trajectory_controller/joint_trajectory`

### Exercise 2: Intelligent Agent

1. Create a new Python file called `intelligent_agent.py` with the intelligent agent code
2. If running with a robot simulator (like Gazebo), connect the agent to the simulation
3. Add a simple obstacle to the simulation and observe the agent's response
4. Modify the decision-making logic to implement different behaviors

### Exercise 3: Custom Behavior

1. Modify the agent to implement a specific behavior (e.g., walking pattern, reaching motion)
2. Add more joints to control based on your robot's configuration
3. Implement a state machine for more complex behaviors

## Summary

In this chapter, we've explored how to create Python AI agents that can publish joint commands to control robots via ROS 2. We've covered:

- The structure of an rclpy-based robot control agent
- How to publish joint trajectory commands
- How to incorporate sensor feedback for closed-loop control
- How to implement AI decision-making that translates to physical robot actions

These concepts bridge the gap between AI algorithms and physical robot control, forming the core of embodied AI systems.

## Next Steps

In the next chapter, we'll explore how to define humanoid robots using URDF (Unified Robot Description Format), which is essential for understanding how robots are modeled and controlled in ROS.