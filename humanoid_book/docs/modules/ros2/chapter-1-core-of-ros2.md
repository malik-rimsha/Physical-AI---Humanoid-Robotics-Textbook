---
title: Chapter 1 - The Core of ROS 2
sidebar_label: Chapter 1 - Core of ROS 2
description: Understanding ROS 2 fundamentals - Nodes, Topics, and Services
---

# Chapter 1: The Core of ROS 2

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the fundamental concepts of ROS 2 including nodes, topics, and services
- Create and run basic publisher/subscriber communication patterns
- Use rclpy to implement ROS 2 nodes in Python
- Understand the role of these concepts in robotic systems

## Prerequisites

Before starting this chapter, you should:
- Have basic Python programming knowledge
- Understand general concepts of robotics and control systems
- Have ROS 2 Humble Hawksbill installed on your system

## Introduction

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. In this chapter, we'll explore the core concepts that make ROS 2 powerful: Nodes, Topics, and Services.

## 1. ROS 2 Nodes

A ROS 2 node is the fundamental unit of execution in ROS. It's a process that performs computation and communicates with other nodes. All ROS 2 programs are composed of multiple nodes that work together to achieve complex behaviors.

### Creating Your First Node

Let's create a simple ROS 2 node using rclpy, the Python client library for ROS 2:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclpy.init(args=args)

    minimal_node = MinimalNode()

    rclpy.spin(minimal_node)

    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This is the most basic ROS 2 node. It initializes the ROS client library, creates a node, spins to keep it running, and then shuts down when interrupted.

## 2. ROS 2 Topics

Topics enable asynchronous message passing between nodes using a publish/subscribe pattern. Nodes can publish messages to a topic or subscribe to messages from a topic.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. ROS 2 Services

Services provide synchronous request/response communication between nodes. A service client sends a request to a service server, which processes the request and sends back a response.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(2, 3)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (2, 3, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercises for ROS 2 Fundamentals

### Exercise 1: Publisher/Subscriber Communication

1. Create a new Python file called `talker.py` with the publisher code
2. Create a new Python file called `listener.py` with the subscriber code
3. Open two terminal windows
4. In the first terminal, source your ROS 2 environment and run: `python3 talker.py`
5. In the second terminal, source your ROS 2 environment and run: `python3 listener.py`
6. Observe the communication between the nodes

#### Expected Outcome

You should see the publisher sending messages and the subscriber receiving them, demonstrating the basic ROS 2 communication pattern.

### Exercise 2: Custom Message Topic

Modify the publisher and subscriber to use a custom message type or change the frequency of publishing:

1. Change the timer period in the publisher from 0.5 seconds to 1.0 second
2. Modify the message content to include your name or a custom string
3. Run the publisher and subscriber again to see the changes

#### Expected Outcome

The messages should now appear at a different frequency and contain your custom content.

### Exercise 3: Service Communication

1. Create a service server file `add_server.py` with the service server code
2. Create a service client file `add_client.py` with the service client code
3. Run the service server in one terminal: `python3 add_server.py`
4. Run the client in another terminal: `python3 add_client.py`
5. Observe the request/response communication

#### Expected Outcome

The client should send a request to add two numbers, and the server should respond with the sum.

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2:
- **Nodes**: The basic execution units that perform computation
- **Topics**: Asynchronous communication via publish/subscribe pattern
- **Services**: Synchronous request/response communication

These concepts form the foundation of ROS 2 and enable the creation of complex robotic systems through modular, distributed components.

## Next Steps

In the next chapter, we'll explore how to create Python AI agents that can publish joint commands to control robots via ROS 2, building on the concepts learned here.