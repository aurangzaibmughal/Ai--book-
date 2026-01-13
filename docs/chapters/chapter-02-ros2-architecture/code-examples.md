---
id: code-examples
title: "Code Examples: ROS 2 Architecture"
sidebar_label: "Code Examples"
sidebar_position: 6
description: "Practical ROS 2 code examples demonstrating nodes, topics, and QoS policies"
---

# Code Examples: ROS 2 Architecture

This section provides working code examples that demonstrate core ROS 2 concepts. All examples are tested with ROS 2 Humble on Ubuntu 22.04.

## Example 1: Minimal Publisher Node

This example creates a simple publisher that sends temperature sensor data at 10Hz.

### Code: `temperature_publisher.py`

```python
#!/usr/bin/env python3
"""
Temperature Sensor Publisher
Simulates a temperature sensor publishing data at 10Hz
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random


class TemperatureSensorNode(Node):
    """Publishes simulated temperature readings"""

    def __init__(self) -> None:
        # Initialize the node with a unique name
        super().__init__('temperature_sensor')

        # Create publisher for temperature topic
        # QoS depth=10 means buffer last 10 messages if subscriber is slow
        self.publisher_ = self.create_publisher(
            Temperature,           # Message type
            'robot/temperature',   # Topic name
            10                     # QoS depth
        )

        # Create timer that calls publish_temperature every 0.1 seconds (10Hz)
        self.timer = self.create_timer(0.1, self.publish_temperature)

        # Initialize simulated temperature
        self.base_temperature: float = 25.0  # Celsius

        self.get_logger().info('Temperature sensor node started')

    def publish_temperature(self) -> None:
        """Callback function called by timer to publish temperature"""
        msg = Temperature()

        # Set message header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Simulate temperature with small random variation
        # NOTE: We use random values here to simulate real sensor noise.
        # Real sensors have measurement uncertainty and environmental variations.
        # In production code, you would read from actual hardware sensors.
        # For deterministic testing, replace with: msg.temperature = self.base_temperature
        msg.temperature = self.base_temperature + random.uniform(-0.5, 0.5)
        msg.variance = 0.1  # Sensor accuracy (±0.1°C)

        # Publish the message
        self.publisher_.publish(msg)

        # Log at DEBUG level (won't show by default)
        self.get_logger().debug(f'Published: {msg.temperature:.2f}°C')


def main(args: Optional[list] = None) -> None:
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = TemperatureSensorNode()

    # Spin (keep the node running and processing callbacks)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Example

```bash
# Make the script executable
chmod +x temperature_publisher.py

# Run the node
python3 temperature_publisher.py

# In another terminal, echo the topic to see messages
ros2 topic echo /robot/temperature

# Check publishing rate
ros2 topic hz /robot/temperature
```

### Key Concepts Demonstrated

1. **Node Initialization**: `super().__init__('temperature_sensor')`
2. **Publisher Creation**: `create_publisher(msg_type, topic, qos_depth)`
3. **Timer-Based Publishing**: Ensures consistent 10Hz rate
4. **Message Headers**: Timestamp and frame_id for sensor fusion
5. **Logging**: Using ROS 2's built-in logger

---

## Example 2: Minimal Subscriber Node

This subscriber receives and processes temperature data.

### Code: `temperature_subscriber.py`

```python
#!/usr/bin/env python3
"""
Temperature Monitor Subscriber
Receives temperature data and triggers alerts if too high
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class TemperatureMonitorNode(Node):
    """Monitors temperature and logs warnings if threshold exceeded"""

    def __init__(self):
        super().__init__('temperature_monitor')

        # Declare parameters with default values
        self.declare_parameter('warning_threshold', 30.0)
        self.declare_parameter('critical_threshold', 35.0)

        # Get parameter values
        self.warning_temp = self.get_parameter('warning_threshold').value
        self.critical_temp = self.get_parameter('critical_threshold').value

        # Create subscriber
        self.subscription = self.create_subscription(
            Temperature,
            'robot/temperature',
            self.temperature_callback,
            10  # QoS depth
        )

        self.get_logger().info(
            f'Temperature monitor started '
            f'(warning: {self.warning_temp}°C, critical: {self.critical_temp}°C)'
        )

    def temperature_callback(self, msg):
        """Called whenever a new temperature message arrives"""
        temp = msg.temperature

        # Check thresholds and log appropriately
        if temp >= self.critical_temp:
            self.get_logger().error(
                f'CRITICAL: Temperature {temp:.2f}°C exceeds {self.critical_temp}°C!'
            )
        elif temp >= self.warning_temp:
            self.get_logger().warn(
                f'WARNING: Temperature {temp:.2f}°C exceeds {self.warning_temp}°C'
            )
        else:
            self.get_logger().debug(f'Temperature normal: {temp:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running with Parameters

```bash
# Run with default thresholds
python3 temperature_subscriber.py

# Run with custom thresholds
ros2 run <package_name> temperature_subscriber \
    --ros-args \
    -p warning_threshold:=28.0 \
    -p critical_threshold:=32.0

# Set log level to see DEBUG messages
ros2 run <package_name> temperature_subscriber \
    --ros-args --log-level debug
```

### Key Concepts Demonstrated

1. **Subscriber Creation**: `create_subscription(msg_type, topic, callback, qos)`
2. **Callback Functions**: Executed when messages arrive
3. **Parameters**: Runtime configuration without code changes
4. **Logging Levels**: DEBUG, INFO, WARN, ERROR

---

## Example 3: QoS Policies in Action

This example demonstrates different QoS configurations for different use cases.

### Code: `qos_demo.py`

```python
#!/usr/bin/env python3
"""
QoS Policy Demonstration
Shows how different QoS settings affect message delivery
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import time


class QoSDemoNode(Node):
    """Demonstrates various QoS configurations"""

    def __init__(self):
        super().__init__('qos_demo')

        # 1. RELIABLE QoS - Guaranteed delivery (like TCP)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.reliable_pub = self.create_publisher(
            String,
            'reliable_topic',
            reliable_qos
        )

        # 2. BEST_EFFORT QoS - No delivery guarantee (like UDP)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest
        )

        self.best_effort_pub = self.create_publisher(
            String,
            'best_effort_topic',
            best_effort_qos
        )

        # 3. TRANSIENT_LOCAL - Late-joiners get last message
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.config_pub = self.create_publisher(
            String,
            'config_topic',
            transient_qos
        )

        # Publish configuration once (late-joiners will still receive it)
        config_msg = String()
        config_msg.data = "Robot Configuration: Mode=Autonomous, Speed=1.5m/s"
        self.config_pub.publish(config_msg)
        self.get_logger().info(f'Published config: {config_msg.data}')

        # Timer for continuous publishing
        self.timer = self.create_timer(0.5, self.publish_messages)
        self.counter = 0

    def publish_messages(self):
        """Publish messages with different QoS"""
        self.counter += 1

        # Reliable message (critical command)
        reliable_msg = String()
        reliable_msg.data = f"COMMAND #{self.counter}: Execute task"
        self.reliable_pub.publish(reliable_msg)

        # Best-effort message (high-frequency sensor data)
        sensor_msg = String()
        sensor_msg.data = f"SENSOR #{self.counter}: x=1.2, y=3.4, z=5.6"
        self.best_effort_pub.publish(sensor_msg)

        self.get_logger().info(f'Published message set #{self.counter}')


class QoSSubscriberNode(Node):
    """Subscribes with matching QoS policies"""

    def __init__(self):
        super().__init__('qos_subscriber')

        # Subscribe to reliable topic
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            String,
            'reliable_topic',
            lambda msg: self.get_logger().info(f'RELIABLE: {msg.data}'),
            reliable_qos
        )

        # Subscribe to best-effort topic
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            String,
            'best_effort_topic',
            lambda msg: self.get_logger().info(f'BEST_EFFORT: {msg.data}'),
            best_effort_qos
        )

        # Subscribe to config topic (will receive last message even if late)
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            String,
            'config_topic',
            lambda msg: self.get_logger().info(f'CONFIG: {msg.data}'),
            transient_qos
        )

        self.get_logger().info('QoS Subscriber started')


def main(args=None):
    rclpy.init(args=args)

    # Choose which node to run based on command-line argument
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'sub':
        node = QoSSubscriberNode()
    else:
        node = QoSDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the QoS Demo

```bash
# Terminal 1: Start publisher
python3 qos_demo.py

# Terminal 2: Start subscriber immediately
python3 qos_demo.py sub

# Terminal 3: Start subscriber after 5 seconds (late-joiner)
sleep 5 && python3 qos_demo.py sub
# Notice: Late-joiner still receives config message due to TRANSIENT_LOCAL!
```

### Experiment: QoS Incompatibility

```bash
# Terminal 1: Publisher with BEST_EFFORT
ros2 topic pub /test std_msgs/String "data: 'hello'" \
    --qos-reliability best_effort

# Terminal 2: Subscriber demanding RELIABLE (won't connect!)
ros2 topic echo /test --qos-reliability reliable

# Check why they're not connecting
ros2 topic info /test -v
```

---

## Example 4: Multi-Node System

This example shows multiple nodes working together in a simple perception pipeline.

### Code: `camera_simulator.py`

```python
#!/usr/bin/env python3
"""
Camera Simulator Node
Publishes simulated camera images at 30Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class CameraSimulatorNode(Node):
    def __init__(self):
        super().__init__('camera_simulator')

        # Use sensor_data QoS profile (BEST_EFFORT for high-frequency data)
        self.publisher_ = self.create_publisher(
            Image,
            'camera/image_raw',
            qos_profile_sensor_data
        )

        # 30Hz camera
        self.timer = self.create_timer(1.0/30.0, self.publish_image)
        self.frame_count = 0

        self.get_logger().info('Camera simulator started (30Hz)')

    def publish_image(self):
        """Publish simulated image"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3

        # Simulate image data (normally from actual camera)
        msg.data = np.random.randint(0, 255, (msg.height, msg.width, 3), dtype=np.uint8).tobytes()

        self.publisher_.publish(msg)
        self.frame_count += 1

        if self.frame_count % 30 == 0:  # Log every second
            self.get_logger().debug(f'Published frame {self.frame_count}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Code: `image_processor.py`

```python
#!/usr/bin/env python3
"""
Image Processor Node
Receives images and performs simple processing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish detection results
        self.detection_pub = self.create_publisher(
            String,
            'detections',
            10
        )

        self.frame_count = 0
        self.get_logger().info('Image processor started')

    def image_callback(self, msg):
        """Process incoming image"""
        self.frame_count += 1

        # Simulate processing (normally: object detection, etc.)
        # Here we just check image size
        if msg.height == 480 and msg.width == 640:
            if self.frame_count % 30 == 0:  # Detect "object" every second
                detection_msg = String()
                detection_msg.data = f"Object detected at frame {self.frame_count}"
                self.detection_pub.publish(detection_msg)
                self.get_logger().info(detection_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Multi-Node System

```bash
# Terminal 1: Camera simulator
python3 camera_simulator.py

# Terminal 2: Image processor
python3 image_processor.py

# Terminal 3: Monitor detections
ros2 topic echo /detections

# Terminal 4: Visualize node graph
ros2 run rqt_graph rqt_graph

# Terminal 5: Check system performance
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw  # Bandwidth usage
```

---

## Example 5: Using ROS 2 CLI Tools

### Introspection Commands

```bash
# List all running nodes
ros2 node list

# Get info about a specific node
ros2 node info /temperature_sensor

# List all topics
ros2 topic list

# Get detailed topic info (including QoS)
ros2 topic info /robot/temperature -v

# Echo topic messages
ros2 topic echo /robot/temperature

# Publish from command line
ros2 topic pub /robot/temperature sensor_msgs/Temperature \
    "{header: {frame_id: 'base_link'}, temperature: 25.0, variance: 0.1}"

# Check publishing rate
ros2 topic hz /robot/temperature

# Check bandwidth usage
ros2 topic bw /camera/image_raw

# View message type definition
ros2 interface show sensor_msgs/msg/Temperature

# List all parameters of a node
ros2 param list /temperature_monitor

# Get parameter value
ros2 param get /temperature_monitor warning_threshold

# Set parameter value
ros2 param set /temperature_monitor warning_threshold 28.0
```

---

## Common Pitfalls and Solutions

### Pitfall 1: QoS Incompatibility

**Problem**: Subscriber not receiving messages

```bash
# Check QoS settings
ros2 topic info /my_topic -v
```

**Solution**: Ensure publisher and subscriber QoS are compatible

### Pitfall 2: Forgetting to Source Setup

**Problem**: `ros2: command not found`

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Pitfall 3: Node Name Conflicts

**Problem**: `Node name already exists`

**Solution**: Each node must have a unique name
```python
# Add namespace or unique suffix
super().__init__('temperature_sensor_1')
```

### Pitfall 4: Callback Blocking

**Problem**: Slow callback blocks other callbacks

**Solution**: Keep callbacks fast; offload heavy work to threads
```python
import threading

def image_callback(self, msg):
    # Quick: just queue the work
    threading.Thread(target=self.process_image, args=(msg,)).start()

def process_image(self, msg):
    # Heavy processing here
    pass
```

---

## Summary

These examples demonstrate:
1. ✅ Creating publishers and subscribers
2. ✅ Configuring QoS policies for different use cases
3. ✅ Using parameters for runtime configuration
4. ✅ Building multi-node systems
5. ✅ Using ROS 2 CLI tools for debugging

**Next Steps**:
- Try modifying these examples
- Combine them into a larger system
- Complete the [Hands-On Lab](./hands-on-lab) to build a full application
- Test your understanding with [Exercises](./exercises)

:::tip Best Practice
Always start with working examples and modify incrementally. Test each change before adding more complexity.
:::
