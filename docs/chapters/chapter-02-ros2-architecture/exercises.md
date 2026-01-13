---
id: exercises
title: "Exercises: ROS 2 Architecture"
sidebar_label: "Exercises"
sidebar_position: 8
description: "Practice exercises to reinforce ROS 2 architecture concepts"
---

# Exercises: ROS 2 Architecture

These exercises reinforce the concepts learned in this chapter. Complete them in order, as they build on each other.

## Exercise 1: QoS Policy Configuration (Beginner)

**Objective**: Understand how different QoS policies affect message delivery.

### Task

Create two nodes:
1. **Publisher**: Publishes messages with different QoS profiles
2. **Subscriber**: Subscribes with various QoS profiles to test compatibility

### Requirements

```python
# Create a publisher that publishes to three topics:
# - /reliable_topic (RELIABLE QoS)
# - /best_effort_topic (BEST_EFFORT QoS)
# - /transient_topic (TRANSIENT_LOCAL durability)

# Create subscribers that:
# 1. Successfully receive from /reliable_topic
# 2. Successfully receive from /best_effort_topic
# 3. Fail to receive due to QoS mismatch
# 4. Receive historical data from /transient_topic when started late
```

### Expected Output

```
[Publisher] Publishing to reliable_topic: Message 1
[Publisher] Publishing to best_effort_topic: Message 1
[Publisher] Publishing to transient_topic: Config data

[Subscriber 1] Received from reliable_topic: Message 1
[Subscriber 2] Received from best_effort_topic: Message 1
[Subscriber 3] ERROR: No data (QoS mismatch)
[Subscriber 4] Received historical data: Config data
```

### Hints

- Use `QoSProfile` to configure reliability and durability
- Test QoS compatibility with `ros2 topic info -v`
- Start subscriber 4 after publisher has been running for 5 seconds

### Solution Verification

```bash
# Check QoS settings
ros2 topic info /reliable_topic -v
ros2 topic info /best_effort_topic -v
ros2 topic info /transient_topic -v

# Verify incompatibility
ros2 topic echo /reliable_topic --qos-reliability best_effort
# Should show: "Could not connect to topic"
```

---

## Exercise 2: Multi-Sensor Fusion (Intermediate)

**Objective**: Combine data from multiple sensors with different publishing rates.

### Task

Create a sensor fusion node that:
1. Subscribes to camera images (30Hz, BEST_EFFORT)
2. Subscribes to LIDAR scans (10Hz, BEST_EFFORT)
3. Subscribes to IMU data (100Hz, RELIABLE)
4. Publishes synchronized sensor data (10Hz, RELIABLE)

### Requirements

```python
#!/usr/bin/env python3
"""
Sensor Fusion Node
Demonstrates multi-sensor data fusion with different rates and QoS policies
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import String
from collections import deque
from typing import Optional


class SensorFusionNode(Node):
    """
    Fuses data from multiple sensors
    - Buffers high-frequency data (camera, IMU)
    - Synchronizes to LIDAR rate (10Hz)
    - Publishes combined sensor state
    """

    def __init__(self):
        super().__init__('sensor_fusion')

        # Create subscribers with appropriate QoS policies
        # Camera: 30Hz, BEST_EFFORT (occasional frame drops acceptable)
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.camera_sub = self.create_subscription(
            Image,
            'camera/image',
            self.camera_callback,
            camera_qos
        )

        # LIDAR: 10Hz, BEST_EFFORT (drives fusion rate)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'lidar/scan',
            self.lidar_callback,
            lidar_qos
        )

        # IMU: 100Hz, RELIABLE (critical for robot stability)
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50  # Larger buffer for high-frequency data
        )
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            imu_qos
        )

        # Create publisher for fused data (10Hz, RELIABLE)
        # RELIABLE because downstream nodes need guaranteed delivery
        fused_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.fused_pub = self.create_publisher(
            String,  # In production, use custom message type
            'sensor/fused',
            fused_qos
        )

        # Implement time-based synchronization
        # Buffer recent messages from high-frequency sensors
        # We keep last 1 second of data for synchronization
        self.camera_buffer = deque(maxlen=30)  # 30 frames at 30Hz = 1 second
        self.imu_buffer = deque(maxlen=100)    # 100 samples at 100Hz = 1 second

        # Track latest LIDAR scan (slowest sensor, drives fusion)
        self.latest_lidar: Optional[LaserScan] = None
        self.latest_lidar_time: Optional[float] = None

        # Counters for logging
        self.camera_count = 0
        self.lidar_count = 0
        self.imu_count = 0

        self.get_logger().info('Sensor fusion node started')
        self.get_logger().info('Waiting for sensor data...')

    def camera_callback(self, msg: Image):
        """
        Callback for camera images (30Hz)
        Buffers frames for time-based synchronization
        """
        timestamp = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        self.camera_buffer.append((timestamp, msg))
        self.camera_count += 1

        self.get_logger().debug(
            f'Received camera frame {self.camera_count} at t={timestamp:.3f}s'
        )

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU data (100Hz)
        Buffers samples for time-based synchronization
        """
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.imu_buffer.append((timestamp, msg))
        self.imu_count += 1

        self.get_logger().debug(
            f'Received IMU sample {self.imu_count} at t={timestamp:.3f}s'
        )

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for LIDAR scans (10Hz)
        Triggers sensor fusion when new scan arrives

        LIDAR is the slowest sensor, so we use it to drive the fusion rate.
        When a LIDAR scan arrives, we find the closest camera frame and
        IMU samples within a time window and publish fused data.
        """
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.latest_lidar = msg
        self.latest_lidar_time = timestamp
        self.lidar_count += 1

        self.get_logger().info(
            f'Received LIDAR scan {self.lidar_count} at t={timestamp:.3f}s'
        )

        # Trigger fusion when LIDAR data arrives
        self.publish_fused_data()

    def publish_fused_data(self):
        """
        Publishes synchronized sensor data

        Synchronization strategy:
        1. Use LIDAR timestamp as reference (slowest sensor)
        2. Find closest camera frame within 50ms window
        3. Collect all IMU samples within 100ms window
        4. Publish combined sensor state
        """
        if self.latest_lidar is None:
            return

        if not self.camera_buffer or not self.imu_buffer:
            self.get_logger().warn('Waiting for all sensors to publish data')
            return

        lidar_time = self.latest_lidar_time
        time_tolerance = 0.05  # 50ms tolerance for camera sync

        # Find closest camera frame to LIDAR timestamp
        closest_camera = None
        min_time_diff = float('inf')

        for cam_time, cam_msg in self.camera_buffer:
            time_diff = abs(cam_time - lidar_time)
            if time_diff < min_time_diff and time_diff < time_tolerance:
                min_time_diff = time_diff
                closest_camera = (cam_time, cam_msg)

        # Collect IMU samples within 100ms window around LIDAR timestamp
        imu_window = 0.1  # 100ms window
        synchronized_imu = [
            (imu_time, imu_msg)
            for imu_time, imu_msg in self.imu_buffer
            if abs(imu_time - lidar_time) < imu_window
        ]

        # Publish fused data
        if closest_camera is not None:
            fused_msg = String()
            fused_msg.data = (
                f'Fused data: '
                f'camera_frame={self.camera_count}, '
                f'lidar_scan={self.lidar_count}, '
                f'imu_samples={len(synchronized_imu)}, '
                f'sync_error={min_time_diff*1000:.1f}ms'
            )

            self.fused_pub.publish(fused_msg)

            self.get_logger().info(
                f'Publishing fused data: '
                f'camera_frame={self.camera_count}, '
                f'lidar_scan={self.lidar_count}, '
                f'imu_samples={len(synchronized_imu)}'
            )
        else:
            self.get_logger().warn(
                f'No camera frame within {time_tolerance*1000}ms of LIDAR scan'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Expected Behavior

```
[Fusion] Received camera frame 1 at t=0.033s
[Fusion] Received camera frame 2 at t=0.066s
[Fusion] Received camera frame 3 at t=0.100s
[Fusion] Received LIDAR scan at t=0.100s
[Fusion] Publishing fused data: camera_frame=3, lidar_scan=1, imu_samples=10
```

### Hints

- Use `self.get_clock().now()` for timestamps
- Buffer recent messages from high-frequency sensors
- Publish when LIDAR data arrives (slowest sensor)

### Bonus Challenge

Implement a time-based synchronization algorithm that matches sensor data within 10ms tolerance.

---

## Exercise 3: Node Lifecycle Management (Intermediate)

**Objective**: Implement proper node initialization and shutdown.

### Task

Create a camera driver node with lifecycle management:
1. **Configuring**: Load parameters, validate settings
2. **Activating**: Initialize camera, start publishing
3. **Deactivating**: Stop publishing, keep camera open
4. **Cleaning up**: Close camera, release resources

### Requirements

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

class CameraDriverNode(LifecycleNode):
    """Camera driver with lifecycle management"""

    def on_configure(self, state: LifecycleState):
        # Load parameters
        # Validate camera settings
        # Return SUCCESS or FAILURE
        pass

    def on_activate(self, state: LifecycleState):
        # Initialize camera
        # Start publishing
        pass

    def on_deactivate(self, state: LifecycleState):
        # Stop publishing
        pass

    def on_cleanup(self, state: LifecycleState):
        # Release camera resources
        pass
```

### Expected Output

```bash
# Terminal 1: Run lifecycle node
ros2 run my_package camera_driver

# Terminal 2: Control lifecycle
ros2 lifecycle set /camera_driver configure
# [Camera] Configuring... Parameters loaded

ros2 lifecycle set /camera_driver activate
# [Camera] Activating... Camera initialized, publishing started

ros2 lifecycle set /camera_driver deactivate
# [Camera] Deactivating... Publishing stopped

ros2 lifecycle set /camera_driver cleanup
# [Camera] Cleaning up... Camera released
```

### Hints

- Use `LifecycleNode` instead of `Node`
- Return `TransitionCallbackReturn.SUCCESS` or `FAILURE`
- Test with `ros2 lifecycle` commands

---

## Exercise 4: Custom Message Types (Intermediate)

**Objective**: Define and use custom ROS 2 message types.

### Task

Create a custom message for robot status:

```
# RobotStatus.msg
std_msgs/Header header
float32 battery_percentage
float32 temperature
string health_status
geometry_msgs/Pose current_pose
string[] active_tasks
```

### Requirements

1. Create message definition in `msg/RobotStatus.msg`
2. Update `CMakeLists.txt` and `package.xml`
3. Build and verify message generation
4. Create publisher and subscriber using custom message

### Steps

```bash
# 1. Create message directory
mkdir -p ~/ros2_ws/src/my_robot/msg

# 2. Create RobotStatus.msg file

# 3. Update package.xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

# 4. Update CMakeLists.txt (for C++ packages)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 5. Build
colcon build --packages-select my_robot

# 6. Verify
ros2 interface show my_robot/msg/RobotStatus
```

### Expected Output

```python
from my_robot.msg import RobotStatus

msg = RobotStatus()
msg.battery_percentage = 85.5
msg.temperature = 26.3
msg.health_status = "HEALTHY"
# ... publish msg
```

---

## Exercise 5: Parameter Server Integration (Advanced)

**Objective**: Implement dynamic parameter updates with validation.

### Task

Create a motor controller node that:
1. Declares parameters with constraints
2. Validates parameter changes
3. Applies parameters dynamically
4. Saves parameters to file

### Requirements

```python
class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters with descriptors
        self.declare_parameter(
            'max_speed',
            1.0,
            ParameterDescriptor(
                description='Maximum speed in m/s',
                type=ParameterType.PARAMETER_DOUBLE,
                read_only=False,
                additional_constraints='Must be between 0.1 and 5.0'
            )
        )

        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Validate and apply parameter changes"""
        for param in params:
            if param.name == 'max_speed':
                if param.value < 0.1 or param.value > 5.0:
                    return SetParametersResult(successful=False)
                # Apply new speed limit
                self.max_speed = param.value

        return SetParametersResult(successful=True)
```

### Expected Behavior

```bash
# Set valid parameter
ros2 param set /motor_controller max_speed 2.5
# Output: Set parameter successful

# Set invalid parameter
ros2 param set /motor_controller max_speed 10.0
# Output: Setting parameter failed: Value out of range

# Dump parameters to file
ros2 param dump /motor_controller --output-dir ~/params

# Load parameters from file
ros2 run my_package motor_controller --ros-args --params-file ~/params/motor_controller.yaml
```

---

## Exercise 6: Multi-Robot System (Advanced)

**Objective**: Create a system that coordinates multiple robots using ROS domains.

### Task

Simulate a warehouse with 3 robots:
1. Each robot has its own ROS domain
2. A central coordinator communicates with all robots
3. Robots publish their positions
4. Coordinator assigns tasks based on robot locations

### Requirements

```python
# Robot node (runs in domain 1, 2, or 3)
class WarehouseRobotNode(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}')
        # Publish position in own domain
        # Subscribe to task assignments
        pass

# Coordinator node (runs in domain 0, bridges to 1, 2, 3)
class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')
        # Subscribe to all robot positions
        # Publish task assignments
        pass
```

### Running the System

```bash
# Terminal 1: Robot 1 (domain 1)
export ROS_DOMAIN_ID=1
ros2 run warehouse robot --ros-args -p robot_id:=1

# Terminal 2: Robot 2 (domain 2)
export ROS_DOMAIN_ID=2
ros2 run warehouse robot --ros-args -p robot_id:=2

# Terminal 3: Robot 3 (domain 3)
export ROS_DOMAIN_ID=3
ros2 run warehouse robot --ros-args -p robot_id:=3

# Terminal 4: Coordinator (domain 0, bridges to all)
export ROS_DOMAIN_ID=0
ros2 run warehouse coordinator
```

### Hints

- Use domain bridging or run coordinator in each domain
- Consider using `ros2 topic list` with `--include-hidden-topics`
- Implement task assignment algorithm (e.g., nearest robot)

---

## Exercise 7: Performance Optimization (Advanced)

**Objective**: Optimize a ROS 2 system for real-time performance.

### Task

Given a slow image processing pipeline, optimize it to achieve 30Hz throughput:

```python
# Original (slow) implementation
class ImageProcessorNode(Node):
    def image_callback(self, msg):
        # Blocking operation - processes image synchronously
        processed = self.process_image(msg)  # Takes 50ms
        self.publisher_.publish(processed)
```

### Requirements

Optimize using:
1. **Callback groups**: Parallel callback execution
2. **Executors**: Multi-threaded processing
3. **Intra-process communication**: Zero-copy message passing
4. **QoS tuning**: Reduce latency

### Solution Approach

```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

class OptimizedImageProcessorNode(Node):
    def __init__(self):
        super().__init__('optimized_processor')

        # Use reentrant callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe with callback group
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        # Thread pool for processing
        self.executor = ThreadPoolExecutor(max_workers=4)

    def image_callback(self, msg):
        # Non-blocking: submit to thread pool
        self.executor.submit(self.process_and_publish, msg)

    def process_and_publish(self, msg):
        processed = self.process_image(msg)
        self.publisher_.publish(processed)

# Run with multi-threaded executor
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

### Performance Targets

- **Latency**: < 35ms (camera to processed output)
- **Throughput**: 30Hz sustained
- **CPU Usage**: < 50% on single core
- **Memory**: No memory leaks over 1 hour

### Measurement

```bash
# Measure latency
ros2 topic delay /processed_image

# Measure throughput
ros2 topic hz /processed_image

# Profile CPU usage
ros2 run performance_test perf_test
```

---

## Exercise 8: Debugging Challenge (Advanced)

**Objective**: Debug a broken ROS 2 system using systematic troubleshooting.

### Scenario

You're given a robot system that isn't working correctly:
- Camera node publishes images
- Object detector subscribes but receives no data
- Motion planner crashes randomly
- System becomes unresponsive after 5 minutes

### Your Task

Use ROS 2 tools to diagnose and fix all issues.

### Debugging Checklist

```bash
# 1. Check node status
ros2 node list
ros2 node info /object_detector

# 2. Check topic connections
ros2 topic list
ros2 topic info /camera/image -v

# 3. Check QoS compatibility
ros2 topic echo /camera/image --qos-reliability reliable

# 4. Monitor system resources
ros2 topic bw /camera/image
ros2 topic hz /camera/image

# 5. Check for errors
ros2 run rqt_console rqt_console

# 6. Inspect parameters
ros2 param list /motion_planner
ros2 param get /motion_planner max_speed

# 7. Record and replay
ros2 bag record -a
ros2 bag play <bag_file>
```

### Common Issues to Find

1. **QoS Mismatch**: Detector uses RELIABLE, camera uses BEST_EFFORT
2. **Memory Leak**: Motion planner doesn't destroy old messages
3. **Callback Blocking**: Long-running callback blocks executor
4. **Parameter Error**: Invalid parameter causes crash

### Solution Template

```markdown
## Issue 1: Object Detector Not Receiving Data
**Root Cause**: QoS incompatibility
**Evidence**: `ros2 topic info -v` shows mismatched QoS
**Fix**: Change detector to BEST_EFFORT QoS

## Issue 2: Motion Planner Crashes
**Root Cause**: Invalid parameter value
**Evidence**: Error log shows "max_speed must be positive"
**Fix**: Add parameter validation

## Issue 3: System Unresponsive After 5 Minutes
**Root Cause**: Memory leak in image buffer
**Evidence**: `top` shows increasing memory usage
**Fix**: Properly destroy old messages
```

---

## Bonus Exercise: Build a Complete System

**Objective**: Integrate all concepts into a functional robot system.

### Task

Build a "Security Robot" that:
1. Patrols a simulated environment
2. Detects intruders using camera
3. Sounds alarm when threat detected
4. Logs all events to database
5. Provides web dashboard for monitoring

### Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Camera    │────>│   Detector   │────>│   Alarm     │
│   Driver    │     │   (AI Model) │     │   System    │
└─────────────┘     └──────────────┘     └─────────────┘
                            │
                            ↓
                    ┌──────────────┐
                    │   Logger     │
                    │   (Database) │
                    └──────────────┘
                            │
                            ↓
                    ┌──────────────┐
                    │   Web API    │
                    │   (FastAPI)  │
                    └──────────────┘
```

### Requirements

- All nodes use appropriate QoS policies
- System handles node failures gracefully
- Performance: 10Hz detection rate
- Logging: All events persisted to SQLite
- Dashboard: Real-time status display

---

## Submission Guidelines

For each exercise, submit:
1. **Source code**: All Python files
2. **Package files**: `package.xml`, `setup.py`
3. **Documentation**: README with build/run instructions
4. **Screenshots**: Terminal output, rqt_graph
5. **Answers**: Response to reflection questions

### Grading Rubric

| Criteria | Points |
|----------|--------|
| Code Quality | 25% |
| Functionality | 35% |
| Documentation | 20% |
| Testing | 20% |

---

## Additional Challenges

1. **Performance**: Optimize Exercise 7 to achieve 60Hz
2. **Scalability**: Extend Exercise 6 to 10 robots
3. **Reliability**: Add fault tolerance to Exercise 8
4. **Integration**: Combine Exercises 2, 3, and 5

:::tip Learning Strategy
Don't rush through these exercises. Each one teaches important concepts you'll use throughout the course. Take time to experiment and understand why things work the way they do.
:::
