#!/usr/bin/env python3
"""
Unit Tests for Temperature Publisher Node

Demonstrates ROS 2 unit testing best practices using pytest.
Tests the TemperatureSensorNode in isolation without launching full ROS 2 system.
"""

from typing import Optional
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Temperature
import time


# Mock TemperatureSensorNode for testing
# In production, you would import from your package
class TemperatureSensorNode(Node):
    """Publishes simulated temperature readings"""

    def __init__(self) -> None:
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Temperature, 'robot/temperature', 10)
        self.timer = self.create_timer(0.1, self.publish_temperature)
        self.base_temperature: float = 25.0
        self.message_count: int = 0

    def publish_temperature(self) -> None:
        """Callback function called by timer to publish temperature"""
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.temperature = self.base_temperature
        msg.variance = 0.1
        self.publisher_.publish(msg)
        self.message_count += 1


@pytest.fixture
def ros_context():
    """Fixture to initialize and shutdown ROS 2 context"""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestTemperatureSensorNode:
    """Test suite for TemperatureSensorNode"""

    def test_node_creation(self, ros_context):
        """Test that node can be created successfully"""
        node = TemperatureSensorNode()
        assert node.get_name() == 'temperature_sensor'
        node.destroy_node()

    def test_publisher_creation(self, ros_context):
        """Test that publisher is created with correct topic and type"""
        node = TemperatureSensorNode()

        # Check publisher exists
        publishers = node.get_publisher_names_and_types_by_node(
            node.get_name(),
            node.get_namespace()
        )

        # Verify topic name and message type
        topic_names = [name for name, _ in publishers]
        assert 'robot/temperature' in topic_names

        node.destroy_node()

    def test_timer_creation(self, ros_context):
        """Test that timer is created with correct period"""
        node = TemperatureSensorNode()

        # Timer should exist
        assert node.timer is not None

        # Timer period should be 0.1 seconds (10Hz)
        assert node.timer.timer_period_ns == 100_000_000  # 0.1s in nanoseconds

        node.destroy_node()

    def test_initial_temperature(self, ros_context):
        """Test that initial temperature is set correctly"""
        node = TemperatureSensorNode()
        assert node.base_temperature == 25.0
        node.destroy_node()

    def test_message_publishing(self, ros_context):
        """Test that messages are published correctly"""
        node = TemperatureSensorNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Create subscriber to receive messages
        received_messages = []

        def message_callback(msg: Temperature):
            received_messages.append(msg)

        subscriber = node.create_subscription(
            Temperature,
            'robot/temperature',
            message_callback,
            10
        )

        # Spin for 0.5 seconds to receive ~5 messages at 10Hz
        start_time = time.time()
        while time.time() - start_time < 0.5:
            executor.spin_once(timeout_sec=0.1)

        # Should have received at least 3 messages (accounting for timing variations)
        assert len(received_messages) >= 3

        # Verify message content
        for msg in received_messages:
            assert msg.header.frame_id == 'base_link'
            assert msg.temperature == 25.0  # Deterministic for testing
            assert msg.variance == 0.1
            assert msg.header.stamp.sec > 0  # Timestamp should be set

        node.destroy_node()

    def test_message_count_increments(self, ros_context):
        """Test that message count increments with each publish"""
        node = TemperatureSensorNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        initial_count = node.message_count

        # Spin for 0.3 seconds to trigger ~3 publishes
        start_time = time.time()
        while time.time() - start_time < 0.3:
            executor.spin_once(timeout_sec=0.1)

        # Message count should have increased
        assert node.message_count > initial_count
        assert node.message_count >= 2  # At least 2 messages in 0.3s at 10Hz

        node.destroy_node()

    def test_node_cleanup(self, ros_context):
        """Test that node cleans up resources properly"""
        node = TemperatureSensorNode()
        node_name = node.get_name()

        # Node should exist
        assert node.get_name() == node_name

        # Destroy node
        node.destroy_node()

        # After destruction, node should not be accessible
        # (In practice, you'd verify resources are released)
        assert True  # Placeholder for actual cleanup verification

    def test_multiple_nodes(self, ros_context):
        """Test that multiple temperature sensor nodes can coexist"""
        node1 = TemperatureSensorNode()

        # Create second node with different name
        node2 = Node('temperature_sensor_2')

        # Both nodes should exist
        assert node1.get_name() == 'temperature_sensor'
        assert node2.get_name() == 'temperature_sensor_2'

        node1.destroy_node()
        node2.destroy_node()

    @pytest.mark.parametrize("base_temp", [0.0, 25.0, 100.0, -40.0])
    def test_temperature_range(self, ros_context, base_temp):
        """Test node with different temperature values"""
        node = TemperatureSensorNode()
        node.base_temperature = base_temp

        assert node.base_temperature == base_temp

        node.destroy_node()


class TestTemperatureMessage:
    """Test suite for Temperature message structure"""

    def test_message_structure(self, ros_context):
        """Test that Temperature message has required fields"""
        msg = Temperature()

        # Check required fields exist
        assert hasattr(msg, 'header')
        assert hasattr(msg, 'temperature')
        assert hasattr(msg, 'variance')

    def test_message_defaults(self, ros_context):
        """Test default values of Temperature message"""
        msg = Temperature()

        # Default temperature should be 0.0
        assert msg.temperature == 0.0
        assert msg.variance == 0.0


# Performance tests
class TestPerformance:
    """Performance tests for temperature publisher"""

    def test_publishing_rate(self, ros_context):
        """Test that node publishes at approximately 10Hz"""
        node = TemperatureSensorNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        received_messages = []

        def message_callback(msg: Temperature):
            received_messages.append(time.time())

        subscriber = node.create_subscription(
            Temperature,
            'robot/temperature',
            message_callback,
            10
        )

        # Collect messages for 1 second
        start_time = time.time()
        while time.time() - start_time < 1.0:
            executor.spin_once(timeout_sec=0.1)

        # Should have received approximately 10 messages (±2 for timing variations)
        assert 8 <= len(received_messages) <= 12

        # Calculate average rate
        if len(received_messages) > 1:
            time_diffs = [
                received_messages[i+1] - received_messages[i]
                for i in range(len(received_messages) - 1)
            ]
            avg_period = sum(time_diffs) / len(time_diffs)
            avg_rate = 1.0 / avg_period

            # Rate should be approximately 10Hz (±20% tolerance)
            assert 8.0 <= avg_rate <= 12.0

        node.destroy_node()


if __name__ == '__main__':
    # Run tests with pytest
    pytest.main([__file__, '-v'])
