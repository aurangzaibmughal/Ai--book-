#!/usr/bin/env python3
"""
Integration Tests for ROS 2 Multi-Node Systems

Demonstrates integration testing with multiple nodes using launch_testing.
Tests communication between publisher and subscriber nodes.
"""

import unittest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Temperature
import time
from typing import List


# Mock nodes for integration testing
class TemperaturePublisherNode(Node):
    """Publisher node for integration tests"""

    def __init__(self) -> None:
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Temperature, 'robot/temperature', 10)
        self.timer = self.create_timer(0.1, self.publish_temperature)
        self.base_temperature: float = 25.0

    def publish_temperature(self) -> None:
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.temperature = self.base_temperature
        msg.variance = 0.1
        self.publisher_.publish(msg)


class TemperatureSubscriberNode(Node):
    """Subscriber node for integration tests"""

    def __init__(self) -> None:
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Temperature,
            'robot/temperature',
            self.temperature_callback,
            10
        )
        self.received_messages: List[Temperature] = []
        self.latest_temperature: float = 0.0

    def temperature_callback(self, msg: Temperature) -> None:
        self.received_messages.append(msg)
        self.latest_temperature = msg.temperature
        self.get_logger().info(f'Received: {msg.temperature:.2f}°C')


@pytest.fixture
def ros_context():
    """Fixture to initialize and shutdown ROS 2 context"""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestPublisherSubscriberIntegration:
    """Integration tests for publisher-subscriber communication"""

    def test_basic_communication(self, ros_context):
        """Test that publisher and subscriber can communicate"""
        # Create publisher and subscriber nodes
        publisher = TemperaturePublisherNode()
        subscriber = TemperatureSubscriberNode()

        # Create executor to spin both nodes
        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        # Spin for 0.5 seconds to allow communication
        start_time = time.time()
        while time.time() - start_time < 0.5:
            executor.spin_once(timeout_sec=0.1)

        # Subscriber should have received messages
        assert len(subscriber.received_messages) > 0
        assert subscriber.latest_temperature == 25.0

        publisher.destroy_node()
        subscriber.destroy_node()

    def test_message_ordering(self, ros_context):
        """Test that messages are received in order"""
        publisher = TemperaturePublisherNode()
        subscriber = TemperatureSubscriberNode()

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        # Spin for 1 second
        start_time = time.time()
        while time.time() - start_time < 1.0:
            executor.spin_once(timeout_sec=0.1)

        # Check that timestamps are monotonically increasing
        timestamps = [msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                     for msg in subscriber.received_messages]

        for i in range(len(timestamps) - 1):
            assert timestamps[i] <= timestamps[i+1], "Messages not in order"

        publisher.destroy_node()
        subscriber.destroy_node()

    def test_multiple_subscribers(self, ros_context):
        """Test that multiple subscribers can receive from same publisher"""
        publisher = TemperaturePublisherNode()
        subscriber1 = TemperatureSubscriberNode()

        # Create second subscriber with different name
        subscriber2 = Node('temperature_subscriber_2')
        received_messages_2 = []

        def callback2(msg):
            received_messages_2.append(msg)

        subscriber2.create_subscription(
            Temperature,
            'robot/temperature',
            callback2,
            10
        )

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber1)
        executor.add_node(subscriber2)

        # Spin for 0.5 seconds
        start_time = time.time()
        while time.time() - start_time < 0.5:
            executor.spin_once(timeout_sec=0.1)

        # Both subscribers should have received messages
        assert len(subscriber1.received_messages) > 0
        assert len(received_messages_2) > 0

        # Both should have received similar number of messages (±1)
        assert abs(len(subscriber1.received_messages) - len(received_messages_2)) <= 1

        publisher.destroy_node()
        subscriber1.destroy_node()
        subscriber2.destroy_node()

    def test_late_subscriber(self, ros_context):
        """Test that late subscriber receives messages after publisher starts"""
        publisher = TemperaturePublisherNode()

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)

        # Let publisher run for 0.3 seconds before subscriber joins
        start_time = time.time()
        while time.time() - start_time < 0.3:
            executor.spin_once(timeout_sec=0.1)

        # Now create subscriber (late joiner)
        subscriber = TemperatureSubscriberNode()
        executor.add_node(subscriber)

        # Spin for another 0.5 seconds
        start_time = time.time()
        while time.time() - start_time < 0.5:
            executor.spin_once(timeout_sec=0.1)

        # Late subscriber should still receive messages
        assert len(subscriber.received_messages) > 0

        publisher.destroy_node()
        subscriber.destroy_node()

    def test_publisher_restart(self, ros_context):
        """Test that subscriber continues receiving after publisher restarts"""
        publisher = TemperaturePublisherNode()
        subscriber = TemperatureSubscriberNode()

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        # Spin for 0.3 seconds
        start_time = time.time()
        while time.time() - start_time < 0.3:
            executor.spin_once(timeout_sec=0.1)

        initial_count = len(subscriber.received_messages)
        assert initial_count > 0

        # Destroy and recreate publisher
        publisher.destroy_node()
        publisher = TemperaturePublisherNode()
        executor.add_node(publisher)

        # Spin for another 0.3 seconds
        start_time = time.time()
        while time.time() - start_time < 0.3:
            executor.spin_once(timeout_sec=0.1)

        # Subscriber should have received more messages
        assert len(subscriber.received_messages) > initial_count

        publisher.destroy_node()
        subscriber.destroy_node()


class TestQoSCompatibility:
    """Integration tests for QoS policy compatibility"""

    def test_reliable_communication(self, ros_context):
        """Test RELIABLE QoS communication"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        # Create nodes with RELIABLE QoS
        publisher = Node('reliable_publisher')
        subscriber = Node('reliable_subscriber')

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        pub = publisher.create_publisher(Temperature, 'test_topic', reliable_qos)

        received = []
        def callback(msg):
            received.append(msg)

        sub = subscriber.create_subscription(
            Temperature,
            'test_topic',
            callback,
            reliable_qos
        )

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        # Publish messages
        for i in range(5):
            msg = Temperature()
            msg.temperature = float(i)
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.1)

        # All messages should be received with RELIABLE QoS
        assert len(received) == 5

        publisher.destroy_node()
        subscriber.destroy_node()


class TestErrorHandling:
    """Integration tests for error handling"""

    def test_no_publisher(self, ros_context):
        """Test subscriber behavior when no publisher exists"""
        subscriber = TemperatureSubscriberNode()

        executor = SingleThreadedExecutor()
        executor.add_node(subscriber)

        # Spin for 0.5 seconds with no publisher
        start_time = time.time()
        while time.time() - start_time < 0.5:
            executor.spin_once(timeout_sec=0.1)

        # Subscriber should not crash, just receive no messages
        assert len(subscriber.received_messages) == 0

        subscriber.destroy_node()

    def test_rapid_publish(self, ros_context):
        """Test system behavior under rapid publishing"""
        publisher = Node('rapid_publisher')
        subscriber = TemperatureSubscriberNode()

        pub = publisher.create_publisher(Temperature, 'robot/temperature', 10)

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        # Publish 100 messages rapidly
        for i in range(100):
            msg = Temperature()
            msg.temperature = float(i)
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.001)

        # Subscriber should handle rapid messages without crashing
        # (May not receive all due to queue depth)
        assert len(subscriber.received_messages) > 0
        assert len(subscriber.received_messages) <= 100

        publisher.destroy_node()
        subscriber.destroy_node()


if __name__ == '__main__':
    # Run tests with pytest
    pytest.main([__file__, '-v'])
