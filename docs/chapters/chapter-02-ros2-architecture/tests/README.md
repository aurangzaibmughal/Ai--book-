# ROS 2 Testing Examples

This directory contains test examples for Chapter 2 (ROS 2 Architecture). These tests demonstrate best practices for testing ROS 2 nodes.

## Test Types

### Unit Tests (`test_temperature_publisher.py`)
Tests individual node functionality in isolation using pytest and ROS 2 testing utilities.

### Integration Tests (`test_integration.py`)
Tests multiple nodes working together using launch_testing framework.

---

## Prerequisites

Install testing dependencies:

```bash
# Install pytest and ROS 2 testing tools
sudo apt install python3-pytest ros-humble-launch-testing ros-humble-launch-testing-ament-cmake

# Install pytest plugins for ROS 2
pip3 install pytest-timeout
```

---

## Running Tests

### Run All Tests

```bash
# From the workspace root
cd ~/ros2_ws
colcon test --packages-select my_robot_package
colcon test-result --verbose
```

### Run Specific Test File

```bash
# Run unit tests only
pytest src/my_robot_package/tests/test_temperature_publisher.py -v

# Run integration tests only
pytest src/my_robot_package/tests/test_integration.py -v
```

### Run with Coverage

```bash
# Install coverage tool
pip3 install pytest-cov

# Run tests with coverage report
pytest src/my_robot_package/tests/ --cov=my_robot_package --cov-report=html
```

---

## Test Structure

```
tests/
├── README.md                      # This file
├── test_temperature_publisher.py  # Unit tests for publisher node
└── test_integration.py            # Integration tests for multi-node systems
```

---

## Writing Your Own Tests

### Unit Test Template

```python
import pytest
import rclpy
from rclpy.node import Node

def test_my_node():
    rclpy.init()
    node = MyNode()
    # Test node functionality
    assert node.some_property == expected_value
    node.destroy_node()
    rclpy.shutdown()
```

### Integration Test Template

```python
import launch
import launch_testing
import pytest

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        # Launch your nodes here
    ])

class TestIntegration:
    def test_nodes_communicate(self):
        # Test inter-node communication
        pass
```

---

## Best Practices

1. **Isolate Tests**: Each test should be independent and not rely on other tests
2. **Use Fixtures**: Use pytest fixtures for common setup/teardown
3. **Mock External Dependencies**: Mock hardware sensors, external services
4. **Test Edge Cases**: Test error conditions, boundary values, timeouts
5. **Keep Tests Fast**: Unit tests should run in milliseconds, integration tests in seconds
6. **Use Descriptive Names**: Test names should clearly describe what they test

---

## Common Issues

### Issue: "rclpy already initialized"
**Solution**: Ensure `rclpy.shutdown()` is called in teardown or use fixtures

### Issue: "Node not found in test"
**Solution**: Check that node is properly launched in test description

### Issue: "Test timeout"
**Solution**: Increase timeout or check for deadlocks in node callbacks

---

## Resources

- [ROS 2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [pytest Documentation](https://docs.pytest.org/)
- [launch_testing Documentation](https://github.com/ros2/launch/tree/humble/launch_testing)

---

**Note**: These are educational examples. In production, you would have more comprehensive test coverage including edge cases, error handling, and performance tests.
