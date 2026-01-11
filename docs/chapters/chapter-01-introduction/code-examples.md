---
id: code-examples
title: Code Examples
sidebar_position: 7
---

# Code Examples

This section provides executable code examples to help you understand Physical AI concepts through practical implementation.

## Example 1: Hello Physical AI

A simple Python script to verify your environment setup.

```python
#!/usr/bin/env python3
"""
Hello Physical AI - Environment Verification
This script verifies your Python environment is ready for Physical AI development.
"""

import sys
import platform

def check_environment():
    """Check and display environment information."""
    print("=" * 50)
    print("Physical AI Environment Check")
    print("=" * 50)

    # Python version
    print(f"\n✓ Python Version: {sys.version}")
    print(f"✓ Platform: {platform.system()} {platform.release()}")
    print(f"✓ Architecture: {platform.machine()}")

    # Check required modules
    required_modules = ['numpy', 'matplotlib']
    print("\nChecking required modules:")

    for module in required_modules:
        try:
            __import__(module)
            print(f"  ✓ {module}: installed")
        except ImportError:
            print(f"  ✗ {module}: NOT installed")

    print("\n" + "=" * 50)
    print("Environment check complete!")
    print("=" * 50)

if __name__ == "__main__":
    check_environment()
```

**Expected Output:**
```
==================================================
Physical AI Environment Check
==================================================

✓ Python Version: 3.11.x
✓ Platform: Linux 5.15.x
✓ Architecture: x86_64

Checking required modules:
  ✓ numpy: installed
  ✓ matplotlib: installed

==================================================
Environment check complete!
==================================================
```

**How to Run:**
```bash
python3 hello_physical_ai.py
```

---

## Example 2: Simple Robot State Representation

Understanding how robots represent their state in Physical AI systems.

```python
#!/usr/bin/env python3
"""
Robot State Representation
Demonstrates how to represent and update robot state in Physical AI.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple

@dataclass
class RobotState:
    """Represents the state of a simple mobile robot."""
    position: np.ndarray  # [x, y] in meters
    orientation: float    # theta in radians
    velocity: float       # linear velocity in m/s
    angular_velocity: float  # angular velocity in rad/s

    def __str__(self):
        return (f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}) m\n"
                f"Orientation: {np.degrees(self.orientation):.2f}°\n"
                f"Velocity: {self.velocity:.2f} m/s\n"
                f"Angular Velocity: {self.angular_velocity:.2f} rad/s")

def update_state(state: RobotState, dt: float) -> RobotState:
    """
    Update robot state using simple kinematic model.

    Args:
        state: Current robot state
        dt: Time step in seconds

    Returns:
        Updated robot state
    """
    # Update orientation
    new_orientation = state.orientation + state.angular_velocity * dt

    # Update position based on velocity and orientation
    dx = state.velocity * np.cos(state.orientation) * dt
    dy = state.velocity * np.sin(state.orientation) * dt
    new_position = state.position + np.array([dx, dy])

    return RobotState(
        position=new_position,
        orientation=new_orientation,
        velocity=state.velocity,
        angular_velocity=state.angular_velocity
    )

def simulate_robot_motion():
    """Simulate simple robot motion over time."""
    # Initial state: robot at origin, facing east, moving forward
    state = RobotState(
        position=np.array([0.0, 0.0]),
        orientation=0.0,  # 0 radians = facing east
        velocity=1.0,     # 1 m/s forward
        angular_velocity=0.1  # 0.1 rad/s turning left
    )

    print("Initial State:")
    print(state)
    print("\n" + "=" * 50 + "\n")

    # Simulate for 5 seconds with 0.5 second time steps
    dt = 0.5
    total_time = 5.0
    steps = int(total_time / dt)

    for step in range(1, steps + 1):
        state = update_state(state, dt)
        print(f"After {step * dt:.1f} seconds:")
        print(state)
        print("\n" + "-" * 50 + "\n")

if __name__ == "__main__":
    simulate_robot_motion()
```

**Expected Output:**
```
Initial State:
Position: (0.00, 0.00) m
Orientation: 0.00°
Velocity: 1.00 m/s
Angular Velocity: 0.10 rad/s

==================================================

After 0.5 seconds:
Position: (0.50, 0.00) m
Orientation: 2.87°
Velocity: 1.00 m/s
Angular Velocity: 0.10 rad/s

--------------------------------------------------
...
```

---

## Example 3: Sensor Data Processing

Processing sensor data is fundamental to Physical AI systems.

```python
#!/usr/bin/env python3
"""
Sensor Data Processing
Demonstrates basic sensor data filtering and processing for Physical AI.
"""

import numpy as np
import matplotlib.pyplot as plt

class SimpleSensor:
    """Simulates a noisy distance sensor."""

    def __init__(self, true_distance: float, noise_std: float = 0.1):
        """
        Initialize sensor.

        Args:
            true_distance: Actual distance to object in meters
            noise_std: Standard deviation of sensor noise
        """
        self.true_distance = true_distance
        self.noise_std = noise_std

    def read(self) -> float:
        """Read sensor with added Gaussian noise."""
        noise = np.random.normal(0, self.noise_std)
        return self.true_distance + noise

class MovingAverageFilter:
    """Simple moving average filter for sensor data."""

    def __init__(self, window_size: int = 5):
        """
        Initialize filter.

        Args:
            window_size: Number of samples to average
        """
        self.window_size = window_size
        self.buffer = []

    def update(self, value: float) -> float:
        """
        Add new value and return filtered result.

        Args:
            value: New sensor reading

        Returns:
            Filtered value
        """
        self.buffer.append(value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return np.mean(self.buffer)

def demonstrate_filtering():
    """Demonstrate sensor noise filtering."""
    # Create sensor measuring 2.0 meters with noise
    sensor = SimpleSensor(true_distance=2.0, noise_std=0.2)
    filter = MovingAverageFilter(window_size=10)

    # Collect data
    num_samples = 100
    raw_readings = []
    filtered_readings = []

    for _ in range(num_samples):
        raw = sensor.read()
        filtered = filter.update(raw)
        raw_readings.append(raw)
        filtered_readings.append(filtered)

    # Display statistics
    print("Sensor Data Analysis")
    print("=" * 50)
    print(f"True Distance: {sensor.true_distance:.2f} m")
    print(f"\nRaw Readings:")
    print(f"  Mean: {np.mean(raw_readings):.3f} m")
    print(f"  Std Dev: {np.std(raw_readings):.3f} m")
    print(f"\nFiltered Readings:")
    print(f"  Mean: {np.mean(filtered_readings):.3f} m")
    print(f"  Std Dev: {np.std(filtered_readings):.3f} m")
    print(f"\nNoise Reduction: {(1 - np.std(filtered_readings)/np.std(raw_readings))*100:.1f}%")

    # Plot results (optional - requires matplotlib)
    try:
        plt.figure(figsize=(10, 6))
        plt.plot(raw_readings, 'b-', alpha=0.5, label='Raw Sensor Data')
        plt.plot(filtered_readings, 'r-', linewidth=2, label='Filtered Data')
        plt.axhline(y=sensor.true_distance, color='g', linestyle='--', label='True Distance')
        plt.xlabel('Sample Number')
        plt.ylabel('Distance (m)')
        plt.title('Sensor Data Filtering for Physical AI')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig('sensor_filtering.png')
        print("\n✓ Plot saved as 'sensor_filtering.png'")
    except ImportError:
        print("\n(matplotlib not available - skipping plot)")

if __name__ == "__main__":
    demonstrate_filtering()
```

**Expected Output:**
```
Sensor Data Analysis
==================================================
True Distance: 2.00 m

Raw Readings:
  Mean: 2.003 m
  Std Dev: 0.198 m

Filtered Readings:
  Mean: 2.001 m
  Std Dev: 0.089 m

Noise Reduction: 55.1%

✓ Plot saved as 'sensor_filtering.png'
```

---

## Running the Examples

### Setup
```bash
# Create a directory for code examples
mkdir -p ~/robotics-learning/chapter-01
cd ~/robotics-learning/chapter-01

# Install required packages
pip install numpy matplotlib
```

### Execute
```bash
# Run each example
python3 hello_physical_ai.py
python3 robot_state.py
python3 sensor_processing.py
```

## Key Takeaways

1. **Environment Verification**: Always verify your setup before starting development
2. **State Representation**: Robots maintain state (position, orientation, velocity)
3. **Sensor Processing**: Real sensors are noisy; filtering is essential
4. **Simulation**: Test algorithms in simulation before deploying to hardware

---

**Next Steps**: Try modifying these examples! Change parameters, add features, and experiment with the code to deepen your understanding.
