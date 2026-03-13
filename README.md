# kangaroo_hw_interface

A ROS2 `ros2_control` **hardware interface plugin** for the  Kangaroo x2 motor controller, communicating over UART serial.

Tested on **ROS2 Humble**.

---

## What it does

Implements `hardware_interface::SystemInterface` to expose two wheel joints:
| Interface | Type | Description |
|---|---|---|
| `joint_wheel_fl` / `joint_wheel_fr` | `velocity` (command) | rad/s setpoint sent to Kangaroo channels 1 & 2 |
| `joint_wheel_fl` / `joint_wheel_fr` | `velocity` (state) | rad/s feedback read from Kangaroo |
| `joint_wheel_fl` / `joint_wheel_fr` | `position` (state) | rad accumulated position read from Kangaroo |

The states are based on motor encoder reading and they're translated into the **/joint_states** ROS topic

---

## Hardware parameters

Configured in the `<hardware>` block of your `ros2_control` URDF:

| Parameter | Type | Example | Description |
|---|---|---|---|
| `serial_port` | string | `/dev/ttyTHS1` | Serial device connected to Kangaroo TX/RX |
| `baudrate` | int | `9600` | Must match software configuration |

---

## Mechanical constants

Defined at the top of `src/kangaroo_system.cpp` — adjust for your drivetrain:

| Constant | Value | Description |
|---|---|---|
| `GEAR_RATIO` | 51.0 | Motor-to-wheel gear reduction |
| `PPR` | 12.0 | Encoder pulses per motor revolution |
| `TPP` | 4.0 | Ticks per pulse (quadrature) — currently unused, reserved |

---

## Building

```bash
# In your ROS2 workspace
cd ~/<your_ros2_ws>/src
git clone https://github.com/Moaaz997/kangaroo_hw_interface_ros kangaroo_hw_interface
cd ~/<your_ros2_ws>
colcon build --packages-select kangaroo_hw_interface
source install/setup.bash
```

---

## Integration

Include the example xacro in `config/ros2_control_example.xacro` in your robot URDF, **adjusting joint names** as needed. Then load your controllers normally via `controller_manager`.