# Simulating a 2-Wheel Differential Drive URDF Robot with Nav2

🔗 **Jump to testing directly** → [Testing Steps]([## 📈 Testing the Complete Robot](https://github.com/pratikhrohane/ros2-diffbot-navigation?tab=readme-ov-file#-testing-the-complete-robot))

---

## 🚀 Overview

It includes building the URDF, creating a custom Gazebo world, adding essential sensors, mapping, localization, and navigation with obstacle avoidance.

## 🧱 Folder Structure

```
├── launch/
│   ├── robot_world.launch.py
│   ├── robot_spawn.launch.py
│   ├── robot_state_publisher.launch.py
│   ├── my_robot.launch.py
│   └── navigation2.launch.py
├── map/
│   └── map.yaml, map.pgm
├── param/
│   └── nav2_params.yaml
├── urdf/
│   └── diff_bot.urdf
├── world/
│   └── home.world
├── model/
│   ├── model.config
│   └── model.sdf
```

---

## 🛠️ Installation Requirements

Make sure to install these before running:

```bash
sudo apt update && sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard
```

---

## 📦 Package Creation

```bash
ros2 pkg create allien_diff_bot --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs nav2_bringup gazebo_ros
```

---

## 🤖 Building the URDF Robot

**File:** `urdf/diff_bot.urdf`

* Includes:

  * Base link
  * Differential wheels
  * Caster wheel (continuous joint)
  * LIDAR (top-mounted)
  * IMU
* Visual, collision, and inertial properties are defined.
* Gazebo plugins used for sensor data and wheel odometry.

🔍 Display and test in RViz:

```bash
ros2 launch allien_diff_bot robot_state_publisher.launch.py
```

Then add **RobotModel** and set the topic to `/robot_description`

---

## 🌍 Creating the World

**Tool:** Gazebo Building Editor
Save the world to `world/home.world`

Add your world to Gazebo:

```bash
ros2 launch allien_diff_bot robot_world.launch.py
```

Spawn the robot:

```bash
ros2 launch allien_diff_bot robot_spawn.launch.py
```

---

## 🧭 SLAM and Map Saving

Use **slam\_toolbox**:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

Control robot using keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After mapping:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

Saved to: `map/map.yaml`, `map.pgm`

---

## 🗺️ Nav2 Integration

**File:** `param/nav2_params.yaml`

Parameter file sourced from community GitHub repo (can customize)

Launch Navigation Stack:

```bash
ros2 launch allien_diff_bot navigation2.launch.py
```

This internally loads `nav2_bringup/bringup_launch.py` and opens RViz with config.

---

## 🔁 Launch Summary

| Purpose             | Command                                                       |
| ------------------- | ------------------------------------------------------------- |
| Load Gazebo world   | `ros2 launch allien_diff_bot robot_world.launch.py`           |
| Spawn robot         | `ros2 launch allien_diff_bot robot_spawn.launch.py`           |
| Publish robot state | `ros2 launch allien_diff_bot robot_state_publisher.launch.py` |
| Combined launch     | `ros2 launch allien_diff_bot my_robot.launch.py`              |
| Start Navigation    | `ros2 launch allien_diff_bot navigation2.launch.py`           |

---

## 📈 Testing the Complete Robot

1. Run these 3 terminals:

   ```bash
   ros2 launch allien_diff_bot robot_world.launch.py
   ros2 launch allien_diff_bot my_robot.launch.py
   ros2 launch allien_diff_bot navigation2.launch.py
   ```

2. In **RViz2**:

   * Add **RobotModel**, select topic: `/robot_description`
   * Click **2D Pose Estimate** (set initial pose)
   * Click **2D Nav Goal** (set target goal)

3. Observe:

   * Robot in Gazebo follows the path avoiding obstacles
   * Path appears in RViz

---

## ✅ Done!
