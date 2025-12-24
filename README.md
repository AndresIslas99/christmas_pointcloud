# 🎄 Christmas Point Cloud - ROS2 Package

A festive ROS2 package that generates animated Christmas-themed LiDAR-style point clouds. Perfect for holiday demos, learning ROS2 point cloud publishing, or just spreading some holiday cheer!

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron%20%7C%20Jazzy-blue)
![License](https://img.shields.io/badge/License-MIT-green)

## ✨ Features

- **🌲 3D Christmas Tree**: Realistic cone-shaped tree with layered foliage
- **⭐ Twinkling Star**: Animated golden star on top with glow effect
- **🔴 Colorful Ornaments**: Spherical ornaments in red, gold, blue, silver, and more
- **💡 Blinking Lights**: Spiral string lights with alternating colors
- **❄️ Falling Snowflakes**: Animated snowfall with realistic drift
- **🎁 Gift Boxes**: Wrapped presents under the tree with ribbons
- **🏔️ Snowy Ground**: White ground plane for the winter scene

## 📋 Requirements

- ROS2 (Humble, Iron, or Jazzy)
- PCL (Point Cloud Library)
- RViz2

## 🚀 Installation

### 1. Clone or copy to your workspace

```bash
cd ~/ros2_ws/src
# Copy the christmas_pointcloud folder here
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the package

```bash
colcon build --packages-select christmas_pointcloud
source install/setup.bash
```

## 🎮 Usage

### Launch with RViz2 (Recommended)

```bash
ros2 launch christmas_pointcloud christmas.launch.py
```

### Run node only

```bash
ros2 run christmas_pointcloud christmas_cloud_node
```

Then open RViz2 separately and add a PointCloud2 display subscribed to `/christmas_cloud`.

### Custom Parameters

```bash
ros2 launch christmas_pointcloud christmas.launch.py \
    tree_height:=5.0 \
    num_snowflakes:=1000 \
    num_ornaments:=100
```

## ⚙️ Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tree_height` | 3.0 | Height of the tree (meters) |
| `tree_base_radius` | 1.5 | Base radius of the tree |
| `num_tree_points` | 15000 | Density of foliage points |
| `num_ornaments` | 50 | Number of ornaments |
| `num_snowflakes` | 500 | Number of snowflakes |
| `snowflake_area` | 8.0 | Area of snowfall (meters) |
| `animation_speed` | 0.05 | Animation speed multiplier |

## 📡 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/christmas_cloud` | `sensor_msgs/PointCloud2` | RGB point cloud output |

## 🎨 RViz2 Tips

For the best visualization:

1. Set background color to dark blue (20, 30, 50)
2. Use `RGB8` color transformer
3. Set point size to 3 pixels
4. Use Orbit view and position camera for good angle

## 📁 Package Structure

```
christmas_pointcloud/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── christmas_params.yaml
├── launch/
│   └── christmas.launch.py
├── rviz/
│   └── christmas.rviz
└── src/
    └── christmas_cloud_publisher.cpp
```

## 🔧 Extending the Package

Want to add more festive elements? The code is modular - just add new `generate*` methods:

```cpp
void generateSnowman(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    // Your snowman generation code here!
}
```

Then call it in `publishCloud()`.

## 📄 License

MIT License - Feel free to use, modify, and spread the holiday cheer!

## 🎅 Author

Created with ❤️ for the robotics community.

---

```
    ⭐
   /  \
  / 🔴 \
 / 🟡 🔵 \
/🔴 🟢 🟡 🔴\
    |||
  🎁🎁🎁

¡Feliz Navidad y próspero año nuevo!
Merry Christmas and Happy New Year!
```
