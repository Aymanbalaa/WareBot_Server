# WareBot Server

*This project is based on Vizanti.*

A mission planner and visualizer for controlling outdoor ROS robots with a web-based interface.

## Overview

WareBot Server is a ROS 2 package that provides a powerful web-based mission planner and visualizer for outdoor robotics applications. It offers an intuitive browser-based interface for monitoring and controlling ROS robots, making it easy to visualize sensor data, plan missions, and interact with robot systems.

**Note**: The ROS 2 package name is `vizanti_server`, while the repository is named `WareBot_Server`. All ROS 2 commands will reference the package name `vizanti_server`.

## Features

- 🌐 **Web-Based Interface**: Access your robot control and visualization through any modern web browser
- 🗺️ **Mission Planning**: Plan and execute robot missions with an intuitive interface
- 📊 **Real-Time Visualization**: View live data from various ROS topics and sensors
- 🔧 **Widget System**: Extensible architecture with customizable widgets for different functionalities
- 🌍 **Satellite Mapping**: Integration with satellite imagery for outdoor navigation
- 🤖 **Multi-Robot Support**: Monitor and control multiple robots simultaneously
- 📡 **ROS Bridge Integration**: Seamless communication with ROS 2 through rosbridge

## Prerequisites

Before installing WareBot Server, ensure you have the following:

- **ROS 2** (tested with ROS 2 distributions)
- **Python 3** with the following packages:
  - `flask`
  - `waitress`
  - `rclpy`
- **ROS 2 Packages**:
  - `rclcpp`
  - `ros2pkg`
  - `rosbridge_suite`
  - `std_srvs`, `std_msgs`, `tf2_msgs`, `nav_msgs`, `geometry_msgs`
  - `rcl_interfaces`
- **Build Tools**:
  - `ament_cmake`
  - `ament_cmake_python`
  - `rosidl_default_generators`

## Installation

1. **Clone the repository into your ROS 2 workspace**:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Aymanbalaa/WareBot_Server.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select vizanti_server
   ```

4. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Starting the Server

Launch the WareBot Server with rosbridge:

```bash
ros2 launch vizanti_server vizanti_server.launch.py
```

Or launch with custom parameters:

```bash
ros2 launch vizanti_server vizanti_server.launch.py port:=5000 rosbridge_port:=5001
```

### Accessing the Web Interface

Once the server is running, open your web browser and navigate to:

```
http://localhost:5000
```

Replace `localhost` with your robot's IP address if accessing remotely.

## Project Structure

```
WareBot_Server/
├── CMakeLists.txt          # CMake build configuration
├── package.xml             # ROS 2 package manifest
├── Contributing.md         # Contribution guidelines
├── launch/                 # ROS 2 launch files
│   ├── vizanti_server.launch.py
│   └── vizanti_rws.launch.py
├── scripts/                # Python scripts
│   ├── server.py          # Main Flask server
│   ├── service_handler.py # ROS service handler
│   └── rqt_reconfigure_param_api.py
├── src/                    # C++ source files
│   └── tf_consolidator.cpp # TF transform consolidator
├── srv/                    # Service definitions
│   ├── GetNodeParameters.srv
│   ├── SetNodeParameter.srv
│   ├── SaveMap.srv
│   ├── LoadMap.srv
│   ├── RecordRosbag.srv
│   ├── ManageNode.srv
│   ├── ListPackages.srv
│   ├── ListExecutables.srv
│   └── ListLifecycles.srv
├── public/                 # Web interface files
│   ├── index.html
│   ├── js/                # JavaScript modules
│   ├── assets/            # Images and resources
│   └── templates/         # Widget templates
└── vizanti_server/         # Additional server resources
```

## Architecture

### Server Components

1. **Flask Web Server**: Serves the static web content and provides the HTTP interface
2. **Rosbridge**: Enables WebSocket communication between the browser and ROS
3. **Service Handler**: Custom ROS node providing additional functionality
4. **TF Consolidator**: C++ node that consolidates TF transforms for efficient visualization

### Client Components

The web interface is built with:
- Vanilla JavaScript (ES6 modules)
- Jinja2 templating (server-side with Flask)
- Canvas-based rendering for visualizations
- Modular widget system for extensibility

## Services

The package provides several custom ROS services:

- `GetNodeParameters` - Retrieve node parameters
- `SetNodeParameter` - Update node parameters
- `SaveMap` - Save the current map
- `LoadMap` - Load a saved map
- `RecordRosbag` - Control rosbag recording
- `ManageNode` - Start/stop ROS nodes
- `ListPackages` - List available ROS packages
- `ListExecutables` - List executable nodes in packages
- `ListLifecycles` - Manage lifecycle nodes

## Contributing

Contributions are welcome! Please see [Contributing.md](Contributing.md) for detailed guidelines on:
- Adding new widgets
- Helper modules and singletons
- Code structure and conventions
- Development workflow

## License

This project is licensed under the BSD License. See the package.xml for details.

## Support

For issues, questions, or feature requests, please use the [GitHub Issues](https://github.com/Aymanbalaa/WareBot_Server/issues) page.
