# Image Conversion ROS 2 Package

This project is a ROS 2 package that converts video streams from a USB or laptop camera. It provides functionality to switch between grayscale and color modes using a ROS 2 service. The processed video stream is published on a separate topic.

---

## Features

1. **USB Camera Integration**:
   - Uses the `usb_cam` ROS 2 package to capture video streams from a USB or laptop camera.
2. **Dynamic Mode Switching**:
   - A ROS 2 service allows switching between grayscale (Mode 1) and color (Mode 2).
3. **Processed Video Output**:
   - Publishes the converted video stream on a separate topic.
4. **Launch File**:
   - Simplifies starting the nodes and setting custom input/output topics.

---

## Installation

### Prerequisites
Ensure you have the following installed:
- ROS 2 (Humble, Rolling, or compatible distribution)
- `usb_cam` package:
  ```bash
  sudo apt install ros-$ROS_DISTRO-usb-cam
  ```
- OpenCV and cv_bridge:
  ```bash
  sudo apt install ros-$ROS_DISTRO-cv-bridge libopencv-dev
  ```

### Clone the Repository
1. Navigate to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   ```
2. Clone the repository:
   ```bash
   git clone <repository_url> image_conversion
   ```
3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select image_conversion
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Package Structure

```
image_conversion/
├── CMakeLists.txt           # Build instructions
├── package.xml              # Package metadata
├── src/
│   └── image_conversion_node.cpp  # Node implementation
├── launch/
│   └── image_conversion_launch.py # Launch file
└── README.md                # Project documentation
```

---

## Usage

### Launch the Nodes
To run the package with both the USB camera node and the image conversion node:
```bash
ros2 launch image_conversion image_conversion_launch.py
```

### Topics
- **Input Topic**:
  - `/camera/image_raw` (default from `usb_cam` package)
- **Output Topic**:
  - `/image_converted`

### Service
- **Service Name**:
  - `/image_conversion/set_mode`
- **Service Type**:
  - `std_srvs/srv/SetBool`
- **Usage**:
  - Switch to grayscale mode:
    ```bash
    ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: true}"
    ```
  - Switch to color mode:
    ```bash
    ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: false}"
    ```

---

## Visualizing Output

Use `rqt` to view the processed video stream:
```bash
rqt
```
1. Open `Plugins > Visualization > Image View`.
2. Select the topic `/image_converted` to view the video stream.

---

## Code Explanation

### **Node Implementation (`src/image_conversion_node.cpp`):**
- Subscribes to `/camera/image_raw` for input video frames.
- Provides a service `/image_conversion/set_mode` to toggle between modes.
- Publishes processed frames to `/image_converted`.
- Uses OpenCV for image conversion (grayscale or color).

### **Launch File (`launch/image_conversion_launch.py`):**
- Launches the `usb_cam` node and the `image_conversion_node`.
- Allows configuration of input and output topics via parameters.

---

## Building the Package
To rebuild the package after changes:
```bash
cd ~/ros2_ws
colcon build --packages-select image_conversion
source install/setup.bash
```

---

## Debugging

### Check Active Nodes
```bash
ros2 node list
```

### Inspect Topics
```bash
ros2 topic list
ros2 topic echo /camera/image_raw
ros2 topic echo /image_converted
```

### Check Service
```bash
ros2 service list
ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: true}"
```

---

## Dependencies
- ROS 2
- OpenCV
- `cv_bridge`
- `usb_cam` ROS 2 package

---

## License
This project is licensed under the [MIT License](LICENSE).

---

## Contributions
Feel free to fork this repository and submit pull requests for improvements or bug fixes.

