# ArUco Pose Detector (ROS 2)

This ROS 2 package detects ArUco markers from camera images (`/camera/image_raw`), estimates their 6-DoF pose using `cv2.solvePnP`, and publishes the transform to the ROS TF tree. The results can be visualized in RViz.

## Features
- Detects ArUco markers using OpenCV.
- Estimates pose and publishes as `TransformStamped`.
- Converts OpenCV camera frame to ROS coordinate conventions.
- Visualizes marker pose using TF and optional image overlay.
## Outputs
![ArUco Detection Example](https://github.com/oscarpoudel/aruco_pose_detector/raw/main/image/1.png)

## Clone and Build
1. Clone the repository:
   ```
   git clone https://github.com/oscarpoudel/aruco_pose_detector.git
   ```

2. Navigate to your ROS 2 workspace and build:
   ```
   cd ~/ros2_ws/src
   mv /path/to/aruco_pose_detector .
   cd ..
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select aruco_pose_detector
   source install/setup.bash
   ```

## Running the Node
**Terminal 1 – Camera driver (e.g. usb_cam):**
```
ros2 run image_tools cam2image --ros-args -p image_width:=640 -p image_height:=360
```

**Terminal 2 – ArUco pose estimator:**
```
ros2 run aruco_pose_detector aruco_pose_estimator.py
```

## Parameters
- `image_topic` (default: `/camera/image_raw`) – Input image topic
- `marker_length` (default: `0.1778`) – Marker side length in meters

Example:
```
ros2 run aruco_pose_detector aruco_pose_estimator.py \
  --ros-args -p marker_length:=0.10 -p image_topic:=/my_camera/image_raw
```

## TF Frames
- Parent: `camera_link`
- Children: `aruco_marker_<id>`

These transforms can be visualized in RViz using the TF and Image display plugins.

