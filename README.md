# jazzy_camera_issues

## Reproducing Issue

Start the image publisher with a VGA image:

```
ros2 run image_publisher image_publisher_node frame0001.jpg --ros-args --log-level DEBUG
```

With reliable transport, we can see the image:

```
ros2 run jazzy_camera_issues image_sub --ros-args -p --best_effort:=false --log-level DEBUG
```

With best effort, few and usually no images come through:

```
ros2 run jazzy_camera_issues image_sub --ros-args -p --best_effort:=true --log-level DEBUG
```

Switching to a smaller image being published works with best effort or reliable:

```
ros2 run image_publisher image_publisher_node /opt/ros/jazzy/share/rviz_common/images/splash.png
```
