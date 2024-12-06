### Steps to collect hand-guiding data:

### After collecting data:



colcon build --symlink-install

source install/setup.bash

ros2 launch realsense2_camera rs_multi_camera_launch_sync.py publish_tf:=false serial_no1:="'840412060409'" serial_no2:="'932122060300'"

ros2 launch lbr_bringup bringup.launch.py model:=iiwa14 sim:=false rviz:=true moveit:=true

ros2 run data_collector kuka_data_recorder
