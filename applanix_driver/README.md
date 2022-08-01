# applanix_driver

# Overview

This package contains the applanix [POS LVX](https://leodrive.ai/products/applanix-pos-lvx) GNSSINS driver. The current master branch works with ROS Galactic.


## Installation
Install applanix repository:
```
https://gitlab.com/leo-aios/applanix.git

cd existing_repo
git remote add origin https://gitlab.com/leo-aios/applanix.git
git branch -M main
git push -uf origin main
```
## Usage
The Applanix driver is used with the applanix message package. Therefore, the applanix_msgs package should also be compiled and the driver should be used. <br/>
You can run applanix driver with command below:
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install --packages-up-to applanix
cd existing_workspace
source install/setup.bash
ros2 launch applanix_driver lvx_client.launch.py
```

## Parameters
 * base_frame_id : This parameter determines the frame name of the vehicle.
 * gnss_ins_frame_id : This parameter determines the frame name of the sensor. 
 * publish_gsof_msgs: GSOF(General Serial Output Format) is default applanix message format. 
   > /applanix/lvx_client/gsof/ins_solution_49        (applanix_msgs::msg::NavigationSolutionGsof49) <br/>
   > /applanix/lvx_client/gsof/ins_solution_rms_50    (applanix_msgs::msg::NavigationPerformanceGsof50) <br/>

 * publish_ros_msgs: This parameter related to ros messages. When you set this parameter to true, the following ros messages are published. <br/>
   > /applanix/lvx_client/gnss/fix (sensor_msgs::msg::NavSatFix) <br/>
   > /applanix/lvx_client/imu_raw  (sensor_msgs::msg::Imu) <br/>
   
 * publish_autoware_msgs: This package contains [autoware messages](https://github.com/autowarefoundation/autoware_msgs). You can activate these messages by setting this parameter to true.
   > applanix/lvx_client/autoware_orientation (autoware_sensing_msgs::msg::GnssInsOrientationStamped)

## Published Topics
 * /applanix/lvx_client/gsof/ins_solution_49         (applanix_msgs::msg::NavigationSolutionGsof49) <br/>
 * /applanix/lvx_client/gsof/ins_solution_rms_50     (applanix_msgs::msg::NavigationPerformanceGsof50) <br/>
 * /applanix/lvx_client/gnss/fix                     (sensor_msgs::msg::NavSatFix) <br/>
 * /applanix/lvx_client/imu_raw                      (sensor_msgs::msg::Imu) <br/>
 * /applanix/lvx_client/autoware_orientation          (autoware_sensing_msgs::msg::GnssInsOrientationStamped)


 
