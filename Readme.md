# Usage
## 1. Required Libraries 
* vtk (A dependency library for PCL installation, need to check Qt during compilation)
* PCL
* `px4_msgs` built in the workspace or sourced from an overlay. For PX4 use message definitions matching your firmware.

## 2. Prerequisites
It might be due to some incorrect settings in my publish/subscribe configurations. Using ROS2's default FastDDS causes significant lag during program execution. The reason hasn't been identified yet. Please follow the steps below to change the DDS to cyclonedds.

### 2.1 Install cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 Change default DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 Verify the change
```
ros2 doctor --report | grep "RMW middleware"
```
If the output shows rmw_cyclonedds_cpp, the modification is successful.

## 3. Running the Code
### 3.1 Launch Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 Run the planning program
Open a new terminal and execute:
* Single drone
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* Additional parameters (optional):
    * use_mockamap:Map generation method. Default: False (uses Random Forest), True uses mockamap.
    * use_dynamic:Whether to consider dynamics. Default: False (disabled), True enables dynamics.
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```
### 3.3 Gate trajectory execution on PX4 Offboard

Pass this parameter file to **both** `ego_planner_node` and `traj_server` in your PX4 launch configuration.

```yaml
/**:
  ros__parameters:
    px4/require_offboard: true
    px4/vehicle_status_topic: /fmu/out/vehicle_status
    px4/vehicle_status_timeout: 1.0
```

Set the status topic to the exact name exported by your uXRCE-DDS bridge, for example `/fmu/out/vehicle_status_v1` for a versioned firmware or a name with the vehicle namespace. The timeout is in seconds.

You can send a goal before switching to Offboard. The trajectory waits without publishing `/position_cmd` and starts from the beginning when PX4 reports Offboard. Leaving Offboard or losing the status stream discards the goal. Send a fresh goal to start another trajectory. The node that sends PX4's `OffboardControlMode` heartbeat must keep doing so independently of `/position_cmd` so PX4 can enter Offboard. [PX4 Offboard documentation](https://docs.px4.io/main/en/flight_modes/offboard)

Rebuild and source the workspace after updating, including any nodes that exchange `traj_utils/msg/Bspline` messages.

# 使用方法
## 1. 需要的库 
* vtk(是安装PCL的依赖库，编译时需要勾选Qt)
* PCL

## 2. 前置条件
可能是我一些发布订阅的设置写的不太对，使用ROS2默认的FastDDS会导致程序运行很卡，目前还没找到原因，所以请按照下述方法将DDS修改为cyclonedds

### 2.1 安装cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 修改默认的DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 检查是否修改成功
```
ros2 doctor --report | grep "RMW middleware"
```
输出显示rmw_cyclonedds_cpp则说明修改成功

## 3. 代码运行
### 3.1 运行Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 运行规划程序
新开一个终端，输入以下指令
* 单机
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* 附加参数，可以选择地图生成模式以及是否考虑动力学
    * use_mockamap:地图生成方式，默认为False，False时使用Random Forest, True时使用mockamap
    * use_dynamic:是否考虑动力学，默认为False, False时不考虑, True时考虑
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```