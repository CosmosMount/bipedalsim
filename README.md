# RM26_Gazebo
Used for bipedal simulation
## 配置
### 安装
安装依赖
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```
安装Gazebo
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```
安装ROS-Gazebo相关依赖
```bash
sudo apt-get install ros-humble-ros-gz
```
### workspace
由于编译和运行可能产生较多文件，且每次修改、在不同电脑上都需要重新编译，我们使用软链接的方式将仓库链接到自己的workspace
```bash
mkdir gazebo_ws && cd gazebo_ws
mkdir src && cd src
ln -s /yourpathto/RM26_Gazebo ./
```
### package
创建ros2 package
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```
## 使用
声明模型路径
```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/cosmosmount/Desktop/RM26_Gazebo/model
```
启动Gazebo
```bash
cd model && ign gazebo bipedal.sdf
```
启动Gazebo-ROS2 bridge
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```
启动节点
```bash
ros2 run lowlevel
```