# ROS环境搭建

1、在Anaconda中创建ROS虚拟环境

```bash
conda create -n ros_env python=2.7
```

2、切换到ros_env环境

```bash
conda activate ros_env
```

3、添加.bashrc环境

```bash
source /home/taowenyin/MyCode/Python/ros_env/devel/setup.bash
```

4、安装相关包

```bash
pip install -U rospkg
pip install empy
conda install numpy
conda install pyqt
conda install pydot
conda install pyqtgraph
```

# 案例说明

1、ex1_topic：Topic通信案例

2、ex2_service：Service通信案例

3、ex3_movelt：以Dobot Magician为基础讲解Movelt!

（1）安装依赖包

```bash
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-publisher-gui
```

（2）安装Movelt!

```bash
sudo apt-get install ros-melodic-moveit
```

（3）启动Movelt Setup Assistant

```bash
rosrun moveit_setup_assistant moveit_setup_assistant
```

4、ex3_movelt_config：以Dobot Magician为基础的Movelt!配置功能包