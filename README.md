# ROS环境搭建

1、在Anaconda中创建ROS虚拟环境

```python
conda create -n ros_env python=2.7
```

2、切换到ros_env环境

```python
conda activate ros_env
```

3、添加.bashrc环境

```python
source /home/taowenyin/MyCode/Python/ros_env/devel/setup.bash
```

4、安装相关包

```python
pip install -U rospkg
pip install empy
conda install numpy
conda install pyqt
conda install pydot
conda install pyqtgraph
```