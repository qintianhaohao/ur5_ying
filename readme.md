# moveit静态障碍物下的规划
```php
roscore
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
rosrun marm_planning moveit_obstacles_demo_ur5.py
```

# moveit障碍物移动
```php
roscore
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
rosrun marm_planning move_collision.py
```

# moveit动态障碍物的规划
```php
roscore
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
rosrun marm_planning moveit_dynamic_collision_ur5.py
```

# moveit动态障碍物的规划(一键启动)
```php
roscore
roslaunch marm_planning start_moveit_gazebo_rviz_ur5.launch
rosrun marm_planning moveit_dynamic_collision_ur5_2.py
```

# 显示octomap测试
```php
roscore
roslaunch marm_planning ur5_octomap_demo.launch
```

# 障碍物移动作为独立节点，并使用rrt规划(通过更改ompl_planning.yaml实现)
```php
roslaunch marm_planning start_moveit_gazebo_rviz_ur5.launch
rosrun marm_planning moveit_dynamic_collision_ur5_3.py
rosrun marm_planning moveit_move_obstacle.py
```

# 把机械臂和障碍物的移动放到一个launch中
```php
roslaunch marm_planning start_moveit_gazebo_rviz_ur5.launch

roslaunch marm_planning start_planning.launch
```

# 把机械臂和障碍物的移动放到一个launch中(使用新包)
```php
roslaunch fake_ur5_planning start_moveit_gazebo_rviz_ur5.launch

roslaunch fake_ur5_planning start_planning.launch
```

# 杨颖测试用
roslaunch fake_ur5_planning start_moveit_gazebo_rviz_ur5.launch

rosrun fake_ur5_planning moveit_dynamic_collision_ur5_3.py

rosrun fake_ur5_planning moveit_move_obstacle_7.py

# 在gazebo中加入两个turtlebot，并用键盘控制其移动
```php
roslaunch fake_ur5_planning start_moveit_gazebo_rviz_ur5.launch

ROS_NAMESPACE=turtlebot1 roslaunch fake_turtlebot_teleop keyboard_teleop.launch

ROS_NAMESPACE=turtlebot2 roslaunch fake_turtlebot_teleop keyboard_teleop.launch
```




# 参考资料
>http://wiki.ros.org/ur_gazebo
>https://blog.csdn.net/shenyan0712/article/details/90509905
>https://blog.csdn.net/weixin_44205772/article/details/89014756
>https://www.guyuehome.com/4889






