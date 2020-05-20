#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from copy import deepcopy
from moveit_msgs.msg import MotionPlanRequest

class MoveItObstaclesDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_obstacles_demo')

        # 等待场景准备就绪
        rospy.sleep(1)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('manipulator')

        # 设置机械臂工作空间
        self.arm.set_workspace([-100, -100, 0, 100, 0.3, 100])

        # 设置机械臂最大速度
        self.arm.set_max_velocity_scaling_factor(value=0.1)

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo('end effector link: {}'.format(self.end_effector_link))

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        # 当运动规划失败后，允许重新规划
        # self.arm.allow_replanning(True)
        self.arm.set_num_planning_attempts(10)
        # self.arm.allow_looking(True)

        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)

        # 设置每次运动规划的时间限制：5s
        self.arm.set_planning_time(3)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(2)

    def planning(self, start_point, end_point):
        """
        功能：动态避障
        :param start_point: 起始点, type: dict
        :param end_point: 终点, type: dict
        :return: None
        """
        # 移动机械臂到指定位置，并获取当前位姿数据为机械臂运动的起始位姿
        if start_point:
            self.move_arm(p=start_point)
        self.start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # 使用moveit自带的求解器规划出A到B的离散路径点, 并存到列表way_points中
        # way_points = self.get_way_points(start_point, end_point)

        while True:
            print('set planner id: RRT')
            self.arm.set_planner_id('TRRTkConfigDefault')
            self.arm.set_named_target('up')
            self.arm.go()
            rospy.sleep(5)


            print('set planner id: PRM')
            self.arm.set_planner_id('TRRTkConfigDefault')
            self.arm.set_named_target('home')
            self.arm.go()
            rospy.sleep(5)


    # -------------------------------------------------------------------

    def get_way_points(self, a, b):
        way_points = []

        # plan 1
        self.arm.set_named_target('up')
        traj = self.arm.plan()
        if traj.joint_trajectory.points:  # True if trajectory contains points
            rospy.loginfo("get trajectory success")
        else:
            rospy.logerr("Trajectory is empty. Planning false!")
        self.arm.clear_pose_targets()

        # plan 2
        # traj = self.moveit_planning(p=b)

        for i, p in enumerate(traj.joint_trajectory.points):
            # rospy.loginfo('waypoint: {}'.format(i))
            if i%2 == 0:
                point = {
                    'x': p.positions[0],
                    'y': p.positions[1],
                    'z': p.positions[2],
                    'ox': p.positions[3],
                    'oy': p.positions[4],
                    'oz': p.positions[5]
                }
                way_points.append(point)
        rospy.loginfo('waypoint: \n {}'.format(way_points))
        rospy.loginfo(len(way_points))
        return way_points

    def moveit_planning(self, p):
        """
        使用moveit求解器规划路径
        :param p: dict, e.g., {'x': 0, 'y': 0, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0, 'ow': 0}
        :return:
        """
        rospy.loginfo('start moveit planning...')
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose.position.x = p['x']
        target_pose.pose.position.y = p['y']
        target_pose.pose.position.z = p['z']
        if 'ox' in p.keys() and p['ox']:
            target_pose.pose.orientation.x = p['ox']
        if 'oy' in p.keys() and p['oy']:
            target_pose.pose.orientation.y = p['oy']
        if 'oz' in p.keys() and p['oz']:
            target_pose.pose.orientation.z = p['oz']
        if 'ow' in p.keys() and p['ow']:
            target_pose.pose.orientation.w = p['ow']
        # 传入一个PoseStamped
        # self.arm.set_pose_target(target_pose, self.end_effector_link)

        # 尝试直接传入一个列表
        self.arm.set_pose_target([p['x'], p['y'], p['z'], p['ox'], p['oy'], p['oz']], self.end_effector_link)
        traj = self.arm.plan()
        if traj.joint_trajectory.points:  # True if trajectory contains points
            rospy.loginfo("get trajectory success")
            return traj
        else:
            rospy.logerr("Trajectory is empty. Planning false!")

    def move_arm(self, p):
        """
        移动机械臂到p点
        :param p: dict, e.g., {'x': 0, 'y': 0, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0, 'ow': 0}
        :return:
        """
        rospy.loginfo('start arm moving...')

        traj = self.moveit_planning(p)
        self.arm.execute(traj)

        # 设置当前位置为起始位置
        self.arm.set_start_state_to_current_state()
        rospy.sleep(1)

    def stop_arm(self):
        """
        急停
        :return:
        """
        pass

    def exist_danger_obstacle(self):
        """
        环境中是否存在危险的障碍物
        :return: True or False
        """
        return False

    def get_obstacle_octomap(self):
        """
        获取环境的octomap信息
        :return:
        """
        pass


if __name__ == "__main__":
    try:
        end = {
            'x': 0.14,
            'y': 0.17,
            'z': 0.66,
            'ox': 0.71,
            'oy': 0.71,
            'oz': 0.02,
            'ow': -0.02
        }
        m = MoveItObstaclesDemo()
        m.planning(start_point=None, end_point=end)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    except KeyboardInterrupt:
        raise