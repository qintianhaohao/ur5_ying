#!/usr/bin/env python
# -*- coding: utf-8 -*-

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
import rospy
import numpy as np
import copy


class MoveitMoveObstacle(object):
    """move obstacle
    """
    def __init__(self):
        # 设置参考坐标系
        self.reference_frame = 'base_link'

        # 初始化场景
        self.scene = PlanningSceneInterface()

        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()

        # 移除已存在障碍物
        self.scene.remove_world_object()

        # 添加障碍物
        self.add_obstacles()

    def add_obstacles(self):
        """
        在场景中增加障碍物
        """
        # 在场景中添加一个立方体
        self.box1_id = 'box1'
        self.box1_size = [0.1, 1.5, 0.1]
        self.box1_pose = PoseStamped()
        self.box1_pose.header.frame_id = self.reference_frame
        self.box1_pose.pose.position.x = 0.8
        self.box1_pose.pose.position.y = 1.6
        self.box1_pose.pose.position.z = 0.31
        self.box1_pose.pose.orientation.w = 1.0   
        self.scene.add_box(self.box1_id, self.box1_pose, self.box1_size)
        self.setColor(self.box1_id, 0.8, 0.4, 0, 1.0)
        
        # 在场景中添加一个桌子
        self.desk1_id = 'desk1'
        self.desk1_size = [3, 3, 0.1]
        self.desk1_pose = PoseStamped()
        self.desk1_pose.header.frame_id = self.reference_frame
        self.desk1_pose.pose.position.x = 0.0
        self.desk1_pose.pose.position.y = 0.0
        self.desk1_pose.pose.position.z = -0.25
        self.desk1_pose.pose.orientation.w = 1.0   
        self.scene.add_box(self.desk1_id, self.desk1_pose, self.desk1_size)
        self.setColor(self.desk1_id, 0.1, 0.5, 0.1, 1.0)
        
        # 在桌子上添加一个底座
        self.base1_id = 'base1'
        self.base1_size = [0.2, 0.2, 0.2]
        self.base1_pose = PoseStamped()
        self.base1_pose.header.frame_id = self.reference_frame
        self.base1_pose.pose.position.x = 0.0
        self.base1_pose.pose.position.y = 0.0
        self.base1_pose.pose.position.z = -0.1
        self.base1_pose.pose.orientation.w = 1.0   
        self.scene.add_box(self.base1_id, self.base1_pose, self.base1_size)
        self.setColor(self.base1_id, 0.5, 0.5, 0, 1.0)

        # 发送场景消息
        self.sendColors()
        rospy.sleep(1)

    def setColor(self, name, r, g, b, a = 0.9):
        """
        设置场景物体的颜色
        """
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    def sendColors(self):
        """
        将颜色设置发送并应用到moveit场景当中
        """
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

    def move_obstacle(self):
        """移动障碍物
        """

        # 在场景中增加物体
        for i in range(100):
            self.scene.add_box(self.box1_id, self.box1_pose, self.box1_size)
            self.scene.add_box(self.desk1_id, self.desk1_pose, self.desk1_size)
            self.scene.add_box(self.base1_id, self.base1_pose, self.base1_size)
            self.sendColors()
            rospy.sleep(0.1)

        # 连续移动障碍物
        def move_box1_continuous(start_pose, end_pose, pose_number, sleep_time):
            rospy.loginfo('start move box1...')
            poses = self.get_3d_linspace(start_pose, end_pose, pose_number)
            for p in poses:
                rospy.loginfo('new pose:\n {}'.format(p))
                self.scene.remove_world_object(self.box1_id)
                self.scene.add_box(self.box1_id, p, self.box1_size)
                rospy.sleep(sleep_time)

        # 把障碍物从pose_0移动到pose_1， 分5步走完，每步停顿0.2秒
        pose_0 = copy.deepcopy(self.box1_pose)
        pose_1 = copy.deepcopy(pose_0)
        pose_1.pose.position.y -= 0.8
        move_box1_continuous(pose_0, pose_1, 5, 0.2)

        # 把障碍物从pose_2移动到pose_3， 分10步走完，每步停顿0.5秒
        pose_2 = copy.deepcopy(pose_1)
        pose_2.pose.position.x += 0.8       # 沿x轴移动
        pose_2.pose.orientation.x -= 0.3    # 沿x轴旋转

        pose_3 = copy.deepcopy(pose_2)
        pose_3.pose.position.z += 0.8       # 沿z轴移动
        
        move_box1_continuous(pose_2, pose_3, 10, 0.5)

        # 如果还需要增加路径，按照上面的方法继续写

    def get_3d_linspace(self, start_p, end_p, p_number):
        """calculate 3d linspace between 2 points
        :param start_p: PoseStamped()
        :param end_p: PoseStamped()
        :param p_number: points number
        :return: points list
        """
        x_new = np.linspace(start_p.pose.position.x, end_p.pose.position.x, p_number)
        y_new = np.linspace(start_p.pose.position.y, end_p.pose.position.y, p_number)
        z_new = np.linspace(start_p.pose.position.z, end_p.pose.position.z, p_number)

        points = []
        for i in range(len(x_new)):
            p = copy.deepcopy(start_p)
            p.pose.position.x = float(x_new[i])
            p.pose.position.y = float(y_new[i])
            p.pose.position.z = float(z_new[i])
            points.append(p)
        
        return points




if __name__ == "__main__":
    rospy.init_node("moveit_move_obstacle")
    app = MoveitMoveObstacle()
    app.move_obstacle()
    rospy.spin()
