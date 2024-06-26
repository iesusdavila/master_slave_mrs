#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from navigation_client import NavigationRobot
from navigate_slave import NavigateSlave
from pose_utils import PoseUtils
from data_robots import DataRobots
import asyncio
import sys
import random, string

system_master_slave = {}

def generate_id():
    id_task = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(8))
    return id_task 

async def main(args=None):
    if args is None:
        args = sys.argv

    if len(args) != 2:
        print("Usage: ros2 run multi_robot_master_slave nav_master_slave.py <path_to_yaml_file>")
        return

    yaml_file_path = args[1]

    rclpy.init(args=args)

    data_nav_robots = DataRobots(yaml_file_path)

    list_nav_func = []
    configs_robots = data_nav_robots.generate_robots()
    for robot in configs_robots:

        name_robot = robot['name']
        is_master = robot['is_master']
        
        if is_master and not(name_robot in system_master_slave):
            nav_master = NavigationRobot(namespace=name_robot)

            system_master_slave[name_robot] = {
                "nav_class": nav_master, 
                "has_camera": robot['has_camera'],
                "same_time_task": robot['same_time_task'], 
                "slaves": {}, 
                "slave_tasks": {}, 
                "status": True
            }
        
        if not is_master:
            nav_slave = NavigationRobot(namespace=name_robot)

            list_poses_wo_process = data_nav_robots.get_list_poses(robot)  # obtener lista de poses sin convertir en PoseStamped
            goal_poses_robot = PoseUtils.create_poses(list_poses_wo_process)  # convertir a PoseStamped
            
            name_master_robot = robot['name_master']                

            if not(name_master_robot in system_master_slave):
                nav_master = NavigationRobot(namespace=name_master_robot)
                has_camera_master = data_nav_robots.get_has_camera(configs_robots, name_master_robot)
                same_time_task_master = data_nav_robots.get_time_task_master(configs_robots, name_master_robot)

                system_master_slave[name_master_robot] = {
                    "nav_class": nav_master, 
                    "has_camera": has_camera_master,
                    "same_time_task": same_time_task_master, 
                    "slaves": {}, 
                    "slave_tasks": {}, 
                    "status": True
                }
            
            dict_master = system_master_slave[name_master_robot]
            id_task = generate_id()
            dict_master["slaves"][name_robot] = {
                "nav_class": nav_slave, 
                "master": name_master_robot, 
                "has_camera": robot['has_camera'],
                "task_queue": 
                    {
                        id_task: 
                            {
                                'name_robot': name_robot,
                                'has_max_time': robot['has_max_time'],
                                'duration_max_time': robot['duration_max_time'], 
                                'use_camera': robot['use_camera'],
                                'goal_poses': goal_poses_robot,
                                'old_robots_execution': [name_robot],
                            }
                    },
                "status": True
            }
            
            slave_robot = NavigateSlave(nav_slave, name_master_robot, name_robot)
            list_nav_func.append(slave_robot.navigate_robot_slave(system_master_slave, id_task))
    
    await asyncio.gather(*list_nav_func)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
