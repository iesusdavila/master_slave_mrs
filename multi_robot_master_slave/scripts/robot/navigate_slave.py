#!/usr/bin/env python3

from delegate_task import FreeSlaveHandler, SlaveWithOneTaskHandler, MasterHandler
from navigate_master import NavigateMaster
from navigation_client import TaskResult
from robot import Robot
from rclpy.duration import Duration
import asyncio

class NavigateSlave(Robot):
    def __init__(self, nav_slave, name_master, name_slave_pend=None):
        self.nav_slave = nav_slave
        self.name_master = name_master
        self.name_slave_pend = name_slave_pend
        
    async def navigate_robot_slave(self, system_master_slave: dict, id_task: str) -> None:

        name_slave = self.nav_slave.getNameRobot()
        dict_slave = system_master_slave[self.name_master]["slaves"][name_slave]
        task_queue = dict_slave["task_queue"]
        while task_queue:

            id_first_slave_task = list(task_queue)[0]
            name_first_slave_task = task_queue[id_first_slave_task]['name_robot']

            if self.name_slave_pend is None:
                self.name_slave_pend = name_slave

            if id_first_slave_task == id_task and name_first_slave_task == self.name_slave_pend:
                
                goal_poses_robot = task_queue[id_first_slave_task]["goal_poses"]
                self.nav_slave.followWaypoints(goal_poses_robot)
                nav_start = self.nav_slave.get_clock().now()

                has_max_time = task_queue[id_first_slave_task]["has_max_time"]
                use_camera = task_queue[id_first_slave_task]["use_camera"]
                duration_max_time_m = task_queue[id_first_slave_task]["duration_max_time"]
                current_waypoint = 0
                
                is_task_terminated = self.nav_slave.isTaskComplete()
                is_task_completed = False
                while not is_task_terminated:
                    
                    await asyncio.sleep(1)
                    feedback = self.nav_slave.getFeedback()
                    
                    if feedback:
                        now = self.nav_slave.get_clock().now()
                        nav_time = self.nav_slave.getTimeNav(now.nanoseconds - nav_start.nanoseconds)

                        current_waypoint = feedback.current_waypoint

                        if has_max_time:
                            duration_max_time=Duration(seconds=duration_max_time_m*60)
                            max_time = self.nav_slave.getTimeNav(duration_max_time.nanoseconds)
                            
                            if self.name_slave_pend == name_slave:
                                super().generate_message(name_slave, current_waypoint, len(goal_poses_robot), nav_time, max_time)
                            else:
                                super().generate_message(name_slave, current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave_pend)
                            
                            if now - nav_start >= duration_max_time:
                                await self.exceed_max_time(task_queue, dict_slave, id_first_slave_task, use_camera, goal_poses_robot, has_max_time, duration_max_time_m, current_waypoint, system_master_slave)

                                return
                        else:
                            if self.name_slave_pend == name_slave:
                                super().generate_message(name_slave, current_waypoint, len(goal_poses_robot), nav_time)
                            else:
                                super().generate_message(name_slave, current_waypoint, len(goal_poses_robot), nav_time, name_slave=self.name_slave_pend)
    
                        is_task_terminated = self.nav_slave.isTaskComplete()
                        missing_points = self.nav_slave.getMissionPoints()

                        if missing_points != None:
                            if len(missing_points) == 0 and is_task_terminated:
                                is_task_completed = True

                if self.nav_slave.getResult() == TaskResult.SUCCEEDED and is_task_completed:
                    self.task_complete(dict_slave, id_first_slave_task)
                else:
                    self.nav_slave.info("Tarea NO completada")
                    await self.exceed_max_time(task_queue, dict_slave, id_first_slave_task, use_camera, goal_poses_robot, has_max_time, duration_max_time_m, current_waypoint, system_master_slave)
                    
                    return
                break
            elif id_first_slave_task != id_task and name_first_slave_task == self.name_slave_pend:
                self.nav_slave.info("El esclavo " + self.name_slave_pend + " está esperando que la tarea enviada al esclavo " + name_first_slave_task + " sea completada una vez que dicho esclavo complete su tarea interna. Es el mismo esclavo pero con otro ID de tarea")
                await asyncio.sleep(1)
            else:
                if self.name_slave_pend != name_slave:
                    print("El esclavo " + self.name_slave_pend + " está esperando que la tarea enviada al esclavo " + name_first_slave_task + " sea completada una vez que dicho esclavo complete su tarea interna.")
                await asyncio.sleep(1)

    async def exceed_max_time(self, task_queue: dict, dict_slave: dict, id_first_slave_task: str, use_camera: bool, goal_poses_robot: list, has_max_time: bool, duration_max_time_m: str, current_waypoint: int, system_master_slave: dict) -> None:
        super().cancel_task(self.nav_slave)

        old_robots_execution = task_queue[id_first_slave_task]["old_robots_execution"]
        
        self.nav_slave.info("Tarea NO completada en el tiempo establecido.")
        dict_slave["task_queue"].pop(id_first_slave_task)
        self.nav_slave.info("Tarea eliminada de la lista de tareas pendientes")

        status_send_goal = await self.send_goal_other_robot(id_first_slave_task, old_robots_execution, use_camera, goal_poses_robot, has_max_time, duration_max_time_m, current_waypoint, system_master_slave)

        if not status_send_goal:
            self.nav_slave.info("Terminada toda ejecucion, problemas de efectuar la tarea.")

    def task_complete(self, dict_slave: dict, id_first_slave_task: str) -> None:
        self.nav_slave.info("Tarea completada")
        dict_slave["task_queue"].pop(id_first_slave_task)
        self.nav_slave.info("Tarea eliminada de la lista de tareas pendientes.")

    async def send_goal_other_robot(self, id_task: str, old_robots_execution: list, use_camera: bool, goal_poses_robot: list, has_max_time: bool, duration_max_time: str, current_waypoint: int, system_master_slave: dict) -> bool:
        await asyncio.sleep(1)

        name_slave = self.nav_slave.getNameRobot()
        nav_master = system_master_slave[self.name_master]["nav_class"]
        list_slaves = system_master_slave[self.name_master]["slaves"]
        
        request = {
            'dict_master': system_master_slave[self.name_master],
            'nav_slave': self.nav_slave,
            'id_task': id_task,
            'use_camera': use_camera,
            'goal_poses': goal_poses_robot,
            'has_max_time': has_max_time,
            'duration_max_time': duration_max_time,
            'old_robots_execution': old_robots_execution,
            'current_waypoint': current_waypoint,
        }

        free_slave_handler = FreeSlaveHandler()
        slave_with_one_task_handler = SlaveWithOneTaskHandler()
        master_handler = MasterHandler()

        free_slave_handler.set_next(slave_with_one_task_handler).set_next(master_handler)
        
        handler, is_free_or_one_task, finish_task = free_slave_handler.handle(request)
        
        #name_slave: nombre del robot que esta pidiendo ayuda de su tarea pendiente
        #handler: robot que esta socorriendo a la ayuda de la tarea pendiente
        if finish_task:
            return False
        else:
            if is_free_or_one_task:
                slave_robot = NavigateSlave(handler, self.name_master, name_slave)
                await asyncio.gather(slave_robot.navigate_robot_slave(system_master_slave, id_task))
                return True
            else:
                master_robot = NavigateMaster(handler, name_slave)
                await asyncio.gather(master_robot.navigate_robot_master(system_master_slave, id_task))
                return True