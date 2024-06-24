#!/usr/bin/env python3

from delegate_task import FreeSlaveHandler, SlaveWithOneTaskHandler, MasterHandler
from navigation_client import TaskResult
from robot import Robot
from rclpy.duration import Duration
import asyncio

class NavigateMaster(Robot):
    def __init__(self, nav_master, name_slave):
        self.nav_master = nav_master
        self.name_slave = name_slave

    async def navigate_robot_master(self, system_master_slave: dict, id_task: str) -> None:
        name_master = self.nav_master.getNameRobot()
        dict_master = system_master_slave[name_master]
        list_slave_tasks = dict_master["slave_tasks"]
        while list_slave_tasks:
            
            id_first_slave = list(list_slave_tasks)[0]
            name_first_slave = list_slave_tasks[id_first_slave]["name_robot"]

            if id_first_slave == id_task and name_first_slave == self.name_slave:
                self.nav_master.info("Completando tarea pendiente del robot esclavo: " + self.name_slave)

                goal_poses_robot = list_slave_tasks[id_first_slave]["goal_poses"]
                self.nav_master.followWaypoints(goal_poses_robot)

                nav_start = self.nav_master.get_clock().now()

                while not self.nav_master.isTaskComplete():
                    
                    await asyncio.sleep(1)
                    feedback = self.nav_master.getFeedback()
                    
                    if feedback:
                        now = self.nav_master.get_clock().now()
                        nav_time = self.nav_master.getTimeNav(now.nanoseconds - nav_start.nanoseconds)
                        
                        current_waypoint = feedback.current_waypoint
                        has_max_time = list_slave_tasks[id_first_slave]["has_max_time"]
                        if dict_master["same_time_task"] and has_max_time:
                            duration_max_time_m = list_slave_tasks[id_first_slave]["duration_max_time"]
                            duration_max_time = Duration(seconds=duration_max_time_m*60)
                            max_time = self.nav_master.getTimeNav(duration_max_time.nanoseconds)
                                                                             
                            super().generate_message(name_master, current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave)

                            if now - nav_start >= duration_max_time:
                                await self.exceed_max_time(list_slave_tasks, id_first_slave, goal_poses_robot, has_max_time, duration_max_time_m, current_waypoint, system_master_slave)
                                
                                return 
                        else:
                            super().generate_message(name_master, current_waypoint, len(goal_poses_robot), nav_time, name_slave=name_first_slave)

                if self.nav_master.getResult() == TaskResult.SUCCEEDED:
                    self.task_complete(list_slave_tasks, id_first_slave)
                
                break
            elif id_first_slave != id_task and name_first_slave == self.name_slave:
                self.nav_slave.info("El esclavo " + self.name_slave + " está esperando que la tarea enviada al esclavo " + name_first_slave + " sea completada una vez que dicho esclavo complete su tarea interna. Es el mismo esclavo pero con otro ID de tarea")
                await asyncio.sleep(1)
            else:
                print("MSJ del maestro: " + name_master + " => El esclavo " + self.name_slave + " está en cola de espera. Ahora ejecuto la tarea del robot " + name_first_slave)
                await asyncio.sleep(1)

    async def exceed_max_time(self, list_slave_tasks: dict, id_first_slave: str, goal_poses_robot: list, has_max_time:bool, duration_max_time_m: str, current_waypoint: int, system_master_slave: dict) -> None:
        super().cancel_task(self.nav_master)

        old_robots_execution = list_slave_tasks[id_first_slave]["old_robots_execution"]

        self.nav_master.info("Tarea NO completada en el tiempo establecido.")
        list_slave_tasks.pop(id_first_slave)
        self.nav_master.info("Tarea eliminada de la lista de tareas pendientes.")

        status_send_goal = await self.send_goal_other_robot(id_first_slave, old_robots_execution, goal_poses_robot, has_max_time, duration_max_time_m, current_waypoint, system_master_slave)

        if not status_send_goal:
            self.nav_master.info("Terminada toda ejecucion, problemas de efectuar la tarea.")

    def task_complete(self, list_slave_tasks: dict, id_first_slave: str) -> None:
        self.nav_master.info("Tarea completada")
        list_slave_tasks.pop(id_first_slave)
        self.nav_master.info("Tarea eliminada de la lista de tareas pendientes.")

    async def send_goal_other_robot(self, id_task: str, old_robots_execution: list, goal_poses_robot: list, has_max_time: bool, duration_max_time: str, current_waypoint: int, system_master_slave: dict) -> bool:
        await asyncio.sleep(1)

        nav_master = self.nav_master
        name_master = nav_master.getNameRobot()
        list_slaves = system_master_slave[name_master]["slaves"]
        nav_slave = list_slaves[self.name_slave]["nav_class"]
        name_slave = nav_slave.getNameRobot()
        
        request = {
            'dict_master': system_master_slave[name_master],
            'nav_slave': nav_slave,
            'id_task': id_task,
            'goal_poses': goal_poses_robot,
            'has_max_time': has_max_time,
            'duration_max_time': duration_max_time,
            'old_robots_execution': old_robots_execution,
            'current_waypoint': current_waypoint,
        }

        master_handler = MasterHandler()
        
        handler, is_free_or_one_task, finish_task = master_handler.handle(request)
        
        #name_slave: nombre del robot que esta pidiendo ayuda de su tarea pendiente
        #handler: robot que esta socorriendo a la ayuda de la tarea pendiente
        if finish_task:
            return False
        else:
            if is_free_or_one_task:
                slave_robot = NavigateSlave(handler, name_master, name_slave)
                await asyncio.gather(slave_robot.navigate_robot_slave(system_master_slave, id_task))
                return True
            else:
                master_robot = NavigateMaster(handler, name_slave)
                await asyncio.gather(master_robot.navigate_robot_master(system_master_slave, id_task))
                return True