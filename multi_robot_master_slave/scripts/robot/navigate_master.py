#!/usr/bin/env python3

from rclpy.duration import Duration
from navigation_client import TaskResult
import asyncio
from robot import Robot

class NavigateMaster(Robot):
    def __init__(self, nav_master, name_slave):
        self.nav_master = nav_master
        self.name_slave = name_slave

    async def navigate_robot_master(self, system_master_slave, id_task):
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
                        
                        if dict_master["same_time_task"]:
                            duration_max_time_m = list_slave_tasks[id_first_slave]["duration_max_time"]
                            duration_max_time = Duration(seconds=duration_max_time_m*60)
                            max_time = self.nav_master.getTimeNav(duration_max_time.nanoseconds)
                            
                            super().generate_message(name_master, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave)

                            if now - nav_start >= duration_max_time:
                                super().cancel_task(self.nav_master)
                                self.nav_master.info("Tarea NO completada en el tiempo establecido.")
                                list_slave_tasks.pop(id_first_slave)
                                self.nav_master.info("Tarea eliminada de la lista de tareas pendientes.")
                        else:
                            super().generate_message(name_master, feedback.current_waypoint, len(goal_poses_robot), nav_time, name_slave=name_first_slave)

                if self.nav_master.getResult() == TaskResult.SUCCEEDED:
                    self.nav_master.info("Tarea completada")
                    list_slave_tasks.pop(id_first_slave)
                    self.nav_master.info("Tarea eliminada de la lista de tareas pendientes.")

                break
            elif id_first_slave != id_task and name_first_slave == self.name_slave:
                self.nav_slave.info("El esclavo " + self.name_slave + " está esperando que la tarea enviada al esclavo " + name_first_slave + " sea completada una vez que dicho esclavo complete su tarea interna. Es el mismo esclavo pero con otro ID de tarea")
                await asyncio.sleep(1)
            else:
                print("MSJ del maestro: " + name_master + " => El esclavo " + self.name_slave + " está en cola de espera. Ahora ejecuto la tarea del robot " + name_first_slave)
                await asyncio.sleep(1)