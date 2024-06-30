from abc import ABC, abstractmethod
from typing import Any, Optional

class Handler(ABC):
    @abstractmethod
    def set_next(self, handler: 'Handler') -> 'Handler':
        pass

    @abstractmethod
    def handle(self, request: dict) -> Optional[Any]:
        pass

class AbstractHandler(Handler):
    _next_handler: Handler = None

    def set_next(self, handler: Handler) -> Handler:
        self._next_handler = handler
        return handler

    @abstractmethod
    def handle(self, request: dict) -> Optional[Any]:
        if self._next_handler:
            return self._next_handler.handle(request)
        return None

class FreeSlaveHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:
        nav_slave = request["nav_slave"]
        nav_slave.info("---> Buscando esclavo libre para realizar la tarea... <---")
        
        self.name_slave = nav_slave.getNameRobot()
        id_task = request['id_task']

        nav_master = request["dict_master"]["nav_class"]
        self.list_slaves = request["dict_master"]["slaves"]
        self.old_robots_execution = request['old_robots_execution']
        self.use_camera = request['use_camera']

        free_slave, found_free_slave = self.find_free_slave()
        if found_free_slave:
            nav_free_slave = self.list_slaves[free_slave]["nav_class"]
            nav_free_slave.info(f'Para el esclavo {self.name_slave}, yo estoy libre.')

            goal_poses = request['goal_poses'][request['current_waypoint']:]
            has_max_time = request['has_max_time']
            duration_max_time = request['duration_max_time']

            self.old_robots_execution.append(free_slave)

            self.list_slaves[free_slave]["task_queue"][id_task] = {
                'name_robot': self.name_slave, 
                'use_camera': self.use_camera,
                'goal_poses': goal_poses,
                'has_max_time': has_max_time,
                'duration_max_time': duration_max_time,
                'old_robots_execution': self.old_robots_execution,
            }
            
            return nav_free_slave, True, False
        else:
            nav_slave.info("No encontre esclavos libres y que no hayan realizado esta tarea...")
            return super().handle(request)
    
    def find_free_slave(self) -> (str, bool):
        find_free_slave = False
        slave = None
        
        for name_slave_iter in self.list_slaves:
            if self.name_slave != name_slave_iter:
                if len(self.list_slaves[name_slave_iter]["task_queue"]) == 0 and name_slave_iter not in self.old_robots_execution:
                    if self.use_camera:
                        if self.list_slaves[name_slave_iter]["has_camera"]:
                            return name_slave_iter, True
                        else:
                            print("La tarea requiere de la cámara y el esclavo no la tiene.")
                    else:
                        return name_slave_iter, True
        
        return slave, find_free_slave

class SlaveWithOneTaskHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:
        nav_slave = request["nav_slave"]
        nav_slave.info("---> Buscando esclavo con una tarea pendiente para realizar la tarea... <---")
        
        self.name_slave = nav_slave.getNameRobot()
        
        nav_master = request["dict_master"]["nav_class"]
        name_master = nav_master.getNameRobot()
        self.list_slaves = request["dict_master"]["slaves"]
        id_task = request["id_task"]
        self.old_robots_execution = request['old_robots_execution']
        self.use_camera = request['use_camera']
        
        slave_with_one_task, found_slave_with_one_task = self.find_slave_with_one_task()
        if found_slave_with_one_task:
            nav_slave_with_one_task = self.list_slaves[slave_with_one_task]["nav_class"]
            nav_slave_with_one_task.info(f'Para el esclavo {self.name_slave}, yo tengo una tarea pendiente.')
            
            goal_poses = request['goal_poses'][request['current_waypoint']:]
            has_max_time = request['has_max_time']
            duration_max_time = request['duration_max_time']

            self.old_robots_execution.append(slave_with_one_task)

            self.list_slaves[slave_with_one_task]["task_queue"][id_task] = {
                'name_robot': self.name_slave, 
                'use_camera': self.use_camera,
                'goal_poses': goal_poses,
                'has_max_time': has_max_time,
                'duration_max_time': duration_max_time,
                'old_robots_execution': self.old_robots_execution,
            }
            
            return nav_slave_with_one_task, True, False
        else:
            nav_slave.info("No encontre esclavos con una tarea pendiente y que no hayan realizado esta tarea...")
            return super().handle(request)

    def find_slave_with_one_task(self) -> (str, bool):
        find_slave_with_one_task = False
        slave = None
        
        for name_slave_iter in self.list_slaves:
            if self.name_slave != name_slave_iter:

                list_task_queue = self.list_slaves[name_slave_iter]["task_queue"]
                for id_task_iter in list_task_queue:
                    
                    name_robot_task_iter = list_task_queue[id_task_iter]["name_robot"]
                    goal_poses = len(list_task_queue[id_task_iter]["goal_poses"])
                    slave_class = self.list_slaves[name_robot_task_iter]["nav_class"]
                    current_pose = slave_class.getFeedback().current_waypoint

                    if (goal_poses - current_pose) == 1 and name_slave_iter not in self.old_robots_execution:
                        if self.use_camera:
                            if self.list_slaves[name_slave_iter]["has_camera"]:
                                return name_slave_iter, True
                            else:
                                print("La tarea requiere de la cámara y el esclavo no la tiene.")
                        else:
                            return name_slave_iter, True
        
        return slave, find_slave_with_one_task 

class MasterHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:

        self.name_slave = request["nav_slave"].getNameRobot()
        id_task = request["id_task"]

        self.nav_master = request["dict_master"]["nav_class"]
        self.name_master = self.nav_master.getNameRobot()
        self.has_camera_master = request["dict_master"]["has_camera"]

        self.goal_poses = request['goal_poses'][request['current_waypoint']:]
        self.has_max_time = request['has_max_time']
        self.duration_max_time = request['duration_max_time']
        self.old_robots_execution = request['old_robots_execution']
        self.use_camera = request['use_camera']

        list_slaves = request["dict_master"]["slaves"]
        self.find_other_slave = None

        self.nav_master.info("---> Verificando si el master está disponible u otro esclavo... <---")
        for slave in list_slaves:
            if slave not in self.old_robots_execution:
                if self.use_camera:
                    if list_slaves[slave]["has_camera"]:
                        self.find_other_slave = slave
                    else:
                        self.nav_master.info("Existe un esclavo disponible, pero la tarea es con cámara y el esclavo no la tiene.")
                else:
                    self.find_other_slave = slave

        task_use_camera_and_master_hasn_camera = False
        if self.name_master not in self.old_robots_execution:
            if self.use_camera:
                if self.has_camera_master:
                    request['dict_master']["slave_tasks"][id_task] = self.master_not_executed()
                    return self.nav_master, False, False
                else:
                    self.nav_master.info("La tarea requiere de la cámara y el master no la tiene.")
                    task_use_camera_and_master_hasn_camera = True
            else:
                request['dict_master']["slave_tasks"][id_task] = self.master_not_executed()
                return self.nav_master, False, False

        if self.find_other_slave != None and task_use_camera_and_master_hasn_camera:
            nav_slave = list_slaves[self.find_other_slave]["nav_class"]
            list_slaves[self.find_other_slave]["task_queue"][id_task] = self.master_exect_slave_not_executed()

            return nav_slave, True, False

        elif self.find_other_slave != None and self.name_master in self.old_robots_execution:
            nav_slave = list_slaves[self.find_other_slave]["nav_class"]
            list_slaves[self.find_other_slave]["task_queue"][id_task] = self.master_exect_slave_not_executed()

            return nav_slave, True, False

        elif self.find_other_slave == None and self.name_master in self.old_robots_execution:
            self.master_and_slave_exect()

            return self.nav_master, False, True

        elif self.find_other_slave == None and task_use_camera_and_master_hasn_camera:
            self.mast_hasn_cam_slave_not_executed()

            return self.nav_master, False, True

        else:
            return super().handle(request)

    def is_master_busy(self) -> None:
        if self.nav_master.getFeedback() is None:
            self.nav_master.info("Master disponible, se efectuará la tarea de manera inmediata.")
        else:                
            self.nav_master.info("Master ocupado, está en línea de espera.")

    def master_not_executed(self) -> dict:
        self.is_master_busy()

        self.old_robots_execution.append(self.name_master)

        task_queue = {
            'name_robot': self.name_slave,
            'use_camera': self.use_camera,
            'goal_poses': self.goal_poses, 
            'has_max_time': self.has_max_time, 
            'duration_max_time': self.duration_max_time,
            'old_robots_execution': self.old_robots_execution,
        }

        return task_queue

    def master_exect_slave_not_executed(self) -> dict:
        self.nav_master.info(f'El maestro {self.name_master} anteriormente ejecutó la tarea, el esclavo {self.find_other_slave} ejecutara esta tarea.')

        self.old_robots_execution.append(self.find_other_slave)

        task_queue = {
            'name_robot': self.name_slave, 
            'use_camera': self.use_camera,
            'goal_poses': self.goal_poses, 
            'has_max_time': self.has_max_time,
            'duration_max_time': self.duration_max_time,
            'old_robots_execution': self.old_robots_execution,
        }

        return task_queue

    def master_and_slave_exect(self) -> None:
        
        self.nav_master.info(f'El maestro {self.name_master} y todos los esclavos anteriormente ya ejecutaron esta intentaron cumplir esta tarea.')
        self.nav_master.info("Revisarla manualmente.")

    def mast_hasn_cam_slave_not_executed(self) -> None:

        self.nav_master.info("No hay esclavos disponibles para realizar la tarea con cámara.")
        self.nav_master.info("El master no tiene la cámara y la tarea la requiere.")