import ctypes
import pdb
import copy

import numpy as np
import matplotlib.pyplot as plt

class VehicleState(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
               ("y", ctypes.c_float),
               ("theta", ctypes.c_float),
               ("kappa", ctypes.c_float)]

class WayPoint(ctypes.Structure):
     _fields_ = [("x", ctypes.c_float),
               ("y", ctypes.c_float)]

class TrajectoryGenerator():
    def __init__(self):
        self.lib = ctypes.cdll.LoadLibrary('../build/libligTrajectoryGenerator.so')

        self.lib.trajectory_generator_new.argtypes = None
        self.lib.trajectory_generator_new.restype = ctypes.c_void_p
        
        self.lib.plan.argtypes = [ctypes.c_void_p,
                                    ctypes.Structure,
                                    ctypes.Structure
                                    ]
        self.lib.plan.restype = None

        self.lib.get_path.argtypes = [ctypes.c_void_p]
        self.lib.get_path.restype = ctypes.POINTER(WayPoint)

        self.lib.get_path_size.argtypes = [ctypes.c_void_p]
        self.lib.get_path_size.restype = ctypes.c_int

        self.lib.delete_trajectory_generator.argtypes = [ctypes.c_void_p]
        self.lib.delete_trajectory_generator.restype = None

        self.lib.free_path.argtypes = [ctypes.POINTER(WayPoint)]
        self.lib.free_path_restype = None

        self.handler = self.lib.trajectory_generator_new()
        
        self.path = WayPoint()
        self.path_size = 0
    
    def plan(self, initial_state, target_state):
        self.lib.plan(self.handler, initial_state, target_state)
        self.path_size = self.lib.get_path_size(self.handler)

    def get_path(self):
        self.path = (WayPoint*self.path_size)()
        self.path = self.lib.get_path(self.handler)
    
    def free_path(self):
        self.lib.free_path(self.path)
        self.path_size = 0



if __name__ == "__main__":
    trajectory_generator = TrajectoryGenerator()

    # path_size = ctypes.c_int
    
    initial_state = VehicleState()
    initial_state.x = 0.0
    initial_state.y = 0.0
    initial_state.theta = 0.0
    initial_state.kappa = 0.0

    target_state = VehicleState()
    target_state.x = 30.0
    target_state.y = 0.0
    target_state.theta = 0.0
    target_state.kappa = 0.0   

    waypoint_array = []
    waypoint_list = []

    vertical_sample_resolution = 2
    original_target_y = target_state.y
    
    for sample in range(-5,5): 

        target_state.y = original_target_y + sample*vertical_sample_resolution
        if(trajectory_generator.path_size > 0):
            trajectory_generator.free_path()
        trajectory_generator.plan(initial_state, target_state)
        trajectory_generator.get_path()

        for i in range(0, trajectory_generator.path_size):
            waypoint_list.append(
                copy.deepcopy(
                    [trajectory_generator.path[i].x ,trajectory_generator.path[i].y]
                )
            )

        waypoint_array.append(np.asarray(waypoint_list))

    for i in range(0, len(waypoint_array)):
        plt.plot(waypoint_array[i][:,0], waypoint_array[i][:,1],'o')
    plt.show()

    # pdb.set_trace()