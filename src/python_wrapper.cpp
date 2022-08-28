#include <iostream>
#include <vector>
#include "trajectory_generator.h"

extern "C"
{
    TrajectoryGenerator* trajectory_generator_new()
    {
        return new TrajectoryGenerator;
    }

    void plan(
        TrajectoryGenerator* trajectory_generator,
        VehicleState initial_state,
        VehicleState target_state)
    {
        trajectory_generator -> plan(initial_state, target_state);
    }

    int get_path_size(
        TrajectoryGenerator* trajectory_generator)
    {
        return (trajectory_generator -> get_path()).size();
    }

    WayPoint* get_path(
        TrajectoryGenerator* trajectory_generator)
    {
        int path_size = (trajectory_generator -> get_path()).size();
        WayPoint* way_point_list = new WayPoint[path_size];
        std::memcpy(way_point_list, 
                    (trajectory_generator->get_path().data()), 
                    path_size* sizeof(WayPoint)
                    );
        return way_point_list;
    }

    // To-be checked
    // Do we need to free the pointer from ctypes??
    void free_path(
        WayPoint* way_point_list)
    {
        std::free(way_point_list);
    }

    void delete_trajectory_generator(
        TrajectoryGenerator* trajectory_generator)
    {
        delete trajectory_generator;
    }

} // extern C
