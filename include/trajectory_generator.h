
#ifndef TRACTORY_GENERATOR_H
#define TRACTORY_GENERATOR_H

#include <iostream>
#include <vector>

// for jacobian calculation
#include <Eigen/Core>   // Eigen 核心部分
#include <Eigen/Dense>  // 稠密矩阵的代数运算（逆，特征值等）

static const float PI = 3.14159265359;

struct OptimizationParameter {
    float acceptable_dx;
    float acceptable_dy;
    float acceptable_dtheta;
    float acceptable_dkappa;

    float pertubate_value;
    float pertubate_s_value;
};

struct SplineParameter{
    float a;
    float b;
    float c;
    float s;
};

struct VehicleState {
    float x;
    float y;
    float theta;
    float kappa;
}; 

struct WayPoint{
    float x;
    float y;
};

class TrajectoryGenerator
{
    public:
        // constructor
        TrajectoryGenerator();

        // desctructor
        ~TrajectoryGenerator();


        // main planning function, return a vector with sampled waypoint list
        void plan(
              const VehicleState& initial_state,
              const VehicleState& target_state
        );

        // Only resample the path again
        void path_sampling(
            const VehicleState& initial_state
        );
        
        // get current sample resolution
        float get_sample_resolution();

        std::vector<WayPoint> get_path();

        // set sample resolution
        int set_sample_resolution(float desired_resolution);
        
        // set optimization parameters
        int set_optimization_parameter(
            const OptimizationParameter& optimization_parameter 
        );
        OptimizationParameter get_optimization_parameter();
        SplineParameter get_spline_parameter();

    private:
        
        void m_initialize_spline(
            const VehicleState& target_state
        );
        void m_compute_spline(
            const VehicleState& initial_state,
            const VehicleState& target_state
        );
        void m_do_motion_update_one_shot(
            const VehicleState& initial_state,
            float a,
            float b,
            float c,
            float s, 
            VehicleState* updated_state
        );

        int m_check_converge(
            const VehicleState& pred_target_state,
            const VehicleState& target_state
        );

        Eigen::Matrix<float, 3,1> m_compute_correction(
            const VehicleState& initial_state, 
            const VehicleState& target_state
        );
        
        float m_compute_x(
            float x_0, 
            float theta_0, 
            float a_p, 
            float b_p, 
            float c_p, 
            float d_p, 
            float s
        );
        float m_compute_y(
            float y_0, 
            float theta_0, 
            float a_p, 
            float b_p, 
            float c_p, 
            float d_p, 
            float s
        );
        float m_compute_theta(
            float theta, 
            float a_p, 
            float b_p, 
            float c_p, 
            float d_p, 
            float s
        );


        OptimizationParameter  m_optimization_parameter;
        SplineParameter m_spline_parameter;

        std::vector<WayPoint> m_path;

        float m_sample_resolution;
        int m_max_iteration;
        int m_is_converge;

}; // class TrajectoryGenerator

#endif // TRACTORY_GENERATOR_H