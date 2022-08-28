
#include <iostream>
#include <vector>
#include <cmath>

// for jacobian calculation
#include <Eigen/Core>   // Eigen 核心部分
#include <Eigen/Dense>  // 稠密矩阵的代数运算（逆，特征值等）

#include "../include/trajectory_generator.h"

// Constructor
TrajectoryGenerator::TrajectoryGenerator()
{

    // initialize optimization parameters
    m_optimization_parameter.acceptable_dx = 0.01F;
    m_optimization_parameter.acceptable_dy = 0.01F;
    m_optimization_parameter.acceptable_dtheta = 1.0F*PI/180.0F; // 1 degree in rad
    m_optimization_parameter.acceptable_dkappa = 1.0F*PI/180.0F; // 1 degree in rad
    
    m_optimization_parameter.pertubate_value = 0.0001F;
    m_optimization_parameter.pertubate_s_value = 0.001F;

    // clear the path
    m_path.clear();

    m_sample_resolution = 1.0F; // sample resolution of the path
    m_max_iteration = 50;       // max iteration of optimization
    m_is_converge = 0;          // bool to show is converged or not

}

TrajectoryGenerator::~TrajectoryGenerator()
{}

float TrajectoryGenerator::get_sample_resolution()
{
    return m_sample_resolution;
}
int TrajectoryGenerator::set_sample_resolution(float desired_resolution)
{
    m_sample_resolution = desired_resolution;
    return 1;
}
int TrajectoryGenerator::set_optimization_parameter(
    const OptimizationParameter& optimization_parameter)
{
    m_optimization_parameter = optimization_parameter;
    return 1;
}

OptimizationParameter TrajectoryGenerator::get_optimization_parameter()
{
    return m_optimization_parameter;
}

SplineParameter TrajectoryGenerator::get_spline_parameter(){
    return m_spline_parameter;
}

void TrajectoryGenerator::m_initialize_spline(
            const VehicleState& target_state)
{
    // initialize spline parameter with target state
    // in here, it assumes the vehicle is at origin (0, 0, 0, 0)
    float x_f = target_state.x;
    float y_f = target_state.y;
    float theta_f = target_state.theta;
    float kappa_f = target_state.kappa;

    float d = std::sqrt(std::pow(x_f, 2)+ std::pow(y_f,2));
    float theta_delta = std::abs(theta_f);
    
    m_spline_parameter.s = d*((std::pow(theta_delta,2)/5.0f)+ 1.0F)+ 2*theta_delta/5.0F;
    m_spline_parameter.a = 0.0;
    m_spline_parameter.b = 0.0;
    m_spline_parameter.c = kappa_f;

}

float TrajectoryGenerator::m_compute_theta(
    float theta, 
    float a_p, 
    float b_p, 
    float c_p, 
    float d_p, 
    float s)
{
    float theta_final = theta + d_p*(std::pow(s,4))/4.0F + \
                        c_p*(std::pow(s,3))/3.0F + \
                        b_p*(std::pow(s,2))/2.0F + \
                        a_p*s;
    return theta_final;
}


float TrajectoryGenerator::m_compute_y(
    float y_0, 
    float theta_0, 
    float a_p, 
    float b_p, 
    float c_p, 
    float d_p, 
    float s)
{
    float theta_s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, s/8.0F);
    float theta_2s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 2.0F*s/8.0F);
    float theta_3s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 3.0F*s/8.0F);
    float theta_4s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 4.0F*s/8.0F);
    float theta_5s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 5.0F*s/8.0F);
    float theta_6s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 6.0F*s/8.0F);
    float theta_7s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 7.0F*s/8.0F);
    float theta_s = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, s);

    float y_final = y_0 + 
                    (std::sin(theta_0) + \
                    4.0F*std::sin(theta_s_8) + \
                    2.0F*std::sin(theta_2s_8) + \
                    4.0F*std::sin(theta_3s_8) + \
                    2.0F*std::sin(theta_4s_8) + \
                    4.0F*std::sin(theta_5s_8) + \
                    2.0F*std::sin(theta_6s_8) + \
                    4.0F*std::sin(theta_7s_8) + \
                    std::sin(theta_s))* \
                    s/24.0F;
        
    return y_final;
}

float TrajectoryGenerator::m_compute_x(
    float x_0, 
    float theta_0, 
    float a_p, 
    float b_p, 
    float c_p, 
    float d_p, 
    float s)
{
    float theta_s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, s/8.0F);
    float theta_2s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 2.0F*s/8.0F);
    float theta_3s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 3.0F*s/8.0F);
    float theta_4s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 4.0F*s/8.0F);
    float theta_5s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 5.0F*s/8.0F);
    float theta_6s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 6.0F*s/8.0F);
    float theta_7s_8 = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, 7.0F*s/8.0F);
    float theta_s = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, s);

    float x_final = x_0 + 
                    (std::cos(theta_0) + \
                    4.0F*std::cos(theta_s_8) + \
                    2.0F*std::cos(theta_2s_8) + \
                    4.0F*std::cos(theta_3s_8) + \
                    2.0F*std::cos(theta_4s_8) + \
                    4.0F*std::cos(theta_5s_8) + \
                    2.0F*std::cos(theta_6s_8) + \
                    4.0F*std::cos(theta_7s_8) + \
                    std::cos(theta_s))* \
                    s/24.0F;
    
    return x_final;
}

void TrajectoryGenerator::m_do_motion_update_one_shot(
    const VehicleState& initial_state, 
    float a,
    float b,
    float c,
    float s,
    VehicleState* updated_state)
{

    float x_0 = initial_state.x;
    float y_0 = initial_state.y;
    float theta_0 = initial_state.theta;
    float kappa_0 = initial_state.kappa;

    float a_p = kappa_0;
    float b_p = -(11.0F*kappa_0 - 18.0F*a + 9.0F*b - 2.0F*c)/ (2.0F* s);
    float c_p = 9.0F* (2.0F*kappa_0- 5.0F*a+ 4.0F*b - c)/ (2.0F* std::pow(s,2));
    float d_p = -9.0F* (kappa_0 - 3.0F*a + 3.0F*b -c)/ (2.0F* std::pow(s,3));
    float kappa_final = a_p + b_p*s + c_p*std::pow(s,2)+ d_p*std::pow(s,3);

    float theta_final = m_compute_theta(theta_0, a_p, b_p, c_p, d_p, s);
    float x_final = m_compute_x(x_0, theta_0, a_p, b_p, c_p, d_p, s);
    float y_final = m_compute_y(y_0, theta_0, a_p, b_p, c_p, d_p, s);

    updated_state->x = x_final;
    updated_state->y = y_final;
    updated_state->theta = theta_final;
    updated_state->kappa = kappa_final;

}

int TrajectoryGenerator::m_check_converge(
    const VehicleState& target_state,
    const VehicleState& predicted_target_state)
{
    // To-do
    // need to add nan handle, sometimes the matrix will become 
    // singular and the inverse will cause nan

    if(std::abs(target_state.x - predicted_target_state.x) >= \
        m_optimization_parameter.acceptable_dx)
    {
        return 0; //not converged
    }
    else if (std::abs(target_state.y - predicted_target_state.y) >= \
        m_optimization_parameter.acceptable_dy)
    {
        return 0; //not converged
    } 
    else if (std::abs(target_state.theta - predicted_target_state.theta) >= \
        m_optimization_parameter.acceptable_dtheta)
    {
        return 0; //not converged
    } 
    else if (std::abs(target_state.kappa - predicted_target_state.kappa) >= \
        m_optimization_parameter.acceptable_dkappa)
    {
        return 0; //not converged
    } 
    else
    {
        return 1; // converged
    }
}

Eigen::Matrix<float, 3, 1> TrajectoryGenerator::m_compute_correction(
    const VehicleState& initial_state, 
    const VehicleState& target_state)
{
    float pertubate = m_optimization_parameter.pertubate_value;
    float pertubate_s = m_optimization_parameter.pertubate_s_value;

    float a = m_spline_parameter.a;
    float b = m_spline_parameter.b;
    float c = m_spline_parameter.c;
    float s = m_spline_parameter.s;

    VehicleState* predict_no_pertubate = new VehicleState;
    VehicleState* predict_pertubate_a = new VehicleState;
    VehicleState* predict_pertubate_b = new VehicleState;
    VehicleState* predict_pertubate_s = new VehicleState;

    m_do_motion_update_one_shot(initial_state, a, b, c, s, predict_no_pertubate);
    m_do_motion_update_one_shot(initial_state, a+ pertubate, b, c, s, predict_pertubate_a);
    m_do_motion_update_one_shot(initial_state, a, b+ pertubate, c, s, predict_pertubate_b);
    m_do_motion_update_one_shot(initial_state, a, b, c, s+ pertubate_s, predict_pertubate_s);

    Eigen::Matrix<float, 3, 1> d_state;
    Eigen::Matrix<float, 3, 3> d_pertubate_state;
    Eigen::Matrix<float, 3, 3> jacobian_matrix;

    d_state(0,0) = target_state.x - predict_no_pertubate->x;
    d_pertubate_state(0,0) = target_state.x - predict_pertubate_a->x;
    d_pertubate_state(0,1) = target_state.x - predict_pertubate_b->x;
    d_pertubate_state(0,2) = target_state.x - predict_pertubate_s->x;
    jacobian_matrix(0,0) = (d_pertubate_state(0,0) - d_state(0,0))/pertubate;
    jacobian_matrix(0,1) = (d_pertubate_state(0,1) - d_state(0,0))/pertubate;
    jacobian_matrix(0,2) = (d_pertubate_state(0,2) - d_state(0,0))/pertubate_s;
    
    d_state(1,0) = target_state.y - predict_no_pertubate->y;
    d_pertubate_state(1,0) = target_state.y - predict_pertubate_a->y;
    d_pertubate_state(1,1) = target_state.y - predict_pertubate_b->y;
    d_pertubate_state(1,2) = target_state.y - predict_pertubate_s->y;
    jacobian_matrix(1,0) = (d_pertubate_state(1,0) - d_state(1,0))/pertubate;
    jacobian_matrix(1,1) = (d_pertubate_state(1,1) - d_state(1,0))/pertubate;
    jacobian_matrix(1,2) = (d_pertubate_state(1,2) - d_state(1,0))/pertubate_s;
    
    d_state(2,0) = target_state.theta - predict_no_pertubate->theta;
    d_pertubate_state(2,0) = target_state.theta - predict_pertubate_a->theta;
    d_pertubate_state(2,1) = target_state.theta - predict_pertubate_b->theta;
    d_pertubate_state(2,2) = target_state.theta - predict_pertubate_s->theta;
    jacobian_matrix(2,0) = (d_pertubate_state(2,0) - d_state(2,0))/pertubate;
    jacobian_matrix(2,1) = (d_pertubate_state(2,1) - d_state(2,0))/pertubate;
    jacobian_matrix(2,2) = (d_pertubate_state(2,2) - d_state(2,0))/pertubate_s;

    delete predict_no_pertubate;
    delete predict_pertubate_a;
    delete predict_pertubate_b;
    delete predict_pertubate_s;

    // here use inverse only
    // might need to change to pseudo inverse in the future
    Eigen::Matrix<float, 3, 1> correction = jacobian_matrix.inverse() * d_state;
    return correction; 
}

void TrajectoryGenerator::m_compute_spline(
            const VehicleState& initial_state,
            const VehicleState& target_state)
{
    int current_iter = 0;
    Eigen::Matrix<float, 3,1> correction;

    VehicleState* predicted_state = new VehicleState;

    // restart optimization
    m_is_converge = 0;

    while((m_is_converge != 1) && (current_iter <= m_max_iteration))
    {
        correction = m_compute_correction(initial_state, target_state);
        m_spline_parameter.a -= correction(0,0);
        m_spline_parameter.b -= correction(1,0);
        m_spline_parameter.s -= correction(2,0);

        m_do_motion_update_one_shot(
            initial_state, 
            m_spline_parameter.a, 
            m_spline_parameter.b,
            m_spline_parameter.c,
            m_spline_parameter.s,
            predicted_state);

        m_is_converge = m_check_converge(target_state, (*predicted_state));
        current_iter += 1;

        std::cout << "iter = " << current_iter << std::endl;
        std::cout << "(" << predicted_state->x << "," << predicted_state->y << ")" << std::endl;
    }

    delete predicted_state;
}

void TrajectoryGenerator::path_sampling(
    const VehicleState& initial_state)
{

    float a = m_spline_parameter.a;
    float b = m_spline_parameter.b;
    float c = m_spline_parameter.c;
    float s = m_spline_parameter.s;
    
    float s_iter = 0.0F;
    int total_sample = int(s/m_sample_resolution);

    m_path.clear();

    WayPoint wp;
    // first point
    wp.x = initial_state.x;
    wp.y = initial_state.y;
    
    m_path.push_back(wp);

    // non_converge happened
    // only return the first point (current vehicle position)
    if(m_is_converge == 0 || m_spline_parameter.s < 0.0F)
        return;

    // converge succesfully, generate waypoint list
    VehicleState* predicted_state = new VehicleState;
    
    for(int i=1; i<total_sample; i++)
    {
        s_iter += m_sample_resolution;
        m_do_motion_update_one_shot(initial_state, a, b, c, s_iter, predicted_state);
        wp.x = predicted_state->x;
        wp.y = predicted_state->y;
        m_path.push_back(wp);
    }

    delete predicted_state;

    return;

}

void TrajectoryGenerator::plan(
    const VehicleState& initial_state,
    const VehicleState& target_state)
{
    m_initialize_spline(target_state);
    m_compute_spline(initial_state, target_state);
    path_sampling(initial_state);
}

std::vector<WayPoint> TrajectoryGenerator::get_path()
{
    return m_path;
}

// main function to check the implementation
int main(int argc, char** argv)
{
    TrajectoryGenerator trajectory_generator;
    std::vector<WayPoint> path;

    VehicleState initial_state, target_state;
    
    initial_state.x = 0.0F;     // meter
    initial_state.y = 0.0F;     // meter
    initial_state.theta = 0.0F; // rad
    initial_state.kappa = 0.0F;

    target_state.x = 10.0F;      // meter
    target_state.y = 5.0F;     // meter
    target_state.theta = 90.0F*PI/180.0F; // rad
    target_state.kappa = 0.0F;

    trajectory_generator.plan(initial_state, target_state);
    path = trajectory_generator.get_path();
    std::cout << "number of waypoint = " << path.size() << std::endl;
    for(int i=0; i<path.size(); i++)
        std::cout << "(x, y) = " <<\
                    "(" << path[i].x <<\ 
                    "," << path[i].y << \
                    ")" << std::endl;
    
    return 0;
}

