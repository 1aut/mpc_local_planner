#ifndef MPC_CONFIG_H_
#define MPC_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <mpc_local_planner/MpcLocalPlannerReconfigureConfig.h>

namespace mpc_local_planner
{

class MpcConfig
{
public:
  
  std::string odom_topic;

  struct CollisionAvoidance
  {
    double costmap_obstacles_behind_robot_dist;
    double cutoff_dist;
    bool enable_dynamic_obstacles;
    double force_inclusion_dist;
    bool include_costmap_obstacles;
    double min_obstacle_dist;
  } collision_avoidance;

  struct Controller
  {
    bool allow_init_with_backward_motion;
    double force_reinit_new_goal_angular;
    double force_reinit_new_goal_dist;
    int force_reinit_num_steps;
    bool global_plan_overwrite_orientation;
    double global_plan_prune_distance;
    double global_plan_viapoint_sep;
    double max_global_plan_lookahead_dist;
    int outer_ocp_iterations;
    bool prefer_x_feedback;
    bool publish_ocp_results;
    double xy_goal_tolerance;
    double yaw_goal_tolerance;
    bool print_cpu_time;
  } controller;

  struct FootprintModel
  {
    std::string type;
    bool is_footprint_dynamic;
  } footprint_model;

  struct Grid
  {
    std::string collocation_method;
    std::string cost_integration_method;
    double dt_ref;
    int grid_size_ref;
    std::string type;
    bool warm_start;
    std::vector<bool> xf_fixed;
    struct VariableGrid
    {
      bool enable;
      double min_dt;
      double max_dt;
      struct GridAdaption
      {
        bool enable;
        double dt_hyst_ratio;
        int min_grid_size;
        int max_grid_size;
      } grid_adaption;
    } variable_grid;
  } grid;

  struct Planning
  {
    struct Objective
    {
      std::string type;
      struct MinimumTimeViaPoints
      {
        double orientation_weight;
        double position_weight;
        bool via_points_ordered;
      } minimum_time_via_points;
      struct QuadraticForm
      {
        std::vector<double> control_weights;
        std::vector<double> state_weights;
        bool integral_form;
        bool hybrid_cost_minimum_time;
      } quadratic_form;
    } objective;
    struct TerminalConstraint
    {
      std::string type;
      struct L2Ball
      {
        double radius;
        std::vector<double> weight_matrix;
      } l2_ball;
    } terminal_constraint;
    struct TerminalCost
    {
      std::string type;
      struct Quadratic
      {
        std::vector<double> final_state_weights;
      } quadratic;
    } terminal_cost;
  } planning;      
 
  struct Robot 
  {
    std::string type;
    struct Unicycle
    {
      double acc_lim_x;
      double dec_lim_x;
      double max_vel_x;
      double max_vel_x_backwards;
      double acc_lim_theta;
      double max_vel_theta;
    } unicycle;
    struct SimpleCar
    {
      double wheelbase;
      bool front_wheel_driving;
      double acc_lim_x;
      double dec_lim_x;
      double max_vel_x;
      double max_vel_x_backwards;
      double max_steering_angle;
      double max_steering_rate;
    } simple_car;
    struct KinematicBicycleVelInput
    {
      double length_rear;
      double length_front;
      double acc_lim_x;
      double dec_lim_x;
      double max_vel_x;
      double max_vel_x_backwards;
      double max_steering_angle;
      double max_steering_rate;
    } kinematic_bicycle_vel_input;
  } robot;

  struct Solver
  {
    std::string type;
    struct Ipopt
    {
      int iterations;
      double max_cpu_time;
      std::map<std::string, double> ipopt_numeric_options;
      std::map<std::string, std::string> ipopt_string_options;
      std::map<std::string, int> ipopt_integer_options;
    } ipopt;
    struct LsqLm
    {
      int iterations;
      double weight_adapt_factor_bounds;
      double weight_adapt_factor_eq;
      double weight_adapt_factor_ineq;
      double weight_adapt_max_bounds;
      double weight_adapt_max_eq;
      double weight_adapt_max_ineq;
      double weight_init_bounds;
      double weight_init_eq;
      double weight_init_ineq;
    } lsq_lm;
  } solver;

  MpcConfig()
  {

    odom_topic = "odom";


    collision_avoidance.costmap_obstacles_behind_robot_dist = 1.5;
    collision_avoidance.cutoff_dist = 5;
    collision_avoidance.enable_dynamic_obstacles = false;
    collision_avoidance.force_inclusion_dist = 1.5;
    collision_avoidance.include_costmap_obstacles = true;
    collision_avoidance.min_obstacle_dist = 0.3;


    controller.allow_init_with_backward_motion = true;
    controller.force_reinit_new_goal_angular = M_PI/2;
    controller.force_reinit_new_goal_dist = 1.0;
    controller.force_reinit_num_steps = 0;
    controller.global_plan_overwrite_orientation = true;
    controller.global_plan_prune_distance = 1.0;
    controller.global_plan_viapoint_sep = 0.5;
    controller.max_global_plan_lookahead_dist = 1.5;
    controller.outer_ocp_iterations = 1;
    controller.prefer_x_feedback = false;
    controller.publish_ocp_results = false;
    controller.xy_goal_tolerance = 0.25;
    controller.yaw_goal_tolerance = 0.15;
    controller.print_cpu_time = false;


    footprint_model.type = "point";
    footprint_model.is_footprint_dynamic = false;


    grid.collocation_method = "forward_differences";
    grid.cost_integration_method = "left_sum";
    grid.dt_ref = 0.3;
    grid.grid_size_ref = 20;
    grid.type = "fd_grid";
    grid.warm_start = true;
    grid.xf_fixed = {true, true, true};
      
    grid.variable_grid.enable = true;
    grid.variable_grid.min_dt = 0.0;
    grid.variable_grid.max_dt = 10.0;
    
    grid.variable_grid.grid_adaption.enable = true;
    grid.variable_grid.grid_adaption.dt_hyst_ratio = 0.1;
    grid.variable_grid.grid_adaption.min_grid_size = 2;
    grid.variable_grid.grid_adaption.max_grid_size = 50;

      
    planning.objective.type = "minimum_time";
    planning.objective.minimum_time_via_points.orientation_weight = 0.0;
    planning.objective.minimum_time_via_points.position_weight = 10.5;
    planning.objective.minimum_time_via_points.via_points_ordered = false;
    planning.objective.quadratic_form.control_weights = {1.0, 1.0};
    planning.objective.quadratic_form.state_weights = {2.0, 2.0, 2.0};
    planning.objective.quadratic_form.integral_form = false;
    planning.objective.quadratic_form.hybrid_cost_minimum_time = false;
    
    planning.terminal_constraint.type = "none";
    planning.terminal_constraint.l2_ball.radius = 5;
    planning.terminal_constraint.l2_ball.weight_matrix = {1.0, 1.0, 1.0};
  
    planning.terminal_cost.type = "none";
    planning.terminal_cost.quadratic.final_state_weights = {2.0, 2.0, 2.0};  
 
    
    robot.type = "unicycle";

    robot.unicycle.acc_lim_x = 2.0;
    robot.unicycle.dec_lim_x = 2.0;
    robot.unicycle.max_vel_x = 0.35;
    robot.unicycle.max_vel_x_backwards = 0.35;
    robot.unicycle.acc_lim_theta = 2.0;
    robot.unicycle.max_vel_theta = 1.0;

    robot.simple_car.acc_lim_x = 2.0;
    robot.simple_car.dec_lim_x = 2.0;
    robot.simple_car.max_vel_x = 0.35;
    robot.simple_car.max_vel_x_backwards = 0.35;
    robot.simple_car.wheelbase = 0.5;
    robot.simple_car.front_wheel_driving = false;
    robot.simple_car.max_steering_angle = 1.5;
    robot.simple_car.max_steering_rate = 0.0;

    robot.kinematic_bicycle_vel_input.acc_lim_x = 2.0;
    robot.kinematic_bicycle_vel_input.dec_lim_x = 2.0;
    robot.kinematic_bicycle_vel_input.max_vel_x = 0.35;
    robot.kinematic_bicycle_vel_input.max_vel_x_backwards = 0.35;
    robot.kinematic_bicycle_vel_input.length_rear = 1.0;
    robot.kinematic_bicycle_vel_input.length_front = 1.0;
    robot.kinematic_bicycle_vel_input.max_steering_angle = 1.5;
    robot.kinematic_bicycle_vel_input.max_steering_rate = 0.0;


    solver.type = "ipopt";

    solver.ipopt.iterations = 100;
    solver.ipopt.max_cpu_time = -1.0;
    solver.ipopt.ipopt_numeric_options["tol"] = 0.0001;
    solver.ipopt.ipopt_string_options["hessian_approximation"] = "exact";
    solver.ipopt.ipopt_string_options["linear_solver"] = "mumps";

    solver.lsq_lm.iterations = 10;
    solver.lsq_lm.weight_adapt_factor_bounds = 1.5;
    solver.lsq_lm.weight_adapt_factor_eq = 1.5;
    solver.lsq_lm.weight_adapt_factor_ineq = 1.5;
    solver.lsq_lm.weight_adapt_max_bounds = 500;
    solver.lsq_lm.weight_adapt_max_eq = 500;
    solver.lsq_lm.weight_adapt_max_ineq = 500;
    solver.lsq_lm.weight_init_bounds = 2.0;
    solver.lsq_lm.weight_init_eq = 2.0;
    solver.lsq_lm.weight_init_ineq = 2.0;

  }

  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);
  void reconfigure(MpcLocalPlannerReconfigureConfig& cfg);

  boost::mutex& configMutex() {return config_mutex_;}

private:
  boost::mutex config_mutex_; //!< Mutex for config accesses and changes

};

}

#endif
