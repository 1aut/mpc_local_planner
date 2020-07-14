#include <mpc_local_planner/mpc_config.h>

namespace mpc_local_planner
{


void MpcConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
  nh.param("controller/xy_goal_tolerance", controller.xy_goal_tolerance, controller.xy_goal_tolerance);
  nh.param("controller/yaw_goal_tolerance", controller.yaw_goal_tolerance, controller.yaw_goal_tolerance);
  nh.param("controller/global_plan_overwrite_orientation", controller.global_plan_overwrite_orientation,
           controller.global_plan_overwrite_orientation);
  nh.param("controller/global_plan_prune_distance", controller.global_plan_prune_distance, controller.global_plan_prune_distance);
  nh.param("controller/max_global_plan_lookahead_dist", controller.max_global_plan_lookahead_dist, controller.max_global_plan_lookahead_dist);
  nh.param("controller/global_plan_viapoint_sep", controller.global_plan_viapoint_sep, controller.global_plan_viapoint_sep);
  nh.param("controller/outer_ocp_iterations", controller.outer_ocp_iterations, controller.outer_ocp_iterations);
  nh.param("controller/force_reinit_new_goal_dist", controller.force_reinit_new_goal_dist, controller.force_reinit_new_goal_dist);
  nh.param("controller/force_reinit_new_goal_angular", controller.force_reinit_new_goal_angular, controller.force_reinit_new_goal_angular);
  nh.param("controller/allow_init_with_backward_motion", controller.allow_init_with_backward_motion, controller.allow_init_with_backward_motion);
  nh.param("controller/force_reinit_num_steps", controller.force_reinit_num_steps, controller.force_reinit_num_steps);
  nh.param("controller/prefer_x_feedback", controller.prefer_x_feedback, controller.prefer_x_feedback);
  nh.param("controller/publish_ocp_results", controller.publish_ocp_results, controller.publish_ocp_results);
  nh.param("controller/print_cpu_time", controller.print_cpu_time, controller.print_cpu_time);


  nh.param("odom_topic", odom_topic, odom_topic);
  nh.param("footprint_model/is_footprint_dynamic", footprint_model.is_footprint_dynamic, footprint_model.is_footprint_dynamic);

  nh.param("collision_avoidance/include_costmap_obstacles", collision_avoidance.include_costmap_obstacles, collision_avoidance.include_costmap_obstacles);
  nh.param("collision_avoidance/costmap_obstacles_behind_robot_dist", collision_avoidance.costmap_obstacles_behind_robot_dist,
           collision_avoidance.costmap_obstacles_behind_robot_dist);
  nh.param("collision_avoidance/min_obstacle_dist", collision_avoidance.min_obstacle_dist, collision_avoidance.min_obstacle_dist);
  nh.param("collision_avoidance/enable_dynamic_obstacles", collision_avoidance.enable_dynamic_obstacles, collision_avoidance.enable_dynamic_obstacles);
  nh.param("collision_avoidance/force_inclusion_dist", collision_avoidance.force_inclusion_dist, collision_avoidance.force_inclusion_dist);
  nh.param("collision_avoidance/cutoff_dist", collision_avoidance.cutoff_dist, collision_avoidance.cutoff_dist);

////////////////////////////// weird parameters, TODO: check //////////////////////////////////////////////////////////////////////////////
//  nh.param("collision_avoidance/collision_check_no_poses", _params.collision_check_no_poses, _params.collision_check_no_poses);        //
//  nh.param("collision_avoidance/collision_check_min_resolution_angular", _params.collision_check_min_resolution_angular,               //
//           _params.collision_check_min_resolution_angular);                                                                            //
//  nh.param("costmap_converter_plugin", _costmap_conv_params.costmap_converter_plugin, _costmap_conv_params.costmap_converter_plugin);  //
//  nh.param("costmap_converter_rate", _costmap_conv_params.costmap_converter_rate, _costmap_conv_params.costmap_converter_rate);        //
//  nh.param("costmap_converter_spin_thread", _costmap_conv_params.costmap_converter_spin_thread,                                        //
//           _costmap_conv_params.costmap_converter_spin_thread);                                                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  nh.param("grid/type", grid.type, grid.type);
  nh.param("grid/variable_grid/enable", grid.variable_grid.enable, grid.variable_grid.enable);
  nh.param("grid/variable_grid/min_dt", grid.variable_grid.min_dt, grid.variable_grid.min_dt);
  nh.param("grid/variable_grid/max_dt", grid.variable_grid.max_dt, grid.variable_grid.max_dt);
  nh.param("grid/variable_grid/grid_adaptation/enable", grid.variable_grid.grid_adaption.enable, grid.variable_grid.grid_adaption.enable);
  nh.param("grid/variable_grid/grid_adaptation/max_grid_size", grid.variable_grid.grid_adaption.max_grid_size, grid.variable_grid.grid_adaption.max_grid_size);
  nh.param("grid/variable_grid/grid_adaptation/dt_hyst_ratio", grid.variable_grid.grid_adaption.dt_hyst_ratio, grid.variable_grid.grid_adaption.dt_hyst_ratio);
  nh.param("grid/variable_grid/grid_adaptation/min_grid_size", grid.variable_grid.grid_adaption.min_grid_size, grid.variable_grid.grid_adaption.min_grid_size);
  nh.param("grid/grid_size_ref", grid.grid_size_ref, grid.grid_size_ref);
  nh.param("grid/dt_ref", grid.dt_ref, grid.dt_ref);
  nh.param("grid/xf_fixed", grid.xf_fixed, grid.xf_fixed);
  nh.param("grid/warm_start", grid.warm_start, grid.warm_start);
  nh.param("grid/collocation_method", grid.collocation_method, grid.collocation_method);
  nh.param("grid/cost_integration_method", grid.cost_integration_method, grid.cost_integration_method);

  nh.param("robot/type", robot.type, robot.type);

  nh.param("robot/unicycle/max_vel_x", robot.unicycle.max_vel_x, robot.unicycle.max_vel_x);
  nh.param("robot/unicycle/max_vel_x_backwards", robot.unicycle.max_vel_x_backwards, robot.unicycle.max_vel_x_backwards);
  nh.param("robot/unicycle/max_vel_theta", robot.unicycle.max_vel_theta, robot.unicycle.max_vel_theta);
  nh.param("robot/unicycle/acc_lim_x", robot.unicycle.acc_lim_x, robot.unicycle.acc_lim_x);
  nh.param("robot/unicycle/dec_lim_x", robot.unicycle.dec_lim_x, robot.unicycle.dec_lim_x);
  nh.param("robot/unicycle/acc_lim_theta", robot.unicycle.acc_lim_theta, robot.unicycle.acc_lim_theta);

  nh.param("robot/simple_car/max_vel_x", robot.simple_car.max_vel_x, robot.simple_car.max_vel_x);
  nh.param("robot/simple_car/max_vel_x_backwards", robot.simple_car.max_vel_x_backwards, robot.simple_car.max_vel_x_backwards);
  nh.param("robot/simple_car/max_steering_angle", robot.simple_car.max_steering_angle, robot.simple_car.max_steering_angle);
  nh.param("robot/simple_car/acc_lim_x", robot.simple_car.acc_lim_x, robot.simple_car.acc_lim_x);
  nh.param("robot/simple_car/dec_lim_x", robot.simple_car.dec_lim_x, robot.simple_car.dec_lim_x);
  nh.param("robot/simple_car/max_steering_rate", robot.simple_car.max_steering_rate, robot.simple_car.max_steering_rate);
  nh.param("robot/simple_car/wheelbase", robot.simple_car.wheelbase, robot.simple_car.wheelbase);
  nh.param("robot/simple_car/front_wheel_driving", robot.simple_car.front_wheel_driving, robot.simple_car.front_wheel_driving);

  nh.param("robot/kinematic_bicycle_vel_input/max_vel_x", robot.kinematic_bicycle_vel_input.max_vel_x, robot.kinematic_bicycle_vel_input.max_vel_x);
  nh.param("robot/kinematic_bicycle_vel_input/max_vel_x_backwards", robot.kinematic_bicycle_vel_input.max_vel_x_backwards, robot.kinematic_bicycle_vel_input.max_vel_x_backwards);
  nh.param("robot/kinematic_bicycle_vel_input/max_steering_angle", robot.kinematic_bicycle_vel_input.max_steering_angle, robot.kinematic_bicycle_vel_input.max_steering_angle);
  nh.param("robot/kinematic_bicycle_vel_input/acc_lim_x", robot.kinematic_bicycle_vel_input.acc_lim_x, robot.kinematic_bicycle_vel_input.acc_lim_x);
  nh.param("robot/kinematic_bicycle_vel_input/dec_lim_x", robot.kinematic_bicycle_vel_input.dec_lim_x, robot.kinematic_bicycle_vel_input.dec_lim_x);
  nh.param("robot/kinematic_bicycle_vel_input/max_steering_rate", robot.kinematic_bicycle_vel_input.max_steering_rate, robot.kinematic_bicycle_vel_input.max_steering_rate);
  nh.param("robot/kinematic_bicycle_vel_input/length_rear", robot.kinematic_bicycle_vel_input.length_rear, robot.kinematic_bicycle_vel_input.length_rear);
  nh.param("robot/kinematic_bicycle_vel_input/length_front", robot.kinematic_bicycle_vel_input.length_front, robot.kinematic_bicycle_vel_input.length_front);

  nh.param("solver/type", solver.type, solver.type);
  nh.param("solver/ipopt/iterations", solver.ipopt.iterations, solver.ipopt.iterations);
  nh.param("solver/ipopt/max_cpu_time", solver.ipopt.max_cpu_time, solver.ipopt.max_cpu_time);
  nh.param("solver/ipopt/ipopt_numeric_options", solver.ipopt.ipopt_numeric_options, solver.ipopt.ipopt_numeric_options);
  nh.param("solver/ipopt/ipopt_string_options", solver.ipopt.ipopt_string_options, solver.ipopt.ipopt_string_options);
  nh.param("solver/ipopt/ipopt_integer_options", solver.ipopt.ipopt_integer_options, solver.ipopt.ipopt_integer_options);

  nh.param("solver/lsq_lm/iterations", solver.lsq_lm.iterations, solver.lsq_lm.iterations);
  nh.param("solver/lsq_lm/weight_init_eq", solver.lsq_lm.weight_init_eq, solver.lsq_lm.weight_init_eq);
  nh.param("solver/lsq_lm/weight_init_ineq", solver.lsq_lm.weight_init_ineq, solver.lsq_lm.weight_init_ineq);
  nh.param("solver/lsq_lm/weight_init_bounds", solver.lsq_lm.weight_init_bounds, solver.lsq_lm.weight_init_bounds);
  nh.param("solver/lsq_lm/weight_adapt_factor_eq", solver.lsq_lm.weight_adapt_factor_eq, solver.lsq_lm.weight_adapt_factor_eq);
  nh.param("solver/lsq_lm/weight_adapt_factor_ineq", solver.lsq_lm.weight_adapt_factor_ineq, solver.lsq_lm.weight_adapt_factor_ineq);
  nh.param("solver/lsq_lm/weight_adapt_factor_bounds", solver.lsq_lm.weight_adapt_factor_bounds, solver.lsq_lm.weight_adapt_factor_bounds);
  nh.param("solver/lsq_lm/weight_adapt_max_eq", solver.lsq_lm.weight_adapt_max_eq, solver.lsq_lm.weight_adapt_max_eq);
  nh.param("solver/lsq_lm/weight_init_eq", solver.lsq_lm.weight_adapt_max_ineq, solver.lsq_lm.weight_adapt_max_ineq);
  nh.param("solver/lsq_lm/weight_adapt_max_bounds", solver.lsq_lm.weight_adapt_max_bounds, solver.lsq_lm.weight_adapt_max_bounds);

  nh.param("planning/objective/type", planning.objective.type, planning.objective.type);
  nh.param("planning/objective/quadratic_form/state_weights", planning.objective.quadratic_form.state_weights, planning.objective.quadratic_form.state_weights);
  nh.param("planning/objective/quadratic_form/control_weights", planning.objective.quadratic_form.control_weights, planning.objective.quadratic_form.control_weights);
  nh.param("planning/objective/quadratic_form/integral_form", planning.objective.quadratic_form.integral_form, planning.objective.quadratic_form.integral_form);
  nh.param("planning/objective/quadratic_form/hybrid_cost_minimum_time", planning.objective.quadratic_form.hybrid_cost_minimum_time, planning.objective.quadratic_form.hybrid_cost_minimum_time);
  nh.param("planning/objective/minimum_time_via_points/via_points_ordered", planning.objective.minimum_time_via_points.via_points_ordered, planning.objective.minimum_time_via_points.via_points_ordered);
  nh.param("planning/objective/minimum_time_via_points/position_weight", planning.objective.minimum_time_via_points.position_weight, planning.objective.minimum_time_via_points.position_weight);
  nh.param("planning/objective/minimum_time_via_points/orientation_weight", planning.objective.minimum_time_via_points.orientation_weight, planning.objective.minimum_time_via_points.orientation_weight);

  nh.param("planning/terminal_cost/type", planning.terminal_cost.type, planning.terminal_cost.type);
  nh.param("planning/terminal_cost/quadratic/final_state_weights", planning.terminal_cost.quadratic.final_state_weights, planning.terminal_cost.quadratic.final_state_weights);
  nh.param("planning/terminal_constraint/type", planning.terminal_constraint.type, planning.terminal_constraint.type);
  nh.param("planning/terminal_constraint/l2_ball/weight_matrix", planning.terminal_constraint.l2_ball.weight_matrix, planning.terminal_constraint.l2_ball.weight_matrix);
  nh.param("planning/terminal_constraint/l2_ball/radius", planning.terminal_constraint.l2_ball.radius, planning.terminal_constraint.l2_ball.radius);

}

void MpcConfig::reconfigure(MpcLocalPlannerReconfigureConfig& cfg)
{  
  boost::mutex::scoped_lock l(config_mutex_);

  collision_avoidance.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
  collision_avoidance.cutoff_dist = cfg.cutoff_dist;
  collision_avoidance.enable_dynamic_obstacles = cfg.enable_dynamic_obstacles;
  collision_avoidance.force_inclusion_dist = cfg.force_inclusion_dist;
  collision_avoidance.include_costmap_obstacles = cfg.include_costmap_obstacles;
  collision_avoidance.min_obstacle_dist = cfg.min_obstacle_dist;


  controller.allow_init_with_backward_motion = cfg.allow_init_with_backward_motion;
  controller.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
  controller.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  controller.force_reinit_num_steps = cfg.force_reinit_num_steps;
  controller.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  controller.global_plan_prune_distance = cfg.global_plan_prune_distance;
  controller.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
  controller.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
  controller.outer_ocp_iterations = cfg.outer_ocp_iterations;
  controller.prefer_x_feedback = cfg.prefer_x_feedback;
  controller.publish_ocp_results = cfg.publish_ocp_results;
  controller.xy_goal_tolerance = cfg.xy_goal_tolerance;
  controller.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  controller.print_cpu_time = cfg.print_cpu_time;

  grid.collocation_method = cfg.collocation_method;
  grid.cost_integration_method = cfg.cost_integration_method;
  grid.dt_ref = cfg.dt_ref;
  grid.grid_size_ref = cfg.grid_size_ref;
  grid.type = cfg.grid_type;
  grid.warm_start = cfg.warm_start;
  grid.xf_fixed = {true, true, true};
  std::cout << cfg.xf_fixed;    

  grid.variable_grid.enable = cfg.variable_grid_enable;
  grid.variable_grid.min_dt = cfg.variable_grid_min_dt;
  grid.variable_grid.max_dt = cfg.variable_grid_max_dt;
    
  grid.variable_grid.grid_adaption.enable = cfg.grid_adaption_enable;
  grid.variable_grid.grid_adaption.dt_hyst_ratio = cfg.dt_hyst_ratio;
  grid.variable_grid.grid_adaption.min_grid_size = cfg.min_grid_size;
  grid.variable_grid.grid_adaption.max_grid_size = cfg.max_grid_size;

      
  planning.objective.type = cfg.planning_objective_type;
  planning.objective.minimum_time_via_points.orientation_weight = cfg.orientation_weight;
  planning.objective.minimum_time_via_points.position_weight = cfg.position_weight;
  planning.objective.minimum_time_via_points.via_points_ordered = cfg.via_points_ordered;
  planning.objective.quadratic_form.control_weights = {1.0, 1.0};
  planning.objective.quadratic_form.state_weights = {2.0, 2.0, 2.0};
  std::cout << cfg.control_weights << " " << cfg.state_weights;
  planning.objective.quadratic_form.integral_form = cfg.integral_form;
  //planning.objective.quadratic_form.hybrid_cost_minimum_time = cfg.hybrid_cost_minimum_time;
    
  planning.terminal_constraint.type = cfg.terminal_constraint_type;
  planning.terminal_constraint.l2_ball.radius = cfg.radius;
  planning.terminal_constraint.l2_ball.weight_matrix = {1.0, 1.0, 1.0};
  std::cout << cfg.weight_matrix;
  
  planning.terminal_cost.type = cfg.terminal_cost_type;
  planning.terminal_cost.quadratic.final_state_weights = {2.0, 2.0, 2.0};
  std::cout << cfg.final_state_weights;  
 
    
  robot.type = cfg.robot_type;

  robot.unicycle.acc_lim_x = cfg.acc_lim_x;
  robot.unicycle.dec_lim_x = cfg.dec_lim_x;
  robot.unicycle.max_vel_x = cfg.max_vel_x;
  robot.unicycle.max_vel_x_backwards = cfg.max_vel_x_backwards;
  robot.unicycle.acc_lim_theta = cfg.acc_lim_theta;
  robot.unicycle.max_vel_theta = cfg.max_vel_theta;


  solver.type = cfg.solver_type;

  solver.ipopt.iterations = cfg.ipopt_iterations;
  solver.ipopt.max_cpu_time = cfg.max_cpu_time;
  solver.ipopt.ipopt_numeric_options["tol"] = 0.0001;
  solver.ipopt.ipopt_string_options["hessian_approximation"] = "exact";
  solver.ipopt.ipopt_string_options["linear_solver"] = "mumps";
  std::cout << cfg.ipopt_string_options;

  solver.lsq_lm.iterations = cfg.lsq_lm_iterations;
  solver.lsq_lm.weight_adapt_factor_bounds = cfg.weight_adapt_factor_bounds;
  solver.lsq_lm.weight_adapt_factor_eq = cfg.weight_adapt_factor_eq;
  solver.lsq_lm.weight_adapt_factor_ineq = cfg.weight_adapt_factor_ineq;
  solver.lsq_lm.weight_adapt_max_bounds = cfg.weight_adapt_max_bounds;
  solver.lsq_lm.weight_adapt_max_eq = cfg.weight_adapt_max_eq;
  solver.lsq_lm.weight_adapt_max_ineq = cfg.weight_adapt_max_ineq;
  solver.lsq_lm.weight_init_bounds = cfg.weight_init_bounds;
  solver.lsq_lm.weight_init_eq = cfg.weight_init_eq;
  solver.lsq_lm.weight_init_ineq = cfg.weight_init_ineq;
}

}
