#include "Controller.hpp"

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace mappi 
{

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

MPPIcROS::~MPPIcROS()
{
    mappi_.shutdown();
}

void MPPIcROS::configure( const nav2::LifecycleNode::WeakPtr & parent,
                            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;

    auto node = node_.lock();

    tf_ = tf;
    costmap_ros_ptr_ = costmap_ros;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    parameters_handler_ = std::make_unique<ParametersHandler>(parent);

    RCLCPP_INFO(logger_, "mappi:: Initializing...");

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
    local_pub_ = node->create_publisher<nav_msgs::msg::Path>("interpolated_plan", 1);

    // Srv client
    // costmap_client_ = nh_upper.serviceClient<std_srvs::Empty>("clear_costmaps");

    /* To-Do:
            - set up parameters via parameter handler
    */

    // Set up MPPI config
    config::MPPIc config;
    this->setUpParameters(config);
    config.print_out(); // print out config (debug)
    
    // Initialize MPPI controller
    mappi_.configure(config, costmap_ros_ptr_);

    // Set up Visualizer instance
    visualizer_ptr_ = std::make_unique<Visualizer>(&mappi_, &nh);

    // Set up Odometry Helper instance
    // odom_helper_ptr_ = std::make_unique<OdomHelper>(name);

    initialized_ = true;

    RCLCPP_INFO(logger_, "mappi:: Finished initializing...");
}

void MPPIcROS::cleanup()
{
    RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type mappi::MPPIcROS",
    plugin_name_.c_str());
    global_pub_.reset();
    // parameters_handler_.reset();
    mappi_.shutdown();
}

void MPPIcROS::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type mappi::MPPIcROS\"  %s",
        plugin_name_.c_str(),plugin_name_.c_str());
    global_pub_->on_activate();
    // parameters_handler_->start();
}

void MPPIcROS::deactivate()
{
    RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type mappi::MPPIcROS\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
    global_pub_->on_deactivate();
    mappi_.reset();
}

void MPPIcROS::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
    (void) speed_limit;
    (void) percentage;
}

geometry_msgs::msg::TwistStamped MPPIcROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                                                    const geometry_msgs::msg::Twist & velocity,
                                                                    nav2_core::GoalChecker * goal_checker)
{
    if(not is_initialized()) return false;

    RCLCPP_INFO(logger_, "mappi:: computing velocity commands...");

    std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

    // Transform global plan w.r.t. current pose
    auto transformed_plan = transformGlobalPlan(pose);
    
    // Get current pose (transform to mappi type)
    ros_utils::ros2mppic(pose, current_odom_); 
    ros_utils::ros2mppic(velocity, current_odom_); 

    /*To-Do:
            - fill current_odom_ with current steering angle
    */
    // odom_helper_ptr_->fillSteering(current_odom_); // get current steering

    auto start_time = std::chrono::system_clock::now();

    RCLCPP_INFO(logger_, "mappi:: calling MPPI controller...");

    objects::Path plan;
    ros_utils::ros2mppic(global_plan_, plan); // transform global plan to mappi type
    objects::Control cmd = mappi_.getControl(current_odom_, plan);
    // NOTE: if no control is found, cmd variable will be returned filled with 0s
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x  = cmd.vx;
    cmd_vel.twist.linear.y  = cmd.vy;
    cmd_vel.twist.angular.z = cmd.wz;

    auto end_time = std::chrono::system_clock::now();
    static std::chrono::duration<double> elapsed_time;
    elapsed_time = end_time - start_time;

    RCLCPP_INFO(logger_, "mappi:: Elapsed time: %f ms", elapsed_time.count()*1000.0);

    // Publish generated trajectories
    visualizer_ptr_->publish();

    RCLCPP_INFO(logger_, "mappi:: published trajectories");

    // Publish interpolated plan
    static nav_msgs::msg::Path path_msg;
    ros_utils::mppic2ros(mappi_.getCurrentPlan(), path_msg, local_frame_);
    local_pub_->publish(path_msg);

    RCLCPP_INFO(logger_, "mappi:: published local plan");

    return cmd_vel;
}

void MPPIcROS::setPlan(const nav_msgs::msg::Path & path)
{
    if(not is_initialized()){
        RCLCPP_ERROR(logger_, "mappi: This controller has not been initialized, please call configure() before using %s", plugin_name_.c_str());
        return false;
    }

    RCLCPP_INFO(logger_, "mappi:: Setting new global plan");

    global_plan_ = path;

    // ROS_INFO("mappi:: shifting global plan");

    // (optional) Shift local plan to avoid planning inside the robot's footprint 
    // aux::shiftPlan(global_plan_, dist_shift_);
    
    // (optional) Clear costmaps
    // static std_srvs::Empty srv;
    // if (costmap_client_.call(srv))
    // {
    //     ROS_INFO("NANO_MPPIC:: Resetting costmaps");
    //     ros::Duration(0.05).sleep(); // wait until costmap is reset (avoid checking costmap pointer when is empty)
    // }
    // else
    // {
    //     ROS_ERROR("NANO_MPPIC:: Failed to call service ~clear_costmaps");
    // }

    // Publish received global plan
    global_pub_->publish(path);

}

bool MPPIcROS::isGoalReached()
{
    ROS_INFO("mappi:: is goal reached?");

    if (not is_initialized()) {
        ROS_ERROR("mappi: This planner/controller has not been initialized, please call initialize() before using this planner");
        return false;
    }

    if(aux::robotNearGoal(this->goal_tolerance_, current_odom_, global_plan_))
    {
        ROS_WARN("mappi: Goal reached!");
        return true;
    }
    else
        return false;
}

bool MPPIcROS::is_initialized()
{
    return initialized_;
}

nav_msgs::msg::Path
MPPIcROS::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
    // Original mplementation taken fron nav2_dwb_controller

    if (global_plan_.poses.empty()) {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
        tf_, global_plan_.header.frame_id, pose,
        robot_pose, transform_tolerance_))
    {
        throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ptr_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin = min_by( global_plan_.poses.begin(), global_plan_.poses.end(),
                                        [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
                                            return euclidean_distance(robot_pose, ps);
                                        });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if( transformation_begin, end(global_plan_.poses),
                                            [&](const auto & global_plan_pose) {
                                                return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
                                            });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
        // We took a copy of the pose, let's lookup the transform at the current time
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        transformPose(
            tf_, costmap_ros_ptr_->getBaseFrameID(),
            stamped_pose, transformed_pose, transform_tolerance_);
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_ptr_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool MPPIcROS::transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                            const std::string frame,
                            const geometry_msgs::msg::PoseStamped & in_pose,
                            geometry_msgs::msg::PoseStamped & out_pose,
                            const rclcpp::Duration & transform_tolerance
                            ) const
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    if (in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        tf->transform(in_pose, out_pose, frame);
        return true;
    } catch (tf2::ExtrapolationException & ex) {
        auto transform = tf->lookupTransform(
            frame,
            in_pose.header.frame_id,
            tf2::TimePointZero
        );
        if ( (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Transform data too old when converting from %s to %s",
                in_pose.header.frame_id.c_str(),
                frame.c_str()
            );
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Data time: %ds %uns, Transform time: %ds %uns",
                in_pose.header.stamp.sec,
                in_pose.header.stamp.nanosec,
                transform.header.stamp.sec,
                transform.header.stamp.nanosec
            );
            return false;
        } else {
            tf2::doTransform(in_pose, out_pose, transform);
            return true;
        }
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Exception in transformPose: %s",
            ex.what()
        );
        return false;
    }
    return false;
}

void MPPIcROS::setUpParameters(config::MPPIc& config)
{
    // Default
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist",
        rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

    // MaPPI
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.goal_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.plan_shift",     rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.global_frame",   rclcpp::ParameterValue("map"));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.local_frame",    rclcpp::ParameterValue("odom"));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.batch_size",     rclcpp::ParameterValue(1000));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.num_iterations", rclcpp::ParameterValue(4));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.time_steps",     rclcpp::ParameterValue(40));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.num_retry",      rclcpp::ParameterValue(2));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.control_offset", rclcpp::ParameterValue(1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.use_splines",    rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.model_dt",       rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.temperature",    rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".GeneralSettings.gamma",          rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Ackermann.min_radius",           rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".BicycleKin.length",              rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".BicycleKin.max_steer",           rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".MotionModel",                    rclcpp::ParameterValue("BicycleKin"));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".NoiseGeneration.std_vx",         rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".NoiseGeneration.std_vy",         rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".NoiseGeneration.std_wz",         rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Constraints.max_vx",             rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Constraints.min_vx",             rclcpp::ParameterValue(-1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Constraints.max_vy",             rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Constraints.min_vy",             rclcpp::ParameterValue(-1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Constraints.max_wz",             rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Constraints.min_wz",             rclcpp::ParameterValue(-0.3));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Critics.Goal.power",         rclcpp::ParameterValue(1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Critics.Goal.active",            rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Critics.Goal.weight",            rclcpp::ParameterValue(5.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".Critics.Goal.threshold",         rclcpp::ParameterValue(0.5));

    

    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    // Set up ROS wrapper params
    // nh.param<float>("GeneralSettings/goal_tolerance", goal_tolerance_, 0.1f);
    // nh.param<float>("GeneralSettings/plan_shift", dist_shift_, 1.0f);
    // nh.param<bool>("GeneralSettings/use_local_planner", use_local_planner_, true);

    // nh.param<std::string>("GeneralSettings/global_frame", global_frame_, ""); 
    // nh.param<std::string>("GeneralSettings/local_frame", local_frame_, ""); 
    // if(local_frame_ == "")
    //     nh_upper.param<std::string>("local_costmap/global_frame", local_frame_, "odom");
    // if(global_frame_ == "")
    //     nh_upper.param<std::string>("global_costmap/global_frame", global_frame_, "map");

    // int batch_size, num_iters, time_steps, num_retry, offset;

    // nh.param<bool>("GeneralSettings/use_splines", config.settings.use_splines, false);

    // config.settings.num_iters  = static_cast<unsigned int>(num_iters);
    // config.settings.num_retry   = static_cast<unsigned int>(num_retry);
    // config.settings.offset      = static_cast<unsigned int>(offset);
    
    // config.noise.batch_size = static_cast<unsigned int>(batch_size);
    // config.noise.time_steps = static_cast<unsigned int>(time_steps);

    // nh.param<float>("GeneralSettings/model_dt",     config.model_dt,    0.01f);
    // nh.param<float>("GeneralSettings/temperature",  config.temperature, 1.0f);
    // nh.param<float>("GeneralSettings/gamma",        config.gamma,       1.0f);

    // nh.param<float>("Ackermann/min_radius",  config.ackermann.min_r,  3.0f);
    // nh.param<float>("BicycleKin/length",     config.bicycleKin.length,  1.0f);
    // nh.param<float>("BicycleKin/max_steer",  config.bicycleKin.max_steer,  0.3f);

    // nh.param<std::string>("MotionModel", config.settings.motion_model, "BicycleKin");

    // nh.param<float>("NoiseGeneration/std_vx", config.noise.std_vx, 0.01f);
    // nh.param<float>("NoiseGeneration/std_vy", config.noise.std_vy, 0.01f);
    // nh.param<float>("NoiseGeneration/std_wz", config.noise.std_wz, 0.01f);

    // nh.param<float>("Constraints/max_vx", config.bounds.max_vx, 2.0f);
    // nh.param<float>("Constraints/min_vx", config.bounds.min_vx, -1.0f);
    // nh.param<float>("Constraints/max_vy", config.bounds.max_vy, 1.0f);
    // nh.param<float>("Constraints/min_vy", config.bounds.min_vy, -1.0f);
    // nh.param<float>("Constraints/max_wz", config.bounds.max_wz, 0.3f);
    // nh.param<float>("Constraints/min_wz", config.bounds.min_wz, -0.3f);
    
    //     // Goal critic config
    // int power;
    // nh.param<int>("Critics/Goal/power", power, 1);
    // config.goal_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/Goal/active",       config.goal_crtc.common.active,     true);
    // nh.param<float>("Critics/Goal/weight",      config.goal_crtc.common.weight,     5.0f);
    // nh.param<float>("Critics/Goal/threshold",   config.goal_crtc.common.threshold,  0.5f);

    //     // PathDist critic config
    // nh.param<int>("Critics/PathDist/power", power, 1);
    // config.pathdist_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/PathDist/active",       config.pathdist_crtc.common.active,     true);
    // nh.param<float>("Critics/PathDist/weight",      config.pathdist_crtc.common.weight,     5.0f);
    // nh.param<float>("Critics/PathDist/threshold",   config.pathdist_crtc.common.threshold,  1.0f);
    // nh.param<int>("Critics/PathDist/stride",        config.pathdist_crtc.traj_stride,       2);

    //     // GoalAngle critic config
    // nh.param<int>("Critics/GoalAngle/power", power, 1);
    // config.goalangle_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/GoalAngle/active",      config.goalangle_crtc.common.active,    true);
    // nh.param<float>("Critics/GoalAngle/weight",     config.goalangle_crtc.common.weight,    5.0f);
    // nh.param<float>("Critics/GoalAngle/threshold",  config.goalangle_crtc.common.threshold, 1.0f);

    //     // Twirling critic config
    // nh.param<int>("Critics/Twirling/power", power, 1);
    // config.twir_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/Twirling/active",   config.twir_crtc.common.active, true);
    // nh.param<float>("Critics/Twirling/weight",  config.twir_crtc.common.weight, 10.0f);

    //     // Forward critic config
    // nh.param<int>("Critics/Forward/power", power, 1);
    // config.forward_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/Forward/active",   config.forward_crtc.common.active, true);
    // nh.param<float>("Critics/Forward/weight",  config.forward_crtc.common.weight, 10.0f);

    //     // PathFollow critic config
    // nh.param<int>("Critics/PathFollow/power", power, 1);
    // config.pathfollow_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/PathFollow/active",     config.pathfollow_crtc.common.active,   true);
    // nh.param<float>("Critics/PathFollow/weight",    config.pathfollow_crtc.common.weight,    5.0f);
    // nh.param<float>("Critics/PathFollow/threshold", config.pathfollow_crtc.common.threshold, 0.5f);
    // nh.param<int>("Critics/PathFollow/offset_from_furthest", offset, 3);
    // config.pathfollow_crtc.offset_from_furthest = static_cast<size_t>(offset);

    //     // PathFollow critic config
    // nh.param<int>("Critics/PathAngle/power", power, 1);
    // config.pathangle_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/PathAngle/active",     config.pathangle_crtc.common.active,   true);
    // nh.param<float>("Critics/PathAngle/weight",    config.pathangle_crtc.common.weight,    15.0f);
    // nh.param<float>("Critics/PathAngle/threshold", config.pathangle_crtc.common.threshold, 0.5f);
    // nh.param<float>("Critics/PathAngle/angle_threshold", config.pathangle_crtc.angle_threshold, 90.0f);
    // config.pathangle_crtc.angle_threshold *= M_PI/180.0; // from deg to rad
    // nh.param<int>("Critics/PathAngle/offset_from_furthest", offset, 3);
    // config.pathangle_crtc.offset_from_furthest = static_cast<size_t>(offset);

    //     // Obstacles critic config
    // nh.param<int>("Critics/Obstacles/power", power, 1);
    // config.obs_crtc.common.power = static_cast<unsigned int>(power);
    // nh.param<bool>("Critics/Obstacles/active",                  config.obs_crtc.common.active,          true);
    // nh.param<float>("Critics/Obstacles/weight",                 config.obs_crtc.common.weight,          5.0f);
    // nh.param<float>("Critics/Obstacles/threshold",              config.obs_crtc.common.threshold,       1.0f);
    // nh.param<float>("Critics/Obstacles/repulsive_weight",       config.obs_crtc.repulsive_weight,       5.0f);
    // nh.param<float>("Critics/Obstacles/collision_cost",         config.obs_crtc.collision_cost,         100000.0f);
    // nh.param<float>("Critics/Obstacles/collision_margin_dist",  config.obs_crtc.collision_margin_dist,  0.1f);

    // nh_upper.param<float>("local_costmap/inflation_layer/inflation_radius",    
    //                         config.obs_crtc.inflation_radius, 
    //                         0.0f);
    // nh_upper.param<float>("local_costmap/inflation_layer/cost_scaling_factor", 
    //                         config.obs_crtc.inflation_scale_factor, 
    //                         0.0f);
}

} // namespace mappi

//register this planner as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mappi::MPPIcROS, nav2_core::Controller)
