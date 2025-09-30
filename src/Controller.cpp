#include "Controller.hpp"

// #include "nav2_core/controller_exceptions.hpp"
// #include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
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

void MPPIcROS::configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;

    auto node = node_.lock();

    tf_ = tf;
    costmap_ros_ptr_ = costmap_ros;
    costmap_mappi_ = mappi::make_shared<mappi::ROS2CostmapAdapter>(costmap_ros_ptr_);
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    RCLCPP_INFO(logger_, "mappi:: Initializing with plugin name %s", plugin_name_.c_str());

    parameters_handler_ = std::make_unique<ParametersHandler>(parent);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
    local_pub_ = node->create_publisher<nav_msgs::msg::Path>("interpolated_plan", 1);

    // Srv client
    // costmap_client_ = nh_upper.serviceClient<std_srvs::Empty>("clear_costmaps");

    // Set up MPPI config
    this->setUpParameters(config_);
    config_.print_out(); // print out config (debug)
    
    // Initialize MPPI controller
    mappi_.configure(config_, costmap_mappi_);

    // Set up Visualizer instance
    visualizer_ptr_ = std::make_unique<Visualizer>(parent, name, &mappi_, config_.visual, parameters_handler_.get());

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
    local_pub_.reset();
    parameters_handler_.reset();
    mappi_.shutdown();
}

void MPPIcROS::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type mappi::MPPIcROS\"  %s",
        plugin_name_.c_str(),plugin_name_.c_str());
    // global_pub_->on_activate();
    parameters_handler_->start();
}

void MPPIcROS::deactivate()
{
    RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type mappi::MPPIcROS\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
    // global_pub_->on_deactivate();
    mappi_.reset();
}

void MPPIcROS::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
    (void) speed_limit;
    (void) percentage;
}

geometry_msgs::msg::TwistStamped MPPIcROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                                                    const geometry_msgs::msg::Twist & velocity,
                                                                    nav2_core::GoalChecker * /*goal_checker*/)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x  = 0.0;
    cmd_vel.twist.linear.y  = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    if(not is_initialized()) return cmd_vel;

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
    ros_utils::mppic2ros(mappi_.getCurrentPlan(), path_msg, local_frame_, clock_);
    local_pub_->publish(path_msg);

    RCLCPP_INFO(logger_, "mappi:: published local plan");

    return cmd_vel;
}

void MPPIcROS::setPlan(const nav_msgs::msg::Path & path)
{
    if(not is_initialized()){
        RCLCPP_ERROR(logger_, "mappi: This controller has not been initialized, please call configure() before using %s", plugin_name_.c_str());
        return;
    }

    RCLCPP_INFO(logger_, "mappi:: Setting new global plan");

    global_plan_ = path;

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

// bool MPPIcROS::isGoalReached()
// {
//     ROS_INFO("mappi:: is goal reached?");

//     if (not is_initialized()) {
//         ROS_ERROR("mappi: This planner/controller has not been initialized, please call initialize() before using this planner");
//         return false;
//     }

//     if(aux::robotNearGoal(this->goal_tolerance_, current_odom_, global_plan_))
//     {
//         ROS_WARN("mappi: Goal reached!");
//         return true;
//     }
//     else
//         return false;
// }

bool MPPIcROS::is_initialized()
{
    return initialized_;
}

nav_msgs::msg::Path
MPPIcROS::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
    // Original mplementation taken fron nav2_dwb_controller

    if (global_plan_.poses.empty()) {
        throw std::runtime_error("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
        tf_, global_plan_.header.frame_id, pose,
        robot_pose, transform_tolerance_))
    {
        throw std::runtime_error("Unable to transform robot pose into global plan's frame");
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
        throw std::runtime_error("Resulting plan has 0 poses in it.");
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
    // Visualization
    auto getParamVis = parameters_handler_->getParamGetter(plugin_name_ + ".Visualization");
    getParamVis(config.visual.active, "active", false, ParameterType::Static);
    getParamVis(config.visual.batch_stride, "batch_stride", 100, ParameterType::Static);
    getParamVis(config.visual.time_stride, "time_stride", 3, ParameterType::Static);
    getParamVis(config.visual.default_z, "default_z", 0.2, ParameterType::Static);
    getParamVis(config.visual.scale, "scale", 0.05, ParameterType::Static);

    // General
    auto getParamGen = parameters_handler_->getParamGetter(plugin_name_ + ".GeneralSettings");
    getParamGen(config.settings.global_frame, "global_frame", std::string(""), ParameterType::Static);
    getParamGen(config.settings.local_frame, "local_frame", std::string(""), ParameterType::Static);
    getParamGen(config.settings.num_iters, "num_iterations", 1);
    getParamGen(config.settings.num_retry, "num_retry", 4);
    getParamGen(config.settings.offset, "control_offset", 1);
    getParamGen(config.settings.goal_tolerance, "goal_tolerance", 0.5);
    getParamGen(config.settings.dist_shift, "plan_shift", 1.0);
    getParamGen(config.settings.use_splines, "use_splines", false);

    double transform_tolerance;
    getParamGen(transform_tolerance, "transform_tolerance", false, ParameterType::Static);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    getParamGen(config.noise.batch_size, "batch_size", 1000);
    getParamGen(config.noise.time_steps, "time_steps", 100);

    getParamGen(config.model_dt, "model_dt", 0.05);
    getParamGen(config.temperature, "temperature", 0.3);
    getParamGen(config.gamma, "gamma", 0.05);

    // Constraints
    auto getParamCnstr = parameters_handler_->getParamGetter(plugin_name_ + ".Constraints");
    getParamCnstr(config.bounds.max_vx, "max_vx", 0.75);
    getParamCnstr(config.bounds.min_vx, "min_vx", 0.75);
    getParamCnstr(config.bounds.max_vy, "max_vy", 0.5);
    getParamCnstr(config.bounds.min_vy, "min_vy", -0.5);
    getParamCnstr(config.bounds.max_wz, "max_wz", 0.5);
    getParamCnstr(config.bounds.min_wz, "min_wz", -0.5);

    // Motion Model
    auto getParamModel = parameters_handler_->getParamGetter(plugin_name_);
    getParamModel(config.settings.motion_model, "MotionModel", std::string("BicycleKin"));

    // Ackermann model
    auto getParamAck = parameters_handler_->getParamGetter(plugin_name_ + ".Ackermann");
    getParamAck(config.ackermann.min_r, "min_radius", 3.0);

    // BicycleKin model
    auto getParamBicycle = parameters_handler_->getParamGetter(plugin_name_ + ".BicycleKin");
    getParamBicycle(config.bicycleKin.length, "length", 0.5);
    getParamBicycle(config.bicycleKin.max_steer, "max_steer", 0.3);

    // BicycleKin model
    auto getParamNoise = parameters_handler_->getParamGetter(plugin_name_ + ".NoiseGeneration");
    getParamNoise(config.noise.std_vx, "std_vx", 0.5);
    getParamNoise(config.noise.std_vy, "std_vy", 0.3);
    getParamNoise(config.noise.std_wz, "std_wz", 0.3);

    // Critics
    auto getParamCrtc = parameters_handler_->getParamGetter(plugin_name_ + ".Critics");
    getParamCrtc(config.goal_crtc.common.active, "Goal.active", true);
    getParamCrtc(config.goal_crtc.common.power, "Goal.power", 1);
    getParamCrtc(config.goal_crtc.common.weight, "Goal.weight", 1.0);
    getParamCrtc(config.goal_crtc.common.threshold, "Goal.threshold", 1.5);

    getParamCrtc(config.goalangle_crtc.common.active, "GoalAngle.active", false);
    getParamCrtc(config.goalangle_crtc.common.power, "GoalAngle.power", 1);
    getParamCrtc(config.goalangle_crtc.common.weight, "GoalAngle.weight", 1.0);
    getParamCrtc(config.goalangle_crtc.common.threshold, "GoalAngle.threshold", 1.5);

    getParamCrtc(config.pathdist_crtc.common.active, "PathDist.active", false);
    getParamCrtc(config.pathdist_crtc.common.power, "PathDist.power", 1);
    getParamCrtc(config.pathdist_crtc.common.weight, "PathDist.weight", 1.0);
    getParamCrtc(config.pathdist_crtc.common.threshold, "PathDist.threshold", 1.5);
    getParamCrtc(config.pathdist_crtc.traj_stride, "PathDist.traj_stride", 2);

    getParamCrtc(config.twir_crtc.common.active, "Twirling.active", false);
    getParamCrtc(config.twir_crtc.common.power, "Twirling.power", 1);
    getParamCrtc(config.twir_crtc.common.weight, "Twirling.weight", 1.0);
    getParamCrtc(config.twir_crtc.common.threshold, "Twirling.threshold", 1.5);

    getParamCrtc(config.forward_crtc.common.active, "Forward.active", false);
    getParamCrtc(config.forward_crtc.common.power, "Forward.power", 1);
    getParamCrtc(config.forward_crtc.common.weight, "Forward.weight", 1.0);
    getParamCrtc(config.forward_crtc.common.threshold, "Forward.threshold", 1.5);

    getParamCrtc(config.pathfollow_crtc.common.active, "PathFollow.active", false);
    getParamCrtc(config.pathfollow_crtc.common.power, "PathFollow.power", 1);
    getParamCrtc(config.pathfollow_crtc.common.weight, "PathFollow.weight", 1.0);
    getParamCrtc(config.pathfollow_crtc.common.threshold, "PathFollow.threshold", 1.5);
    getParamCrtc(config.pathfollow_crtc.offset_from_furthest, "PathFollow.offset_from_furthest", 3);

    getParamCrtc(config.pathangle_crtc.common.active, "PathAngle.active", false);
    getParamCrtc(config.pathangle_crtc.common.power, "PathAngle.power", 1);
    getParamCrtc(config.pathangle_crtc.common.weight, "PathAngle.weight", 1.0);
    getParamCrtc(config.pathangle_crtc.common.threshold, "PathAngle.threshold", 1.5);
    getParamCrtc(config.pathangle_crtc.offset_from_furthest, "PathAngle.offset_from_furthest", 3);
    getParamCrtc(config.pathangle_crtc.angle_threshold, "PathAngle.angle_threshold", 1.57);

    getParamCrtc(config.obs_crtc.common.active, "Obstacles.active", false);
    getParamCrtc(config.obs_crtc.common.power, "Obstacles.power", 1);
    getParamCrtc(config.obs_crtc.common.weight, "Obstacles.weight", 1.0);
    getParamCrtc(config.obs_crtc.common.threshold, "Obstacles.threshold", 1.5);
    getParamCrtc(config.obs_crtc.repulsive_weight, "Obstacles.repulsive_weight", 0.0);
    getParamCrtc(config.obs_crtc.collision_cost, "Obstacles.collision_cost", 100000.0);
    getParamCrtc(config.obs_crtc.collision_margin_dist, "Obstacles.collision_margin_dist", 0.2);
    config.obs_crtc.inflation_radius = 0.0;
    config.obs_crtc.inflation_scale_factor = 0.0;
    // To-Do:
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
