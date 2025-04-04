#include "Controller.hpp"

#include <pluginlib/class_list_macros.h>
// #include <base_local_planner/goal_functions.h>

namespace mappi {

MPPIcROS::MPPIcROS() : tf_(NULL), initialized_(false)
{

}

MPPIcROS::MPPIcROS(std::string name, 
                    tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros)
    : tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

MPPIcROS::~MPPIcROS()
{
    mappi_.shutdown();
}

void MPPIcROS::initialize(std::string name, 
                            tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros)
{

    ROS_INFO("mappi:: Initializing...");

    // Set up private ROS handlers (tf, costmap, node)
    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle nh_upper("~/");

    tf_ = tf;
    costmap_ros_ptr_ = mappi::shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros);

        // Publishers
    global_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
    local_pub_  = nh.advertise<nav_msgs::Path>("strided_plan", 1);

    // Set up ROS wrapper params
    nh.param<float>("GeneralSettings/goal_tolerance", goal_tolerance_, 0.1f);
    nh.param<float>("GeneralSettings/plan_shift", dist_shift_, 1.0f);
    nh.param<bool>("GeneralSettings/use_local_planner", use_local_planner_, true);

    nh.param<std::string>("GeneralSettings/global_frame", global_frame_, ""); 
    nh.param<std::string>("GeneralSettings/local_frame", local_frame_, ""); 
    if(local_frame_ == "")
        nh_upper.param<std::string>("local_costmap/global_frame", local_frame_, "odom");
    if(global_frame_ == "")
        nh_upper.param<std::string>("global_costmap/global_frame", global_frame_, "map");

    // Set up MPPI config
    config::MPPIc config;

    int batch_size, num_iters, time_steps, num_retry, offset;
    nh.param<int>("GeneralSettings/batch_size",        batch_size,  1000);
    nh.param<int>("GeneralSettings/num_iterations",    num_iters,   4);
    nh.param<int>("GeneralSettings/time_steps",        time_steps,  40);
    nh.param<int>("GeneralSettings/num_retry",         num_retry,   2);
    nh.param<int>("GeneralSettings/control_offset",    offset,      1);

    nh.param<bool>("GeneralSettings/use_splines", config.settings.use_splines, false);

    config.settings.num_iters  = static_cast<unsigned int>(num_iters);
    config.settings.num_retry   = static_cast<unsigned int>(num_retry);
    config.settings.offset      = static_cast<unsigned int>(offset);
    
    config.noise.batch_size = static_cast<unsigned int>(batch_size);
    config.noise.time_steps = static_cast<unsigned int>(time_steps);

    nh.param<float>("GeneralSettings/model_dt",     config.model_dt,    0.01f);
    nh.param<float>("GeneralSettings/temperature",  config.temperature, 1.0f);
    nh.param<float>("GeneralSettings/gamma",        config.gamma,       1.0f);

    nh.param<float>("Ackermann/min_radius",  config.ackermann.min_r,  3.0f);
    nh.param<float>("BicycleKin/length",     config.bicycleKin.length,  1.0f);
    nh.param<float>("BicycleKin/max_steer",  config.bicycleKin.max_steer,  0.3f);

    nh.param<std::string>("MotionModel", config.settings.motion_model, "BicycleKin");

    nh.param<float>("NoiseGeneration/std_vx", config.noise.std_vx, 0.01f);
    nh.param<float>("NoiseGeneration/std_vy", config.noise.std_vy, 0.01f);
    nh.param<float>("NoiseGeneration/std_wz", config.noise.std_wz, 0.01f);

    nh.param<float>("Constraints/max_vx", config.bounds.max_vx, 2.0f);
    nh.param<float>("Constraints/min_vx", config.bounds.min_vx, -1.0f);
    nh.param<float>("Constraints/max_vy", config.bounds.max_vy, 1.0f);
    nh.param<float>("Constraints/min_vy", config.bounds.min_vy, -1.0f);
    nh.param<float>("Constraints/max_wz", config.bounds.max_wz, 0.3f);
    nh.param<float>("Constraints/min_wz", config.bounds.min_wz, -0.3f);
    
        // Goal critic config
    int power;
    nh.param<int>("Critics/Goal/power", power, 1);
    config.goal_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/Goal/active",       config.goal_crtc.common.active,     true);
    nh.param<float>("Critics/Goal/weight",      config.goal_crtc.common.weight,     5.0f);
    nh.param<float>("Critics/Goal/threshold",   config.goal_crtc.common.threshold,  0.5f);

        // PathDist critic config
    nh.param<int>("Critics/PathDist/power", power, 1);
    config.pathdist_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/PathDist/active",       config.pathdist_crtc.common.active,     true);
    nh.param<float>("Critics/PathDist/weight",      config.pathdist_crtc.common.weight,     5.0f);
    nh.param<float>("Critics/PathDist/threshold",   config.pathdist_crtc.common.threshold,  1.0f);
    nh.param<int>("Critics/PathDist/stride",        config.pathdist_crtc.traj_stride,       2);

        // GoalAngle critic config
    nh.param<int>("Critics/GoalAngle/power", power, 1);
    config.goalangle_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/GoalAngle/active",      config.goalangle_crtc.common.active,    true);
    nh.param<float>("Critics/GoalAngle/weight",     config.goalangle_crtc.common.weight,    5.0f);
    nh.param<float>("Critics/GoalAngle/threshold",  config.goalangle_crtc.common.threshold, 1.0f);

        // Twirling critic config
    nh.param<int>("Critics/Twirling/power", power, 1);
    config.twir_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/Twirling/active",   config.twir_crtc.common.active, true);
    nh.param<float>("Critics/Twirling/weight",  config.twir_crtc.common.weight, 10.0f);

        // Forward critic config
    nh.param<int>("Critics/Forward/power", power, 1);
    config.forward_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/Forward/active",   config.forward_crtc.common.active, true);
    nh.param<float>("Critics/Forward/weight",  config.forward_crtc.common.weight, 10.0f);

        // PathFollow critic config
    nh.param<int>("Critics/PathFollow/power", power, 1);
    config.pathfollow_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/PathFollow/active",     config.pathfollow_crtc.common.active,   true);
    nh.param<float>("Critics/PathFollow/weight",    config.pathfollow_crtc.common.weight,    5.0f);
    nh.param<float>("Critics/PathFollow/threshold", config.pathfollow_crtc.common.threshold, 0.5f);
    nh.param<int>("Critics/PathFollow/offset_from_furthest", offset, 3);
    config.pathfollow_crtc.offset_from_furthest = static_cast<size_t>(offset);

        // PathFollow critic config
    nh.param<int>("Critics/PathAngle/power", power, 1);
    config.pathangle_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/PathAngle/active",     config.pathangle_crtc.common.active,   true);
    nh.param<float>("Critics/PathAngle/weight",    config.pathangle_crtc.common.weight,    15.0f);
    nh.param<float>("Critics/PathAngle/threshold", config.pathangle_crtc.common.threshold, 0.5f);
    nh.param<float>("Critics/PathAngle/angle_threshold", config.pathangle_crtc.angle_threshold, 90.0f);
    config.pathangle_crtc.angle_threshold *= M_PI/180.0; // from deg to rad
    nh.param<int>("Critics/PathAngle/offset_from_furthest", offset, 3);
    config.pathangle_crtc.offset_from_furthest = static_cast<size_t>(offset);

        // Obstacles critic config
    nh.param<int>("Critics/Obstacles/power", power, 1);
    config.obs_crtc.common.power = static_cast<unsigned int>(power);
    nh.param<bool>("Critics/Obstacles/active",                  config.obs_crtc.common.active,          true);
    nh.param<float>("Critics/Obstacles/weight",                 config.obs_crtc.common.weight,          5.0f);
    nh.param<float>("Critics/Obstacles/threshold",              config.obs_crtc.common.threshold,       1.0f);
    nh.param<float>("Critics/Obstacles/repulsive_weight",       config.obs_crtc.repulsive_weight,       5.0f);
    nh.param<float>("Critics/Obstacles/collision_cost",         config.obs_crtc.collision_cost,         100000.0f);
    nh.param<float>("Critics/Obstacles/collision_margin_dist",  config.obs_crtc.collision_margin_dist,  0.1f);

    nh_upper.param<float>("local_costmap/inflation_layer/inflation_radius",    
                            config.obs_crtc.inflation_radius, 
                            0.0f);
    nh_upper.param<float>("local_costmap/inflation_layer/cost_scaling_factor", 
                            config.obs_crtc.inflation_scale_factor, 
                            0.0f);

    config.print_out(); // print out config (debug)

    // Initialize MPPI controller
    mappi_.configure(config, costmap_ros_ptr_);

    // Set up Visualizer instance
    visualizer_ptr_ = std::make_unique<Visualizer>(&mappi_, &nh);

    // Set up Odometry Helper instance
    odom_helper_ptr_ = std::make_unique<OdomHelper>(name);

    // Set up NavFn Wrapper (if available)
    #ifdef HAS_NAVFN
        navfn_wrapper_.configure("navfn_wrapper", costmap_ros_ptr_);
    #endif

    // Set up dynamic reconfigure server
    dyn_srv_ = new dynamic_reconfigure::Server<mappi::MPPIPlannerROSConfig>(nh);
    dynamic_reconfigure::Server
        <mappi::MPPIPlannerROSConfig>
            ::CallbackType callback = boost::bind(&MPPIcROS::reconfigure_callback, this, _1, _2);
    dyn_srv_->setCallback(callback);

    initialized_ = true;

    ROS_INFO("mappi:: Finished initializing");
}

bool MPPIcROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(not is_initialized()) return false;

    ROS_INFO("mappi:: computing velocity commands...");

    static geometry_msgs::PoseStamped current_pose;
    if(not costmap_ros_ptr_->getRobotPose(current_pose)){
        ROS_ERROR("mappi:: Could not get robot pose!");
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }
    ros_utils::ros2mppic(current_pose, current_odom_); // get current pose (from costmap_2d)

    // odom_helper_ptr_->fillTwistAndSteering(current_odom_); // get current velocity & steering
    // odom_helper_ptr_->fillTwist(current_odom_); // get current velocity
    // odom_helper_ptr_->fillSteering(current_odom_); // get current steering

    auto start_time = std::chrono::system_clock::now();

    ROS_INFO("mappi:: calling MPPI controller...");

    objects::Control cmd = mappi_.getControl(current_odom_, local_plan_);
    // NOTE: if no control is found, cmd variable will be returned filled with 0s
    
    cmd_vel.linear.x  = cmd.vx;
    cmd_vel.linear.y  = cmd.vy;
    cmd_vel.angular.z  = cmd.wz;

    auto end_time = std::chrono::system_clock::now();
    static std::chrono::duration<double> elapsed_time;
    elapsed_time = end_time - start_time;

    ROS_INFO("mappi:: Elapsed time: %f ms", elapsed_time.count()*1000.0);

    // Publish generated trajectories
    visualizer_ptr_->publish();

    ROS_INFO("mappi:: published trajectories");

    // Publish interpolated plan
    static nav_msgs::Path path_msg;
    ros_utils::mppic2ros(mappi_.getCurrentPlan(), path_msg, local_frame_);
    local_pub_.publish(path_msg);

    ROS_INFO("mappi:: published local plan");

    return true;
}

bool MPPIcROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    if(not is_initialized()){
        ROS_ERROR("mappi: This planner/controller has not been initialized, please call initialize() before using this planner");
        return false;
    }

    ROS_INFO("mappi:: Setting new global plan");

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tf_->lookupTransform(local_frame_, global_frame_,
                               ros::Time(0));
        ros_utils::ros2mppic(global_plan, global_plan_, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        ros_utils::ros2mppic(global_plan, global_plan_);
        ROS_ERROR("mappi::\n %s",ex.what());
        ROS_ERROR("mappi::\t Assuming global and local frame are the same!");
    }

    ROS_INFO("mappi:: Setting local plan");
    auto start_time = std::chrono::system_clock::now();

    local_plan_ = global_plan_;
    #ifdef HAS_NAVFN
        if(not aux::robotNearGoal(goal_tolerance_+static_cast<float>(navfn_wrapper_.getTolerance()), 
                                    current_odom_, 
                                    global_plan_ ) 
            && use_local_planner_ )
        {
            if( not navfn_wrapper_.getPlan(current_odom_, global_plan_, local_plan_) ){
                local_plan_.reset(0); // set local_plan_ to null
            }
        }

    #endif

    auto end_time = std::chrono::system_clock::now();
    static std::chrono::duration<double> elapsed_time;
    elapsed_time = end_time - start_time;

    ROS_INFO("mappi:: Local plan elapsed time: %f ms", elapsed_time.count()*1000.0);

    ROS_INFO("mappi:: shifting local plan");

    // (optional) Shift local plan to avoid planning inside the robot's footprint 
    aux::shiftPlan(local_plan_, dist_shift_);

    // Publish received global plan
    static nav_msgs::Path path_msg;
    ros_utils::mppic2ros(global_plan_, path_msg, global_frame_);
    global_pub_.publish(path_msg);

    return true;
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

void MPPIcROS::reconfigure_callback(mappi::MPPIPlannerROSConfig &dyn_cfg, uint32_t level)
{

    ROS_WARN("mappi: Dynamic reconfigure called");

    if(not is_initialized())
        return;

    try {

    goal_tolerance_ = static_cast<float>(dyn_cfg.goal_tolerance);
    dist_shift_     = static_cast<float>(dyn_cfg.plan_shift);
    use_local_planner_ = dyn_cfg.use_local_planner;

    config::MPPIc config;
    config.settings.num_iters  = static_cast<unsigned int>(dyn_cfg.num_iterations);
    config.settings.num_retry   = static_cast<unsigned int>(dyn_cfg.num_retry);
    config.settings.offset      = static_cast<unsigned int>(dyn_cfg.control_offset);

    config.settings.use_splines = dyn_cfg.use_splines;
    
    config.noise.batch_size = static_cast<unsigned int>(dyn_cfg.batch_size);
    config.noise.time_steps = static_cast<unsigned int>(dyn_cfg.time_steps);

    config.model_dt = static_cast<float>(dyn_cfg.model_dt);
    config.temperature = static_cast<float>(dyn_cfg.temperature);
    config.gamma = static_cast<float>(dyn_cfg.gamma);

    config.ackermann.min_r = static_cast<float>(dyn_cfg.min_radius);
    config.bicycleKin.length = static_cast<float>(dyn_cfg.length);
    config.bicycleKin.max_steer = static_cast<float>(dyn_cfg.max_steer);

    config.settings.motion_model = dyn_cfg.MotionModel;

    config.noise.std_vx = static_cast<float>(dyn_cfg.std_vx);
    config.noise.std_vy = static_cast<float>(dyn_cfg.std_vy);
    config.noise.std_wz = static_cast<float>(dyn_cfg.std_wz);

    config.bounds.max_vx = static_cast<float>(dyn_cfg.max_vx);
    config.bounds.min_vx = static_cast<float>(dyn_cfg.min_vx);
    config.bounds.max_vy = static_cast<float>(dyn_cfg.max_vy);
    config.bounds.min_vy = static_cast<float>(dyn_cfg.min_vy);
    config.bounds.max_wz = static_cast<float>(dyn_cfg.max_wz);
    config.bounds.min_wz = static_cast<float>(dyn_cfg.min_wz);

    // Goal critic config
    config.goal_crtc.common.power     = static_cast<unsigned int>(dyn_cfg.goal_power);
    config.goal_crtc.common.active    = static_cast<unsigned int>(dyn_cfg.goal_active);
    config.goal_crtc.common.weight    = static_cast<float>(dyn_cfg.goal_weight);
    config.goal_crtc.common.threshold = static_cast<float>(dyn_cfg.goal_threshold);

     // GoalAngle critic config
    config.goalangle_crtc.common.power = static_cast<unsigned int>(dyn_cfg.goalangle_power);
    config.goalangle_crtc.common.active = static_cast<unsigned int>(dyn_cfg.goalangle_active);
    config.goalangle_crtc.common.weight = static_cast<float>(dyn_cfg.goalangle_weight);
    config.goalangle_crtc.common.threshold = static_cast<float>(dyn_cfg.goalangle_threshold);

    // Twirling critic config
    config.twir_crtc.common.power = static_cast<unsigned int>(dyn_cfg.twir_power);
    config.twir_crtc.common.active = static_cast<unsigned int>(dyn_cfg.twir_active);
    config.twir_crtc.common.weight = static_cast<float>(dyn_cfg.twir_weight);

    // Forward critic config
    config.forward_crtc.common.power = static_cast<unsigned int>(dyn_cfg.frwd_power);
    config.forward_crtc.common.active = static_cast<unsigned int>(dyn_cfg.frwd_active);
    config.forward_crtc.common.weight = static_cast<float>(dyn_cfg.frwd_weight);

    // PathDist critic config
    config.pathdist_crtc.common.power = static_cast<unsigned int>(dyn_cfg.pathdist_power);
    config.pathdist_crtc.common.active = static_cast<unsigned int>(dyn_cfg.pathdist_active);
    config.pathdist_crtc.common.weight = static_cast<float>(dyn_cfg.pathdist_weight);
    config.pathdist_crtc.common.threshold = static_cast<float>(dyn_cfg.pathdist_threshold);
    config.pathdist_crtc.traj_stride = dyn_cfg.pathdist_stride;

    // PathFollow critic config
    config.pathfollow_crtc.common.power     = static_cast<unsigned int>(dyn_cfg.pathfollow_power);
    config.pathfollow_crtc.common.active     = static_cast<unsigned int>(dyn_cfg.pathfollow_active);
    config.pathfollow_crtc.common.weight    = static_cast<float>(dyn_cfg.pathfollow_weight);
    config.pathfollow_crtc.common.threshold = static_cast<float>(dyn_cfg.pathfollow_threshold);
    config.pathfollow_crtc.offset_from_furthest = static_cast<size_t>(dyn_cfg.pathfollow_offset);

    // PathFollow critic config
    config.pathangle_crtc.common.power     = static_cast<unsigned int>(dyn_cfg.pathangle_power);
    config.pathangle_crtc.common.active     = static_cast<unsigned int>(dyn_cfg.pathangle_active);
    config.pathangle_crtc.common.weight    = static_cast<float>(dyn_cfg.pathangle_weight);
    config.pathangle_crtc.common.threshold = static_cast<float>(dyn_cfg.pathangle_threshold);
    config.pathangle_crtc.offset_from_furthest = static_cast<size_t>(dyn_cfg.pathangle_offset);
    config.pathangle_crtc.angle_threshold = static_cast<float>(dyn_cfg.pathangle_angle_threshold) * M_PI/180.0;

    // Obstacles critic config
    config.obs_crtc.common.power     = static_cast<unsigned int>(dyn_cfg.obs_power);
    config.obs_crtc.common.active    = static_cast<unsigned int>(dyn_cfg.obs_active);
    config.obs_crtc.common.weight    = static_cast<float>(dyn_cfg.obs_weight);
    config.obs_crtc.common.threshold = static_cast<float>(dyn_cfg.obs_threshold);
    config.obs_crtc.repulsive_weight = static_cast<float>(dyn_cfg.obs_repulsive_weight);
    config.obs_crtc.collision_cost   = static_cast<float>(dyn_cfg.obs_collision_cost);
    config.obs_crtc.collision_margin_dist = static_cast<float>(dyn_cfg.obs_collision_margin_dist);

    ros::NodeHandle nh_upper("~/");
    nh_upper.param<float>("local_costmap/inflation_layer/inflation_radius",    
                            config.obs_crtc.inflation_radius, 
                            0.0f);
    nh_upper.param<float>("local_costmap/inflation_layer/cost_scaling_factor", 
                            config.obs_crtc.inflation_scale_factor, 
                            0.0f);

    config.print_out(); // print out config (debug)

    // Reconfigure MPPI controller
    mappi_.setConfig(config);

    } catch (...) {
        ROS_ERROR("mappi:: Dynamic Reconfigure parameters could not be read!");
    }
}

} // namespace mappi

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mappi::MPPIcROS, nav_core::BaseLocalPlanner)