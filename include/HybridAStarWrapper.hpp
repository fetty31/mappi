#ifndef __MAPPI_HYBRIDASTAR_WRAPPER_HPP__
#define __MAPPI_HYBRIDASTAR_WRAPPER_HPP__

// Hybrid A* implementation: https://github.com/karlkurzer/path_planner
#include <constants.h>
#include <helper.h>
#include <collisiondetection.h>
#include <dynamicvoronoi.h>
#include <algorithm.h>
#include <node3d.h>
#include <path.h>
#include <smoother.h>
#include <visualize.h>
#include <lookup.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <vector>
#include <string>
#include <mutex>
#include <memory>

class HybridAStarWrapper {

    public:
        HybridAStarWrapper();
        HybridAStarWrapper(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>&);
        ~HybridAStarWrapper();

        void configure(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>&);

        bool getPlan(mappi::objects::Odometry2d odom, 
                    mappi::objects::Path& goals,
                    mappi::objects::Path& out_plan);

        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        mappi::objects::Path& out_plan);

    protected:
        mappi::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_{nullptr};


    private:
        
        // The path produced by the hybrid A* algorithm
        HybridAStar::Path path; 
        
        // The smoother used for optimizing the path
        HybridAStar::Smoother smoother; 

        // The path smoothed and ready for the controller
        HybridAStar::Path smoothedPath = HybridAStar::Path(true); 
        
        // The visualization used for search visualization
        HybridAStar::Visualize visualization; 
        
        // The collission detection for testing specific configurations
        HybridAStar::CollisionDetection configurationSpace;

        /// The voronoi diagram (To-Do: probably not used!!)
        HybridAStar::DynamicVoronoi voronoiDiagram;

        /// A pointer to the grid the planner runs on
        nav_msgs::OccupancyGrid::Ptr grid;

        /// The start pose set through RViz
        geometry_msgs::PoseWithCovarianceStamped start;

        /// The goal pose set through RViz
        geometry_msgs::PoseStamped goal;

        /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
        HybridAStar::Constants::config collisionLookup[HybridAStar::Constants::headings * HybridAStar::Constants::positions];

        /// A lookup of analytical solutions (Dubin's paths)
        float* dubinsLookup = new float [HybridAStar::Constants::headings * 
                                        HybridAStar::Constants::headings * 
                                        HybridAStar::Constants::dubinsWidth * 
                                        HybridAStar::Constants::dubinsWidth
                                        ];
        
        // Custom functions to choose Goal & Start points from received path
        geometry_msgs::PoseStamped chooseGoal(mappi::objects::Path&);
        geometry_msgs::PoseStamped chooseStart(mappi::objects::Odometry2d&);

        // From map [pixel] to world [meters] func.
        void mapToWorld(double mx, double my, double& wx, double& wy);

        // Clear cost of cell [mx,my]
        void clearCell(unsigned int mx, unsigned int my);

        // Transform costmap into nav_msgs::OccupancyGrid
        void fromCostmapToGrid(nav_msgs::OccupancyGrid&);

        // Transform HybridAStar::Node3D vector into mappi::objects::Path
        void fromAStarToMappi(const std::vector<HybridAStar::Node3D>&, mappi::objects::Path&);
        
        // Start point shifting [meters]
        double start_shift_;

        // Aux. mutex
        std::mutex mutex_;
        
        // Cost translation LUT
        char* cost_translation_table_;
        
        // Global frame ID
        std::string global_frame_;
};

HybridAStarWrapper::HybridAStarWrapper() 
{
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;     // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100; // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++)
        cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
}

HybridAStarWrapper::HybridAStarWrapper(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros)
{
    configure(name, costmap_ros);
}

HybridAStarWrapper::~HybridAStarWrapper() { delete costmap_; }

void HybridAStarWrapper::configure(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros)
{
    costmap_ros_ptr_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param<double>("start_shift", start_shift_, 0.0);
}

bool HybridAStarWrapper::getPlan(mappi::objects::Odometry2d odom, 
                                mappi::objects::Path& goals,
                                mappi::objects::Path& out_plan
                                )
{
    geometry_msgs::PoseStamped start;
    start = chooseStart(odom); 

    geometry_msgs::PoseStamped goal;
    goal = chooseGoal(goals);

    if(this->makePlan(start, goal, out_plan)){
        return true;
    }
    else
    {
        ROS_ERROR("HybridAStarWrapper::ERROR couldn't find any path!");
        return false;
    }
    
}

geometry_msgs::PoseStamped HybridAStarWrapper::chooseGoal(mappi::objects::Path& goals)
{
    geometry_msgs::PoseStamped chosen_goal;
    chosen_goal.header.frame_id = global_frame_;
    
    size_t N = goals.x.size()-1;
    unsigned int mx, my;

    // Search goals vector until the chosen goal is inside the local costmap
    while( (N > 0) && 
        (not costmap_ros_ptr_->getCostmap()->worldToMap(goals.x(N), goals.y(N), mx, my)) 
    ){
        N--;
    }

    chosen_goal.pose.position.x = goals.x(N);
    chosen_goal.pose.position.y = goals.y(N);

    return chosen_goal;
}

geometry_msgs::PoseStamped HybridAStarWrapper::chooseStart(mappi::objects::Odometry2d& odom)
{
    geometry_msgs::PoseStamped chosen_start;
    chosen_start.header.frame_id = global_frame_;
    chosen_start.pose.position.x = odom.x;
    chosen_start.pose.position.y = odom.y;

    tf2::Quaternion q;
    q.setRPY( 0, 0, odom.yaw ); 
    q.normalize();
    chosen_start.pose.orientation = tf2::toMsg(q);

    return chosen_start;
}

bool HybridAStarWrapper::makePlan(const geometry_msgs::PoseStamped& start, 
                                    const geometry_msgs::PoseStamped& goal, 
                                    mappi::objects::Path& out_plan)
{

    ROS_INFO("mappi::HybridAStarWrapper checking start & goal are valid...");

    // Check goal & start are valid points
    unsigned int sx, sy; // start position
    if(not costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy))
        throw std::runtime_error("mappi::HybridAStarWrapper ERROR: start position is not inside the given costmap");

    unsigned int gx, gy; // start position
    if(not costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
        throw std::runtime_error("mappi::HybridAStarWrapper ERROR: goal position is not inside the given costmap");

    int height = costmap_->getSizeInCellsX();
    int width = costmap_->getSizeInCellsY();
    int depth = HybridAStar::Constants::headings;
    int length = width * height * depth;

    ROS_INFO("mappi::HybridAStarWrapper initializing Node lists...");

    // Define list pointers and initialize lists
    HybridAStar::Node3D* nodes3D = new HybridAStar::Node3D[length]();
    HybridAStar::Node2D* nodes2D = new HybridAStar::Node2D[width * height]();

    ROS_INFO("mappi::HybridAStarWrapper Defining Goal & Start...");
    
    // Retrieving goal position
    float x = static_cast<float>(gx);
    float y = static_cast<float>(gy);
    float t = tf::getYaw(goal.pose.orientation);

    const HybridAStar::Node3D nGoal(x, y, t, 0, 0, nullptr);
    
    // Retrieving start position
    x = static_cast<float>(sx);
    y = static_cast<float>(sy);
    t = tf::getYaw(start.pose.orientation);

    HybridAStar::Node3D nStart(x, y, t, 0, 0, nullptr);

    ros::Time t0 = ros::Time::now();

    ROS_INFO("mappi::HybridAStarWrapper clear paths...");
    
    // Clear paths
    path.clear();
    smoothedPath.clear();

    ROS_INFO("mappi::HybridAStarWrapper from costmap to grid...");

    // Update occupancy grid
    nav_msgs::OccupancyGrid::Ptr grid(boost::make_shared<nav_msgs::OccupancyGrid>());
    this->fromCostmapToGrid(*grid);
    configurationSpace.updateGrid(grid);

    ROS_INFO("mappi::HybridAStarWrapper solve...");

    // Solve
    HybridAStar::Visualize visualization; // not used
    HybridAStar::Node3D* nSolution = HybridAStar::Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    
    ROS_INFO("mappi::HybridAStarWrapper tracing path...");

    // Trace the path
    smoother.tracePath(nSolution);

    // Create updated path
    path.updatePath(smoother.getPath());

    ROS_INFO("mappi::HybridAStarWrapper smooth path after defining voronoi grid...");

// DEBUG
height = grid->info.height;
width = grid->info.width;
bool** binMap;
binMap = new bool*[width];

for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
    binMap[x][y] = grid->data[y * width + x] ? true : false;
    }
}

voronoiDiagram.initializeMap(width, height, binMap);
voronoiDiagram.update();
voronoiDiagram.visualize();

// Smooth the path
smoother.smoothPath(voronoiDiagram);

// Create smooth updated path
smoothedPath.updatePath(smoother.getPath());

    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    ROS_INFO_STREAM("mappi::HybridAStarWrapper TIME in ms: " << d * 1000);

    // Fill output plan
    this->fromAStarToMappi(smoother.getPath(), out_plan);
    
    ROS_INFO("mappi::HybridAStarWrapper publish visualize...");

// Publish
path.publishPath();
path.publishPathNodes();
path.publishPathVehicles();
smoothedPath.publishPath();
smoothedPath.publishPathNodes();
smoothedPath.publishPathVehicles();
visualization.publishNode3DCosts(nodes3D, width, height, depth);
visualization.publishNode2DCosts(nodes2D, width, height);

    // Collect garbage
    delete [] nodes3D;
    delete [] nodes2D;

    return true;
}

void HybridAStarWrapper::fromCostmapToGrid(nav_msgs::OccupancyGrid& grid)
{
    double resolution = costmap_->getResolution();

    grid.header.frame_id = global_frame_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = costmap_->getSizeInCellsX();
    grid.info.height = costmap_->getSizeInCellsY();

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);

    unsigned char* data = costmap_->getCharMap();
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        grid.data[i] = cost_translation_table_[ data[ i ]];
    }
}

void HybridAStarWrapper::fromAStarToMappi(const std::vector<HybridAStar::Node3D>& in_path, mappi::objects::Path& out_path)
{
    out_path.reset(in_path.size());

    for(int i=0; i < in_path.size(); i++){
        out_path.x(i)    = in_path[i].getX();
        out_path.y(i)    = in_path[i].getY();
        out_path.yaw(i)  = 0.0;
        out_path.free(i) = true;
    }
}

#endif