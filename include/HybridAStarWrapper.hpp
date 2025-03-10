#ifndef __MAPPI_HYBRIDASTAR_WRAPPER_HPP__
#define __MAPPI_HYBRIDASTAR_WRAPPER_HPP__

// Hybrid A* implementation: https://github.com/karlkurzer/path_planner
#include <hybrid_astar/constants.h>
#include <hybrid_astar/helper.h>
#include <hybrid_astar/collisiondetection.h>
#include <hybrid_astar/dynamicvoronoi.h>
#include <hybrid_astar/algorithm.h>
#include <hybrid_astar/node3d.h>
#include <hybrid_astar/path.h>
#include <hybrid_astar/smoother.h>
#include <hybrid_astar/visualize.h>
#include <hybrid_astar/lookup.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <vector>
#include <string>
#include <mutex>
#include <memory>

typedef HybridAStar::Constants Constants;

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
                        std::vector<geometry_msgs::PoseStamped>& plan);

    protected:
        mappi::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_{nullptr};


    private:
        
        // The path produced by the hybrid A* algorithm
        HybridAStar::Path path; 
        
        // The smoother used for optimizing the path
        HybridAStar::Smoother smoother; 

        // The path smoothed and ready for the controller
        HybridAStar::Path smoothedPath = Path(true); 
        
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
        Constants::config collisionLookup[Constants::headings * Constants::positions];

        /// A lookup of analytical solutions (Dubin's paths)
        float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
        
        // Custom functions to choose Goal & Start points from received path
        geometry_msgs::PoseStamped chooseGoal(mappi::objects::Path&);
        geometry_msgs::PoseStamped chooseStart(mappi::objects::Odometry2d&, int mode=0);

        // From map [pixel] to world [meters] func.
        void mapToWorld(double mx, double my, double& wx, double& wy);

        // Clear cost of cell [mx,my]
        void clearCell(unsigned int mx, unsigned int my);
        
        // Start point shifting [meters]
        double start_shift_;

        // Aux. mutex
        std::mutex mutex_;
        
        // Global frame ID
        std::string global_frame_;
};

#endif