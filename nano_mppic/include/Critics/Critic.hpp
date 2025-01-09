#ifndef __NANO_MPPI_CRITIC_HPP__
#define __NANO_MPPI_CRITIC_HPP__

#include <string>
#include <memory>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace nano_mppic::critics {

class Critic {

    // VARIABLES

    protected:
        std::string name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        nav2_costmap_2d::Costmap2D * costmap_ptr_{nullptr};

    // FUNCTIONS

    public:
        Critic() = default;

        virtual ~Critic() = default;

        virtual void configure(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros){
            name_ = name;
            costmap_ros_ptr_ = costmap_ros;
            costmap_ptr_ = costmap_ros->getCostmap();
        }

        virtual void score(nano_mppic::objects::State&,
                            nano_mppic::objects::Trajectory&,
                            xt::xtensor<float,1>&,
                            bool& ) = 0;

        std::string getName(){ return name_; }

};

} // namespace nano_mppic::critics

#endif