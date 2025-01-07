#ifndef __NANO_MPPI_CRITIC_HPP__
#define __NANO_MPPI_CRITIC_HPP__

#include <string>
#include <memory>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

namespace nano_mppic::critics {

class Critic {

    // VARIABLES

    protected:
        std::string name_;
        std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_ptr_{nullptr};

    // FUNCTIONS

    public:
        Critic() = default;

        virtual ~Critic() = default;

        virtual void configure(std::string name, costmap_2d::Costmap2DROS costmap_ros);

        virtual void score();

        std::string getName(){ return name_; }

};

} // namespace nano_mppic::critics

#endif