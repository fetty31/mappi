#ifndef __NANO_MPPI_CRITIC_HPP__
#define __NANO_MPPI_CRITIC_HPP__

#include <string>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>

#include <ros/common.h> // to-do: avoid ros dependency

namespace nano_mppic {
#if ROS_VERSION_MAJOR > 1

    #include <memory> // Use std::shared_ptr for ROS2
	 
	template <typename T>
	using shared_ptr = std::shared_ptr<T>;

	template <typename T, typename... Args>
	std::shared_ptr<T> make_shared(Args&&... args) {
    	return std::make_shared<T>(std::forward<Args>(args)...);
	}

#else

    #include <boost/shared_ptr.hpp> // Use boost::shared_ptr for ROS
    
	template <typename T>
	using shared_ptr = boost::shared_ptr<T>;

	template <typename T, typename... Args>
	boost::shared_ptr<T> make_shared(Args&&... args) {
    	return boost::make_shared<T>(std::forward<Args>(args)...);
	}

#endif
}

namespace nano_mppic::critics {

class Critic {

    // VARIABLES

    protected:
        std::string name_;
        nano_mppic::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_ptr_{nullptr};

    // FUNCTIONS

    public:
        Critic() = default;

        virtual ~Critic() = default;

        virtual void configure(std::string name, nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros){
            name_ = name;
            costmap_ros_ptr_ = costmap_ros;
            costmap_ptr_ = costmap_ros->getCostmap();
        }

        virtual void score(nano_mppic::objects::State&,
                            nano_mppic::objects::Trajectory&,
                            nano_mppic::objects::Path&,
                            xt::xtensor<float,1>&,
                            bool& ) = 0;

        std::string getName(){ return name_; }

};

} // namespace nano_mppic::critics

#endif