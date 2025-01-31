#ifndef __NANO_MPPI_shared_ptr_HPP__
#define __NANO_MPPI_shared_ptr_HPP__

#include <ros/common.h> 

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

#endif