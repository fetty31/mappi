/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Shared Pointer template
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_shared_ptr_HPP__
#define __MAPPI_shared_ptr_HPP__

// Try ROS1 first
#if __has_include(<ros/version.h>)
    #include <ros/version.h>
    #include <boost/shared_ptr.hpp> // Use boost::shared_ptr for ROS1
    #include <boost/make_shared.hpp>
#else
    #include <memory> // Use std::shared_ptr for ROS2 / non-ROS
#endif

namespace mappi {

#if defined(MAPPI_ROS1)

    template <typename T>
    using shared_ptr = boost::shared_ptr<T>;

    template <typename T, typename... Args>
    boost::shared_ptr<T> make_shared(Args&&... args) {
        return boost::make_shared<T>(std::forward<Args>(args)...);
    }

#else // Assume ROS2 or non-ROS project

    template <typename T>
	using shared_ptr = std::shared_ptr<T>;

	template <typename T, typename... Args>
	std::shared_ptr<T> make_shared(Args&&... args) {
    	return std::make_shared<T>(std::forward<Args>(args)...);
	}

#endif

} // namespace mappi

#endif