#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <functional>

#include "pratham_path_planner/test.hpp"

#include "ros_api/occupancy_grid.hpp"

#include "path_planning/d_star_lite.hpp"
#include "../include/path_planning_lib/src/d_star_lite.cpp"

#include "utils/utils.hpp"
#include "../include/path_planning_lib/lib/utils/src/utils.cpp"

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
    
class OccupancyGrid : public rclcpp::Node
{
    public:
        OccupancyGrid(): rclcpp::Node("Test_Occupancy_Grid")

        goal_publisher=this->create_publisher<geometry_msgs::msg::Pose>("goal_local",10)        
        grid_publisher=this->create_publisher<nav_msgs::msg::OccupancyGrid>("yasmap",10);
        grid_publisher_timer=this->create_wall_timer(50ms, std::bind(&OccupancyGrid::publisher_callback));


        
}