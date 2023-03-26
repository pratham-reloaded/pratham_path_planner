#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <functional>
#include <string>

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


#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

using namespace::std::chrono_literals;
using std::placeholders::_1;

double x_translation=0.0, y_translation=0.0; 
double x_quaternion=0.0,y_quaternion=0.0,z_quaternion=0.0,w_quaternion=0.0;

int time_sec=0;
unsigned int time_microsec=0;
int time_count=0;

int goal_x=0;
int goal_y=0;

int temp_0=0;
int temp_1=0;
int counter_0=0;
int counter_1=0;

constexpr int n = 4;
std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
std::vector<std::vector<int>> path(0, std::vector<int>(2,0));


static grid_map::GridMapRosConverter conv;

class PathPlanner : public rclcpp::Node
{
  public:
    PathPlanner(): rclcpp::Node("path_planner")
    {
      //Subscriptions
      goal_subscription=this->create_subscription<geometry_msgs::msg::Pose>("goal_local",10,std::bind(&PathPlanner::local_goal_callback,this,_1));
      costmap_subscription=this->create_subscription<nav_msgs::msg::OccupancyGrid>("anymap",10,std::bind(&PathPlanner::anymap_callback,this,_1));

      //Publishers
      path_publisher=this->create_publisher<nav_msgs::msg::Path>("path_local",10);
      path_publish_timer=this->create_wall_timer(50ms, std::bind(&PathPlanner::path_publisher_callback, this));

      //tf_listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      timer_tf = this->create_wall_timer(9000s, std::bind(&PathPlanner::tf_listener, this));
    }

  private:

      void tf_listener()
      {
        std::string fromFrameRel = "map_link";
        std::string toFrameRel = "base_link";
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
        x_translation=t.transform.translation.x;
        y_translation=t.transform.translation.y;
        x_quaternion=t.transform.rotation.x;
        y_quaternion=t.transform.rotation.y;
        z_quaternion=t.transform.rotation.z;
        w_quaternion=t.transform.rotation.w;
      }

      void anymap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr local_map) const
      {
        for (int i=0; i<(n*n); i++){
          if(temp_0==4){
            temp_0=0;
            temp_1++;
          };

          if(local_map->data[i]>75){
          grid[n-1-temp_1][temp_0]=1;}
          temp_0++;
        }

        Grid start(3, 0, 0, 0, 0, 0);
        Grid goal(n-1-goal_y, goal_x, 0, 0, 0, 0);

        start.id_ = start.x_ * n + start.y_;
        start.pid_ = start.x_ * n + start.y_;
        goal.id_ = goal.x_ * n + goal.y_;
        start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

        DStarLite d_star_lite(grid);
        {
          const auto [path_found, path_vector] = d_star_lite.Plan(start, goal);
          path.resize(path_vector.size());
          for (int i=0; i<path_vector.size(); i++) {
            printf("%d %d\n", path_vector[i].x_, path_vector[i].y_);
            path[i][0]=n-1-path_vector[i].y_;
            path[i][1]=path_vector[i].x_;
          }
        }

      }

      void local_goal_callback(const geometry_msgs::msg::Pose::SharedPtr goal) const
      {
        goal_x=(int) goal->position.x;
        goal_y=(int) goal->position.y;
      }

      void path_publisher_callback()
      {
        auto path_local=nav_msgs::msg::Path();
        time_count++;
        time_microsec=time_microsec+50;
        if(time_count%20==0)
        {
          time_sec++;
        }
        path_local.header.stamp.sec=time_sec;
        path_local.header.stamp.nanosec=time_microsec;
        path_local.header.frame_id="map_link";
        for(int i=0; i<path.size();i++){
          path_local.poses[i].pose.position.x=path[i][0];
          path_local.poses[i].pose.position.y=path[i][1];
        }
        path_publisher->publish(path_local);
      }

     
      //Path_publisher
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
      rclcpp::TimerBase::SharedPtr path_publish_timer;
      

      //Subscriptions
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscription; 


      // gridmap instance
      std::shared_ptr<grid_map::GridMap> anymap_ptr;


      //tf_listener
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      rclcpp::TimerBase::SharedPtr timer_tf{nullptr};


};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();

  return 0;
}
