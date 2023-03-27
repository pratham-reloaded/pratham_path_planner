#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <functional>
#include <string>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"


#include "path_planning/d_star_lite.hpp"
#include "../include/path_planning_lib/src/d_star_lite.cpp"

#include "path_planning/a_star.hpp"
#include "../include/path_planning_lib/src/a_star.cpp"


#include "path_planning/dijkstra.hpp"
#include "../include/path_planning_lib/src/dijkstra.cpp"

// #include "path_planning/rrt_star.hpp"
// #include "../include/path_planning_lib/src/rrt_star.cpp"

#include "path_planning/rrt.hpp"
#include "../include/path_planning_lib/src/rrt.cpp"

#include "path_planning/lpa_star.hpp"
#include "../include/path_planning_lib/src/lpa_star.cpp"

#include "path_planning/jump_point_search.hpp"
#include "../include/path_planning_lib/src/jump_point_search.cpp"

#include "path_planning/genetic_algorithm.hpp"
#include "../include/path_planning_lib/src/genetic_algorithm.cpp"

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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

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

constexpr int n = 320;

static grid_map::GridMapRosConverter conv;



static float occupied_threshold = 0;
float occupancy_grid_to_vec(float x) {
  if (x > occupied_threshold) {
    return 1;
  } else {
    return 0;
  }
}


double getAverageValue(const std::vector<std::vector<int>>& matrix) {
    int count = 0;
    int sum = 0;
    for (const auto& row : matrix) {
        for (const auto& val : row) {
            sum += val;
            count++;
        }
    }
    if (count == 0) return 0;
    return static_cast<double>(sum) / count;
}

class PathPlanner : public rclcpp::Node
{
  public:
    PathPlanner(): rclcpp::Node("path_planner"), grid(n, std::vector<int>(n, 0))
    {
      std::cout << "initializing subscriptions\n";
      //Subscriptions
      goal_subscription=this->create_subscription<geometry_msgs::msg::Pose>("goal_local",10,std::bind(&PathPlanner::local_goal_callback,this,_1));
      costmap_subscription=this->create_subscription<nav_msgs::msg::OccupancyGrid>("anymap",10,std::bind(&PathPlanner::anymap_callback,this,_1));

      std::cout << "initializing publishers\n";
      //Publishers
      path_publisher=this->create_publisher<nav_msgs::msg::Path>("path_local",10);
      path_publish_timer=this->create_wall_timer(50ms, std::bind(&PathPlanner::path_publisher_callback, this));

      std::cout << "initializing tf stuff\n";
      //tf_listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      // timer_tf = this->create_wall_timer(9000s, std::bind(&PathPlanner::tf_listener, this));

      std::cout << "initializing anymap\n";
      this->anymap_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
      this->anymap_ptr->setGeometry(grid_map::Length(8, 8), 0.025);
      this->anymap_ptr->add("anymap", 0.0);
      std::cout << "initialization done\n";
    }

  private:
      //Path_publisher
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
      rclcpp::TimerBase::SharedPtr path_publish_timer;


      //Subscriptions
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscription;

      // the path that will be published
      std::vector<Grid> path;


      // gridmap stuff
      std::shared_ptr<grid_map::GridMap> anymap_ptr;
      std::vector<std::vector<int>> grid;
                //                                   n = 320 ^  ^ fill with zeros

      //tf_listener
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      rclcpp::TimerBase::SharedPtr timer_tf{nullptr};

      //transforms
      geometry_msgs::msg::TransformStamped odom_to_base;
      geometry_msgs::msg::TransformStamped odom_to_map;
      geometry_msgs::msg::TransformStamped map_to_base;


/*      void tf_listener()
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
*/
      void get_odom_to_base_tf() {
        // lookup the latest transform between odom and base_link
        this->odom_to_base = this->tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
      }

      void get_odom_to_map_tf() {
          this->odom_to_map = this->tf_buffer_->lookupTransform("base_link", "map_link", tf2::TimePointZero);
      }

      void get_map_to_base_tf() {
        this->get_map_to_base = this->tf_buffer_->lookupTransform("map_link", "base_link", tf2::TimePointZero);
      }
    

      void anymap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr local_map)
      {
        std::cout << "received a map\n";
        this->anymap_ptr->get("anymap").setConstant(0.0);
        conv.fromOccupancyGrid(*local_map, "anymap", *(this->anymap_ptr));
        std::cout << "added from occupancyGrid to anymap_ptr\n";

        std::cout << "the matrix size is " << this->anymap_ptr->get("anymap").rows()
                  << " " << this->anymap_ptr->get("anymap").cols() << std::endl;

        Eigen::MatrixXf unprocessed_grid_map =
          this->anymap_ptr->get("anymap").cast<float>();
        // .cast<Eigen::Matrix2f::Scalar>();
        // .cast<float>();
        std::cout << "got the unprocessed matrix, now processing\n";
        Eigen::MatrixXf processed_grid_map =
          unprocessed_grid_map.unaryExpr(&occupancy_grid_to_vec);
        std::cout << "converted anymap to Eigen::matrix\n";

        Eigen::Map<
          Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            const_cast<int *>(this->grid[0].data()), processed_grid_map.rows(),
            processed_grid_map.cols()) = processed_grid_map.cast<int>();

        // std::cout << "received grid with avg value " << getAverageValue(this->grid)
        // << std::endl;
        Grid start(155, 160, 10, 1230.02, 0, 0);
        std::cout << "setting the goal to " << n - 1 - goal_y << " " << goal_x
                  << std::endl;
        Grid goal(0, 10, 122, 0.1, 1, 0);

        std::cout << "created nodes for start and goal\n";
        start.id_ = start.x_ * n + start.y_;
        start.pid_ = start.x_ * n + start.y_;
        goal.id_ = goal.x_ * n + goal.y_;
        start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

        std::cout << "instanciating the path planner\n";
        DStarLite d_star_lite(grid);
        // d_star_lite.SetParams(20, 1010);
        {
          const auto [path_found, path_vector] = d_star_lite.Plan(start, goal);
          std::cout << "path planning done? " << path_found << std::endl;
          std::cout << "size of the path found " << path_vector.size() << std::endl;
          this->path = path_vector;
          std::cout << "resized the path now printing\n";
          std::cout << "path printed now it needs to be published\n";
        }
      }

      void local_goal_callback(const geometry_msgs::msg::Pose::SharedPtr goal) const
      {
        goal_x=(int) goal->position.x;
        // goal_y=(int) goal->position.y;
      }

      void path_publisher_callback()
      {
        std::cout << "publishing path : \n";
        auto path_local = nav_msgs::msg::Path();
        path_local.header.frame_id = "map_link";

        for(int i=0; i<path.size();i++){
          geometry_msgs::msg::PoseStamped pose_stamped_msg;
          grid_map::Position position;
          grid_map::Index index(path[i].x_, path[i].y_);
          this->anymap_ptr->getPosition(index, position);
          pose_stamped_msg.pose.position.x = position.x();
          pose_stamped_msg.pose.position.y = position.y();
          path_local.poses.emplace_back(pose_stamped_msg);
          std::cout << "the path is : " << path[i].x_ << " " << path[i].y_ << std::endl;
        }
        // std::cout << path_local << std::endl;
        path_publisher->publish(path_local);
      }

     

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();

  return 0;
}
