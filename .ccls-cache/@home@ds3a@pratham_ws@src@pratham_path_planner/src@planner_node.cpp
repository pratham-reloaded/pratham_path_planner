#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <functional>
#include <string>
#include <math.h>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"


#include "path_planning/d_star_lite.hpp"
#include "../include/path_planning_lib/src/d_star_lite.cpp"

#include "path_planning/a_star.hpp"
#include "../include/path_planning_lib/src/a_star.cpp"


#include "path_planning/dijkstra.hpp"
#include "../include/path_planning_lib/src/dijkstra.cpp"

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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
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


int temp_0=0;
int temp_1=0;
int counter_0=0;
int counter_1=0;

constexpr int n = 320;

static grid_map::GridMapRosConverter conv;



static float occupied_threshold = 0.0;
float occupancy_grid_to_vec(float x) {
  if (x > occupied_threshold) {
    return 1;
  } else {
    return 0;
  }
}

Eigen::MatrixXf createMatrixFromVector(std::vector<std::vector<int>> vectorMat, Eigen::MatrixXf& matrixMat) {
    // std::cout << vectorMat.size() << " " << vectorMat[1].size() << std::endl;
    for(int i = 0; i < vectorMat.size(); i++) {
        for(int j = 0; j < vectorMat[i].size(); j++) {
            matrixMat(i,j) = static_cast<float>(vectorMat[i][j]);
        }
    }
    return matrixMat;
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


      gridmap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid", 10);




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

      // this is a test to see what the node is perceiving
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_publisher;

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

      float goal_x=4;
      float goal_y=4;
      //transforms
      geometry_msgs::msg::TransformStamped odom_to_base;
      geometry_msgs::msg::TransformStamped odom_to_map;
      geometry_msgs::msg::TransformStamped map_to_base;

      void get_odom_to_base_tf() {
        // lookup the latest transform between odom and base_link
        this->odom_to_base = this->tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
      }

      void get_odom_to_map_tf() {
          this->odom_to_map = this->tf_buffer_->lookupTransform("base_link", "map_link", tf2::TimePointZero);
      }

      void get_map_to_base_tf() {
        this->map_to_base = this->tf_buffer_->lookupTransform("map_link", "base_link", tf2::TimePointZero);
      }
    

      void anymap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr local_map)
      {
        try {
          this->get_map_to_base_tf();
          this->get_odom_to_map_tf();
          this->get_odom_to_base_tf();
        } catch (const tf2::TransformException &ex) {
          std::cout << "unable to look up tf. Make sure the localization thingi is running\n";
        }

        // std::cout << "received a map\n";
        this->anymap_ptr->get("anymap").setConstant(0.0);
        conv.fromOccupancyGrid(*local_map, "anymap", *(this->anymap_ptr));
        // std::cout << "added from occupancyGrid to anymap_ptr\n";

        // std::cout << "the matrix size is " << this->anymap_ptr->get("anymap").rows()
                  // << " " << this->anymap_ptr->get("anymap").cols() << std::endl;

        Eigen::MatrixXf unprocessed_grid_map =
          this->anymap_ptr->get("anymap").cast<float>();

/*        grid_map::Position pose_(0, 2);
        grid_map::Index ind_;
        this->anymap_ptr->getIndex(pose_, ind_);
        std::cout << "index of (0, 2) is \n" << ind_ << std::endl;
*/
        // .cast<Eigen::Matrix2f::Scalar>();
        // .cast<float>();
        // std::cout << "got the unprocessed matrix, now processing\n";
        Eigen::MatrixXf processed_grid_map =
          unprocessed_grid_map.unaryExpr(&occupancy_grid_to_vec);
        // std::cout << "converted anymap to Eigen::matrix\n";

        int rows = processed_grid_map.rows();
        int cols = processed_grid_map.cols();
        // std::cout << rows << " " << cols << std::endl;

        Eigen::MatrixXi int_grid_map = processed_grid_map.cast<int>().transpose();
        this->grid.clear();
        for(int i=0; i<cols; i++) {
          const int* begin = &int_grid_map.col(i).data()[0];
          this->grid.push_back(std::vector<int>(begin, begin+rows));
        }

        // std::cout << " the vector<vector<int>> is of shape " << this->grid.size() << " " << this->grid[0].size() << std::endl;

/* // THIS STUFF IS JUST FOR DEBUGGING
        this->anymap_ptr->add("stuff_shown", 0.0);

        nav_msgs::msg::OccupancyGrid grid_msg;
        // std::cout << "the matrix size is: \n";
        // std::cout << createMatrixFromVector(this->grid).size() << std::endl;

        createMatrixFromVector(this->grid, this->anymap_ptr->get("stuff_shown"));

        conv.toOccupancyGrid(*anymap_ptr.get(), "stuff_shown", 0, 1, grid_msg);
        grid_msg.header.frame_id = "map_link";

        gridmap_publisher->publish(grid_msg);
*/
        // std::cout << "received grid with avg value " << getAverageValue(this->grid)


        // This should be the transform between map_link and base_link converted to indices


        grid_map::Position pose(this->map_to_base.transform.translation.x,
                                this->map_to_base.transform.translation.y);
        grid_map::Index ind;
        this->anymap_ptr->getIndex(pose, ind);

        // TODO check if it is ind(0) ind(1) or the opposite
        // DONE, this is correct
        Grid start(ind(0), ind(1), 0, 0, 0, 0);
        // std::cout << "setting the goal to " << n - 1 - goal_y << " " << goal_x << std::endl;
        // Grid goal(0, 319, 0, 0, 0, 0);

        // we have OG stored in (this->goal_x, this->goal_y)
        // we have OM stored in this->odom_to_map
        // MG = OG - OM
        // we need to rotate it by -yaw to supply the goal to the path planner
        tf2::Quaternion q;
        double roll, pitch, yaw;
        q.setX(this->odom_to_map.transform.rotation.x);
        q.setY(this->odom_to_map.transform.rotation.y);
        q.setZ(this->odom_to_map.transform.rotation.z);
        q.setW(this->odom_to_map.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // this returns yaw in radians

        // OG - OM = MG
        float mg_x = this->goal_x - this->odom_to_map.transform.translation.x;
        float mg_y = this->goal_y - this->odom_to_map.transform.translation.y;

        // (mogx, mogy) is MG but with M as a non rotated frame, we can convert this to indices to provide as a goal
        float mogx = (mg_x * std::cos(-yaw)) + (mg_y * (-std::sin(-yaw)));
        float mogy = (mg_x * std::sin(-yaw)) + (mg_y * std::cos(-yaw));

        grid_map::Position mog_pose(mogx, mogy);
        grid_map::Index goal_ind;
        this->anymap_ptr->getIndex(mog_pose, goal_ind);

        Grid goal(goal_ind(0), goal_ind(1), 0, 0, 0, 0);

        // std::cout << "created nodes for start and goal\n";
        start.id_ = start.x_ * n + start.y_;
        start.pid_ = start.x_ * n + start.y_;
        goal.id_ = goal.x_ * n + goal.y_;
        start.h_cost_ = sqrt(powf(start.x_ - goal.x_, 2) + powf(start.y_ - goal.y_, 2));


        // std::cout << "instanciating the path planner\n";
        AStar d_star_lite(this->grid);
        {
          const auto [path_found, path_vector] = d_star_lite.Plan(start, goal);
          // std::cout << "path planning done? " << path_found << std::endl;
          // std::cout << "size of the path found " << path_vector.size() << std::endl;
          this->path = path_vector;
          // std::cout << "received path of length " << path_vector.size() <<std::endl;
          // std::cout << "resized the path now printing\n";
          // std::cout << "path printed now it needs to be published\n";
        }
      }

      void local_goal_callback(const geometry_msgs::msg::Pose::SharedPtr goal)
      {
        std::cout << "received new goal\n";
        this->goal_x = goal->position.x;
        this->goal_y = goal->position.y;

        // NOTE
        // Goals are given with respect to base link
        // while storing them, we need to store it in terms of the odom frame
        // to do that, we first rotate the current goal_x and goal_y in the direction that base link is rotated w.r.t odom
        // then add odom to the rotated matrix and store

        tf2::Quaternion q;
        double roll, pitch, yaw;
        q.setX(this->odom_to_base.transform.rotation.x);
        q.setY(this->odom_to_base.transform.rotation.y);
        q.setZ(this->odom_to_base.transform.rotation.z);
        q.setW(this->odom_to_base.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // this returns yaw in radians

        //now to get the vector BG (B = base_link, G = goal)
        //std trig functions take in radians as inputs
        this->goal_x = (goal->position.x * std::cos(yaw)) + (goal->position.y * (-std::sin(yaw)));
        this->goal_y = (goal->position.x * std::sin(yaw)) + (goal->position.y * std::cos(yaw));

        // We have OB (O = odom, B = base_link), from this->odom_to_base
        // we just have to add OB to BG to get OG, which we can store
        this->goal_x += this->odom_to_base.transform.translation.x;
        this->goal_y += this->odom_to_base.transform.translation.y;
        // (goal_x, goal_y) forms vector OG
      }

      void path_publisher_callback()
      {
        // std::cout << "publishing path : \n";
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
          // std::cout << "the path is : " << path[i].x_ << " " << path[i].y_ << std::endl;
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
