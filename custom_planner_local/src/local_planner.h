#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/odometry_helper_ros.h>


#include <costmap_2d/costmap_2d_ros.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

// transforms
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


// costmap & geometry
#include <stdlib.h>
#include <time.h>
#include <list>


using namespace std;

namespace local_planner{

  class LocalPlanner : public nav_core::BaseLocalPlanner{

    public:
      LocalPlanner();

      LocalPlanner(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      ~LocalPlanner();

      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);


      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();
       
      void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

      double getYawAmcl(geometry_msgs::PoseWithCovarianceStamped msg);

      void goalCallback(const move_base_msgs::MoveBaseActionGoal::Ptr& msg);

      double getYawGoal(move_base_msgs::MoveBaseActionGoal msg);

      void getPlan();
      
      double getYawPlan();
      
      void getVelZ(geometry_msgs::Twist& cmd_vel);

      void getVelX(geometry_msgs::Twist& cmd_vel);

      void addMarker();


    private:
      bool initialized_;
      bool reached;
      
      tf2_ros::Buffer* tf_;
      
      costmap_2d::Costmap2DROS* costmap_ros_;

      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      geometry_msgs::PoseStamped pozisyon;
      geometry_msgs::Twist cmd; // contains the velocity

      visualization_msgs::Marker marker;
      bool end_marker;

      int checker;
      int i = 0;
      
      float length;
      float lcount;
      float dist;
      float dist_end;

      ros::Subscriber amcl_sub;
      ros::Subscriber goal_sub;
      ros::Publisher marker_pub;

      float pose_a_x;
      float pose_a_y;
      float pose_a_z;

      double a[4];
      double g[4];
      double p[4];
      double p_end[4];

      float pose_g_x;
      float pose_g_y;
      float pose_g_z;

      float pose_p_x;
      float pose_p_y;
      float pose_p_z;
      float pose_p_x_end;
      float pose_p_y_end;
      float pose_p_z_end;

      float vel_x; 
      float vel_z;
      float vel_zero;
  };
};

#endif