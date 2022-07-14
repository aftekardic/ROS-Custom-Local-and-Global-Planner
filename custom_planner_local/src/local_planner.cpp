#include "local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

        LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) 
        {
                this->initialized_ = false;
        }

        LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
                initialize(name, tf, costmap_ros);
                this->initialized_ = false;
         }

        LocalPlanner::~LocalPlanner() {}

        void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
        {
                this->costmap_ros_ = costmap_ros;
		this->tf_ = tf;


                ros::NodeHandle nh;
		this->amcl_sub = nh.subscribe("amcl_pose", 100, &LocalPlanner::amclCallback, this);
		this->goal_sub = nh.subscribe("move_base/goal", 100, &LocalPlanner::goalCallback, this);
                this->marker_pub = nh.advertise<visualization_msgs::Marker>("local_marker", 10);

                this->initialized_ = true;
                ROS_INFO("This planner has been initialized");
        }

        bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
        {

                if(!this->initialized_)
                {
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
                }

                this->plan = orig_global_plan; 
		this->length = this->plan.size();  
                this->i = 20;
                this->reached = false;

                this->vel_x = 0.2;
                this->vel_z = 0.3;
                this->vel_zero = 0.0;

                this->end_marker = false;

                return true;
        }

        bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
        {
               
                if(!this->initialized_)
                {
                    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                    return false;
                }
                
                if (this->reached)
                {
                        this->reached = false;
                        this->i = this->i + 5;
                }
                
                

                if (this->i < this->length)
                {                 
                        addMarker();
                        getPlan();
                        
                        this->dist = sqrt(pow((this->pose_p_x-this->pose_a_x), 2) + pow(this->pose_p_y-this->pose_a_y, 2));
                        ROS_INFO("Yaw Robot Z: %f  Yaw Plan Z: %f  Distance: %f  Length: %f", this->pose_a_z, this->pose_p_z, this->dist, this->length);
                        
                        
                        if (this->pose_p_z - 0.12 > this->pose_a_z)
                        {
                                // ROS_INFO("Z DONUS POSITIVE");
                                if (this->pose_a_z >= 2.9 && this->pose_a_z <= 0.12)
                                {
                                        if (this->dist < 0.5) 
                                        {
                                                cmd_vel.linear.x = this->vel_zero;
                                                this->reached = true;
                                        }
                                        else
                                        {
                                                cmd_vel.linear.x = 0.2;
                                                // ROS_INFO("X ILERLEME");
                                                if (this->dist >= 1.0)
                                                {
                                                        cmd_vel.linear.x = this->vel_zero;
                                                        cmd_vel.angular.z = this->vel_zero;
                                                        ROS_INFO("LUTFEN YENI BIR KONUM VERIN!");
                                                }
                                        }
                                }
                                else
                                {
                                        cmd_vel.angular.z = this->vel_z;
                                        cmd_vel.linear.x = this->vel_zero;
                                }
                        }
                        else if (this->pose_a_z >= this->pose_p_z + 0.12)
                        {
                                // ROS_INFO("Z DONUS NEGATIVE");
                                if (this->pose_a_z <= -2.9 && this->pose_a_z <= 0.12)
                                {
                                        this->pose_a_z = -3.13;
                                        cmd_vel.angular.z = this->vel_zero;

                                        if (this->dist < 0.5) 
                                        {
                                                ROS_INFO("reached degisiimmmmdeee");
                                                cmd_vel.linear.x = this->vel_zero;
                                                this->reached = true;
                                        }
                                        else
                                        {
                                                cmd_vel.linear.x = 0.2;
                                                // ROS_INFO("X ILERLEME");
                                                if (this->dist >= 1.0)
                                                {
                                                        cmd_vel.linear.x = this->vel_zero;
                                                        cmd_vel.angular.z = this->vel_zero;
                                                        ROS_INFO("LUTFEN YENI BIR KONUM VERIN!");
                                                }
                                        }
                                }
                                else
                                {
                                        cmd_vel.angular.z = (-1)*this->vel_z;
                                        cmd_vel.linear.x = this->vel_zero;
                                }
                        }
                        else
                        {
                                cmd_vel.angular.z = this->vel_zero;

                                if (this->dist < 0.5) 
                                {
                                        ROS_INFO("reached degisiimmmmdeee");
                                        cmd_vel.linear.x = this->vel_zero;
                                        this->reached = true;
                                }
                                else
                                {
                                        cmd_vel.linear.x = this->vel_x;
                                        // ROS_INFO("X ILERLEME");
                                        if (this->dist >= 1.0)
                                        {
                                                cmd_vel.linear.x = this->vel_zero;
                                                cmd_vel.angular.z = this->vel_zero;
                                                ROS_INFO("LUTFEN YENI BIR KONUM VERIN!");
                                        }
                                }
                        }
                        ROS_INFO("%d", this->i);
                }

                else
                {
                        addMarker();
                        this->dist_end = sqrt(pow((this->pose_p_x_end-this->pose_a_x), 2) + pow(this->pose_p_y_end-this->pose_a_y, 2));
                        
                        if (this->dist_end > 0.7 && this->dist_end < 1.0) 
                        {       
                                cmd_vel.linear.x = this->vel_x;
                                cmd_vel.angular.z = this->vel_zero;

                                ROS_INFO("Yaw Robot Z: %f  Yaw Plan Z: %f  Distance: %f  Length: %f", this->pose_a_z, this->pose_p_z_end, this->dist_end, this->length);

                                this->dist_end = sqrt(pow((this->pose_p_x_end-this->pose_a_x), 2) + pow(this->pose_p_y_end-this->pose_a_y, 2));
                                
                                if (this->dist >= 1.0)
                                {
                                        cmd_vel.linear.x = this->vel_zero;
                                        cmd_vel.angular.z = this->vel_zero;
                                        ROS_INFO("LUTFEN YENI BIR KONUM VERIN!");
                                }
                        }

                        else
                        {
                                cmd_vel.linear.x = this->vel_zero;
                                ROS_INFO("Yaw Robot Z: %f  Yaw Plan Z: %f  Distance: %f  Length: %f", this->pose_a_z, this->pose_p_z_end, this->dist_end, this->length);
                        
                                if (this->pose_p_z_end - 0.12 > this->pose_a_z)
                                {
                                        cmd_vel.angular.z = this->vel_z;
                                        cmd_vel.linear.x = this->vel_zero;
                                        ROS_INFO("Yaw Robot Z: %f  Yaw Plan Z: %f  Distance: %f  Length: %f", this->pose_a_z, this->pose_p_z_end, this->dist_end, this->length);
                                }       
                                else if (this->pose_a_z >= this->pose_p_z_end + 0.12)
                                {
                                        cmd_vel.angular.z = (-1)*this->vel_z;
                                        cmd_vel.linear.x = this->vel_zero;
                                        ROS_INFO("Yaw Robot Z: %f  Yaw Plan Z: %f  Distance: %f  Length: %f", this->pose_a_z, this->pose_p_z_end, this->dist_end, this->length);
                                }
                                else
                                {
                                        cmd_vel.linear.x = this->vel_zero;
                                        cmd_vel.angular.z = this->vel_zero;
                                        ROS_INFO("GOAL REACHED!");
                                }  
                        }
                }
                
                return true;

        }

        bool LocalPlanner::isGoalReached()
        {
                if(!this->initialized_)
                {
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                }

                return false;
        }

        void LocalPlanner::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
	{		
                // ROBOT POSE
		this->pose_a_x = msg->pose.pose.position.x;
                this->pose_a_x = round(this->pose_a_x * 100) / 100;

		this->pose_a_y = msg->pose.pose.position.y;
                this->pose_a_y = round(this->pose_a_y * 100) / 100;

		// this->pose_a_z = msg->pose.pose.orientation.z;
                this->pose_a_z = getYawAmcl(*msg);
                this->pose_a_z = round(this->pose_a_z * 100) / 100;
                // ROS_INFO("pose x: %f", this->pose_x);
                // ROS_INFO("pose y: %f", this->pose_y);
                // ROS_INFO("Robot Z: %f", this->pose_z);
        }

        double LocalPlanner::getYawAmcl(geometry_msgs::PoseWithCovarianceStamped msg)
	{

		
		this->a[0]= msg.pose.pose.orientation.x;
		this->a[1]= msg.pose.pose.orientation.y;
		this->a[2]= msg.pose.pose.orientation.z;
		this->a[3]= msg.pose.pose.orientation.w;

		double t3 = +2.0 * (a[3] * a[2] + a[0] * a[1]);
		double t4 = +1.0 - 2.0 * (a[1] * a[1] + a[2] * a[2]);  

		return std::atan2(t3, t4);
	}
 
        void LocalPlanner::getPlan()
        {
                this->pose_p_x = this->plan[this->i].pose.position.x;
                this->pose_p_x = round(this->pose_p_x * 100) / 100;
                
                this->pose_p_y = this->plan[this->i].pose.position.y;
                this->pose_p_y = round(this->pose_p_y * 100) / 100;

                
                this->p[0]= this->plan[this->i].pose.orientation.x;
		this->p[1]= this->plan[this->i].pose.orientation.y;
		this->p[2]= this->plan[this->i].pose.orientation.z;
		this->p[3]= this->plan[this->i].pose.orientation.w;

                double t3 = +2.0 * (p[3] * p[2] + p[0] * p[1]);
		double t4 = +1.0 - 2.0 * (p[1] * p[1] + p[2] * p[2]);  

                this->pose_p_z = std::atan2(t3, t4);
                this->pose_p_z = round(this->pose_p_z * 100) / 100;

                this->pose_p_x_end = this->plan[this->length-1].pose.position.x;
                this->pose_p_x_end = round(this->pose_p_x_end * 100) / 100;

                this->pose_p_y_end = this->plan[this->length-1].pose.position.y;
                this->pose_p_y_end = round(this->pose_p_y_end * 100) / 100;


                this->p_end[0]= this->plan[this->length-1].pose.orientation.x;
		this->p_end[1]= this->plan[this->length-1].pose.orientation.y;
		this->p_end[2]= this->plan[this->length-1].pose.orientation.z;
		this->p_end[3]= this->plan[this->length-1].pose.orientation.w;

                double t3_end = +2.0 * (p_end[3] * p_end[2] + p_end[0] * p_end[1]);
		double t4_end = +1.0 - 2.0 * (p_end[1] * p_end[1] + p_end[2] * p_end[2]);  

                this->pose_p_z_end = std::atan2(t3_end, t4_end);
                this->pose_p_z_end = round(this->pose_p_z_end * 100) / 100;

        }

        
        void LocalPlanner::goalCallback(const move_base_msgs::MoveBaseActionGoal::Ptr& msg)
	{		
                // GOAL POSE
		this->pose_g_x = msg->goal.target_pose.pose.position.x;
		this->pose_g_y = msg->goal.target_pose.pose.position.y;
		
                this->pose_g_z = getYawGoal(*msg);
                this->pose_g_z = round(this->pose_g_z * 100) / 100;

                // ROS_INFO("pose x: %f", this->pose_m_x);
                // ROS_INFO("pose y: %f", this->pose_m_y);
                // ROS_INFO("Goal Z: %f", this->pose_m_z);
        }

        double LocalPlanner::getYawGoal(move_base_msgs::MoveBaseActionGoal msg)
	{

		
		this->g[0]= msg.goal.target_pose.pose.orientation.x;
		this->g[1]= msg.goal.target_pose.pose.orientation.y;
		this->g[2]= msg.goal.target_pose.pose.orientation.z;
		this->g[3]= msg.goal.target_pose.pose.orientation.w;

		double t3 = +2.0 * (g[3] * g[2] + g[0] * g[1]);
		double t4 = +1.0 - 2.0 * (g[1] * g[1] + g[2] * g[2]);  

		return std::atan2(t3, t4);
	}

        void LocalPlanner::addMarker()
        {       
                if(this->i >= this->length)
                {
                        this->marker.header.frame_id = "map";
                        this->marker.header.stamp = ros::Time();
                        this->marker.ns = "my_namespace";
                        this->marker.id = 0;
                        this->marker.type = visualization_msgs::Marker::ARROW;
                        this->marker.action = visualization_msgs::Marker::ADD;
                        this->marker.pose.position.x = this->pose_p_x_end;
                        this->marker.pose.position.y = this->pose_p_y_end;
                        this->marker.pose.position.z = 0.1;
                        this->marker.pose.orientation.x = this->p_end[0];
                        this->marker.pose.orientation.y = this->p_end[1];
                        this->marker.pose.orientation.z = this->p_end[2];
                        this->marker.pose.orientation.w = this->p_end[3];
                        this->marker.scale.x = 0.7;
                        this->marker.scale.y = 0.1;
                        this->marker.scale.z = 0.1;
                        this->marker.color.a = 1.0;
                        this->marker.color.r = 0.0;
                        this->marker.color.g = 1.0;
                        this->marker.color.b = 0.0;   
                }
                else
                {
                        this->marker.header.frame_id = "map";
                        this->marker.header.stamp = ros::Time();
                        this->marker.ns = "my_namespace";
                        this->marker.id = 0;
                        this->marker.type = visualization_msgs::Marker::ARROW;
                        this->marker.action = visualization_msgs::Marker::ADD;
                        this->marker.pose.position.x = this->pose_p_x;
                        this->marker.pose.position.y = this->pose_p_y;
                        this->marker.pose.position.z = 0.1;
                        this->marker.pose.orientation.x = this->p[0];
                        this->marker.pose.orientation.y = this->p[1];
                        this->marker.pose.orientation.z = this->p[2];
                        this->marker.pose.orientation.w = this->p[3];
                        this->marker.scale.x = 0.7;
                        this->marker.scale.y = 0.1;
                        this->marker.scale.z = 0.1;
                        this->marker.color.a = 1.0;
                        this->marker.color.r = 0.0;
                        this->marker.color.g = 0.0;
                        this->marker.color.b = 1.0;
                }
                marker_pub.publish(this->marker); 
                
        }
}