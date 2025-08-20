#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    // PD control variables
    double prev_lat_err_ = 0.0;
    double prev_yaw_err_ = 0.0;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("自定义局部规划器初始化");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        prev_lat_err_ = 0.0;
        prev_yaw_err_ = 0.0;
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // 获取代价地图的数据
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        for (unsigned int y = 0; y < size_y; y++)
        {
            for (unsigned int x = 0; x < size_x; x++)
            {
                int map_index = y * size_x + x;	//在一维数组中的下标
                unsigned char cost = map_data[map_index];               // 从代价地图数据取值
            }
        }

        // 在代价地图上遍历导航路径点
        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("odom_combined",global_plan_[i],pose_odom);
            double odom_x = pose_odom.pose.position.x;
            double odom_y = pose_odom.pose.position.y;

            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x;
            double local_y = odom_y - origin_y;
            int x = local_x / costmap->getResolution();
            int y = local_y / costmap->getResolution();

            // 检测前方路径点是否在禁行区域或者障碍物里
            if(i >= target_index_ && i < target_index_ + 10)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                if(cost >= 253)
                    return false;
            }
        }

        // 计算最终目标点
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);
        if(pose_adjusting_ == false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.05)
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("调整最终姿态，final_yaw = %.2f",final_yaw);
            cmd_vel.linear.x = pose_final.pose.position.x * 0.5;
            cmd_vel.angular.z = final_yaw * 0.2;
            if(abs(final_yaw) < 0.1)
            {
                goal_reached_ = true;
                ROS_WARN("到达终点！");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            return true;
        }

        geometry_msgs::PoseStamped target_pose;
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist > 0.2) 
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                break;
            }

            if(i == global_plan_.size()-1)
                target_pose = pose_base; 
        }

        // --- PD controller for omni-wheel robot ---
        // PID gains - you can tune these further
        double Kp_lat = 0.65;
        double Kd_lat = 0.15;
        double Kp_yaw = 0.35;
        double Kd_yaw = 0.15;
        double const_v = 0.16; // m/s, desired forward velocity

        // Calculate errors
        // Lateral error is the y-distance to the target point in the base_link frame
        double lat_err = target_pose.pose.position.y;
        // Yaw error is the angle to the target point
        double yaw_err = atan2(target_pose.pose.position.y, target_pose.pose.position.x);

        // Simple fixed-time derivative (assumes a constant control rate)
        // A more robust implementation would use ros::Time to get the actual dt
        double dt = 0.05; 

        // Lateral PD control (for cmd_vel.linear.y)
        double lat_err_d = (lat_err - prev_lat_err_) / dt;
        cmd_vel.linear.y = Kp_lat * lat_err + Kd_lat * lat_err_d;

        // Yaw PD control (for cmd_vel.angular.z)
        double yaw_err_d = (yaw_err - prev_yaw_err_) / dt;
        cmd_vel.angular.z = Kp_yaw * yaw_err + Kd_yaw * yaw_err_d;

        // Update previous errors for the next cycle
        prev_lat_err_ = lat_err;
        prev_yaw_err_ = yaw_err;
        
        // Set a constant forward velocity
        cmd_vel.linear.x = const_v;

        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
} // namespace my_planner
