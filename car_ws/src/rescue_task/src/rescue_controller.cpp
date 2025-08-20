#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatus.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>

// 状态机枚举
enum class RescueState {
    NAVIGATING,    // 导航到目标点
    SEARCHING,     // 原地旋转搜索AprilTag
    APPROACHING,   // 靠近目标物体
    FINAL_APPROACH,// 最终接近状态：等待-前进-等待-抓取
    GRABBING,      // 抓取并返回起点
    FINISHED       // 任务完成
};

class RescueController {
public:
    RescueController() : nh_("~"), move_base_client_("move_base", true) {
        // 加载参数
        loadParameters();
        
        // 初始化状态
        current_state_ = RescueState::NAVIGATING;
        current_target_index_ = 0;
        tag_detected_ = false;
        tag_ever_detected_ = false;
        last_tag_time_ = ros::Time(0);
        search_start_time_ = ros::Time(0);
        approach_completed_ = false;
        final_approach_start_ = ros::Time(0);
        final_approach_phase_ = 0;
        
        // 初始化发布者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        grab_pub_ = nh_.advertise<std_msgs::Int32>("/grab", 1);
        
        // 初始化订阅者
        tag_sub_ = nh_.subscribe("/tag_detections", 1, &RescueController::tagCallback, this);
        
        // 初始化TF监听器
        tf_listener_ = new tf::TransformListener();
        
        // 等待move_base服务器
        ROS_INFO("等待move_base action server...");
        move_base_client_.waitForServer();
        ROS_INFO("move_base action server已连接");
        
        ROS_INFO("========== 救援控制器启动 ==========");
        ROS_INFO("目标AprilTag ID: %d", target_tag_id_);
        ROS_INFO("共有 %zu 个目标点", target_poses_.size());
    }
    
    ~RescueController() {
        delete tf_listener_;
    }
    
    void run() {
        ros::Rate rate(control_rate_);
        
        // 开始导航到第一个目标点
        if (!target_poses_.empty()) {
            navigateToTarget(0);
        } else {
            ROS_ERROR("没有配置目标点，退出");
            return;
        }
        
        while (ros::ok()) {
            ros::spinOnce();
            stateMachine();
            rate.sleep();
        }
    }

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher grab_pub_;
    ros::Subscriber tag_sub_;
    tf::TransformListener* tf_listener_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    
    // 状态变量
    RescueState current_state_;
    int current_target_index_;
    bool tag_detected_;
    bool tag_ever_detected_;
    bool approach_completed_;
    ros::Time last_tag_time_;
    ros::Time search_start_time_;
    ros::Time final_approach_start_;
    int final_approach_phase_;
    
    // AprilTag检测数据
    double saved_tag_x_ = 0.0;
    double saved_tag_z_ = 0.0;
    
    // 参数
    int control_rate_;
    geometry_msgs::Pose start_pose_;
    std::vector<geometry_msgs::Pose> target_poses_;
    
    // 搜索参数
    double search_angular_speed_;
    int search_direction_;  // 1=顺时针, -1=逆时针
    double search_timeout_;
    
    // AprilTag相关参数
    int target_tag_id_;
    double lost_timeout_sec_;
    
    // PID控制参数（从fine_tuning_test.cpp移植）
    double linear_kp_;
    double angular_kp_;
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double stop_distance_z_;
    double x_tolerance_;
    double z_tolerance_;
    double target_x_;
    double x_bias_m_;
    double z_bias_m_;
    double linear_feedforward_;
    double angular_feedforward_;
    
    // 抓取参数
    double grab_wait_time_;
    double approach_wait_time_;
    
    // 最终接近参数
    double forward_distance_;
    double final_approach_speed_;
    
    void loadParameters() {
        // 基本参数
        nh_.param("control_rate", control_rate_, 20);
        
        // 加载起始位置
        loadPoseParameter("start_pose", start_pose_);
        
        // 加载目标位置列表
        loadPoseListParameter("target_poses", target_poses_);
        
        // 搜索参数
        nh_.param("search_angular_speed", search_angular_speed_, 0.3);
        nh_.param("search_direction", search_direction_, 1);
        nh_.param("search_timeout", search_timeout_, 10.0);
        
        // AprilTag参数
        nh_.param("target_tag_id", target_tag_id_, 1);
        nh_.param("lost_timeout_sec", lost_timeout_sec_, 1.0);
        
        // PID控制参数
        nh_.param("linear_kp", linear_kp_, 0.8);
        nh_.param("angular_kp", angular_kp_, 2.0);
        nh_.param("max_linear_speed", max_linear_speed_, 0.2);
        nh_.param("min_linear_speed", min_linear_speed_, 0.05);
        nh_.param("max_angular_speed", max_angular_speed_, 1.0);
        nh_.param("stop_distance_z", stop_distance_z_, 0.30);
        nh_.param("x_tolerance", x_tolerance_, 0.03);
        nh_.param("z_tolerance", z_tolerance_, 0.03);
        nh_.param("target_x", target_x_, 0.0);
        nh_.param("x_bias", x_bias_m_, 0.0);
        nh_.param("z_bias", z_bias_m_, 0.0);
        nh_.param("linear_feedforward", linear_feedforward_, 0.0);
        nh_.param("angular_feedforward", angular_feedforward_, 0.0);
        
        // 抓取参数
        nh_.param("grab_wait_time", grab_wait_time_, 3.0);
        nh_.param("approach_wait_time", approach_wait_time_, 3.0);
        
        // 最终接近参数
        nh_.param("forward_distance", forward_distance_, 0.15);
        nh_.param("final_approach_speed", final_approach_speed_, 0.1);
    }
    
    void loadPoseParameter(const std::string& param_name, geometry_msgs::Pose& pose) {
        XmlRpc::XmlRpcValue pose_param;
        if (nh_.getParam(param_name, pose_param)) {
            pose.position.x = static_cast<double>(pose_param["position"]["x"]);
            pose.position.y = static_cast<double>(pose_param["position"]["y"]);
            pose.position.z = static_cast<double>(pose_param["position"]["z"]);
            pose.orientation.x = static_cast<double>(pose_param["orientation"]["x"]);
            pose.orientation.y = static_cast<double>(pose_param["orientation"]["y"]);
            pose.orientation.z = static_cast<double>(pose_param["orientation"]["z"]);
            pose.orientation.w = static_cast<double>(pose_param["orientation"]["w"]);
        }
    }
    
    void loadPoseListParameter(const std::string& param_name, std::vector<geometry_msgs::Pose>& poses) {
        XmlRpc::XmlRpcValue poses_param;
        if (nh_.getParam(param_name, poses_param)) {
            for (int i = 0; i < poses_param.size(); i++) {
                geometry_msgs::Pose pose;
                pose.position.x = static_cast<double>(poses_param[i]["position"]["x"]);
                pose.position.y = static_cast<double>(poses_param[i]["position"]["y"]);
                pose.position.z = static_cast<double>(poses_param[i]["position"]["z"]);
                pose.orientation.x = static_cast<double>(poses_param[i]["orientation"]["x"]);
                pose.orientation.y = static_cast<double>(poses_param[i]["orientation"]["y"]);
                pose.orientation.z = static_cast<double>(poses_param[i]["orientation"]["z"]);
                pose.orientation.w = static_cast<double>(poses_param[i]["orientation"]["w"]);
                poses.push_back(pose);
            }
        }
    }
    
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        bool found = false;
        for (const auto& det : msg->detections) {
            if (!det.id.empty()) {
                for (const auto& id : det.id) {
                    if (id == target_tag_id_) {
                        saved_tag_x_ = det.pose.pose.pose.position.x;
                        saved_tag_z_ = det.pose.pose.pose.position.z;
                        found = true;
                        break;
                    }
                }
            }
            if (found) break;
        }
        
        tag_detected_ = found;
        if (found) {
            last_tag_time_ = ros::Time::now();
            tag_ever_detected_ = true;  // 标记曾经检测到过tag
        }
    }
    
    void navigateToTarget(int target_index) {
        if (target_index >= target_poses_.size()) {
            ROS_ERROR("目标索引超出范围: %d >= %zu", target_index, target_poses_.size());
            return;
        }
        
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = target_poses_[target_index];
        
        move_base_client_.sendGoal(goal);
        ROS_INFO("导航到目标点 %d: x=%.2f, y=%.2f", 
                 target_index, 
                 target_poses_[target_index].position.x, 
                 target_poses_[target_index].position.y);
    }
    
    void navigateToStart() {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = start_pose_;
        
        move_base_client_.sendGoal(goal);
        ROS_INFO("返回起点: x=%.2f, y=%.2f", start_pose_.position.x, start_pose_.position.y);
    }
    
    bool checkNavigationCompleted() {
        actionlib::SimpleClientGoalState state = move_base_client_.getState();
        return state == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    
    void stopRobot() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }
    
    void searchForTag() {
        // 检查是否检测到AprilTag
        if (tag_detected_) {
            ROS_INFO("检测到目标Tag(id=%d)，停止搜索", target_tag_id_);
            stopRobot();
            current_state_ = RescueState::APPROACHING;
            approach_completed_ = false;
            return;
        }
        
        // 检查搜索超时
        ros::Time now = ros::Time::now();
        if (search_start_time_.isZero()) {
            search_start_time_ = now;
            ROS_INFO("开始搜索AprilTag，%s旋转", search_direction_ > 0 ? "顺时针" : "逆时针");
        }
        
        if ((now - search_start_time_).toSec() > search_timeout_) {
            ROS_WARN("搜索超时，前往下一个目标点");
            stopRobot();
            search_start_time_ = ros::Time(0);
            
            // 移动到下一个目标点
            current_target_index_++;
            if (current_target_index_ < target_poses_.size()) {
                navigateToTarget(current_target_index_);
                current_state_ = RescueState::NAVIGATING;
            } else {
                // 所有目标点都搜索完毕，返回起点
                ROS_INFO("所有目标点搜索完毕，返回起点");
                navigateToStart();
                current_state_ = RescueState::NAVIGATING;
                current_target_index_ = -1; // 标记为返回起点
            }
            return;
        }
        
        // 执行旋转搜索
        geometry_msgs::Twist cmd;
        cmd.angular.z = search_angular_speed_ * search_direction_;
        cmd_vel_pub_.publish(cmd);
        
        ROS_INFO_THROTTLE(2.0, "搜索中... (%.1fs/%.1fs)", 
                         (now - search_start_time_).toSec(), search_timeout_);
    }
    
    double clamp(double value, double min_val, double max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
    
    void approachTarget() {
        // 检查目标丢失 - 如果曾经检测到tag，即使丢失也直接进入最终接近
        if (!tag_detected_ && tag_ever_detected_) {
            ROS_INFO("Tag已丢失但曾经检测到，进入最终接近");
            current_state_ = RescueState::FINAL_APPROACH;
            final_approach_start_ = ros::Time::now();
            final_approach_phase_ = 0;
            stopRobot();
            return;
        }
        
        // 如果从未检测到tag且超时，重新搜索
        if (!tag_detected_ && 
            !last_tag_time_.isZero() && 
            (ros::Time::now() - last_tag_time_).toSec() > lost_timeout_sec_) {
            ROS_WARN("目标丢失超过%.1fs，重新搜索", lost_timeout_sec_);
            current_state_ = RescueState::SEARCHING;
            search_start_time_ = ros::Time(0);
            stopRobot();
            return;
        }
        
        if (!tag_detected_) {
            stopRobot();
            return;
        }
        
        // 应用偏置补偿
        const double x_meas = saved_tag_x_ + x_bias_m_;
        const double z_meas = saved_tag_z_ + z_bias_m_;
        double x_error = x_meas - target_x_;
        double z_error = z_meas - stop_distance_z_;
        
        bool x_ok = std::fabs(x_error) <= x_tolerance_;
        bool z_ok = std::fabs(z_error) <= z_tolerance_;
        
        if (x_ok && z_ok) {
            stopRobot();
            if (!approach_completed_) {
                ROS_INFO("靠近完成: x_err=%.3f(m) z_err=%.3f(m) 目标距离=%.2f(m)", 
                         x_error, z_error, stop_distance_z_);
                approach_completed_ = true;
                current_state_ = RescueState::FINAL_APPROACH;
                final_approach_start_ = ros::Time::now();
                final_approach_phase_ = 0;
            }
            return;
        }
        
        // PID控制（从fine_tuning_test.cpp移植）
        geometry_msgs::Twist cmd;
        
        // 角速度控制
        double ang = -x_error * angular_kp_;
        if (std::fabs(ang) > 1e-6 && angular_feedforward_ > 0.0) {
            ang += (ang >= 0.0 ? angular_feedforward_ : -angular_feedforward_);
        }
        ang = clamp(ang, -max_angular_speed_, max_angular_speed_);
        cmd.angular.z = ang;
        
        // 线速度控制
        double lin = z_error * linear_kp_;
        if (lin < 0.0) lin = 0.0; // 不后退
        if (lin > 0.0 && linear_feedforward_ > 0.0) {
            lin += linear_feedforward_;
        }
        lin = clamp(lin, 0.0, max_linear_speed_);
        if (lin > 0.0 && lin < min_linear_speed_ && std::fabs(z_error) > z_tolerance_) {
            lin = min_linear_speed_;
        }
        cmd.linear.x = lin;
        
        cmd_vel_pub_.publish(cmd);
        ROS_INFO_THROTTLE(0.5, "靠近中: x=%.3f z=%.3f | x_err=%.3f z_err=%.3f | v=%.2f w=%.2f",
                         x_meas, z_meas, x_error, z_error, cmd.linear.x, cmd.angular.z);
    }
    
    void executeFinalApproach() {
        ros::Time now = ros::Time::now();
        double elapsed_time = (now - final_approach_start_).toSec();
        
        switch (final_approach_phase_) {
            case 0: // 第一次等待3秒
                if (elapsed_time >= 3.0) {
                    ROS_INFO("第一次等待完成，开始前进%.2fm", forward_distance_);
                    final_approach_phase_ = 1;
                    final_approach_start_ = now;  // 重置计时器
                }
                else {
                    ROS_INFO_THROTTLE(1.0, "第一次等待中... (%.1f/3.0s)", elapsed_time);
                }
                stopRobot();
                break;
                
            case 1: // 前进阶段
                {
                    // 计算需要的前进时间 = 距离 / 速度
                    double forward_time = forward_distance_ / final_approach_speed_;
                    
                    if (elapsed_time >= forward_time) {
                        ROS_INFO("前进完成，开始第二次等待");
                        final_approach_phase_ = 2;
                        final_approach_start_ = now;  // 重置计时器
                        stopRobot();
                    }
                    else {
                        // 继续前进
                        geometry_msgs::Twist cmd;
                        cmd.linear.x = final_approach_speed_;
                        cmd_vel_pub_.publish(cmd);
                        ROS_INFO_THROTTLE(1.0, "前进中... (%.1f/%.1fs)", elapsed_time, forward_time);
                    }
                }
                break;
                
            case 2: // 第二次等待3秒
                if (elapsed_time >= 3.0) {
                    ROS_INFO("第二次等待完成，开始抓取");
                    current_state_ = RescueState::GRABBING;
                    stopRobot();
                }
                else {
                    ROS_INFO_THROTTLE(1.0, "第二次等待中... (%.1f/3.0s)", elapsed_time);
                }
                stopRobot();
                break;
        }
    }
    
    void executeGrabbing() {
        static ros::Time grab_start_time;
        static bool grab_sent = false;
        static bool returning = false;
        
        if (!grab_sent) {
            // 等待一段时间后执行抓取
            ROS_INFO("等待 %.1f 秒后执行抓取", approach_wait_time_);
            ros::Duration(approach_wait_time_).sleep();
            
            // 发送抓取命令
            std_msgs::Int32 grab_msg;
            grab_msg.data = 1;
            grab_pub_.publish(grab_msg);
            ROS_INFO("已发送抓取命令 /grab = 1");
            
            grab_start_time = ros::Time::now();
            grab_sent = true;
            
            // 等待抓取完成
            ros::Duration(grab_wait_time_).sleep();
            ROS_INFO("抓取完成，开始返回起点");
            
            // 开始返回起点
            navigateToStart();
            returning = true;
            current_state_ = RescueState::NAVIGATING;
            current_target_index_ = -1; // 标记为返回起点
        }
    }
    
    void stateMachine() {
        switch (current_state_) {
            case RescueState::NAVIGATING:
                if (checkNavigationCompleted()) {
                    if (current_target_index_ == -1) {
                        // 返回起点完成
                        ROS_INFO("已返回起点");
                        
                        // 释放夹爪
                        ros::Duration(3.0).sleep();
                        std_msgs::Int32 grab_msg;
                        grab_msg.data = 0;
                        grab_pub_.publish(grab_msg);
                        ROS_INFO("已发送松开夹爪命令 /grab = 0");
                        
                        current_state_ = RescueState::FINISHED;
                    } else {
                        // 到达目标点，开始搜索
                        ROS_INFO("到达目标点 %d，开始搜索", current_target_index_);
                        current_state_ = RescueState::SEARCHING;
                        search_start_time_ = ros::Time(0);
                        tag_detected_ = false;
                    }
                }
                break;
                
            case RescueState::SEARCHING:
                searchForTag();
                break;
                
            case RescueState::APPROACHING:
                approachTarget();
                break;
                
            case RescueState::FINAL_APPROACH:
                executeFinalApproach();
                break;
                
            case RescueState::GRABBING:
                executeGrabbing();
                break;
                
            case RescueState::FINISHED:
                stopRobot();
                ROS_INFO_THROTTLE(5.0, "救援任务完成");
                break;
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "rescue_controller");
    
    RescueController controller;
    controller.run();
    
    return 0;
}