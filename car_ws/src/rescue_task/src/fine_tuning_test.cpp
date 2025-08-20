#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <cmath>
#include <string>

class FineTuningTestNode {
public:
    FineTuningTestNode() : nh_("~") {
        loadParameters();

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        tag_sub_ = nh_.subscribe("/tag_detections", 1, &FineTuningTestNode::tagCallback, this);

        in_view_ = false;
        last_seen_time_ = ros::Time(0);
        state_ = "search";

        ROS_INFO("========== 自动泊车测试节点启动 ==========");
        ROS_INFO("目标AprilTag ID: %d", target_tag_id_);
        ROS_INFO("流程: 旋转搜索 → 对准并前进至目标距离 → 停车完成");
    }

    void run() {
        ros::Rate rate(control_rate_);
        while (ros::ok()) {
            ros::spinOnce();
            stateMachine();
            rate.sleep();
        }
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber tag_sub_;

    // 状态
    std::string state_;
    bool in_view_;
    ros::Time last_seen_time_;

    // 最新检测（相机坐标系，单位: m）
    double saved_tag_x_ = 0.0; // 左右偏移（+ 右，- 左）
    double saved_tag_z_ = 0.0; // 前后距离（+ 前方）

    // 参数
    int control_rate_;
    int target_tag_id_;

    double linear_kp_;
    double angular_kp_;

    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;

    double stop_distance_z_;
    double x_tolerance_;
    double z_tolerance_;

    double search_angular_speed_;
    double lost_timeout_sec_;
   
    // 目标设定点（允许对准非图像中心的横向目标）
    double target_x_;

    // 误差补偿参数
    double x_bias_m_;                // X测量偏置（m），正值向右
    double z_bias_m_;                // Z测量偏置（m），正值向前
    double linear_feedforward_;      // 线速度前馈（m/s）
    double angular_feedforward_;     // 角速度前馈（rad/s）

    void loadParameters() {
        nh_.param("control_rate", control_rate_, 20); // Hz

        nh_.param("target_tag_id", target_tag_id_, 1);

        nh_.param("linear_kp", linear_kp_, 0.8);
        nh_.param("angular_kp", angular_kp_, 2.0);

        nh_.param("max_linear_speed", max_linear_speed_, 0.2);
        nh_.param("min_linear_speed", min_linear_speed_, 0.05);
        nh_.param("max_angular_speed", max_angular_speed_, 1.0);

        nh_.param("stop_distance_z", stop_distance_z_, 0.30);
        nh_.param("x_tolerance", x_tolerance_, 0.03);
        nh_.param("z_tolerance", z_tolerance_, 0.03);
        nh_.param("target_x", target_x_, 0.0); // 横向对准目标（m），默认瞄准相机中心

        nh_.param("search_angular_speed", search_angular_speed_, 0.0); // 静止等待，不旋转
        nh_.param("lost_timeout_sec", lost_timeout_sec_, 1.0);

        // 误差补偿参数
        nh_.param("x_bias", x_bias_m_, 0.0);
        nh_.param("z_bias", z_bias_m_, 0.0);
        nh_.param("linear_feedforward", linear_feedforward_, 0.0);
        nh_.param("angular_feedforward", angular_feedforward_, 0.0);
    }

    void robotStop() {
        geometry_msgs::Twist cmd; cmd.linear.x = 0.0; cmd.angular.z = 0.0; cmd_vel_pub_.publish(cmd);
    }

    double clamp(double value, double min_val, double max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        bool found = false;
        for (const auto& det : msg->detections) {
            if (!det.id.empty()) {
                for (const auto& id : det.id) {
                    if (id == target_tag_id_) {
                        // ROS1 apriltag_ros: 位置在 pose.pose.pose.position
                        saved_tag_x_ = det.pose.pose.pose.position.x;
                        saved_tag_z_ = det.pose.pose.pose.position.z;
                        found = true;
                        break;
                    }
                }
            }
            if (found) break;
        }
        in_view_ = found;
        if (found) {
            last_seen_time_ = ros::Time::now();
        }
    }

    void stateMachine() {
        if (state_ == "search") {
            if (in_view_) {
                ROS_INFO("检测到目标Tag(id=%d)，进入对准前进阶段", target_tag_id_);
                state_ = "approach";
                robotStop();
                return;
            }
            robotStop();
            ROS_INFO_THROTTLE(1.0, "搜索中: 静止等待检测到目标Tag...");
            return;
        }

        if (state_ == "approach") {
            // 丢失检测回退搜索
            if (!in_view_ && (ros::Time::now() - last_seen_time_).toSec() > lost_timeout_sec_) {
                ROS_WARN("目标丢失超过%.1fs，回退至搜索", lost_timeout_sec_);
                state_ = "search";
                robotStop();
                return;
            }

            // 测量补偿 & 误差计算（以相机坐标系为准）
            const double x_meas = saved_tag_x_ + x_bias_m_;
            const double z_meas = saved_tag_z_ + z_bias_m_;
            double x_error = x_meas - target_x_; // 目标横向应为 target_x_
            double z_error = z_meas - stop_distance_z_; // 目标距离应为 stop_distance_z

            bool x_ok = std::fabs(x_error) <= x_tolerance_;
            bool z_ok = std::fabs(z_error) <= z_tolerance_;

            if (x_ok && z_ok) {
                robotStop();
                ROS_INFO("泊车完成: x_err=%.3f(m) z_err=%.3f(m) 目标距离=%.2f(m)", x_error, z_error, stop_distance_z_);
                state_ = "done";
                return;
            }

            geometry_msgs::Twist cmd;

            // 角速度：根据横向偏差闭环 + 前馈补偿
            double ang = -x_error * angular_kp_;
            if (std::fabs(ang) > 1e-6 && angular_feedforward_ > 0.0) {
                ang += (ang >= 0.0 ? angular_feedforward_ : -angular_feedforward_);
            }
            ang = clamp(ang, -max_angular_speed_, max_angular_speed_);
            cmd.angular.z = ang;

            // 线速度：根据距离误差前进 + 前馈补偿（只前进不倒车）
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
            ROS_INFO_THROTTLE(0.5, "对准前进: x=%.3f z=%.3f | x_err=%.3f z_err=%.3f | v=%.2f w=%.2f",
                              x_meas, z_meas, x_error, z_error, cmd.linear.x, cmd.angular.z);
            return;
        }

        if (state_ == "done") {
            robotStop();
            return;
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "fine_tuning_test");
    FineTuningTestNode node;
    node.run();
    return 0;
} 