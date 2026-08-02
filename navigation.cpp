// ==================== 宏清理（放置在所有 include 之前） ====================
#ifdef size
#undef size
#endif
#ifdef new
#undef new
#endif
#include <new>

// ==================== 标准库头文件 ====================
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <cmath>
#include <queue>
#include <atomic>
#include <chrono>
#include <cstdlib>
// ==================== ROS 头文件 ====================
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 航点结构体（增加yaw）
struct Waypoint
{
    std::string name;
    double x;
    double y;
    double yaw; // 目标朝向（弧度）
};

class NavigationController
{
public:
    void bowPathForRegion1();
    void bowPathForRegion2();
    void bowPathForRegion3();
    void bowPathForRegion4();  // 新增：区域4清扫

    NavigationController(double wait_between = 0.2, double goal_timeout = 20.0)
        : move_base_("move_base", true),
          is_navigating_(false),
          current_x_(0.0),
          current_y_(0.0),
          current_yaw_(0.0),
          wait_between_(wait_between),
          goal_timeout_(goal_timeout),
          key_(""),
          xy_tolerance_(0.15),
          max_retries_(2),
          recovery_attempts_(0),
          detected_tag_id_(-1),
          start_x_(0.0),
          start_y_(0.0),
          start_yaw_(0.0),
          reverse_param_modified_(false),
          backup_min_vel_x_(0.0),
          backup_escape_vel_(-0.1),
          backup_traj_min_vel_x_(0.0)
    {
        ROS_INFO("Waiting for move_base action server...");
        while (!move_base_.waitForServer(ros::Duration(60)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Connected to move_base");

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ros::Duration(1.0).sleep();
        ROS_INFO("[INIT] cmd_vel publisher created, subscribers: %d", cmd_vel_pub_.getNumSubscribers());

        odom_sub_ = nh_.subscribe("/odom", 10, &NavigationController::odomCallback, this);
        key_sub_ = nh_.subscribe("key_input", 10, &NavigationController::keyCallback, this);
        trigger_sub_ = nh_.subscribe("auto_slam", 10, &NavigationController::triggerCallback, this);
        apriltag_sub_ = nh_.subscribe("/apriltag_id", 10, &NavigationController::apriltagCallback, this);

        initWaypoints();
        initRegionPathPoints();
        recordStartPosition();
    }

    void initWaypoints()
    {
        Waypoint wp1;
        wp1.name = "Region 1";
        wp1.x = 1.91;
        wp1.y = 0.60;
        wp1.yaw = -1.5708;
        waypoints_.push_back(wp1);

        Waypoint wp2;
        wp2.name = "Region 2";
        wp2.x = 1.85;
        wp2.y = 1.84;
        wp2.yaw = 0.0;
        waypoints_.push_back(wp2);

        Waypoint wp3;
        wp3.name = "Region 3";
        wp3.x = 0.69;
        wp3.y = 1.68;
        wp3.yaw = 1.558;
        waypoints_.push_back(wp3);

        Waypoint wp4;
        wp4.name = "Region 4";
        wp4.x = 0.04;
        wp4.y = -0.15;
        wp4.yaw = 0.0;
        waypoints_.push_back(wp4);

        center_point_.name = "Center Point";
        center_point_.x = 1.15;
        center_point_.y = 1.10;
        center_point_.yaw = 0.0;
    }

    // 初始化四区域的5点路径（所有点yaw与各自起点一致）
    void initRegionPathPoints()
    {
        region1_path_points_.clear();
        region2_path_points_.clear();
        region3_path_points_.clear();
        region4_path_points_.clear();  // 新增

        auto mk = [](const std::string &name, double x, double y, double yaw) {
            Waypoint p;
            p.name = name;
            p.x = x;
            p.y = y;
            p.yaw = yaw;
            return p;
        };

        // Region1
        double y1 = waypoints_[0].yaw;
        region1_path_points_.push_back(mk("R1_P1", 1.87, 0.05, y1));
        region1_path_points_.push_back(mk("R1_P2", 2.22, 0.64, y1));
        region1_path_points_.push_back(mk("R1_P3", 2.22, 0.05, y1));
        region1_path_points_.push_back(mk("R1_P4", 2.4, 0.62, y1));
        region1_path_points_.push_back(mk("R1_P5", 2.4, 0.06, y1));

        // Region2
        double y2 = waypoints_[1].yaw;
        region2_path_points_.push_back(mk("R2_P1", 2.3847, 1.8157, y2));
        region2_path_points_.push_back(mk("R2_P2", 1.8501, 2.1042, y2));
        region2_path_points_.push_back(mk("R2_P3", 2.4797, 1.9996, y2));
        region2_path_points_.push_back(mk("R2_P4", 1.8113, 2.3152, y2));
        region2_path_points_.push_back(mk("R2_P5", 2.4080, 2.3148, y2));

        // Region3
        double y3 = waypoints_[2].yaw;
        region3_path_points_.push_back(mk("R3_P1", 0.69,    2.2,  y3));
        region3_path_points_.push_back(mk("R3_P2", 0.43627, 1.68, y3));
        region3_path_points_.push_back(mk("R3_P3", 0.43627, 2.22,  y3));
        region3_path_points_.push_back(mk("R3_P4", 0.20447, 1.68, y3));
        region3_path_points_.push_back(mk("R3_P5", 0.20447, 2.22,  y3));

        // Region4 (新增)
        double y4 = waypoints_[3].yaw;
        region4_path_points_.push_back(mk("R4_P1", 0.04, -0.15, y4));
        region4_path_points_.push_back(mk("R4_P2", -0.3, -0.15, y4));
        region4_path_points_.push_back(mk("R4_P3", 0.04, -0.5, y4));
        region4_path_points_.push_back(mk("R4_P4", -0.3, -0.5, y4));
        region4_path_points_.push_back(mk("R4_P5", 0.04, -0.15, y4));

        ROS_INFO("[PATH_INIT] Region1 path points: %zu", region1_path_points_.size());
        ROS_INFO("[PATH_INIT] Region2 path points: %zu", region2_path_points_.size());
        ROS_INFO("[PATH_INIT] Region3 path points: %zu", region3_path_points_.size());
        ROS_INFO("[PATH_INIT] Region4 path points: %zu", region4_path_points_.size());
    }

    void recordStartPosition()
    {
        start_x_ = 0.0;
        start_y_ = 0.0;
        start_yaw_ = 0.0;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_yaw_ = tf::getYaw(q);
    }

    void keyCallback(const std_msgs::String::ConstPtr &msg)
    {
        key_ = msg->data;
        ROS_INFO("Received key input: %s", key_.c_str());
    }

    void triggerCallback(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("Received navigation trigger: %s", msg->data.c_str());
        if (!is_navigating_)
        {
            std::thread nav_thread(&NavigationController::startNavigation, this);
            nav_thread.detach();
        }
        else
        {
            ROS_INFO("Navigation is already in progress, ignoring new request");
        }
    }

    void apriltagCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        detected_tag_id_ = msg->data;
        ROS_INFO_THROTTLE(1, "[Apriltag] Detected ID = %d", detected_tag_id_.load());
    }

    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    bool isWithinTolerance(const Waypoint &wp)
    {
        return calculateDistance(current_x_, current_y_, wp.x, wp.y) <= xy_tolerance_;
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    bool alignToTargetYaw(double target_yaw, double tolerance_deg = 3.0)
    {
        ros::spinOnce();
        double yaw_error = normalizeAngle(target_yaw - current_yaw_);
        double tolerance_rad = tolerance_deg * M_PI / 180.0;
        if (std::fabs(yaw_error) <= tolerance_rad) return true;

        rotateRelativeByAngle(yaw_error * 180.0 / M_PI, 0.6);
        ros::Duration(0.05).sleep();
        ros::spinOnce();

        yaw_error = normalizeAngle(target_yaw - current_yaw_);
        return std::fabs(yaw_error) <= tolerance_rad;
    }

    void clearCostmaps()
    {
        ROS_WARN("Clearing costmaps to recover...");
        system("rosservice call /move_base/clear_costmaps \"{}\"");
        ros::Duration(0.3).sleep();
    }

    void rotateRecovery()
    {
        rotateByTime(0.5, 2.0);
        rotateByTime(-0.5, 1.0);
        ros::Duration(0.3).sleep();
        ros::spinOnce();
    }

    void rotateByTime(double angular_speed, double duration)
    {
        geometry_msgs::Twist twist;
        twist.angular.z = angular_speed;
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(20);

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < duration)
        {
            cmd_vel_pub_.publish(twist);
            rate.sleep();
            ros::spinOnce();
        }

        twist.angular.z = 0.0;
        cmd_vel_pub_.publish(twist);
        ros::Duration(0.1).sleep();
    }

    void rotateRelativeByAngle(double angle_deg, double angular_speed = 0.4)
    {
        double angle_rad = std::fabs(angle_deg) * M_PI / 180.0;
        double duration = angle_rad / std::fabs(angular_speed);
        double speed = (angle_deg > 0) ? std::fabs(angular_speed) : -std::fabs(angular_speed);

        geometry_msgs::Twist twist;
        twist.angular.z = speed;

        ros::Time start_time = ros::Time::now();
        ros::Rate rate(20);

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < duration)
        {
            cmd_vel_pub_.publish(twist);
            rate.sleep();
            ros::spinOnce();
        }

        twist.angular.z = 0.0;
        cmd_vel_pub_.publish(twist);
        ros::Duration(0.1).sleep();
    }

    bool navigateToPointSimple(const Waypoint &wp, const std::string &context)
    {
        ROS_INFO("[%s] Navigating to %s (%.3f, %.3f) yaw=%.3f",
                 context.c_str(), wp.name.c_str(), wp.x, wp.y, wp.yaw);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = wp.x;
        goal.target_pose.pose.position.y = wp.y;
        goal.target_pose.pose.position.z = 0.0;

        tf::Quaternion q = tf::createQuaternionFromYaw(wp.yaw);
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        move_base_.sendGoal(goal);

        ros::Time start_time = ros::Time::now();
        ros::Rate check_rate(5);

        while (ros::ok())
        {
            if ((ros::Time::now() - start_time).toSec() > goal_timeout_)
            {
                move_base_.cancelGoal();
                break;
            }

            if (isWithinTolerance(wp))
            {
                move_base_.cancelGoal();
                return true;
            }

            actionlib::SimpleClientGoalState state = move_base_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
            if (state == actionlib::SimpleClientGoalState::ABORTED ||
                state == actionlib::SimpleClientGoalState::REJECTED ||
                state == actionlib::SimpleClientGoalState::RECALLED)
            {
                move_base_.cancelGoal();
                break;
            }

            ros::spinOnce();
            check_rate.sleep();
        }
        return false;
    }

    bool navigateToPoint(const Waypoint &wp, int index, int total)
    {
        ROS_INFO("Navigating to %s (%d/%d)", wp.name.c_str(), index + 1, total);

        for (int retry = 0; retry <= max_retries_; ++retry)
        {
            if (retry > 0)
            {
                ROS_WARN("Retry %d/%d for %s", retry, max_retries_, wp.name.c_str());
                if (retry == 1) clearCostmaps();
                else if (retry == 2) { rotateRecovery(); clearCostmaps(); }
            }

            if (navigateToPointSimple(wp, "RetryNav")) return true;
            if (isWithinTolerance(wp)) return true;
        }
        return false;
    }

    bool navigateToPointNoYaw(const Waypoint &wp, const std::string &context)
    {
        Waypoint tmp = wp;
        tmp.yaw = current_yaw_;
        return navigateToPointSimple(tmp, context);
    }

    bool navigateViaCenter(const Waypoint &target)
    {
        if (!navigateToPointSimple(center_point_, "Center")) return false;
        ros::Duration(0.1).sleep();
        return navigateToPointSimple(target, target.name);
    }

    bool navigateViaCenterNoYaw(const Waypoint &target)
    {
        if (!navigateToPointNoYaw(center_point_, "CenterNoYaw")) return false;
        ros::Duration(0.1).sleep();
        return navigateToPointSimple(target, target.name);
    }

    // ===== 允许倒车参数控制（执行5点路径期间临时开启）=====
    void enableReverseForLocalPlanner()
    {
        if (reverse_param_modified_) return;

        // 读取旧参数（读不到就用默认）
        nh_.param("/move_base/DWAPlannerROS/min_vel_x", backup_min_vel_x_, 0.0);
        nh_.param("/move_base/DWAPlannerROS/escape_vel", backup_escape_vel_, -0.1);
        nh_.param("/move_base/TrajectoryPlannerROS/min_vel_x", backup_traj_min_vel_x_, 0.0);

        // ros::param::set 返回 void，直接调用
        ros::param::set("/move_base/DWAPlannerROS/min_vel_x", -0.20);
        ros::param::set("/move_base/DWAPlannerROS/escape_vel", -0.20);
        ros::param::set("/move_base/TrajectoryPlannerROS/min_vel_x", -0.20);

        // TEB 常见开关（存在则生效，不存在也不影响）
        ros::param::set("/move_base/TebLocalPlannerROS/allow_backward_motion", true);
        ros::param::set("/move_base/TebLocalPlannerROS/max_vel_x_backwards", 0.25);

        reverse_param_modified_ = true;
        ROS_INFO("[REVERSE] Enabled temporary reverse-friendly params.");
    }

    void restoreLocalPlannerParams()
    {
        if (!reverse_param_modified_) return;

        ros::param::set("/move_base/DWAPlannerROS/min_vel_x", backup_min_vel_x_);
        ros::param::set("/move_base/DWAPlannerROS/escape_vel", backup_escape_vel_);
        ros::param::set("/move_base/TrajectoryPlannerROS/min_vel_x", backup_traj_min_vel_x_);

        reverse_param_modified_ = false;
        ROS_INFO("[REVERSE] Restored local planner params.");
    }

    // 通用：执行5点路径（逐点导航 + 失败重试）
    bool runPathPoints(const std::vector<Waypoint> &path, const std::string &tag)
    {
        if (path.empty()) return true;

        enableReverseForLocalPlanner();

        for (size_t i = 0; i < path.size(); ++i)
        {
            const Waypoint &wp = path[i];
            ROS_INFO("[%s] Point %zu/%zu -> (%.4f, %.4f), yaw=%.4f",
                     tag.c_str(), i + 1, path.size(), wp.x, wp.y, wp.yaw);

            bool ok = navigateToPointSimple(wp, tag);
            if (!ok) ok = navigateToPoint(wp, static_cast<int>(i), static_cast<int>(path.size()));
            if (!ok)
            {
                ROS_WARN("[%s] Failed to reach point %zu", tag.c_str(), i + 1);
                restoreLocalPlannerParams();
                return false;
            }

            ros::Duration(0.05).sleep();
        }

        restoreLocalPlannerParams();
        return true;
    }

    void startNavigation()
    {
        if (is_navigating_) return;
        is_navigating_ = true;

        // 先导航到区域2并执行清扫
        Waypoint wp_region2 = waypoints_[1];
        if (!navigateToPoint(wp_region2, 0, 1))
        {
            is_navigating_ = false;
            return;
        }

        alignToTargetYaw(wp_region2.yaw);
        bowPathForRegion2();

        // 根据AprilTag ID决定后续访问顺序
        int final_tag_id = detected_tag_id_.load();
        std::vector<int> targets;
        
        if (final_tag_id == 1) 
        {
            // 如果检测到tag1，先去区域1，再去区域3，最后区域4
            targets = {0, 2, 3};
        }
        else if (final_tag_id == 3) 
        {
            // 如果检测到tag3，先去区域3，再去区域1，最后区域4
            targets = {2, 0, 3};
        }
        else 
        {
            // 默认顺序：区域1 -> 区域3 -> 区域4
            targets = {0, 2, 3};
        }

        // 执行后续区域的访问
        for (size_t i = 0; i < targets.size(); ++i)
        {
            int idx = targets[i];
            Waypoint wp = waypoints_[idx];

            // 导航到目标区域
            if (!navigateViaCenter(wp))
            {
                if (!navigateToPoint(wp, i, targets.size()))
                    ROS_ERROR("Direct navigation failed: %s", wp.name.c_str());
            }

            // 执行对应区域的清扫路径
            if (idx == 0)
            {
                alignToTargetYaw(waypoints_[0].yaw);
                bowPathForRegion1();
                navigateToPointSimple(waypoints_[0], "Region1Return");
            }
            else if (idx == 2)
            {
                alignToTargetYaw(waypoints_[2].yaw);
                bowPathForRegion3();
                navigateToPointSimple(waypoints_[2], "Region3Return");
            }
            else if (idx == 3)
            {
                alignToTargetYaw(waypoints_[3].yaw);
                bowPathForRegion4();  // 执行区域四清扫
                navigateToPointSimple(waypoints_[3], "Region4Return");
            }

            // 如果不是最后一个区域，回到中心点
            if (i < targets.size() - 1)
            {
                navigateToPointSimple(center_point_, "CenterTransition");
                ros::Duration(0.1).sleep();
            }
        }

        cmd_vel_pub_.publish(geometry_msgs::Twist());
        is_navigating_ = false;
        ROS_INFO("[NAVIGATION] All regions completed!");
    }

private:
    ros::NodeHandle nh_;
    MoveBaseClient move_base_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber key_sub_;
    ros::Subscriber trigger_sub_;
    ros::Subscriber apriltag_sub_;

    std::vector<Waypoint> waypoints_;
    Waypoint center_point_;

    std::vector<Waypoint> region1_path_points_;
    std::vector<Waypoint> region2_path_points_;
    std::vector<Waypoint> region3_path_points_;
    std::vector<Waypoint> region4_path_points_;  // 新增

    bool is_navigating_;
    std::string key_;
    double current_x_, current_y_, current_yaw_;
    double wait_between_, goal_timeout_, xy_tolerance_;
    int max_retries_, recovery_attempts_;
    std::atomic<int> detected_tag_id_;
    double start_x_, start_y_, start_yaw_;

    // 倒车参数备份
    bool reverse_param_modified_;
    double backup_min_vel_x_;
    double backup_escape_vel_;
    double backup_traj_min_vel_x_;
};

// 区域1：5点路径（yaw与起点一致）
void NavigationController::bowPathForRegion1()
{
    ROS_INFO("[BOWPATH_REGION1] Start 5-point path (yaw=region1 start yaw)");
    bool ok = runPathPoints(region1_path_points_, "Region1Path");
    if (!ok) ROS_WARN("[BOWPATH_REGION1] path failed");
    ROS_INFO("[BOWPATH_REGION1] finished");
}

// 区域2：5点路径（yaw与起点一致）
void NavigationController::bowPathForRegion2()
{
    ROS_INFO("[BOWPATH_REGION2] Start 5-point path (yaw=region2 start yaw)");
    bool ok = runPathPoints(region2_path_points_, "Region2Path");
    if (!ok) ROS_WARN("[BOWPATH_REGION2] path failed");
    ROS_INFO("[BOWPATH_REGION2] finished");
}

// 区域3：5点路径（yaw与起点一致）
void NavigationController::bowPathForRegion3()
{
    ROS_INFO("[BOWPATH_REGION3] Start 5-point path (yaw=region3 start yaw)");
    bool ok = runPathPoints(region3_path_points_, "Region3Path");
    if (!ok) ROS_WARN("[BOWPATH_REGION3] path failed");
    ROS_INFO("[BOWPATH_REGION3] finished");
}

// 区域4：5点路径（yaw与起点一致）
void NavigationController::bowPathForRegion4()
{
    ROS_INFO("[BOWPATH_REGION4] Start 5-point path (yaw=region4 start yaw)");
    bool ok = runPathPoints(region4_path_points_, "Region4Path");
    if (!ok) ROS_WARN("[BOWPATH_REGION4] path failed");
    ROS_INFO("[BOWPATH_REGION4] finished");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_demo");
    ros::NodeHandle nh;
    double wait_between = nh.param("wait_between_seconds", 0.2);
    double goal_timeout = nh.param("goal_timeout", 20.0);

    NavigationController controller(wait_between, goal_timeout);
    ROS_INFO("Navigation node started, waiting for trigger signal...");
    ros::spin();
    return 0;
}
