// vision_bridge.cpp
// Bridges COIN-LIO odometry to MAVROS for ArduPilot GPS-denied navigation.
//
// COIN-LIO publishes:
//   /Odometry (nav_msgs/Odometry)
//     frame_id:       "camera_init"  (world/map frame)
//     child_frame_id: "body"         (IMU/body frame)
//     pose:  position + orientation (quaternion) in world frame
//     twist: NOT populated by COIN-LIO — will forward zeros
//
// This node publishes:
//   /mavros/vision_pose/pose  (geometry_msgs/PoseStamped)  — EKF2/AHRS position
//   /mavros/vision_speed/speed_twist (geometry_msgs/TwistStamped) — velocity hint
//
// Coordinate frames:
//   COIN-LIO outputs in a right-handed ENU-like world frame (x-forward at init,
//   z-up). MAVROS vision_pose expects ENU. No axis swap is needed in most
//   configurations; a yaw offset param is provided for body frame alignment.
//
// Parameters:
//   ~odom_topic          (string, default "/Odometry")  — source topic
//   ~rate_limit          (double, default 30.0 Hz)      — max publish rate
//   ~body_to_mavros_yaw  (double, default 0.0 rad)      — static yaw rotation
//                         applied to the pose before forwarding; use when the
//                         COIN-LIO init heading differs from the MAVROS body frame
//   ~watchdog_timeout    (double, default 1.0 s)        — warn if no odom received

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf/transform_datatypes.h>

class VisionBridge
{
public:
    VisionBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        // Parameters
        std::string odom_topic;
        pnh.param<std::string>("odom_topic", odom_topic, "/Odometry");
        pnh.param<double>("rate_limit", rate_limit_hz_, 30.0);
        pnh.param<double>("body_to_mavros_yaw", body_to_mavros_yaw_, 0.0);
        pnh.param<double>("watchdog_timeout", watchdog_timeout_s_, 1.0);

        if (rate_limit_hz_ <= 0.0)
        {
            ROS_WARN("[vision_bridge] rate_limit <= 0, disabling rate limiting");
            min_dt_ = ros::Duration(0.0);
        }
        else
        {
            min_dt_ = ros::Duration(1.0 / rate_limit_hz_);
        }

        // Build the static yaw quaternion for body frame alignment
        yaw_quat_ = tf::createQuaternionFromYaw(body_to_mavros_yaw_);

        // Publishers
        pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>(
            "/mavros/vision_pose/pose", 10);
        pub_twist_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/vision_speed/speed_twist", 10);
        pub_diag_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(
            "/diagnostics", 10);

        // Subscriber
        sub_odom_ = nh.subscribe(odom_topic, 10,
                                  &VisionBridge::odomCallback, this);

        // Watchdog timer — fires every watchdog_timeout seconds
        watchdog_timer_ = nh.createTimer(
            ros::Duration(watchdog_timeout_s_),
            &VisionBridge::watchdogCallback, this);

        last_odom_time_ = ros::Time(0);
        last_pub_time_  = ros::Time(0);

        ROS_INFO("[vision_bridge] Subscribing to %s", odom_topic.c_str());
        ROS_INFO("[vision_bridge] rate_limit=%.1f Hz  body_to_mavros_yaw=%.4f rad",
                 rate_limit_hz_, body_to_mavros_yaw_);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        last_odom_time_ = ros::Time::now();

        // Rate limiting
        if ((last_odom_time_ - last_pub_time_) < min_dt_)
            return;
        last_pub_time_ = last_odom_time_;

        // --- PoseStamped ---
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        // Keep frame_id as "camera_init"; MAVROS will use the TF tree or
        // the frame configured in its vision_pose plugin. Set to "map" if
        // ArduPilot requires it — adjust via launch arg if needed.

        if (body_to_mavros_yaw_ == 0.0)
        {
            // Fast path: no rotation needed
            pose_msg.pose = msg->pose.pose;
        }
        else
        {
            // Apply static yaw offset: p_mavros = R_yaw * p_coinlio
            tf::Quaternion q_odom;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, q_odom);
            tf::Quaternion q_out = yaw_quat_ * q_odom;
            q_out.normalize();

            // Rotate position vector by yaw offset
            tf::Vector3 pos(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
            tf::Vector3 pos_rot = tf::quatRotate(yaw_quat_, pos);

            pose_msg.pose.position.x = pos_rot.x();
            pose_msg.pose.position.y = pos_rot.y();
            pose_msg.pose.position.z = pos_rot.z();
            tf::quaternionTFToMsg(q_out, pose_msg.pose.orientation);
        }

        pub_pose_.publish(pose_msg);

        // --- TwistStamped ---
        // Note: COIN-LIO does not populate twist in its odometry message.
        // The twist fields will be zero. If COIN-LIO is updated to publish
        // velocity, this bridge will forward it automatically.
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header = msg->header;
        twist_msg.twist  = msg->twist.twist;
        pub_twist_.publish(twist_msg);
    }

    void watchdogCallback(const ros::TimerEvent&)
    {
        diagnostic_msgs::DiagnosticArray diag_array;
        diag_array.header.stamp = ros::Time::now();

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "coin_lio/vision_bridge";
        status.hardware_id = "vision_bridge";

        if (last_odom_time_.isZero())
        {
            status.level   = diagnostic_msgs::DiagnosticStatus::WARN;
            status.message = "No odometry received yet";
        }
        else
        {
            double age = (ros::Time::now() - last_odom_time_).toSec();
            diagnostic_msgs::KeyValue kv;
            kv.key   = "last_odom_age_s";
            kv.value = std::to_string(age);
            status.values.push_back(kv);

            if (age > watchdog_timeout_s_)
            {
                status.level   = diagnostic_msgs::DiagnosticStatus::WARN;
                status.message = "Odometry stale: " + std::to_string(age) + "s";
                ROS_WARN_THROTTLE(watchdog_timeout_s_,
                    "[vision_bridge] No odometry for %.1f s — MAVROS will lose vision input",
                    age);
            }
            else
            {
                status.level   = diagnostic_msgs::DiagnosticStatus::OK;
                status.message = "OK";
            }
        }

        diag_array.status.push_back(status);
        pub_diag_.publish(diag_array);
    }

    // ROS handles
    ros::Subscriber sub_odom_;
    ros::Publisher  pub_pose_;
    ros::Publisher  pub_twist_;
    ros::Publisher  pub_diag_;
    ros::Timer      watchdog_timer_;

    // Config
    double       rate_limit_hz_;
    double       body_to_mavros_yaw_;
    double       watchdog_timeout_s_;
    ros::Duration min_dt_;
    tf::Quaternion yaw_quat_;

    // State
    ros::Time last_odom_time_;
    ros::Time last_pub_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coin_lio_vision_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    VisionBridge bridge(nh, pnh);
    ros::spin();
    return 0;
}
