// loop_closure.h
// Standalone loop closure module for COIN-LIO.
//
// Adapted from FAST_LIO_SAM (https://github.com/kahowang/FAST_LIO_SAM)
// Original loop closure logic by kahowang; refactored here into a
// self-contained, GTSAM-free class for use with COIN-LIO's iterated EKF.
//
// Design:
//   - Stores keyframes as (world-frame pose, body-frame point cloud) pairs.
//   - A background thread periodically runs radius-based loop detection.
//   - When a loop is found, ICP aligns the current keyframe submap against
//     the historical submap.  On success, the relative drift is computed and
//     published on /loop_closure/correction as geometry_msgs/PoseStamped.
//   - The main node can read the correction via hasCorrection()/getCorrection()
//     OR subscribe to the topic; both paths are supported.
//   - No GTSAM, no Scan Context – just PCL ICP + Euclidean-distance detection.
//
// Usage in laserMapping.cpp:
//
//   #include "loop_closure.h"
//   ...
//   LoopClosure loop_closure;
//   loop_closure.init(nh);
//   ...
//   // inside the per-frame update block, after EKF update:
//   loop_closure.addKeyframe(pose_matrix4d, feats_down_body);
//   ...
//   if (loop_closure.hasCorrection()) {
//       Eigen::Matrix4d corr = loop_closure.getCorrection();
//       // apply correction to current state estimate
//   }
//   ...
//   loop_closure.stop();   // called before node exit

#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// LoopClosure uses the same PointType as the rest of COIN-LIO so it can
// accept clouds produced by feats_down_body without any copying or conversion.
// PointType == pcl::PointXYZINormal  (from common_lib.h)
using LCPointType = pcl::PointXYZINormal;
using LCCloud     = pcl::PointCloud<LCPointType>;

// ---------------------------------------------------------------------------
// Internal keyframe storage
// ---------------------------------------------------------------------------
struct LoopKeyframe {
    int                id;         // sequential index
    double             timestamp;  // lidar end time (seconds)
    Eigen::Matrix4d    pose;       // T_world_body (body = IMU frame)
    LCCloud::Ptr       cloud;      // point cloud in body frame
};

// ---------------------------------------------------------------------------
// LoopClosure class
// ---------------------------------------------------------------------------
class LoopClosure {
public:
    LoopClosure() = default;
    ~LoopClosure() { stop(); }

    // -----------------------------------------------------------------------
    // init() – load ROS params, advertise topic, start background thread.
    // Call once from the main node's init block.
    // -----------------------------------------------------------------------
    void init(ros::NodeHandle& nh)
    {
        nh.param<bool>  ("loop_closure/enable",                  enabled_,                  false);
        nh.param<double>("loop_closure/frequency",               frequency_hz_,             4.0);
        nh.param<double>("loop_closure/search_radius",           search_radius_,            15.0);
        nh.param<double>("loop_closure/time_diff_threshold",     time_diff_threshold_,      30.0);
        nh.param<double>("loop_closure/fitness_score_threshold", fitness_score_threshold_,  0.3);
        nh.param<double>("loop_closure/keyframe_distance",       keyframe_distance_,        1.0);
        nh.param<double>("loop_closure/keyframe_angle",          keyframe_angle_,           0.2);
        nh.param<int>   ("loop_closure/submap_size",             submap_size_,              20);
        nh.param<int>   ("loop_closure/icp_max_iterations",      icp_max_iterations_,       100);
        nh.param<double>("loop_closure/icp_max_correspondence_dist",
                         icp_max_correspondence_dist_, 150.0);

        if (!enabled_) {
            ROS_INFO("[LoopClosure] disabled via params – skipping init.");
            return;
        }

        // Down-sampler used when building ICP submaps
        icp_voxel_filter_.setLeafSize(0.4f, 0.4f, 0.4f);

        // Publisher for the correction pose.
        // Header.frame_id = "camera_init"  (same convention as COIN-LIO)
        // Position = translation correction (world frame)
        // Orientation = rotation correction (world frame), as quaternion
        correction_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "/loop_closure/correction", 1);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
            "/loop_closure/loop_edges", 1);

        // Keyframe position cloud used for KD-tree radius search
        kf_positions_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        kf_positions_copy_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        running_ = true;
        detection_thread_ = std::thread(&LoopClosure::detectionThreadFunc, this);

        ROS_INFO("[LoopClosure] enabled. radius=%.1f m, time_diff=%.1f s, "
                 "icp_thresh=%.3f, freq=%.1f Hz",
                 search_radius_, time_diff_threshold_,
                 fitness_score_threshold_, frequency_hz_);
    }

    // -----------------------------------------------------------------------
    // stop() – signal background thread to exit and join.
    // -----------------------------------------------------------------------
    void stop()
    {
        running_ = false;
        if (detection_thread_.joinable())
            detection_thread_.join();
    }

    // -----------------------------------------------------------------------
    // addKeyframe() – called from the main mapping loop each time a new
    // keyframe is selected.  pose is T_world_body (4×4, double).
    // cloud is the downsampled point cloud in body (IMU) frame.
    //
    // Returns true if the frame was accepted as a new keyframe, false if it
    // was too close (distance/angle) to the previous keyframe.
    // -----------------------------------------------------------------------
    bool addKeyframe(const Eigen::Matrix4d& pose,
                     LCCloud::Ptr           cloud)
    {
        if (!enabled_)
            return false;

        // Keyframe selection: skip if motion since last keyframe is too small.
        if (!keyframes_.empty()) {
            const Eigen::Matrix4d& last_pose = keyframes_.back().pose;
            Eigen::Matrix4d delta = last_pose.inverse() * pose;

            double dist = delta.block<3,1>(0,3).norm();
            // Extract rotation angle from delta rotation matrix
            Eigen::AngleAxisd aa(Eigen::Matrix3d(delta.block<3,3>(0,0)));
            double angle = std::abs(aa.angle());

            if (dist < keyframe_distance_ && angle < keyframe_angle_)
                return false;
        }

        LoopKeyframe kf;
        kf.id        = static_cast<int>(keyframes_.size());
        kf.timestamp = ros::Time::now().toSec();
        kf.pose      = pose;
        kf.cloud     = cloud;

        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            keyframes_.push_back(kf);

            // Update the flat XYZ cloud used for KD-tree search
            pcl::PointXYZ pt;
            pt.x = static_cast<float>(pose(0,3));
            pt.y = static_cast<float>(pose(1,3));
            pt.z = static_cast<float>(pose(2,3));
            kf_positions_->push_back(pt);
        }
        return true;
    }

    // -----------------------------------------------------------------------
    // hasCorrection() / getCorrection()
    //   Poll-based alternative to subscribing to the topic.
    //   After getCorrection() the flag is cleared.
    // -----------------------------------------------------------------------
    bool hasCorrection() const { return correction_ready_.load(); }

    Eigen::Matrix4d getCorrection()
    {
        std::lock_guard<std::mutex> lk(correction_mutex_);
        correction_ready_ = false;
        return latest_correction_;
    }

    // -----------------------------------------------------------------------
    // isEnabled() – convenience accessor
    // -----------------------------------------------------------------------
    bool isEnabled() const { return enabled_; }

private:
    // ========================== Parameters ==================================
    bool   enabled_                    = false;
    double frequency_hz_               = 4.0;
    double search_radius_              = 15.0;
    double time_diff_threshold_        = 30.0;
    double fitness_score_threshold_    = 0.3;
    double keyframe_distance_          = 1.0;
    double keyframe_angle_             = 0.2;
    int    submap_size_                = 20;
    int    icp_max_iterations_         = 100;
    double icp_max_correspondence_dist_= 150.0;

    // ========================== State =======================================
    std::vector<LoopKeyframe>           keyframes_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr kf_positions_;        // world-frame XYZ only
    pcl::PointCloud<pcl::PointXYZ>::Ptr kf_positions_copy_;   // copy for detection thread

    std::unordered_map<int,int> loop_index_map_;  // cur_id -> pre_id (prevent repeat)

    mutable std::mutex kf_mutex_;

    // Correction produced by last successful ICP
    Eigen::Matrix4d    latest_correction_ = Eigen::Matrix4d::Identity();
    std::atomic<bool>  correction_ready_{false};
    mutable std::mutex correction_mutex_;

    // Background thread
    std::thread detection_thread_;
    std::atomic<bool> running_{false};

    // ROS
    ros::Publisher correction_pub_;
    ros::Publisher marker_pub_;

    // ICP voxel filter for submap construction
    pcl::VoxelGrid<LCPointType> icp_voxel_filter_;

    // ========================== Background thread ===========================
    void detectionThreadFunc()
    {
        ros::Rate rate(frequency_hz_);
        while (ros::ok() && running_) {
            rate.sleep();
            performLoopDetection();
            publishLoopMarkers();
        }
    }

    // -----------------------------------------------------------------------
    // detectLoopCandidate() – radius search in keyframe-position KD-tree.
    // Returns true and fills cur_id / pre_id on success.
    // -----------------------------------------------------------------------
    bool detectLoopCandidate(int& cur_id, int& pre_id)
    {
        // Work on a local copy to minimise lock time
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            if (keyframes_.size() < 2)
                return false;
            *kf_positions_copy_ = *kf_positions_;
        }

        const int cur_idx = static_cast<int>(kf_positions_copy_->size()) - 1;

        // Skip if this keyframe has already been matched
        if (loop_index_map_.count(cur_idx))
            return false;

        // Build KD-tree over all keyframe positions
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(kf_positions_copy_);

        std::vector<int>   candidate_idx;
        std::vector<float> candidate_sq_dist;
        kdtree.radiusSearch(kf_positions_copy_->back(),
                            static_cast<float>(search_radius_),
                            candidate_idx, candidate_sq_dist, 0);

        if (candidate_idx.empty())
            return false;

        // Among candidates, pick the one with sufficient time separation
        double cur_time;
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            cur_time = keyframes_[cur_idx].timestamp;
        }

        int best_pre = -1;
        for (int id : candidate_idx) {
            if (id == cur_idx)
                continue;
            double pre_time;
            {
                std::lock_guard<std::mutex> lk(kf_mutex_);
                pre_time = keyframes_[id].timestamp;
            }
            if (std::abs(cur_time - pre_time) > time_diff_threshold_) {
                best_pre = id;
                break;
            }
        }

        if (best_pre < 0)
            return false;

        cur_id = cur_idx;
        pre_id = best_pre;
        return true;
    }

    // -----------------------------------------------------------------------
    // buildSubmap() – collect submap_size_ keyframes around 'center_id',
    // transform them into world frame, then voxel-downsample.
    // -----------------------------------------------------------------------
    LCCloud::Ptr buildSubmap(int center_id, int half_width)
    {
        LCCloud::Ptr submap(new LCCloud());

        std::lock_guard<std::mutex> lk(kf_mutex_);
        const int n = static_cast<int>(keyframes_.size());

        for (int offset = -half_width; offset <= half_width; ++offset) {
            int idx = center_id + offset;
            if (idx < 0 || idx >= n)
                continue;

            const LoopKeyframe& kf = keyframes_[idx];

            // Transform body-frame cloud into world frame using kf.pose
            LCCloud::Ptr cloud_world(new LCCloud());
            pcl::transformPointCloud(*kf.cloud, *cloud_world,
                                     kf.pose.cast<float>());
            *submap += *cloud_world;
        }

        if (submap->empty())
            return submap;

        // Downsample
        LCCloud::Ptr submap_ds(new LCCloud());
        icp_voxel_filter_.setInputCloud(submap);
        icp_voxel_filter_.filter(*submap_ds);
        return submap_ds;
    }

    // -----------------------------------------------------------------------
    // performLoopDetection() – the core routine called by the background
    // thread.  Detects, matches, and publishes corrections.
    // -----------------------------------------------------------------------
    void performLoopDetection()
    {
        int cur_id = -1, pre_id = -1;
        if (!detectLoopCandidate(cur_id, pre_id))
            return;

        ROS_INFO("[LoopClosure] Candidate: cur=%d  pre=%d", cur_id, pre_id);

        // Build submaps
        LCCloud::Ptr cur_submap = buildSubmap(cur_id, 0);           // single frame
        LCCloud::Ptr pre_submap = buildSubmap(pre_id, submap_size_); // neighbourhood

        if (cur_submap->empty() || pre_submap->empty()) {
            ROS_WARN("[LoopClosure] Empty submap – skipping.");
            return;
        }

        // ---- ICP ----
        pcl::IterativeClosestPoint<LCPointType, LCPointType> icp;
        icp.setMaxCorrespondenceDistance(
            static_cast<double>(icp_max_correspondence_dist_));
        icp.setMaximumIterations(icp_max_iterations_);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(cur_submap);
        icp.setInputTarget(pre_submap);

        LCCloud::Ptr aligned(new LCCloud());
        icp.align(*aligned);

        if (!icp.hasConverged()) {
            ROS_WARN("[LoopClosure] ICP did not converge.");
            return;
        }

        double score = icp.getFitnessScore();
        if (score > fitness_score_threshold_) {
            ROS_INFO("[LoopClosure] ICP score %.4f > threshold %.4f – rejected.",
                     score, fitness_score_threshold_);
            return;
        }

        ROS_INFO("[LoopClosure] ICP accepted. score=%.4f", score);

        // Mark this pair to avoid re-detection
        loop_index_map_[cur_id] = pre_id;

        // ---- Compute drift correction ----
        // icp.getFinalTransformation() is the transform that maps
        // cur_submap into alignment with pre_submap, i.e.
        //   T_correction * T_world_cur_estimated = T_world_cur_corrected
        // So the correction to the current keyframe pose is:
        //   T_corrected = T_icp * T_estimated
        Eigen::Matrix4d T_icp =
            icp.getFinalTransformation().cast<double>();

        Eigen::Matrix4d T_cur_estimated;
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            T_cur_estimated = keyframes_[cur_id].pose;
        }

        Eigen::Matrix4d T_cur_corrected = T_icp * T_cur_estimated;

        // The correction transform itself (what needs to be applied to the
        // current estimated pose to obtain the corrected pose):
        Eigen::Matrix4d correction = T_cur_corrected * T_cur_estimated.inverse();

        // Store and publish
        {
            std::lock_guard<std::mutex> lk(correction_mutex_);
            latest_correction_ = correction;
            correction_ready_  = true;
        }

        publishCorrection(correction, cur_id, pre_id, score);
    }

    // -----------------------------------------------------------------------
    // publishCorrection() – publish the drift correction on the ROS topic.
    // -----------------------------------------------------------------------
    void publishCorrection(const Eigen::Matrix4d& correction,
                           int cur_id, int pre_id, double score)
    {
        if (correction_pub_.getNumSubscribers() == 0 &&
            marker_pub_.getNumSubscribers()    == 0 &&
            !correction_ready_.load())
            return;  // nothing listening; correction is still stored internally

        geometry_msgs::PoseStamped msg;
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = "camera_init";

        // Translation
        msg.pose.position.x = correction(0, 3);
        msg.pose.position.y = correction(1, 3);
        msg.pose.position.z = correction(2, 3);

        // Rotation
        Eigen::Matrix3d R = correction.block<3,3>(0,0);
        Eigen::Quaterniond q(R);
        q.normalize();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        correction_pub_.publish(msg);

        ROS_INFO("[LoopClosure] Correction published. "
                 "cur=%d  pre=%d  score=%.4f  dt=[%.3f %.3f %.3f]",
                 cur_id, pre_id, score,
                 correction(0,3), correction(1,3), correction(2,3));
    }

    // -----------------------------------------------------------------------
    // publishLoopMarkers() – visualise loop edges in RViz.
    // -----------------------------------------------------------------------
    void publishLoopMarkers()
    {
        if (marker_pub_.getNumSubscribers() == 0)
            return;
        if (loop_index_map_.empty())
            return;

        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = "camera_init";
        edge_marker.header.stamp    = ros::Time::now();
        edge_marker.ns              = "loop_edges";
        edge_marker.id              = 0;
        edge_marker.type            = visualization_msgs::Marker::LINE_LIST;
        edge_marker.action          = visualization_msgs::Marker::ADD;
        edge_marker.scale.x         = 0.1;
        edge_marker.color.r         = 0.9f;
        edge_marker.color.g         = 0.2f;
        edge_marker.color.b         = 0.2f;
        edge_marker.color.a         = 0.8f;
        edge_marker.pose.orientation.w = 1.0;

        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            for (const auto& kv : loop_index_map_) {
                int cur_id = kv.first;
                int pre_id = kv.second;
                if (cur_id >= static_cast<int>(keyframes_.size()) ||
                    pre_id >= static_cast<int>(keyframes_.size()))
                    continue;

                geometry_msgs::Point p_cur, p_pre;
                p_cur.x = keyframes_[cur_id].pose(0,3);
                p_cur.y = keyframes_[cur_id].pose(1,3);
                p_cur.z = keyframes_[cur_id].pose(2,3);
                p_pre.x = keyframes_[pre_id].pose(0,3);
                p_pre.y = keyframes_[pre_id].pose(1,3);
                p_pre.z = keyframes_[pre_id].pose(2,3);

                edge_marker.points.push_back(p_cur);
                edge_marker.points.push_back(p_pre);
            }
        }

        marker_array.markers.push_back(edge_marker);
        marker_pub_.publish(marker_array);
    }
};  // class LoopClosure
