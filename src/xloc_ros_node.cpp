#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <xloc/xloc_interface.h>
#include <xloc/xloc_common.h>

#include "xloc_ros_wrapper/ActivateMap.h"
#include "xloc_ros_wrapper/StartMapping.h"
#include "xloc_ros_wrapper/StopMapping.h"
#include "xloc_ros_wrapper/StartLocalization.h"
#include "xloc_ros_wrapper/StopLocalization.h"
#include "xloc_ros_wrapper/SetInitialPose.h"
#include "xloc_ros_wrapper/ResetSlamError.h"
#include "xloc_ros_wrapper/GetCurrentPose.h"
#include "xloc_ros_wrapper/SwitchMap.h"
#include "xloc_ros_wrapper/ChangeMapOrigin.h"
#include "xloc_ros_wrapper/StartUpdateMap.h"
#include "xloc_ros_wrapper/StopUpdateMap.h"
#include "xloc_ros_wrapper/Diagnostics.h"
#include <tf3/buffer_core.h>
#include <tf3/compat.h>
#include <tf3/LinearMath/Transform.h>
#include <any>

using namespace std::chrono_literals;

// Helper converters between ROS messages and xloc types
static xloc::Pose ToXlocPose(const geometry_msgs::Pose& p)
{
    xloc::Pose xp;
    xp.position.x = p.position.x;
    xp.position.y = p.position.y;
    xp.position.z = p.position.z;
    xp.orientation.x = p.orientation.x;
    xp.orientation.y = p.orientation.y;
    xp.orientation.z = p.orientation.z;
    xp.orientation.w = p.orientation.w;
    return xp;
}

static geometry_msgs::PoseStamped FromXlocPoseStamped(const xloc::PoseStamped& ps)
{
    geometry_msgs::PoseStamped out;
    out.header.stamp = ros::Time(ps.header.stamp.sec, ps.header.stamp.nsec);
    out.header.frame_id = ps.header.frame_id;
    out.pose.position.x = ps.pose.position.x;
    out.pose.position.y = ps.pose.position.y;
    out.pose.position.z = ps.pose.position.z;
    out.pose.orientation.x = ps.pose.orientation.x;
    out.pose.orientation.y = ps.pose.orientation.y;
    out.pose.orientation.z = ps.pose.orientation.z;
    out.pose.orientation.w = ps.pose.orientation.w;
    return out;
}

static visualization_msgs::MarkerArray FromXlocMarkerArray(const xloc::MarkerArray& xloc_ma)
{
    visualization_msgs::MarkerArray ros_ma;
    for(const auto& xloc_marker : xloc_ma.markers)
    {
        visualization_msgs::Marker ros_marker;
        ros_marker.header.stamp = ros::Time(xloc_marker.header.stamp.sec, xloc_marker.header.stamp.nsec);
        ros_marker.header.frame_id = xloc_marker.header.frame_id;
        ros_marker.ns = xloc_marker.ns;
        ros_marker.id = xloc_marker.id;
        ros_marker.type = xloc_marker.type;
        ros_marker.action = xloc_marker.action;
        ros_marker.pose.position.x = xloc_marker.pose.position.x;
        ros_marker.pose.position.y = xloc_marker.pose.position.y;
        ros_marker.pose.position.z = xloc_marker.pose.position.z;
        ros_marker.pose.orientation.x = xloc_marker.pose.orientation.x;
        ros_marker.pose.orientation.y = xloc_marker.pose.orientation.y;
        ros_marker.pose.orientation.z = xloc_marker.pose.orientation.z;
        ros_marker.pose.orientation.w = xloc_marker.pose.orientation.w;
        ros_marker.scale.x = xloc_marker.scale.x;
        ros_marker.scale.y = xloc_marker.scale.y;
        ros_marker.scale.z = xloc_marker.scale.z;
        ros_marker.color.r = xloc_marker.color.r;
        ros_marker.color.g = xloc_marker.color.g;
        ros_marker.color.b = xloc_marker.color.b;
        ros_marker.color.a = xloc_marker.color.a;
        ros_ma.markers.push_back(ros_marker);
    }
    return ros_ma;
}

std::thread main_thread;
std::thread pub_visualization_thread;
std::thread pub_map_thread;

void MainThread(std::unique_ptr<xloc::XLOCInterface>& xloc, std::shared_ptr<tf3::BufferCore> tf_buffer,
                   tf2_ros::TransformBroadcaster & tf_broadcaster, ros::Publisher & diagnostics_pub)
{
    xloc::Diagnostics diag;
    ros::Rate rate(10.0); // 10 Hz
    while(ros::ok()){
        if(xloc && tf_buffer)
        {
            // publish diagnostics
            try {
                diag = xloc->GetDiagnostics();
                xloc_ros_wrapper::Diagnostics diag_msg;
                diag_msg.header.stamp = ros::Time(diag.header.stamp.sec, diag.header.stamp.nsec);
                diag_msg.header.frame_id = diag.header.frame_id;
                diag_msg.xloc_state = static_cast<uint8_t>(diag.xloc_state.state);
                diag_msg.xloc_state_message = diag.xloc_state.message;
                diag_msg.current_active_map = diag.current_active_map;
                diag_msg.reliability = diag.reliability;
                diag_msg.matching_score = diag.matching_score;
                diagnostics_pub.publish(diag_msg);
                ROS_INFO_THROTTLE(3.0, "XLOC Diagnostics:\nstate:%d\nmessage:%s\nactive_map:%s\nreliability:%.3f\nmatching_score:%.3f",
                    diag_msg.xloc_state,
                    diag_msg.xloc_state_message.c_str(),
                    diag_msg.current_active_map.c_str(),
                    diag_msg.reliability,
                    diag_msg.matching_score);
            } catch (const std::exception &e) {
                ROS_WARN_THROTTLE(5.0, "Failed to get/publish diagnostics: %s", e.what());
            }
            try{
                tf3::TransformStampedMsg map_to_odom_tf = tf_buffer->lookupTransform("map", "odom", tf3::Time());
                std::vector<geometry_msgs::TransformStamped> stamped_transforms;
                geometry_msgs::TransformStamped map_to_odom_ros_tf;
                map_to_odom_ros_tf.header.stamp = ros::Time(map_to_odom_tf.header.stamp.sec, map_to_odom_tf.header.stamp.nsec);
                map_to_odom_ros_tf.header.frame_id = map_to_odom_tf.header.frame_id;
                map_to_odom_ros_tf.child_frame_id = map_to_odom_tf.child_frame_id;
                map_to_odom_ros_tf.transform.translation.x = map_to_odom_tf.transform.translation.x;
                map_to_odom_ros_tf.transform.translation.y = map_to_odom_tf.transform.translation.y;
                map_to_odom_ros_tf.transform.translation.z = map_to_odom_tf.transform.translation.z;
                map_to_odom_ros_tf.transform.rotation.x = map_to_odom_tf.transform.rotation.x;
                map_to_odom_ros_tf.transform.rotation.y = map_to_odom_tf.transform.rotation.y;
                map_to_odom_ros_tf.transform.rotation.z = map_to_odom_tf.transform.rotation.z;
                map_to_odom_ros_tf.transform.rotation.w = map_to_odom_tf.transform.rotation.w;
                stamped_transforms.push_back(map_to_odom_ros_tf);
                tf3::TransformStampedMsg odom_to_base_tf = tf_buffer->lookupTransform("odom", "base_link", tf3::Time());
                geometry_msgs::TransformStamped odom_to_base_ros_tf;
                odom_to_base_ros_tf.header.stamp = ros::Time(odom_to_base_tf.header.stamp.sec, odom_to_base_tf.header.stamp.nsec);
                odom_to_base_ros_tf.header.frame_id = odom_to_base_tf.header.frame_id;
                odom_to_base_ros_tf.child_frame_id = odom_to_base_tf.child_frame_id;
                odom_to_base_ros_tf.transform.translation.x = odom_to_base_tf.transform.translation.x;
                odom_to_base_ros_tf.transform.translation.y = odom_to_base_tf.transform.translation.y;
                odom_to_base_ros_tf.transform.translation.z = odom_to_base_tf.transform.translation.z;
                odom_to_base_ros_tf.transform.rotation.x = odom_to_base_tf.transform.rotation.x;
                odom_to_base_ros_tf.transform.rotation.y = odom_to_base_tf.transform.rotation.y;
                odom_to_base_ros_tf.transform.rotation.z = odom_to_base_tf.transform.rotation.z;
                odom_to_base_ros_tf.transform.rotation.w = odom_to_base_tf.transform.rotation.w;
                stamped_transforms.push_back(odom_to_base_ros_tf);
                tf_broadcaster.sendTransform(stamped_transforms);
            }
            catch(const std::exception & ex){
                ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
                continue;
            }
        }
        rate.sleep();
    }
}

static nav_msgs::OccupancyGrid FromXlocOccupancyGrid(const xloc::OccupancyGrid& xg)
{
    nav_msgs::OccupancyGrid out;
    out.header.stamp = ros::Time(xg.header.stamp.sec, xg.header.stamp.nsec);
    out.header.frame_id = xg.header.frame_id;
    out.info.resolution = xg.info.resolution;
    out.info.width = xg.info.width;
    out.info.height = xg.info.height;
    out.info.origin.position.x = xg.info.origin.position.x;
    out.info.origin.position.y = xg.info.origin.position.y;
    out.info.origin.position.z = xg.info.origin.position.z;
    out.info.origin.orientation.x = xg.info.origin.orientation.x;
    out.info.origin.orientation.y = xg.info.origin.orientation.y;
    out.info.origin.orientation.z = xg.info.origin.orientation.z;
    out.info.origin.orientation.w = xg.info.origin.orientation.w;
    // copy data (xloc uses int8_t vector)
    out.data.assign(xg.data.begin(), xg.data.end());
    return out;
}

void PubVisualizationThread(std::unique_ptr<xloc::XLOCInterface>& xloc, ros::Publisher& traj_node_pub, ros::Publisher& constraint_pub)
{
    ros::Rate rate(1.0); // 1 Hz for visualization markers
    while(ros::ok()){
        if(xloc){
            const xloc::MarkerArray& traj_nodes = xloc->GetTrajectoryNodeListMarker();
            const xloc::MarkerArray& constraints = xloc->GetConstraintListMarker();
            visualization_msgs::MarkerArray ros_traj_nodes = FromXlocMarkerArray(traj_nodes);
            visualization_msgs::MarkerArray ros_constraints = FromXlocMarkerArray(constraints);
            traj_node_pub.publish(ros_traj_nodes);
            constraint_pub.publish(ros_constraints);
        }
        rate.sleep();
    }
}

void PubMapThread(std::unique_ptr<xloc::XLOCInterface>& xloc, ros::Publisher& occupancy_pub)
{
    // Track last published header to avoid publishing unchanged maps
    xloc::UnixTime last_published_time{0,0};
    uint32_t last_published_seq = 0;
    while(ros::ok()){
        if(xloc){
            xloc::Diagnostics diag = xloc->GetDiagnostics();
            if(diag.xloc_state.state == xloc::MAPPING) {
                // publish online grid at 1 Hz
                const xloc::OccupancyGrid& online = xloc->GetOnlineGridMap();
                if(online.info.width > 0 && online.data.size() > 0) {
                    bool changed = (online.header.seq != last_published_seq) || (online.header.stamp != last_published_time);
                    if(changed) {
                        occupancy_pub.publish(FromXlocOccupancyGrid(online));
                        last_published_seq = online.header.seq;
                        last_published_time = online.header.stamp;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } else {
                // publish static grid at 3 Hz
                const xloc::OccupancyGrid& static_map = xloc->GetStaticGridMap(false);
                if(static_map.info.width > 0 && static_map.data.size() > 0) {
                    bool changed = (static_map.header.seq != last_published_seq) || (static_map.header.stamp != last_published_time);
                    if(changed) {
                        occupancy_pub.publish(FromXlocOccupancyGrid(static_map));
                        last_published_seq = static_map.header.seq;
                        last_published_time = static_map.header.stamp;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(333));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xloc_ros_node");
    ros::NodeHandle nh;

    // Create TF buffer core and XLOC instance
    std::shared_ptr<::tf3::BufferCore> tf_buffer = std::make_shared<::tf3::BufferCore>(tf3::Duration(10.0));
    tf3::TransformStampedMsg base_to_scan;
    base_to_scan.header.frame_id = "base_link";
    base_to_scan.child_frame_id = "scan";
    base_to_scan.header.stamp = tf3::Time::now();
    base_to_scan.transform.translation.x = 0.0;
    base_to_scan.transform.translation.y = 0.0;
    base_to_scan.transform.translation.z = 0.0;
    base_to_scan.transform.rotation.x = 0.0;
    base_to_scan.transform.rotation.y = 0.0;
    base_to_scan.transform.rotation.z = 0.0;
    base_to_scan.transform.rotation.w = 1.0;
    tf_buffer->setTransform(base_to_scan, "xloc_ros_wrapper", true);
    tf3::TransformStampedMsg base_to_imu;
    base_to_imu.header.frame_id = "base_link";
    base_to_imu.child_frame_id = "imu";
    base_to_imu.header.stamp = tf3::Time::now();
    base_to_imu.transform.translation.x = 0.0;
    base_to_imu.transform.translation.y = 0.0;
    base_to_imu.transform.translation.z = 0.0;
    base_to_imu.transform.rotation.x = 0.0;
    base_to_imu.transform.rotation.y = 0.0;
    base_to_imu.transform.rotation.z = 0.0;
    base_to_imu.transform.rotation.w = 1.0;
    tf_buffer->setTransform(base_to_imu, "xloc_ros_wrapper", true);
    std::unique_ptr<xloc::XLOCInterface> xloc = xloc::CreateXLOC(tf_buffer);
    if(!xloc){
        ROS_ERROR("Failed to create XLOC instance. Ensure libxloc is available and linked.");
    }

    // Subscribers: scan, odom, imu
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10,
        [&](const sensor_msgs::LaserScan::ConstPtr& msg){
            if(xloc){
                xloc::LaserScan ls;
                ls.header.frame_id = msg->header.frame_id;
                ls.header.stamp = xloc::UnixTime(msg->header.stamp.sec, msg->header.stamp.nsec);
                ls.angle_min = msg->angle_min;
                ls.angle_max = msg->angle_max;
                ls.angle_increment = msg->angle_increment;
                ls.time_increment = msg->time_increment;
                ls.scan_time = msg->scan_time;
                ls.range_min = msg->range_min;
                ls.range_max = msg->range_max;
                ls.ranges.assign(msg->ranges.begin(), msg->ranges.end());
                ls.intensities.assign(msg->intensities.begin(), msg->intensities.end());
                xloc->DispatchSensorData("laser_scan", ls);
            }
        });

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10,
        [&](const nav_msgs::Odometry::ConstPtr& msg){
            if(xloc){
                xloc::Odometry odom;
                odom.header.frame_id = msg->header.frame_id;
                odom.header.stamp = xloc::UnixTime(msg->header.stamp.sec, msg->header.stamp.nsec);
                odom.child_frame_id = msg->child_frame_id;
                odom.pose.pose.position.x = msg->pose.pose.position.x;
                odom.pose.pose.position.y = msg->pose.pose.position.y;
                odom.pose.pose.position.z = msg->pose.pose.position.z;
                odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
                odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
                odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
                odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
                odom.twist.twist.linear.x = msg->twist.twist.linear.x;
                odom.twist.twist.linear.y = msg->twist.twist.linear.y;
                odom.twist.twist.linear.z = msg->twist.twist.linear.z;
                odom.twist.twist.angular.x = msg->twist.twist.angular.x;
                odom.twist.twist.angular.y = msg->twist.twist.angular.y;
                odom.twist.twist.angular.z = msg->twist.twist.angular.z;
                xloc->DispatchSensorData("odom", odom);
            }
        });

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10,
        [&](const sensor_msgs::Imu::ConstPtr& msg){
            if(xloc){
                xloc::Imu imu;
                imu.header.frame_id = msg->header.frame_id;
                imu.header.stamp = xloc::UnixTime(msg->header.stamp.sec, msg->header.stamp.nsec);
                imu.orientation.x = msg->orientation.x;
                imu.orientation.y = msg->orientation.y;
                imu.orientation.z = msg->orientation.z;
                imu.orientation.w = msg->orientation.w;
                imu.angular_velocity.x = msg->angular_velocity.x;
                imu.angular_velocity.y = msg->angular_velocity.y;
                imu.angular_velocity.z = msg->angular_velocity.z;
                imu.linear_acceleration.x = msg->linear_acceleration.x;
                imu.linear_acceleration.y = msg->linear_acceleration.y;
                imu.linear_acceleration.z = msg->linear_acceleration.z;
                imu.orientation_covariance[0] = msg->orientation_covariance[0];
                imu.orientation_covariance[1] = msg->orientation_covariance[1];
                imu.orientation_covariance[2] = msg->orientation_covariance[2];
                imu.orientation_covariance[3] = msg->orientation_covariance[3];
                imu.orientation_covariance[4] = msg->orientation_covariance[4];
                imu.orientation_covariance[5] = msg->orientation_covariance[5];
                imu.orientation_covariance[6] = msg->orientation_covariance[6];
                imu.orientation_covariance[7] = msg->orientation_covariance[7];
                imu.orientation_covariance[8] = msg->orientation_covariance[8];
                imu.angular_velocity_covariance[0] = msg->angular_velocity_covariance[0];
                imu.angular_velocity_covariance[1] = msg->angular_velocity_covariance[1];
                imu.angular_velocity_covariance[2] = msg->angular_velocity_covariance[2];
                imu.angular_velocity_covariance[3] = msg->angular_velocity_covariance[3];
                imu.angular_velocity_covariance[4] = msg->angular_velocity_covariance[4];
                imu.angular_velocity_covariance[5] = msg->angular_velocity_covariance[5];
                imu.angular_velocity_covariance[6] = msg->angular_velocity_covariance[6];
                imu.angular_velocity_covariance[7] = msg->angular_velocity_covariance[7];
                imu.angular_velocity_covariance[8] = msg->angular_velocity_covariance[8];
                imu.linear_acceleration_covariance[0] = msg->linear_acceleration_covariance[0];
                imu.linear_acceleration_covariance[1] = msg->linear_acceleration_covariance[1];
                imu.linear_acceleration_covariance[2] = msg->linear_acceleration_covariance[2];
                imu.linear_acceleration_covariance[3] = msg->linear_acceleration_covariance[3];
                imu.linear_acceleration_covariance[4] = msg->linear_acceleration_covariance[4];
                imu.linear_acceleration_covariance[5] = msg->linear_acceleration_covariance[5];
                imu.linear_acceleration_covariance[6] = msg->linear_acceleration_covariance[6];
                imu.linear_acceleration_covariance[7] = msg->linear_acceleration_covariance[7];
                imu.linear_acceleration_covariance[8] = msg->linear_acceleration_covariance[8];
                xloc->DispatchSensorData("imu", imu);
            }
        });

    // Services
    ros::ServiceServer srv_activate = nh.advertiseService<xloc_ros_wrapper::ActivateMap::Request, xloc_ros_wrapper::ActivateMap::Response>(
        "activate_map",
        [&](xloc_ros_wrapper::ActivateMap::Request& req, xloc_ros_wrapper::ActivateMap::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->ActivateMap(req.map_file_name);
            res.code = s.code;
            res.message = s.message;
            return true;
        }
    );

    ros::ServiceServer srv_start_mapping = nh.advertiseService<xloc_ros_wrapper::StartMapping::Request, xloc_ros_wrapper::StartMapping::Response>(
        "start_mapping",
        [&](xloc_ros_wrapper::StartMapping::Request& req, xloc_ros_wrapper::StartMapping::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->StartMapping();
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_stop_mapping = nh.advertiseService<xloc_ros_wrapper::StopMapping::Request, xloc_ros_wrapper::StopMapping::Response>(
        "stop_mapping",
        [&](xloc_ros_wrapper::StopMapping::Request& req, xloc_ros_wrapper::StopMapping::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->StopMapping(req.map_file_name);
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_start_loc = nh.advertiseService<xloc_ros_wrapper::StartLocalization::Request, xloc_ros_wrapper::StartLocalization::Response>(
        "start_localization",
        [&](xloc_ros_wrapper::StartLocalization::Request& req, xloc_ros_wrapper::StartLocalization::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->StartLocalization();
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_stop_loc = nh.advertiseService<xloc_ros_wrapper::StopLocalization::Request, xloc_ros_wrapper::StopLocalization::Response>(
        "stop_localization",
        [&](xloc_ros_wrapper::StopLocalization::Request& req, xloc_ros_wrapper::StopLocalization::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->StopLocalization();
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_set_initial = nh.advertiseService<xloc_ros_wrapper::SetInitialPose::Request, xloc_ros_wrapper::SetInitialPose::Response>(
        "set_initial_pose",
        [&](xloc_ros_wrapper::SetInitialPose::Request& req, xloc_ros_wrapper::SetInitialPose::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::Pose xp = ToXlocPose(req.initial_pose);
            xloc::StatusResponse s = xloc->SetInitialPose(xp);
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_reset = nh.advertiseService<xloc_ros_wrapper::ResetSlamError::Request, xloc_ros_wrapper::ResetSlamError::Response>(
        "reset_slam_error",
        [&](xloc_ros_wrapper::ResetSlamError::Request& req, xloc_ros_wrapper::ResetSlamError::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->ResetSlamError();
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_get_pose = nh.advertiseService<xloc_ros_wrapper::GetCurrentPose::Request, xloc_ros_wrapper::GetCurrentPose::Response>(
        "get_current_pose",
        [&](xloc_ros_wrapper::GetCurrentPose::Request& req, xloc_ros_wrapper::GetCurrentPose::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            const xloc::PoseStamped& ps = xloc->GetCurrentPose();
            res.pose = FromXlocPoseStamped(ps);
            // No StatusResponse returned by GetCurrentPose; consider code OK
            res.code = xloc::OK;
            res.message = "";
            return true;
        }
    );

    ros::ServiceServer srv_switch_map = nh.advertiseService<xloc_ros_wrapper::SwitchMap::Request, xloc_ros_wrapper::SwitchMap::Response>(
        "switch_map",
        [&](xloc_ros_wrapper::SwitchMap::Request& req, xloc_ros_wrapper::SwitchMap::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::Pose p = ToXlocPose(req.initial_pose);
            xloc::StatusResponse s = xloc->SwitchMap(req.map_file_name, req.use_initial_pose, p);
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_change_origin = nh.advertiseService<xloc_ros_wrapper::ChangeMapOrigin::Request, xloc_ros_wrapper::ChangeMapOrigin::Response>(
        "change_map_origin",
        [&](xloc_ros_wrapper::ChangeMapOrigin::Request& req, xloc_ros_wrapper::ChangeMapOrigin::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::Pose p = ToXlocPose(req.new_map_origin);
            xloc::StatusResponse s = xloc->ChangeMapOrigin(p);
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_start_update = nh.advertiseService<xloc_ros_wrapper::StartUpdateMap::Request, xloc_ros_wrapper::StartUpdateMap::Response>(
        "start_update_map",
        [&](xloc_ros_wrapper::StartUpdateMap::Request& req, xloc_ros_wrapper::StartUpdateMap::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->StartUpdateMap();
            res.code = s.code; res.message = s.message; return true;
        }
    );

    ros::ServiceServer srv_stop_update = nh.advertiseService<xloc_ros_wrapper::StopUpdateMap::Request, xloc_ros_wrapper::StopUpdateMap::Response>(
        "stop_update_map",
        [&](xloc_ros_wrapper::StopUpdateMap::Request& req, xloc_ros_wrapper::StopUpdateMap::Response& res)->bool{
            if(!xloc){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc::StatusResponse s = xloc->StopUpdateMap(req.save_updated_map);
            res.code = s.code; res.message = s.message; return true;
        }
    );
    // Publishers: trajectory nodes, constraints
    ros::Publisher traj_node_pub = nh.advertise<visualization_msgs::MarkerArray>("xloc/trajectory_nodes", 10);
    ros::Publisher constraint_pub = nh.advertise<visualization_msgs::MarkerArray>("xloc/constraints", 10);
    ros::Publisher diagnostics_pub = nh.advertise<xloc_ros_wrapper::Diagnostics>("xloc/diagnostics", 10);
    ros::Publisher occupancy_pub = nh.advertise<nav_msgs::OccupancyGrid>("xloc/occupancy_grid", 10, true);
    // create broadcaster AFTER ros::init
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Start processing thread (pass broadcaster by reference)
    main_thread = std::thread(MainThread, std::ref(xloc), tf_buffer, std::ref(tf_broadcaster), std::ref(diagnostics_pub));
    pub_visualization_thread = std::thread(PubVisualizationThread, std::ref(xloc), std::ref(traj_node_pub), std::ref(constraint_pub));
    pub_map_thread = std::thread(PubMapThread, std::ref(xloc), std::ref(occupancy_pub));
    ROS_INFO("xloc_ros_node ready");
    ros::spin();
    if(main_thread.joinable()){
        main_thread.join();
    }
    if(pub_visualization_thread.joinable()) pub_visualization_thread.join();
    if(pub_map_thread.joinable()) pub_map_thread.join();
    return 0;
}
