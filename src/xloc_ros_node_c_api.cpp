#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <xloc_c_api.h>
#include <xloc/xloc_common.h>
#include <tf3/buffer_core.h>
#include <tf3/compat.h>
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

#include <thread>
#include <vector>
#include <string>

using namespace std::chrono_literals;

static geometry_msgs::PoseStamped FromCpose(const xloc_pose_t* p, const xloc_header_t* h)
{
    geometry_msgs::PoseStamped out;
    if (h) out.header.stamp = ros::Time(h->stamp.sec, h->stamp.nsec);
    if (h && h->frame_id) out.header.frame_id = h->frame_id;
    if (p) {
        out.pose.position.x = p->position[0];
        out.pose.position.y = p->position[1];
        out.pose.position.z = p->position[2];
        out.pose.orientation.x = p->orientation[0];
        out.pose.orientation.y = p->orientation[1];
        out.pose.orientation.z = p->orientation[2];
        out.pose.orientation.w = p->orientation[3];
    }
    return out;
}

static visualization_msgs::MarkerArray FromXlocMarkerArrayPtr(const xloc_marker_array_t* cma)
{
    visualization_msgs::MarkerArray out;
    if (!cma || !cma->internal) return out;
    const xloc::MarkerArray* ma = reinterpret_cast<const xloc::MarkerArray*>(cma->internal);
    for(const auto & m : ma->markers) {
        visualization_msgs::Marker rm;
        rm.header.stamp = ros::Time(m.header.stamp.sec, m.header.stamp.nsec);
        rm.header.frame_id = m.header.frame_id;
        rm.ns = m.ns;
        rm.id = m.id;
        rm.type = m.type;
        rm.action = m.action;
        rm.pose.position.x = m.pose.position.x;
        rm.pose.position.y = m.pose.position.y;
        rm.pose.position.z = m.pose.position.z;
        rm.pose.orientation.x = m.pose.orientation.x;
        rm.pose.orientation.y = m.pose.orientation.y;
        rm.pose.orientation.z = m.pose.orientation.z;
        rm.pose.orientation.w = m.pose.orientation.w;
        rm.scale.x = m.scale.x;
        rm.scale.y = m.scale.y;
        rm.scale.z = m.scale.z;
        rm.color.r = m.color.r;
        rm.color.g = m.color.g;
        rm.color.b = m.color.b;
        rm.color.a = m.color.a;
        rm.lifetime = ros::Duration(m.lifetime.sec + m.lifetime.nsec * 1e-9);
        rm.frame_locked = m.frame_locked;
        for (const auto & pt : m.points) {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            rm.points.push_back(p);
        }
        for (const auto & c : m.colors) {
            std_msgs::ColorRGBA cc;
            cc.r = c.r; cc.g = c.g; cc.b = c.b; cc.a = c.a;
            rm.colors.push_back(cc);
        }
        rm.text = m.text;
        rm.mesh_resource = m.mesh_resource;
        rm.mesh_use_embedded_materials = m.mesh_use_embedded_materials;
        out.markers.push_back(rm);
    }
    return out;
}

static nav_msgs::OccupancyGrid FromCoccupancy(const xloc_occupancy_grid_t* cg)
{
    nav_msgs::OccupancyGrid out;
    if (!cg) return out;
    out.header.stamp = ros::Time(cg->header.stamp.sec, cg->header.stamp.nsec);
    if (cg->header.frame_id) out.header.frame_id = cg->header.frame_id;
    out.info.resolution = cg->resolution;
    out.info.width = cg->width;
    out.info.height = cg->height;
    out.info.origin.position.x = cg->origin.position[0];
    out.info.origin.position.y = cg->origin.position[1];
    out.info.origin.position.z = cg->origin.position[2];
    out.info.origin.orientation.x = cg->origin.orientation[0];
    out.info.origin.orientation.y = cg->origin.orientation[1];
    out.info.origin.orientation.z = cg->origin.orientation[2];
    out.info.origin.orientation.w = cg->origin.orientation[3];
    out.data.resize(cg->data_length);
    for (size_t i = 0; i < cg->data_length; ++i) out.data[i] = static_cast<int8_t>(cg->data[i]);
    return out;
}

void MainThread_C(xloc_handle_t handle, tf3::BufferCore* tf_buffer, tf2_ros::TransformBroadcaster & tf_broadcaster,
                  ros::Publisher & diagnostics_pub, ros::Publisher & pose_pub)
{
    while (ros::ok()) {
        if (!handle) { std::this_thread::sleep_for(100ms); continue; }
        xloc_diagnostics_t* d = nullptr;
        try {
            d = xloc_get_diagnostics(handle);
            if (d) {
                xloc_ros_wrapper::Diagnostics diag_msg;
                diag_msg.header.stamp = ros::Time(d->header.stamp.sec, d->header.stamp.nsec);
                if (d->header.frame_id) diag_msg.header.frame_id = d->header.frame_id;
                diag_msg.xloc_state = d->xloc_state;
                if (d->current_active_map) diag_msg.current_active_map = d->current_active_map;
                diag_msg.reliability = d->reliability;
                diag_msg.matching_score = d->matching_score;
                diagnostics_pub.publish(diag_msg);
                xloc_free_diagnostics(d);
            }
        } catch(...) { if (d) xloc_free_diagnostics(d); }

        xloc_pose_t* p = nullptr;
        try {
            p = xloc_get_current_pose(handle);
            if (p) {
                geometry_msgs::PoseStamped pose_msg = FromCpose(p, nullptr);
                pose_pub.publish(pose_msg);
                xloc_free_pose(p);
            }
        } catch(...) { if (p) xloc_free_pose(p); }

        // publish TFs from buffer if available
        try {
            tf3::TransformStampedMsg map_to_odom = tf_buffer->lookupTransform("map", "odom", tf3::Time());
            tf3::TransformStampedMsg odom_to_base = tf_buffer->lookupTransform("odom", "base_link", tf3::Time());
            std::vector<geometry_msgs::TransformStamped> tv;
            geometry_msgs::TransformStamped t1, t2;
            t1.header.stamp = ros::Time(map_to_odom.header.stamp.sec, map_to_odom.header.stamp.nsec);
            t1.header.frame_id = map_to_odom.header.frame_id;
            t1.child_frame_id = map_to_odom.child_frame_id;
            t1.transform.translation.x = map_to_odom.transform.translation.x;
            t1.transform.translation.y = map_to_odom.transform.translation.y;
            t1.transform.translation.z = map_to_odom.transform.translation.z;
            t1.transform.rotation.x = map_to_odom.transform.rotation.x;
            t1.transform.rotation.y = map_to_odom.transform.rotation.y;
            t1.transform.rotation.z = map_to_odom.transform.rotation.z;
            t1.transform.rotation.w = map_to_odom.transform.rotation.w;
            tv.push_back(t1);
            t2.header.stamp = ros::Time(odom_to_base.header.stamp.sec, odom_to_base.header.stamp.nsec);
            t2.header.frame_id = odom_to_base.header.frame_id;
            t2.child_frame_id = odom_to_base.child_frame_id;
            t2.transform.translation.x = odom_to_base.transform.translation.x;
            t2.transform.translation.y = odom_to_base.transform.translation.y;
            t2.transform.translation.z = odom_to_base.transform.translation.z;
            t2.transform.rotation.x = odom_to_base.transform.rotation.x;
            t2.transform.rotation.y = odom_to_base.transform.rotation.y;
            t2.transform.rotation.z = odom_to_base.transform.rotation.z;
            t2.transform.rotation.w = odom_to_base.transform.rotation.w;
            tv.push_back(t2);
            tf_broadcaster.sendTransform(tv);
        } catch(...) {}

        std::this_thread::sleep_for(100ms);
    }
}

void PubVisualizationThread_C(xloc_handle_t handle, ros::Publisher& traj_pub, ros::Publisher& constraint_pub)
{
    ros::Rate rate(1.0);
    while (ros::ok()) {
        if (!handle) { rate.sleep(); continue; }
        xloc_marker_array_t* tma = xloc_get_trajectory_node_list_marker(handle);
        xloc_marker_array_t* cma = xloc_get_constraint_list_marker(handle);
        visualization_msgs::MarkerArray traj = FromXlocMarkerArrayPtr(tma);
        visualization_msgs::MarkerArray constraints = FromXlocMarkerArrayPtr(cma);
        traj_pub.publish(traj);
        constraint_pub.publish(constraints);
        if (tma) xloc_free_marker_array(tma);
        if (cma) xloc_free_marker_array(cma);
        rate.sleep();
    }
}

void PubMapThread_C(xloc_handle_t handle, ros::Publisher& occ_pub)
{
    xloc_unix_time_t last_published_time{0,0};
    uint32_t last_published_seq = 0;
    while (ros::ok()) {
        if (!handle) { std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue; }
        xloc_diagnostics_t* d = xloc_get_diagnostics(handle);
        bool mapping = false;
        if (d) { mapping = (d->xloc_state == xloc::MAPPING); xloc_free_diagnostics(d); }
        if (mapping) {
            // publish online grid at 1 Hz when mapping
            xloc_occupancy_grid_t* online = xloc_get_online_grid_map(handle);
            if (online && online->width > 0 && online->data_length > 0) {
                bool changed = (online->header.seq != last_published_seq) ||
                               (online->header.stamp.sec != last_published_time.sec) ||
                               (online->header.stamp.nsec != last_published_time.nsec);
                if (changed) {
                    occ_pub.publish(FromCoccupancy(online));
                    last_published_seq = online->header.seq;
                    last_published_time.sec = online->header.stamp.sec;
                    last_published_time.nsec = online->header.stamp.nsec;
                }
            }
            if (online) xloc_free_occupancy_grid(online);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else {
            // publish static grid at ~3 Hz when not mapping
            xloc_occupancy_grid_t* static_map = xloc_get_static_grid_map(handle, 0);
            if (static_map && static_map->width > 0 && static_map->data_length > 0) {
                bool changed = (static_map->header.seq != last_published_seq) ||
                               (static_map->header.stamp.sec != last_published_time.sec) ||
                               (static_map->header.stamp.nsec != last_published_time.nsec);
                if (changed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    occ_pub.publish(FromCoccupancy(static_map));
                    last_published_seq = static_map->header.seq;
                    last_published_time.sec = static_map->header.stamp.sec;
                    last_published_time.nsec = static_map->header.stamp.nsec;
                }
            }
            if (static_map) xloc_free_occupancy_grid(static_map);
            std::this_thread::sleep_for(std::chrono::milliseconds(333));
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xloc_ros_node_capi");
    ros::NodeHandle nh;

    // Create TF buffer and set an example transform like original node
    tf3::BufferCore* tf_buffer = new tf3::BufferCore(tf3::Duration(10.0));
    tf3::TransformStampedMsg base_to_scan_1;
    base_to_scan_1.header.frame_id = "base_link";
    base_to_scan_1.child_frame_id = "scan_1";
    base_to_scan_1.header.stamp = tf3::Time::now();
    base_to_scan_1.transform.translation.x = 0.2575;
    base_to_scan_1.transform.translation.y = 0.0;
    base_to_scan_1.transform.translation.z = 0.0;
    base_to_scan_1.transform.rotation.x = 0.0;
    base_to_scan_1.transform.rotation.y = 0.0;
    base_to_scan_1.transform.rotation.z = 0.0099998;
    base_to_scan_1.transform.rotation.w = 0.99995;
    tf_buffer->setTransform(base_to_scan_1, "xloc_ros_wrapper", true);
    tf3::TransformStampedMsg base_to_scan_2;
    base_to_scan_2.header.frame_id = "base_link";
    base_to_scan_2.child_frame_id = "scan_2";
    base_to_scan_2.header.stamp = tf3::Time::now();
    base_to_scan_2.transform.translation.x = -0.325;
    base_to_scan_2.transform.translation.y = 0.133;
    base_to_scan_2.transform.translation.z = 0.0;
    base_to_scan_2.transform.rotation.x = 0.0;
    base_to_scan_2.transform.rotation.y = 0.0;
    base_to_scan_2.transform.rotation.z = 1.0;
    base_to_scan_2.transform.rotation.w = 0.0;
    tf_buffer->setTransform(base_to_scan_2, "xloc_ros_wrapper", true);
    tf3::TransformStampedMsg base_to_imu;
    base_to_imu.header.frame_id = "base_link";
    base_to_imu.child_frame_id = "imu";
    base_to_imu.header.stamp = tf3::Time::now();
    base_to_imu.transform.translation.x = 0.0;
    base_to_imu.transform.translation.y = 0.0;
    base_to_imu.transform.translation.z = 0.0;
    base_to_imu.transform.rotation.x = 1.0;
    base_to_imu.transform.rotation.y = 0.0;
    base_to_imu.transform.rotation.z = 0.0;
    base_to_imu.transform.rotation.w = 0.0;
    tf_buffer->setTransform(base_to_imu, "xloc_ros_wrapper", true);

    // Create XLOC via C API (pass tf buffer pointer)
    xloc_handle_t handle = xloc_create(reinterpret_cast<TF3_BufferCore>(tf_buffer));

    // Publishers using same topics as original node
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::MarkerArray>("xloc/trajectory_nodes", 10);
    ros::Publisher constraint_pub = nh.advertise<visualization_msgs::MarkerArray>("xloc/constraints", 10);
    ros::Publisher diagnostics_pub = nh.advertise<xloc_ros_wrapper::Diagnostics>("xloc/diagnostics", 10);
    ros::Publisher occ_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("xloc/current_pose", 10);

    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Subscribers: mirror original node and dispatch via C API
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan_1", 10,
        [&](const sensor_msgs::LaserScan::ConstPtr& msg){
            if(!handle) return;
            xloc_laserscan_t scan;
            scan.header.seq = msg->header.seq;
            scan.header.stamp.sec = msg->header.stamp.sec;
            scan.header.stamp.nsec = msg->header.stamp.nsec;
            scan.header.frame_id = const_cast<char*>(msg->header.frame_id.c_str());
            scan.angle_min = msg->angle_min;
            scan.angle_max = msg->angle_max;
            scan.angle_increment = msg->angle_increment;
            scan.time_increment = msg->time_increment;
            scan.scan_time = msg->scan_time;
            scan.range_min = msg->range_min;
            scan.range_max = msg->range_max;
            scan.ranges_length = msg->ranges.size();
            scan.ranges = nullptr;
            if(scan.ranges_length){
                scan.ranges = (float*)std::malloc(sizeof(float)*scan.ranges_length);
                for(size_t i=0;i<scan.ranges_length;++i) scan.ranges[i] = msg->ranges[i];
            }
            scan.intensities_length = msg->intensities.size();
            scan.intensities = nullptr;
            if(scan.intensities_length){
                scan.intensities = (float*)std::malloc(sizeof(float)*scan.intensities_length);
                for(size_t i=0;i<scan.intensities_length;++i) scan.intensities[i] = msg->intensities[i];
            }
            xloc_dispatch_laserscan(handle, "scan_1", &scan);
            if(scan.ranges) std::free(scan.ranges);
            if(scan.intensities) std::free(scan.intensities);
        });

    ros::Subscriber scan_sub_2 = nh.subscribe<sensor_msgs::LaserScan>("scan_2", 10,
        [&](const sensor_msgs::LaserScan::ConstPtr& msg){
            if(!handle) return;
            xloc_laserscan_t scan;
            scan.header.seq = msg->header.seq;
            scan.header.stamp.sec = msg->header.stamp.sec;
            scan.header.stamp.nsec = msg->header.stamp.nsec;
            scan.header.frame_id = const_cast<char*>(msg->header.frame_id.c_str());
            scan.angle_min = msg->angle_min;
            scan.angle_max = msg->angle_max;
            scan.angle_increment = msg->angle_increment;
            scan.time_increment = msg->time_increment;
            scan.scan_time = msg->scan_time;
            scan.range_min = msg->range_min;
            scan.range_max = msg->range_max;
            scan.ranges_length = msg->ranges.size();
            scan.ranges = nullptr;
            if(scan.ranges_length){
                scan.ranges = (float*)std::malloc(sizeof(float)*scan.ranges_length);
                for(size_t i=0;i<scan.ranges_length;++i) scan.ranges[i] = msg->ranges[i];
            }
            scan.intensities_length = msg->intensities.size();
            scan.intensities = nullptr;
            if(scan.intensities_length){
                scan.intensities = (float*)std::malloc(sizeof(float)*scan.intensities_length);
                for(size_t i=0;i<scan.intensities_length;++i) scan.intensities[i] = msg->intensities[i];
            }
            xloc_dispatch_laserscan(handle, "scan_2", &scan);
            if(scan.ranges) std::free(scan.ranges);
            if(scan.intensities) std::free(scan.intensities);
        });

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10,
        [&](const nav_msgs::Odometry::ConstPtr& msg){
            if(!handle) return;
            xloc_odometry_t odom;
            odom.header.seq = msg->header.seq;
            odom.header.stamp.sec = msg->header.stamp.sec;
            odom.header.stamp.nsec = msg->header.stamp.nsec;
            odom.header.frame_id = const_cast<char*>(msg->header.frame_id.c_str());
            odom.child_frame_id = const_cast<char*>(msg->child_frame_id.c_str());
            odom.pose_position[0] = msg->pose.pose.position.x;
            odom.pose_position[1] = msg->pose.pose.position.y;
            odom.pose_position[2] = msg->pose.pose.position.z;
            odom.pose_orientation[0] = msg->pose.pose.orientation.x;
            odom.pose_orientation[1] = msg->pose.pose.orientation.y;
            odom.pose_orientation[2] = msg->pose.pose.orientation.z;
            odom.pose_orientation[3] = msg->pose.pose.orientation.w;
            for(int i=0;i<36;++i) odom.pose_covariance[i] = msg->pose.covariance[i];
            odom.twist_linear[0] = msg->twist.twist.linear.x;
            odom.twist_linear[1] = msg->twist.twist.linear.y;
            odom.twist_linear[2] = msg->twist.twist.linear.z;
            odom.twist_angular[0] = msg->twist.twist.angular.x;
            odom.twist_angular[1] = msg->twist.twist.angular.y;
            odom.twist_angular[2] = msg->twist.twist.angular.z;
            for(int i=0;i<36;++i) odom.twist_covariance[i] = msg->twist.covariance[i];
            xloc_dispatch_odometry(handle, "odom", &odom);
        });

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10,
        [&](const sensor_msgs::Imu::ConstPtr& msg){
            if(!handle) return;
            xloc_imu_t imu;
            imu.header.seq = msg->header.seq;
            imu.header.stamp.sec = msg->header.stamp.sec;
            imu.header.stamp.nsec = msg->header.stamp.nsec;
            imu.header.frame_id = const_cast<char*>(msg->header.frame_id.c_str());
            imu.orientation[0] = msg->orientation.x;
            imu.orientation[1] = msg->orientation.y;
            imu.orientation[2] = msg->orientation.z;
            imu.orientation[3] = msg->orientation.w;
            for(int i=0;i<9;++i) imu.orientation_covariance[i] = msg->orientation_covariance[i];
            imu.angular_velocity[0] = msg->angular_velocity.x;
            imu.angular_velocity[1] = msg->angular_velocity.y;
            imu.angular_velocity[2] = msg->angular_velocity.z;
            for(int i=0;i<9;++i) imu.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
            imu.linear_acceleration[0] = msg->linear_acceleration.x;
            imu.linear_acceleration[1] = msg->linear_acceleration.y;
            imu.linear_acceleration[2] = msg->linear_acceleration.z;
            for(int i=0;i<9;++i) imu.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
            xloc_dispatch_imu(handle, "imu", &imu);
        });

    ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10,
        [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
            if(!handle) return;
            xloc_pose_t ip;
            ip.position[0] = msg->pose.pose.position.x;
            ip.position[1] = msg->pose.pose.position.y;
            ip.position[2] = msg->pose.pose.position.z;
            ip.orientation[0] = msg->pose.pose.orientation.x;
            ip.orientation[1] = msg->pose.pose.orientation.y;
            ip.orientation[2] = msg->pose.pose.orientation.z;
            ip.orientation[3] = msg->pose.pose.orientation.w;
            xloc_status_response_t r = xloc_set_initial_pose(handle, &ip);
            xloc_free_status_response(&r);
        });

    // Services (map control, start/stop, etc.) using C API
    ros::ServiceServer srv_activate = nh.advertiseService<xloc_ros_wrapper::ActivateMap::Request, xloc_ros_wrapper::ActivateMap::Response>("activate_map",
        [&](xloc_ros_wrapper::ActivateMap::Request& req, xloc_ros_wrapper::ActivateMap::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_activate_map(handle, req.map_file_name.c_str());
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_start_mapping = nh.advertiseService<xloc_ros_wrapper::StartMapping::Request, xloc_ros_wrapper::StartMapping::Response>("start_mapping",
        [&](xloc_ros_wrapper::StartMapping::Request& req, xloc_ros_wrapper::StartMapping::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_start_mapping(handle);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_stop_mapping = nh.advertiseService<xloc_ros_wrapper::StopMapping::Request, xloc_ros_wrapper::StopMapping::Response>("stop_mapping",
        [&](xloc_ros_wrapper::StopMapping::Request& req, xloc_ros_wrapper::StopMapping::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_stop_mapping(handle, req.map_file_name.c_str());
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_start_loc = nh.advertiseService<xloc_ros_wrapper::StartLocalization::Request, xloc_ros_wrapper::StartLocalization::Response>("start_localization",
        [&](xloc_ros_wrapper::StartLocalization::Request& req, xloc_ros_wrapper::StartLocalization::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_start_localization(handle);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_stop_loc = nh.advertiseService<xloc_ros_wrapper::StopLocalization::Request, xloc_ros_wrapper::StopLocalization::Response>("stop_localization",
        [&](xloc_ros_wrapper::StopLocalization::Request& req, xloc_ros_wrapper::StopLocalization::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_stop_localization(handle);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_set_initial = nh.advertiseService<xloc_ros_wrapper::SetInitialPose::Request, xloc_ros_wrapper::SetInitialPose::Response>("set_initial_pose",
        [&](xloc_ros_wrapper::SetInitialPose::Request& req, xloc_ros_wrapper::SetInitialPose::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_pose_t ip;
            ip.position[0] = req.initial_pose.position.x;
            ip.position[1] = req.initial_pose.position.y;
            ip.position[2] = req.initial_pose.position.z;
            ip.orientation[0] = req.initial_pose.orientation.x;
            ip.orientation[1] = req.initial_pose.orientation.y;
            ip.orientation[2] = req.initial_pose.orientation.z;
            ip.orientation[3] = req.initial_pose.orientation.w;
            xloc_status_response_t r = xloc_set_initial_pose(handle, &ip);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_reset = nh.advertiseService<xloc_ros_wrapper::ResetSlamError::Request, xloc_ros_wrapper::ResetSlamError::Response>("reset_slam_error",
        [&](xloc_ros_wrapper::ResetSlamError::Request& req, xloc_ros_wrapper::ResetSlamError::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_reset_slam_error(handle);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_get_pose = nh.advertiseService<xloc_ros_wrapper::GetCurrentPose::Request, xloc_ros_wrapper::GetCurrentPose::Response>("get_current_pose",
        [&](xloc_ros_wrapper::GetCurrentPose::Request& req, xloc_ros_wrapper::GetCurrentPose::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_pose_t* p = xloc_get_current_pose(handle);
            if(p){ res.pose = FromCpose(p, nullptr); xloc_free_pose(p); }
            res.code = xloc::OK; res.message = "";
            return true;
        });

    ros::ServiceServer srv_switch_map = nh.advertiseService<xloc_ros_wrapper::SwitchMap::Request, xloc_ros_wrapper::SwitchMap::Response>("switch_map",
        [&](xloc_ros_wrapper::SwitchMap::Request& req, xloc_ros_wrapper::SwitchMap::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_pose_t ip;
            if(req.use_initial_pose){
                ip.position[0] = req.initial_pose.position.x; ip.position[1] = req.initial_pose.position.y; ip.position[2] = req.initial_pose.position.z;
                ip.orientation[0] = req.initial_pose.orientation.x; ip.orientation[1] = req.initial_pose.orientation.y; ip.orientation[2] = req.initial_pose.orientation.z; ip.orientation[3] = req.initial_pose.orientation.w;
            }
            xloc_status_response_t r = xloc_switch_map(handle, req.map_file_name.c_str(), req.use_initial_pose ? 1 : 0, req.use_initial_pose ? &ip : nullptr);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_change_origin = nh.advertiseService<xloc_ros_wrapper::ChangeMapOrigin::Request, xloc_ros_wrapper::ChangeMapOrigin::Response>("change_map_origin",
        [&](xloc_ros_wrapper::ChangeMapOrigin::Request& req, xloc_ros_wrapper::ChangeMapOrigin::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_pose_t np;
            np.position[0] = req.new_map_origin.position.x; np.position[1] = req.new_map_origin.position.y; np.position[2] = req.new_map_origin.position.z;
            np.orientation[0] = req.new_map_origin.orientation.x; np.orientation[1] = req.new_map_origin.orientation.y; np.orientation[2] = req.new_map_origin.orientation.z; np.orientation[3] = req.new_map_origin.orientation.w;
            xloc_status_response_t r = xloc_change_map_origin(handle, &np);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_start_update = nh.advertiseService<xloc_ros_wrapper::StartUpdateMap::Request, xloc_ros_wrapper::StartUpdateMap::Response>("start_update_map",
        [&](xloc_ros_wrapper::StartUpdateMap::Request& req, xloc_ros_wrapper::StartUpdateMap::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_start_update_map(handle);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    ros::ServiceServer srv_stop_update = nh.advertiseService<xloc_ros_wrapper::StopUpdateMap::Request, xloc_ros_wrapper::StopUpdateMap::Response>("stop_update_map",
        [&](xloc_ros_wrapper::StopUpdateMap::Request& req, xloc_ros_wrapper::StopUpdateMap::Response& res)->bool{
            if(!handle){ res.code = 255; res.message = "xloc not available"; return true; }
            xloc_status_response_t r = xloc_stop_update_map(handle, req.save_updated_map ? 1 : 0);
            res.code = r.code; res.message = r.message ? std::string(r.message) : std::string("");
            xloc_free_status_response(&r);
            return true;
        });

    // Start threads
    std::thread main_thread(MainThread_C, handle, tf_buffer, std::ref(tf_broadcaster), std::ref(diagnostics_pub), std::ref(pose_pub));
    std::thread vis_thread(PubVisualizationThread_C, handle, std::ref(traj_pub), std::ref(constraint_pub));
    std::thread map_thread(PubMapThread_C, handle, std::ref(occ_pub));

    ros::spin();

    // Cleanup
    if (handle) xloc_destroy(handle);
    delete tf_buffer;
    if (main_thread.joinable()) main_thread.join();
    if (vis_thread.joinable()) vis_thread.join();
    if (map_thread.joinable()) map_thread.join();
    return 0;
}
