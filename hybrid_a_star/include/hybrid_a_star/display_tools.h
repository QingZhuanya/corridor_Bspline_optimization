

#ifndef HYBRID_A_STAR_DISPLAY_TOOLS_H
#define HYBRID_A_STAR_DISPLAY_TOOLS_H

#include "type.h"
#include <math/vec2d.h>
#include <math/math_utils.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

__attribute__((unused)) static void PublishSearchedTree(const TypeVectorVecd<4> &tree, const std::string &topic_name) {
    static ros::NodeHandle node_handle("~");
    static ros::Publisher tree_pub = node_handle.advertise<visualization_msgs::Marker>(topic_name, 10);

    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 0.8;
    tree_list.color.r = 136;
    tree_list.color.g = 138;
    tree_list.color.b = 133;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    tree_pub.publish(tree_list);
}

__attribute__((unused)) static void PublishPath(ros::Publisher &path_pub, const TypeVectorVecd<3> &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());
        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();

    path_pub.publish(nav_path);
}

__attribute__((unused)) static void PublishPath(const TypeVectorVecd<3> &path, const std::string &topic_name) {
    static ros::NodeHandle node_handle("~");
    static ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>(topic_name, 1);

    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());
        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();

    path_pub.publish(nav_path);
}

__attribute__((unused)) static void PublishPath(const TypeVectorVecd<2> &path, const std::string &topic_name) {
    static ros::NodeHandle node_handle("~");
    static ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>(topic_name, 10);

    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();

    path_pub.publish(nav_path);
}

__attribute__((unused)) static void PublishEllipse(const Eigen::Vector2d &x_center, double &c_best, double dist,
                                                   double theta, const std::string &topic_name) {
    double a = std::sqrt(c_best * c_best - dist * dist) * 0.5;
    double b = c_best * 0.5;
    double angle = M_PI / 2 - theta;

    Mat3d R_z = Eigen::AngleAxisd(-angle, Vec3d::UnitZ()).toRotationMatrix();

    TypeVectorVecd<2> ellipse_path;

    Eigen::Vector2d coord;
    for (double t = 0; t <= 2.0 * M_PI + 0.1;) {
        coord.x() = a * std::cos(t);
        coord.y() = b * std::sin(t);

        coord = R_z.block<2, 2>(0, 0) * coord + x_center;
        ellipse_path.emplace_back(coord);

        t += 0.1;
    }

    static ros::NodeHandle node_handle("~");
    static ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>(topic_name, 10);

    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: ellipse_path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();

    path_pub.publish(nav_path);
}

#endif //HYBRID_A_STAR_DISPLAY_TOOLS_H
