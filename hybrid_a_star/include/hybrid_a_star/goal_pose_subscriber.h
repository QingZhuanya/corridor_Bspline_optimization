

#ifndef HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <deque>
#include <mutex>

class GoalPoseSubscriber2D {
public:
    GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_poses_;

    std::mutex buff_mutex_;
};

#endif //HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
