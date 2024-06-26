

#ifndef HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <deque>
#include <mutex>

class InitPoseSubscriber2D {
public:
    InitPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name,
                         size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_poses_;

    std::mutex buff_mutex_;
};

#endif //HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H
