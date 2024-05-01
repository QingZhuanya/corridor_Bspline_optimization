

// #include "hybrid_a_star/goal_pose_subscriber.h"
#include <hybrid_a_star/goal_pose_subscriber.h>

GoalPoseSubscriber2D::GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &GoalPoseSubscriber2D::MessageCallBack, this
    );
}

void GoalPoseSubscriber2D::MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose_ptr) {
    buff_mutex_.lock();
    goal_poses_.emplace_back(goal_pose_ptr);
    buff_mutex_.unlock();
}

void GoalPoseSubscriber2D::ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff) {
    buff_mutex_.lock();
    if (!goal_poses_.empty()) {
        pose_data_buff.insert(pose_data_buff.end(), goal_poses_.begin(), goal_poses_.end());
        goal_poses_.clear();
    }
    buff_mutex_.unlock();
}



