

#ifndef HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
#define HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <deque>
#include <mutex>
#include <thread>
#include <string>

class CostMapSubscriber {
public:
    CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr);

private:
    void MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::OccupancyGridPtr> deque_costmap_;

    std::mutex buff_mutex_;
};

#endif //HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
