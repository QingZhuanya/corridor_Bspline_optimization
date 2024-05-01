#include <plan_manage/planning_visualization.h>

using std::cout;
using std::endl;
namespace opt_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    // goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    // global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    // init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    path_smooth_pub = nh.advertise<visualization_msgs::Marker>("path_smooth", 2);
    // a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    // sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = 0;
      // pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  // void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  // {
  //   visualization_msgs::Marker sphere;
  //   sphere.header.frame_id = "world";
  //   sphere.header.stamp = ros::Time::now();
  //   sphere.type = visualization_msgs::Marker::SPHERE;
  //   sphere.action = visualization_msgs::Marker::ADD;
  //   sphere.id = id;

  //   sphere.pose.orientation.w = 1.0;
  //   sphere.color.r = color(0);
  //   sphere.color.g = color(1);
  //   sphere.color.b = color(2);
  //   sphere.color.a = color(3);
  //   sphere.scale.x = scale;
  //   sphere.scale.y = scale;
  //   sphere.scale.z = scale;
  //   sphere.pose.position.x = goal_point(0);
  //   sphere.pose.position.y = goal_point(1);
  //   sphere.pose.position.z = goal_point(2);

  //   goal_point_pub.publish(sphere);
  // }

  // void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  // {

  //   if (global_list_pub.getNumSubscribers() == 0)
  //   {
  //     return;
  //   }

  //   Eigen::Vector4d color(0, 0.5, 0.5, 1);
  //   displayMarkerList(global_list_pub, init_pts, scale, color, id);
  // }

  // void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  // {

  //   if (init_list_pub.getNumSubscribers() == 0)
  //   {
  //     return;
  //   }

  //   Eigen::Vector4d color(0, 0, 1, 1);
  //   displayMarkerList(init_list_pub, init_pts, scale, color, id);
  // }
  void PlanningVisualization::PublishPath(ros::Publisher &pub, const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: list) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());//创建四元数

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    pub.publish(nav_path);
  }
  void PlanningVisualization::PublishWayPoint(ros::Publisher &pub, const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id){
    visualization_msgs::MarkerArray way_point_array;
    for (unsigned int i = 0; i < list.size(); i++) {
        visualization_msgs::Marker way_point;

        if (i == 0) {
            way_point.action = visualization_msgs::Marker::DELETEALL;
        }

        way_point.header.frame_id = "world";
        way_point.ns = "way_points";
        way_point.header.stamp = ros::Time(0);
        way_point.type = visualization_msgs::Marker::SPHERE;
        way_point.id = i;
        way_point.scale.x = 0.1;
        way_point.scale.y = 0.1;
        way_point.scale.z = 0.0;
        way_point.color.a = 1.0;

        way_point.color.r = 1.0;
        way_point.color.b = 1.0;
        way_point.color.g = 0.0;

        way_point.pose.position.x = list[i].x();
        way_point.pose.position.y = list[i].y();
        way_point.pose.position.z = 0.0;

        way_point.pose.orientation = tf::createQuaternionMsgFromYaw(list[i].z());
        way_point_array.markers.emplace_back(way_point);
    }
    pub.publish(way_point_array);
  }

  void PlanningVisualization::displayOptimalList(const vector<Eigen::Vector3d>& optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    
    Eigen::Vector4d color(1, 0, 0, 1);
    // PublishPath(optimal_list_pub, optimal_pts, 0.15, color, id);
    displayMarkerList(optimal_list_pub, optimal_pts, 0.15, color, id);
  }

  void PlanningVisualization::displaySmoothPathList(const vector<Eigen::Vector3d>& SmoothPath_pts, int id)
  {

    if (path_smooth_pub.getNumSubscribers() == 0)
    {
      return;
    }

    // vector<Eigen::Vector3d> list;
    // for (int i = 0; i < SmoothPath_pts.size(); i++)
    // {
    //   Eigen::Vector3d pt;
    //   pt.head(2) = SmoothPath_pts[i];
    //   list.push_back(pt);
    // }
    Eigen::Vector4d color(1, 0, 1, 0);
    displayMarkerList(path_smooth_pub, SmoothPath_pts, 0.15, color, id);
  }

  // void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  // {

  //   if (a_star_list_pub.getNumSubscribers() == 0)
  //   {
  //     return;
  //   }

  //   int i = 0;
  //   vector<Eigen::Vector3d> list;

  //   Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
  //   double scale = 0.05 + (double)rand() / RAND_MAX / 10;

  //   // for ( int i=0; i<10; i++ )
  //   // {
  //   //   //Eigen::Vector4d color(1,1,0,0);
  //   //   displayMarkerList(a_star_list_pub, list, scale, color, id+i);
  //   // }

  //   for (auto block : a_star_paths)
  //   {
  //     list.clear();
  //     for (auto pt : block)
  //     {
  //       list.push_back(pt);
  //     }
  //     //Eigen::Vector4d color(0.5,0.5,0,1);
  //     displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
  //     i++;
  //   }
  // }

  // void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  // {
  //   visualization_msgs::MarkerArray array;
  //   // clear
  //   pub.publish(array);

  //   generateArrowDisplayArray(array, list, scale, color, id);

  //   pub.publish(array);
  // }

  // PlanningVisualization::
} // namespace opt_planner