#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <geonav_transform/RequestGPS.h>
#include <cstdlib>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <navgraph/navgraphDriveToXY.h>

namespace GPSGoalTransform
{
class GPSGoalTransform
{
  public:
    GPSGoalTransform();
    ~GPSGoalTransform();
    void run();

  private:
    void transform_gps_to_map();
    void transform_map_to_map_navgraph();
    bool cb_request_gps_srv(geonav_transform::RequestGPS::Request &req, geonav_transform::RequestGPS::Response &res);

    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_map_2_map_navgraph_;
    ros::ServiceServer request_gps_srv_;
    ros::ServiceClient request_gps_client_;
    ros::ServiceClient send_navgraph_goal_client_;

    std::string desired_map_frame_id_;
    std::string gps_map_frame_id_;
};

GPSGoalTransform::GPSGoalTransform():
  // TODO: rosparam frame_id
  gps_map_frame_id_("map"),
  desired_map_frame_id_("map_navgraph")
{ 
  // transform_map_2_map_navgraph_.header.frame_id = gps_map_frame_id_;
  // transform_map_2_map_navgraph_.child_frame_id = desired_map_frame_id_;
  // transform_map_2_map_navgraph_.header.seq = 0;
}

GPSGoalTransform::~GPSGoalTransform()
{
}

void GPSGoalTransform::run()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // variables
  double frequency = 5.0;

  // ROS params here
  nh_priv.param("frequency", frequency, 5.0);
  
  request_gps_srv_ = nh.advertiseService("navgraph/gps_request", &GPSGoalTransform::cb_request_gps_srv, this);
  request_gps_client_ = nh.serviceClient<geonav_transform::RequestGPS>("gps_to_map_xy");
  send_navgraph_goal_client_ = nh.serviceClient<navgraph::navgraphDriveToXY>("navgraph/drive_to_xy");



  // TODO:check if GPS available
  // TODO:check if service available
  // TODO:check if navgraph available

  // Loop
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    try{
      tf_listener_.lookupTransform( gps_map_frame_id_, desired_map_frame_id_, ros::Time(0), transform_map_2_map_navgraph_);
      // ROS_INFO("x:%f, y:%f", transform_map_2_map_navgraph_.getOrigin().x(), transform_map_2_map_navgraph_.getOrigin().y());
    } 
    catch (tf::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ros::spinOnce();
    // TODO: functions here

    rate.sleep();
  }   // end of Loop
}   // end of run()

bool GPSGoalTransform::cb_request_gps_srv(geonav_transform::RequestGPS::Request &req, geonav_transform::RequestGPS::Response &res) 
{

  geonav_transform::RequestGPS gps_srv;
  gps_srv.request.latitude = req.latitude;
  gps_srv.request.longitude = req.longitude;
  ROS_INFO("test %f, %f", gps_srv.request.latitude, gps_srv.request.longitude);
  if (request_gps_client_.call(gps_srv))
  {
    ROS_INFO("( X: %f, Y: %f ) in frame: %s", gps_srv.response.x, gps_srv.response.y, gps_srv.response.frame_id.c_str());
    ROS_INFO("( X: %f, Y: %f ) in frame: %s", gps_srv.response.x - transform_map_2_map_navgraph_.getOrigin().x() , gps_srv.response.y - transform_map_2_map_navgraph_.getOrigin().y(), "map_navgraph");
    navgraph::navgraphDriveToXY navgraph_srv;
    navgraph_srv.request.nearby_pose.x = gps_srv.response.x - transform_map_2_map_navgraph_.getOrigin().x();
    navgraph_srv.request.nearby_pose.y = gps_srv.response.y - transform_map_2_map_navgraph_.getOrigin().y();
    if (send_navgraph_goal_client_.call(navgraph_srv))
    {
      ROS_INFO("destination sent!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
  }
  else 
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  return 0;
} //cb_request_gps_srv

}   // namespace GeonavTransform 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_goal_transform");
  GPSGoalTransform::GPSGoalTransform gps_goal_trans;
  gps_goal_trans.run();
  return 0;
}
