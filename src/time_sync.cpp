#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


void sensorCallback(const sensor_msgs::PointCloud2ConstPtr &cloud1, const sensor_msgs::PointCloud2ConstPtr &cloud2)
{
  ROS_INFO("Time sync is working");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "time_sync");
  ros::NodeHandle nh;

  std::string cloud1, cloud2;
  nh.getParam("input_cloud_topic", cloud1);
  nh.getParam("input_image_topic", cloud2);


  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "cloud1", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "cloud2", 1);
  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(cloud1_sub, cloud2_sub, 10);
  sync.registerCallback(boost::bind(&sensorCallback, _1, _2));

  ros::spin();
  ROS_ERROR("Time sync is working");

  return 0;
}
