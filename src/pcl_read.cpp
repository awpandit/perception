#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>

// This header contains required definitions to load and store coulds to PCD and other file formats.
#include <pcl/io/pcd_io.h>


main(int argc, char **argv)
{
  ros::init (argc, argv, "pcl_read");

  ros::NodeHandle nh;

  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

  sensor_msgs::PointCloud2 output;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  // Load the clouds from the disk
 pcl::io::loadPCDFile ("/home/akpndt/catkin_ws/src/perception/data/dataRGB_pcd_filtered.pcd", cloud);

  //pcl::io::loadPLYFile ("/home/akpndt/catkin_ws/src/perception/data/ganesha_final.ply", cloud);

  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom";
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
