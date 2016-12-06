#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <perception/reconfigConfig.h>

#include <iostream>

// Global variables for dynamic reconfiguration GUI.
double_t leafsize;
double_t filter_mean, filter_threshold, segmentation_threshold, ClusterTolerance;
int32_t segmentation_maxiteration, ClusterMinSize, ClusterMaxSize;
double_t passFilterMin_x, passFilterMin_y, passFilterMin_z, passFilterMax_x, passFilterMax_y, passFilterMax_z;
double_t convexHullHeightMin, convexHullHeightMax;

class cloudHandler
{
public:
  cloudHandler()
  {
    pcl_cloud_sub = nh.subscribe("pcl_output", 10, &cloudHandler::cloudCB, this);

    pcl_downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled", 1);
    pcl_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_processed", 1);
    ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
    coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
  }

  void cloudCB(const sensor_msgs::PointCloud2 &input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mean_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_convex_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    /// Downsampling
    sensor_msgs::PointCloud2 output_downsampled;

    pcl::fromROSMsg(input, *cloud);

    // Creating object for downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
    voxelSampler.setInputCloud(cloud->makeShared());
    voxelSampler.setLeafSize(leafsize, leafsize, leafsize);
    voxelSampler.filter(*cloud_downsampled);

    pcl::toROSMsg(*cloud_downsampled, output_downsampled);
    pcl_downsample_pub.publish(output_downsampled);

    /// Filtering
    sensor_msgs::PointCloud2 output_mean_filtered;
    sensor_msgs::PointCloud2 output_pass_filtered;

    // Creating object for statistical outlier filter.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilter;
    statFilter.setInputCloud(cloud_downsampled->makeShared());
    statFilter.setMeanK(filter_mean);
    statFilter.setStddevMulThresh(filter_threshold);
    statFilter.filter(*cloud_mean_filtered);

    pcl::toROSMsg(*cloud_mean_filtered, output_mean_filtered);

    // Creating object for passthrough filter.
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud_mean_filtered->makeShared());
    filter.setFilterFieldName("x");
    filter.setFilterLimits(passFilterMin_x, passFilterMax_x);
    filter.filter(*cloud_pass_filtered);

    filter.setFilterFieldName("y");
    filter.setFilterLimits(passFilterMin_y, passFilterMax_y);
    filter.filter(*cloud_pass_filtered);

    filter.setFilterFieldName("z");
    filter.setFilterLimits(passFilterMin_z, passFilterMax_z);
    filter.filter(*cloud_pass_filtered);

    pcl::toROSMsg(*cloud_pass_filtered, output_pass_filtered);
    pcl_filter_pub.publish(output_pass_filtered);

    /// Planar segmentation
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(segmentation_maxiteration);
    segmentation.setDistanceThreshold(segmentation_threshold);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setInputCloud(cloud_pass_filtered->makeShared());
    segmentation.segment(*inliers, coefficients);

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    coef_pub.publish(ros_coefficients);

    // Publish the Point Indices
    pcl_msgs::PointIndices ros_inliers;
    pcl_conversions::fromPCL(*inliers, ros_inliers);
    ind_pub.publish(ros_inliers);

    if (inliers->indices.size() == 0)
      std::cout << "Could not find a plane in the scene." << std::endl;
    else
    {
      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_pass_filtered->makeShared());
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*cloud_segmented);

      /// Convex Hull
      // Retrieve the convex hull.
      pcl::ConvexHull<pcl::PointXYZRGB> hull;
      hull.setInputCloud(cloud_segmented->makeShared());
      // Make sure that the resulting hull is bidimensional.
      hull.setDimension(2);
      hull.reconstruct(*cloud_convex_hull);

      // Redundant check.
      if (hull.getDimension() == 2)
      {
        // Prism object.
        pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
        prism.setInputCloud(cloud_pass_filtered->makeShared());
        prism.setInputPlanarHull(cloud_convex_hull->makeShared());

        // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
        // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
        prism.setHeightLimits(convexHullHeightMin, convexHullHeightMax);
        pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

        prism.segment(*objectIndices);

        // Get all points retrieved by the hull.
        extract.setIndices(objectIndices);
        extract.filter(*cluster);

        /// KdTree search
        // kd-tree object.
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

        // Perform the search, and print out results.
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (ClusterTolerance);
        ec.setMinClusterSize (ClusterMinSize);
        ec.setMaxClusterSize (ClusterMaxSize);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud (cluster->makeShared());
        ec.extract (cluster_indices);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cluster->points[*pit]);
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*cloud_cluster, centroid);
          std::cout << "x :" << centroid[0] << std::endl;
          std::cout << "y :" << centroid[1] << std::endl;
          std::cout << "z :" << centroid[2] << std::endl;

          //Publish the new cloud
          sensor_msgs::PointCloud2 output;
          pcl::toROSMsg(*cloud_cluster, output);
          pcl_pub.publish(output);
        }


      }
      else std::cout << "The chosen hull is not planar." << std::endl;
    }
  }

protected:
  // Node handler
  ros::NodeHandle nh;
  // Subscriber node
  ros::Subscriber pcl_cloud_sub;
  // Publisher nodes
  ros::Publisher pcl_downsample_pub, pcl_filter_pub, pcl_pub, ind_pub, coef_pub;
};

// Function for dynamic reconfiguration GUI.
void callback(perception::reconfigConfig &config, uint32_t level)
{
  leafsize = config.leafsize;
  filter_mean = config.filter_mean;
  filter_threshold = config.filter_thresold;
  segmentation_threshold = config.segmentation_thresold;
  segmentation_maxiteration = config.segmentation_maxiteration;
  passFilterMin_x = config.passFilterMin_x;
  passFilterMin_y = config.passFilterMin_y;
  passFilterMin_z = config.passFilterMin_z;
  passFilterMax_x = config.passFilterMax_x;
  passFilterMax_y = config.passFilterMax_y;
  passFilterMax_z = config.passFilterMax_z;
  ClusterTolerance = config.ClusterTolerance;
  ClusterMinSize = config.ClusterMinSize;
  ClusterMaxSize = config.ClusterMaxSize;
  convexHullHeightMin = config.convexHullHeightMin;
  convexHullHeightMax = config.convexHullHeightMax;
}

main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_process");

  dynamic_reconfigure::Server<perception::reconfigConfig> server;
  dynamic_reconfigure::Server<perception::reconfigConfig>::CallbackType fun;
  fun = boost::bind(&callback, _1, _2);
  server.setCallback(fun);

  cloudHandler handler;

  ros::spin();

  return 0;
}



