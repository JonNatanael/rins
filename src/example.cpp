#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    /*sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl_conversions::toPCL(*input, *cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (cloud_filtered);
    pcl_conversions::fromPCL(cloud_filtered, output);
    pub.publish (output);*/


    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered_1;
    pcl_conversions::toPCL(*input, *cloud);

    //pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (cloud_filtered_1);

    pcl::fromPCLPointCloud2 (cloud_filtered_1, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (.01);
  seg.setInputCloud (cloud_filtered);
    
  seg.segment(*inliers, *coefficients);

  // Exit if no plane found
  if (inliers->indices.size () == 0) return;

  // Extract points of found plane
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_f);

  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*cloud_f, outcloud);

    sensor_msgs::PointCloud2 output;  
    pcl_conversions::fromPCL(outcloud, output);
    pub.publish(output);

  // Publish the plane to a new topic.
  /*pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*cloud_f, outcloud);
  pub.publish (outcloud);*/






}

int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "rins");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud (the name can be remapped in the launch file)
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud (the name can be remapped in the launch file)
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin ...
    ros::spin ();
}
