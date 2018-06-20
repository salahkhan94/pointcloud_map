#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <math.h>


class PointCloudMap
{
public:
  PointCloudMap()
  {
      pub1 = n_.advertise<sensor_msgs::PointCloud2>("pointcloud_map", 5);
      pub2 = n_.advertise<sensor_msgs::PointCloud2>("pointcloud_transformed", 5);
      sub1 = n_.subscribe("/elas/point_cloud", 5, &PointCloudMap::transformCallback, this);  
      tf_listener = new tf::TransformListener();
      q = 0;
  }
  
  void ConcatAndPublish()
  {
       sensor_msgs::PointCloud2 pcl_out; 
       pcl::concatenatePointCloud(pcl_map,pcl_transform,pcl_out);
       pcl_map = pcl_out;
       pub1.publish(pcl_map);
  }
  
  void transformCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in) 
  { 
      ROS_INFO("Cloud received"); 
      sensor_msgs::PointCloud2 pcl_out; 
      tf_listener->waitForTransform("/map", (*pcl_in).header.frame_id,(*pcl_in).header.stamp, ros::Duration(5.0));  
   
      pcl_ros::transformPointCloud("/map", *pcl_in, pcl_transform, *tf_listener);
      pcl_out  = filter(pcl_transform);
      ROS_INFO("Transformed Cloud");
      pcl_out.header.frame_id = pcl_transform.header.frame_id;
      pcl_out.header.stamp = pcl_transform.header.stamp;
      pub2.publish(pcl_out);
      
      if(q==0)
      {
        pcl_map = pcl_transform;
        pub1.publish(pcl_map);
        q=1;
      }
      else
      {
        ConcatAndPublish();
      } 
  }
  
  sensor_msgs::PointCloud2 filter(const sensor_msgs::PointCloud2 pcl_in)
  {
      sensor_msgs::PointCloud2 pcl_out; 
      pcl::PointCloud<pcl::PointXYZRGB> test_cloud;
      pcl::fromROSMsg(pcl_in,test_cloud);
      ROS_INFO("Inputcloud size is");
      //ROS_INFO("%d",test_cloud.points.size ());
      float d, x , y, z;
      int j=0;
      filter_pcl_transform = test_cloud;
      for (size_t i = 0; i < test_cloud.points.size (); ++i)
      {
        x = test_cloud.points[i].x;
        y = test_cloud.points[i].y;
        z = test_cloud.points[i].z;
    	d = pow(x,2)+pow(y,2)+pow(z,2) ;
    	if(d<5)
    	{
   	   filter_pcl_transform.points[j].x = x;
   	   filter_pcl_transform.points[j].y = y;
   	   filter_pcl_transform.points[j].z = z;
   	   filter_pcl_transform.points[j].r = test_cloud.points[i].r;
   	   filter_pcl_transform.points[j].g = test_cloud.points[i].g;
   	   filter_pcl_transform.points[j].b = test_cloud.points[i].b;
   	   j++;
   	}
      }
      ROS_INFO("Filtered cloudsize is");
      ROS_INFO("%d",j); 
      filter_pcl_transform.points.resize(j);
      filter_pcl_transform.width = j;
      filter_pcl_transform.height = 1;
      pcl::toROSMsg(filter_pcl_transform,pcl_out);
      return pcl_out;
  }
  
private:
  ros::NodeHandle n_; 
  ros::Publisher pub1,pub2;
  ros::Subscriber sub1;
  tf::TransformListener *tf_listener; 
  sensor_msgs::PointCloud2 pcl_transform,pcl_map;
  pcl::PointCloud<pcl::PointXYZRGB> output_pcl_map, input_pcl_transform,filter_pcl_transform;
  int q;
};


int main(int argc, char **argv) 
{ 

 ros::init(argc, argv, "pointcloud_map_node");
  PointCloudMap cloudobj;
  ros::spin();  
  return 0; 

} 
