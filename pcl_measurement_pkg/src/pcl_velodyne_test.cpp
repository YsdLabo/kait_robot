#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL basic
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// passthrough filter
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
sensor_msgs::PointCloud2 cloud_raw;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_now{new pcl::PointCloud<pcl::PointXYZI>};
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stored{new pcl::PointCloud<pcl::PointXYZI>};
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid{new pcl::PointCloud<pcl::PointXYZI>};
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered{new pcl::PointCloud<pcl::PointXYZI>};

double limit = 0.05;

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//convert
	cloud_raw = *msg;
	pcl::fromROSMsg(*msg, *cloud_now);
	
	// PassThrough Filter
	pcl::PassThrough<pcl::PointXYZI> pass;
	cloud_filtered->clear();
	pass.setInputCloud(cloud_now);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-limit, limit);
	pass.filter(*cloud_mid);
	pass.setInputCloud(cloud_mid);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-limit, limit);
	pass.filter(*cloud_filtered);
	
	// accumulation
	*cloud_stored += *cloud_filtered;
	
	double ave = 0;
	for(int i=0;i<cloud_stored->size(); i++)
	{
		ave += cloud_stored->points[i].x;
	}
	ave /= (double)cloud_stored->size();
	
	//ROS_INFO("size: %d  ave: %lf", cloud_stored->size(), ave);
	
	// Publication
	sensor_msgs::PointCloud2 pcl_out;
	pcl::toROSMsg(*cloud_stored, pcl_out);
	pcl_out.header = cloud_raw.header;
	pub.publish(pcl_out);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_velodyne_test");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("cloud_in", 1, &point_cloud_callback);
	pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
	
	ros::param::param<double>("limit", limit, 0.1);
	limit /= 2.0;
	
	ros::spin();
	
	return 0;
}
