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
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_one{new pcl::PointCloud<pcl::PointXYZI>};
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid{new pcl::PointCloud<pcl::PointXYZI>};
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered{new pcl::PointCloud<pcl::PointXYZI>};

int count = -1;
int packets = 0;
int times = 0;

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//convert
	cloud_raw = *msg;
	pcl::fromROSMsg(*msg, *cloud_now);

  *cloud_one += *cloud_now;
  packets++;
  if(packetes != 76) return;
  packets = 0;
  
	// PassThrough Filter
	pcl::PassThrough<pcl::PointXYZI> pass;
	cloud_filtered->clear();
  pass.setInputCloud(cloud_one);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.05, 1.05);
	pass.filter(*cloud_mid);
	pass.setInputCloud(cloud_mid);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.05, 1.05);
	pass.filter(*cloud_filtered);
  
	// 累積
  if(cloud_filterd->size() > 0) {
	  *cloud_stored += *cloud_filterd;
	  times ++;

    for(double limit_y = -1.05; limit_y < 1.05; limit_y += 0.1)
    {
      for(double limit_z = -1.05; limit_z < 1.05; limit_z += 0.1)
      {
	      cloud_filtered->clear();
  	    pass.setInputCloud(cloud_stored);
	      pass.setFilterFieldName("y");
	      pass.setFilterLimits(limit_y, limit_y+0.1);
	      pass.filter(*cloud_mid);
	      pass.setInputCloud(cloud_mid);
	      pass.setFilterFieldName("z");
	      pass.setFilterLimits(limit_z, limit_z+0.1);
	      pass.filter(*cloud_filtered);
	
        //if((int)cloud_stored->size() / 500 > count)
		    //{
			    // 平均
			    double ave = 0;
			    for(int i=0;i<cloud_stored->size(); i++)
		      {
				    ave += cloud_stored->points[i].x + 0.4505;    // 床から天井までの高さ
			    }
      		ave /= (double)cloud_stored->size();

			    // 標準偏差
			    double sd = 0.0;
			    for(int i=0; i < cloud_stored->size(); i++)
			    {
				    sd += (cloud_stored->points[i].x - ave)*(cloud_stored->points[i].x);
			    }
			    sd /= (double)cloud_stored->size();
			    sd = std::sqrt(sd);
			
		      // 最大・最小
			    double min = 9999, max = -9999;
			    for(int i=0; i < cloud_stored->size(); i++)
			    {
				    if(min > cloud_stored->points[i].x) min = cloud_stored->points[i].x;
				    if(max < cloud_stored->points[i].x ) max = cloud_stored->points[i].x;
			    }

	      	printf("%d,%d,%lf,%lf,%lf,%lf\n", times,(int)cloud_stored->size(), ave, sd, min, max);

      		//count = (int)cloud_stored->size() / 500;
      	//}
      }
    }
  }
	
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
	printf("size,ave,sd,min,max,times\n");
	
	ros::spin();
	
	return 0;
}
