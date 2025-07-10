#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

class PassThroughFilter
{
public:
    PassThroughFilter()
    {
        //Creation of subscriber and publisher of PC
        sub_ = node_handle.subscribe("/camera/depth_registered/points", 1, &PassThroughFilter::cloudCallback, this);
        pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/filtered_points", 1);
    }

private:
    ros::NodeHandle node_handle;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    float maximum_dist;

    /*WARNING -> This method slows down the pointcloud to a around a third of excepted rate*/

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        // Convert the ROS PointCloud2 message to a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*input, *cloud);

        // Apply PassThrough filter
        //pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
        //pcl::PassThrough<pcl::PointXYZ> passthrough;

        if (node_handle.getParam("/detection_parameters/max_range",maximum_dist)){
        }
        else{
            maximum_dist = 3.5; //default_value
        }
        //ROS_INFO("Max. distance set to: %.2f m",maximum_dist);


        for (auto& point : filtered_cloud->points) {
            //ROS_INFO(" %.2f ",point.z);
            if (point.z < 0.0 || point.z > maximum_dist) {
                // Mark points outside the range with NaN -> Ignored in octomap_from_pointcloud.cpp
                point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // Convert the filtered PCL PointCloud back to ROS PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = input->header;  // Preserve the header of the input point cloud

        // Publish the filtered point cloud
        pub_.publish(output);
    }
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "passthrough_filter");
    PassThroughFilter filter;
    ros::spin();

    return 0;
}
