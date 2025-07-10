#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomapRT/octomapRT.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace octomap;

class OctomapRTNode {
public:
    OctomapRTNode() : nh("~"), octomapRT(resolution, maxRange, clampingMin, clampingMax, probHit, probMiss) {
        // Parameters
        nh.param("resolution", resolution, 0.1);
        nh.param("max_range", maxRange, -1.0);
        nh.param("frame_id", frameId, std::string("map"));

        // Configure default sensor model values (adjust as needed)
        clampingMin = 0.12;
        clampingMax = 0.97;
        probHit = 0.7;
        probMiss = 0.4;

        // ROS subscribers and publishers
        pointCloudSub = nh.subscribe("/camera/depth_registered/points", 1, &OctomapRTNode::pointCloudCallback, this);
        octomapPub = nh.advertise<octomap_msgs::Octomap>("/output_octomap", 1);

        // Initialize OctomapRT
        octomapRT.initialize();
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg) {
        // Convert PointCloud2 to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *cloud);

        // Insert points into OctomapRT
        octomap::Pointcloud octomapCloud;
        for (const auto& point : cloud->points) {
            if (maxRange <= 0 || point.getVector3fMap().norm() <= maxRange) {
                octomapCloud.push_back(point.x, point.y, point.z);
            }
        }

        // Insert the point cloud into the OctomapRT
        octomapRT.insertPointCloud(octomapCloud);

        // Synchronize and publish the octree
        octomapRT.sync();

        publishOctomap();
    }

    void publishOctomap() {
        octomap_msgs::Octomap octomapMsg;
        octomapMsg.header.frame_id = frameId;
        octomapMsg.header.stamp = ros::Time::now();

        if (octomap_msgs::fullMapToMsg(*octomapRT.getTree(), octomapMsg)) {
            octomapPub.publish(octomapMsg);
        } else {
            ROS_ERROR("Failed to serialize OctomapRT message.");
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pointCloudSub;
    ros::Publisher octomapPub;

    OctomapRT octomapRT; // OctomapRT object
    double resolution;
    double maxRange;
    double clampingMin, clampingMax;
    double probHit, probMiss;
    std::string frameId;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_octomaprt_node");
    OctomapRTNode node;
    ros::spin();
    return 0;
}
