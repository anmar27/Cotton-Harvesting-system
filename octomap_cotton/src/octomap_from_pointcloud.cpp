#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_ros/conversions.h>  // For pointCloud2ToOctomap conversion
#include <sensor_msgs/PointCloud2.h>

//Libraries for message sinchronization
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Messages from capture node
#include <computation/BoundingBox.h>
#include <computation/BoundingBoxes.h>


class OctomapGenerator {
public:
    float resolution = 0.05;

    OctomapGenerator() : node_handle("~"), octree(resolution), publish_rate(1), timeout(ros::Duration(0.1)) {
        // Initialization of publisher
        octomap_pub = node_handle.advertise<octomap_msgs::Octomap>("/octomap", 10);

        // Subscribe to the point cloud topic
        pointcloud_cache_sub = node_handle.subscribe("/camera/depth_registered/points", 10, &OctomapGenerator::pointCloudCacheCallback, this);
        //Subscriibe to bounding boxes from yolo topic
        //bounding_boxes_sub = node_handle.subscribe("/yolo/bounding_boxes", 10, &OctomapGenerator::boundingBoxesCallback, this);
        pointcloud_sub.subscribe(node_handle, "/camera/depth_registered/points", 10);
        bounding_boxes_sub.subscribe(node_handle, "/yolo/bounding_boxes", 10);

        sync.reset(new Sync(MySyncPolicy(10), pointcloud_sub, bounding_boxes_sub));
        sync->registerCallback(boost::bind(&OctomapGenerator::syncCallback, this, _1, _2));
    }

    void spin() {
        while (ros::ok()) {
            publishOctomap();
            checkTimeout(); // Handle timeout cases
            ros::spinOnce();
            publish_rate.sleep();
        }
    }

private:
    ros::NodeHandle node_handle;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Subscriber<computation::BoundingBoxes> bounding_boxes_sub;

    ros::Subscriber pointcloud_cache_sub;
    sensor_msgs::PointCloud2::ConstPtr cached_pointcloud;

    ros::Publisher octomap_pub;
    ros::Rate publish_rate;
    octomap::OcTree octree;
    std::vector<computation::BoundingBox> bounding_boxes;
    sensor_msgs::PointCloud2::ConstPtr last_cloud_msg;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, computation::BoundingBoxes> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync;

    ros::Time last_bounding_box_time;
    ros::Duration timeout;

    // Private methods

    void syncCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                      const computation::BoundingBoxes::ConstPtr& bb_msg) {
        // Update bounding box list
        bounding_boxes = bb_msg->bounding_boxes;
        ROS_INFO("BB-MSG: %ld",bb_msg->bounding_boxes.size());
        // Process the point cloud (insert entire cloud into Octomap)
        processPointCloudWithoutBoundingBoxes(cloud_msg);

        // If bounding boxes are detected, delete the corresponding voxels
        if (!bounding_boxes.empty()) {
            processPointCloudWithBoundingBoxes(cloud_msg);
        }
        last_bounding_box_time = ros::Time::now(); // Update the timestamp
    }
    
    void checkTimeout() {
        if ((ros::Time::now() - last_bounding_box_time) > timeout) {
            ROS_WARN("Timeout reached. Processing entire point cloud.");
            // Use the point cloud without bounding boxes
            if (cached_pointcloud) {
                processPointCloudWithoutBoundingBoxes(cached_pointcloud);
            } else {
                ROS_WARN("No point cloud available.");
            }
            last_bounding_box_time = ros::Time::now(); // Reset timeout
        }
    }
    

    void processPointCloudWithoutBoundingBoxes(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        octomap::Pointcloud octomap_cloud;
        octomap::pointCloud2ToOctomap(*cloud_msg, octomap_cloud);

        // Insert entire point cloud into the octree
        octree.insertPointCloud(octomap_cloud, octomap::point3d(0.0, 0.0, 0.0));
        ROS_INFO("Entire point cloud inserted into Octomap.");
    }

    void pointCloudCacheCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        cached_pointcloud = cloud_msg;
    }

    //Function in charge of deleting voxels inside the boundingbox
    void processPointCloudWithBoundingBoxes(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        ROS_INFO("STARTING MODIFICATION");
        // Convert the cloud message to an Octomap-compatible point cloud
        octomap::Pointcloud octomap_cloud;
        octomap::pointCloud2ToOctomap(*cloud_msg, octomap_cloud);

        for (const auto& box : bounding_boxes) {
            ROS_INFO("Obtained BB: bytes: %ld size:%ld", sizeof(bounding_boxes), bounding_boxes.size());
            for (int u = box.xmin; u <= box.xmax; ++u) {
                for (int v = box.ymin; v <= box.ymax; ++v) {
                    int index = v * cloud_msg->width + u;

                    //ROS_INFO("Size of cloud_msg w: %d h:%d",cloud_msg->width,cloud_msg->height);

                    // Retrieve the 3D coordinates of the point in the point cloud
                    float x, y, z;
                    memcpy(&x, &cloud_msg->data[index * cloud_msg->point_step + cloud_msg->fields[0].offset], sizeof(float));
                    memcpy(&y, &cloud_msg->data[index * cloud_msg->point_step + cloud_msg->fields[1].offset], sizeof(float));
                    memcpy(&z, &cloud_msg->data[index * cloud_msg->point_step + cloud_msg->fields[2].offset], sizeof(float));
                    

                    // Ignore invalid points (NaNs)
                    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                        continue;
                    }

                    // Create a 3D point
                    octomap::point3d point(x, y, z);
                    octree.deleteNode(point); // Remove voxels in unwanted class areas

                    //ROS_INFO("Deleted point x:%f, y:%f z:%f", point.x(),point.y(),point.z());
                    
                }
                
            }
            ROS_INFO("COUNT");
        }
        ROS_INFO("Processed bounding box points and updated Octomap.");
    }

    void publishOctomap() {
        // Create an Octomap message
        octomap_msgs::Octomap msg;
        msg.header.frame_id = "camera_link";
        msg.header.stamp = ros::Time::now();

        // Convert the OctoMap to a ROS message
        if (octomap_msgs::binaryMapToMsg(octree, msg)) {
            // Publish the OctoMap message
            octomap_pub.publish(msg);
            ROS_INFO("Published OctoMap.");
        } else {
            ROS_ERROR("Failed to convert OctoMap to ROS message.");
        }
    }
};

int main(int argc, char** argv) {
    // Initialization of ROS node
    ros::init(argc, argv, "pointcloud_2_octomap");
    OctomapGenerator octomap_generator;
    octomap_generator.spin();

    return 0;
}
