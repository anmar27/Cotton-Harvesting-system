#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "simple_octomap_node");
    ros::NodeHandle nh;

    // Create publisher for octomap messages
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);

    // Set publishing rate
    ros::Rate rate(1);  // 1 Hz

    // Define the OctoMap resolution
    double resolution = 0.1;
    octomap::OcTree octree(resolution);

    // Example: Add a single occupied cell at the origin (0,0,0)
    octree.updateNode(0.0, 0.0, 0.0, true);
    

    // Example: Add a small cube of occupied cells for visualization
    for (int x = -5; x <= 5; ++x) {
        for (int y = -5; y <= 5; ++y) {
            for (int z = -5; z <= 5; ++z) {
                octree.updateNode(x * resolution, y * resolution, z * resolution, true);
                //octree.expand();
            }
        }
    }

    // Loop to continuously publish the OctoMap
    while (ros::ok()) {
        // Create an Octomap message
        octomap_msgs::Octomap msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        // Convert the OctoMap to a ROS message
        if (octomap_msgs::binaryMapToMsg(octree, msg)) {
            // Publish the OctoMap message
            octomap_pub.publish(msg);
            ROS_INFO("Published OctoMap.");
        } else {
            ROS_ERROR("Failed to convert OctoMap to ROS message.");
        }

        // Sleep according to the rate
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
