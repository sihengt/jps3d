// One-shot publisher used to smoke-test jps3d_node: builds a small octree
// (5x5x5 world, 1m resolution, free except a wall at x=2 blocking y in
// [0,5) at z=0..1, leaving z=2..4 as a gap), serializes it, and publishes
// once on /octomap_full.
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("octomap_test_publisher");
    auto pub = node->create_publisher<octomap_msgs::msg::Octomap>(
        "/octomap_full", rclcpp::QoS(10));

    octomap::OcTree tree(1.0);
    // Free space everywhere in [0,5)^3.
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 5; ++z)
                tree.updateNode(
                    octomap::point3d(x + 0.5, y + 0.5, z + 0.5), false);
    // Wall at x=2 across all y, blocking z=0,1. The +1 floatToInt boundary
    // bleed (see OctomapMapUtil::updateFromOctree) extends the topmost
    // occupied leaf (z=1, world [1.0,2.0]) up into z=2 as well, so the
    // actually-occupied region ends up z=0..2 -- leaving z=3 and z=4 both
    // genuinely free, a 2-cell buffer robust to the bleed.
    for (int y = 0; y < 5; ++y)
        for (int z = 0; z < 2; ++z)
            tree.updateNode(octomap::point3d(2.5, y + 0.5, z + 0.5), true);

    octomap_msgs::msg::Octomap msg;
    octomap_msgs::fullMapToMsg(tree, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = node->now();

    // Give the subscriber time to match before publishing (no history QoS
    // durability here, matching jps3d_node's default depth-10 subscription).
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rclcpp::shutdown();
    return 0;
}
