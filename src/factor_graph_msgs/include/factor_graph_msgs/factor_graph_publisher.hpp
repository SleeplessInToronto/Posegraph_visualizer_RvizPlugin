#pragma once

// factor_graph_publisher.hpp — header-only helper for publishing pose graph
// data to the factor_graph_rviz_plugin.
//
// Usage:
//   1. Construct once, passing your rclcpp::Node pointer.
//   2. Call upsert_pose() on every keyframe creation AND after every optimizer
//      correction (same call handles both — it overwrites the stored position).
//   3. Call add_loop_closure() or add_rejected() once per event.
//   4. Call publish() after every SLAM update to stream the full graph.
//
// Example (inside your SLAM node):
//
//   #include <factor_graph_publisher.hpp>
//   factor_graph_viz::FactorGraphPublisher fg_pub(this, "/factor_graph", "odom");
//
//   // On keyframe creation:
//   fg_pub.upsert_pose(kf_id, x, y, z, qw, qx, qy, qz);
//   fg_pub.publish(now());
//
//   // After PGO corrects all poses:
//   for (int i = 0; i < num_keyframes; ++i)
//       fg_pub.upsert_pose(i, corrected_x[i], ...);
//   fg_pub.publish(now());
//
//   // On accepted loop closure:
//   fg_pub.add_loop_closure(candidate_id, query_id);
//
//   // On rejected candidate:
//   fg_pub.add_rejected(candidate_id, query_id);

#include <rclcpp/rclcpp.hpp>
#include <factor_graph_msgs/msg/factor_graph.hpp>
#include <factor_graph_msgs/msg/factor_graph_node.hpp>
#include <factor_graph_msgs/msg/factor_graph_edge.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace factor_graph_viz {

class FactorGraphPublisher
{
public:
  FactorGraphPublisher(rclcpp::Node * node,
                       const std::string & topic   = "/factor_graph",
                       const std::string & frame_id = "odom")
  : frame_id_(frame_id)
  {
    pub_ = node->create_publisher<factor_graph_msgs::msg::FactorGraph>(topic, 10);
  }

  // Insert a new keyframe or update its position after PGO correction.
  void upsert_pose(uint64_t id,
                   double x,  double y,  double z,
                   double qw, double qx, double qy, double qz)
  {
    auto & n = pose_nodes_[id];
    n.id   = id;
    n.type = factor_graph_msgs::msg::FactorGraphNode::POSE;
    n.pose.pose.position.x    = x;
    n.pose.pose.position.y    = y;
    n.pose.pose.position.z    = z;
    n.pose.pose.orientation.w = qw;
    n.pose.pose.orientation.x = qx;
    n.pose.pose.orientation.y = qy;
    n.pose.pose.orientation.z = qz;
  }

  // Record an accepted loop closure between two keyframes.
  // Call once when your place recognizer + geometric verification both pass.
  void add_loop_closure(uint64_t id_from, uint64_t id_to)
  {
    factor_graph_msgs::msg::FactorGraphEdge e;
    e.id_from = id_from;
    e.id_to   = id_to;
    e.type    = factor_graph_msgs::msg::FactorGraphEdge::LOOP_CLOSURE;
    edges_.push_back(e);
  }

  // Record a rejected loop closure candidate.
  // Call when place recognition fired but ICP / consistency check failed.
  void add_rejected(uint64_t id_from, uint64_t id_to)
  {
    factor_graph_msgs::msg::FactorGraphEdge e;
    e.id_from = id_from;
    e.id_to   = id_to;
    e.type    = factor_graph_msgs::msg::FactorGraphEdge::REJECTED_LOOP_CLOSURE;
    edges_.push_back(e);
  }

  // Publish the full accumulated graph. Call after every SLAM update.
  void publish(const rclcpp::Time & stamp)
  {
    factor_graph_msgs::msg::FactorGraph msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id_;

    msg.nodes.reserve(pose_nodes_.size());
    for (auto & [id, node] : pose_nodes_) {
      node.header.stamp    = stamp;
      node.header.frame_id = frame_id_;
      msg.nodes.push_back(node);
    }

    msg.edges = edges_;
    pub_->publish(msg);
  }

private:
  rclcpp::Publisher<factor_graph_msgs::msg::FactorGraph>::SharedPtr pub_;
  std::string frame_id_;

  // Keyed by ID so upsert_pose() updates in-place without searching a vector.
  std::unordered_map<uint64_t, factor_graph_msgs::msg::FactorGraphNode> pose_nodes_;
  std::vector<factor_graph_msgs::msg::FactorGraphEdge> edges_;
};

}  // namespace factor_graph_viz
