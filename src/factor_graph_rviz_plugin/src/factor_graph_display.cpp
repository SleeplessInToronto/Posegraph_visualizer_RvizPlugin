#include "factor_graph_rviz_plugin/factor_graph_display.hpp"

#include <cmath>
#include <unordered_set>

#include <OgreColourValue.h>
#include <OgreVector3.h>

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace factor_graph_rviz_plugin
{

FactorGraphDisplay::~FactorGraphDisplay() = default;

FactorGraphDisplay::FactorGraphDisplay()
{
  node_size_prop_ = new rviz_common::properties::FloatProperty(
    "Node Size", 0.1f, "Diameter of pose spheres and landmark cubes.", this);
  node_size_prop_->setMin(0.01f);

  edge_width_prop_ = new rviz_common::properties::FloatProperty(
    "Edge Width", 0.02f, "Width of loop closure edges and rings.", this);
  edge_width_prop_->setMin(0.001f);

  show_pose_nodes_prop_ = new rviz_common::properties::BoolProperty(
    "Show Pose Nodes", true, "Show keyframe pose spheres (blue).", this);

  show_loop_closure_prop_ = new rviz_common::properties::BoolProperty(
    "Show Loop Closures", true, "Show accepted loop closure edges and rings (green).", this);

  show_rejected_prop_ = new rviz_common::properties::BoolProperty(
    "Show Rejected Loop Closures", true, "Show rejected loop closure edges (red).", this);
}

void FactorGraphDisplay::onInitialize()
{
  MFDClass::onInitialize();

  connect(node_size_prop_,           &rviz_common::properties::Property::changed, this, &FactorGraphDisplay::updateVisuals);
  connect(edge_width_prop_,          &rviz_common::properties::Property::changed, this, &FactorGraphDisplay::updateVisuals);
  connect(show_pose_nodes_prop_,     &rviz_common::properties::Property::changed, this, &FactorGraphDisplay::updateVisuals);
  connect(show_loop_closure_prop_,   &rviz_common::properties::Property::changed, this, &FactorGraphDisplay::updateVisuals);
  connect(show_rejected_prop_,       &rviz_common::properties::Property::changed, this, &FactorGraphDisplay::updateVisuals);
}

void FactorGraphDisplay::updateVisuals()
{
  if (last_msg_) processMessage(last_msg_);
}

void FactorGraphDisplay::reset()
{
  MFDClass::reset();
  clearVisuals();
  last_msg_.reset();
}

void FactorGraphDisplay::clearVisuals()
{
  node_visuals_.clear();
  edge_visuals_.clear();
}

void FactorGraphDisplay::processMessage(
  factor_graph_msgs::msg::FactorGraph::ConstSharedPtr msg)
{
  last_msg_ = msg;
  clearVisuals();

  // Transform scene node from message frame to the RViz2 fixed frame
  Ogre::Vector3    position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Transform",
      QString::fromStdString(
        "Could not transform from '" + msg->header.frame_id + "' to fixed frame"));
    return;
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // Build node ID → 3-D position lookup table
  std::unordered_map<uint64_t, Ogre::Vector3> id_pos;
  id_pos.reserve(msg->nodes.size());
  for (const auto & node : msg->nodes) {
    using N = factor_graph_msgs::msg::FactorGraphNode;
    if (node.type == N::POSE) {
      id_pos[node.id] = {
        static_cast<float>(node.pose.pose.position.x),
        static_cast<float>(node.pose.pose.position.y),
        static_cast<float>(node.pose.pose.position.z)};
    } else {
      id_pos[node.id] = {
        static_cast<float>(node.position.x),
        static_cast<float>(node.position.y),
        static_cast<float>(node.position.z)};
    }
  }

  const float node_size  = node_size_prop_->getFloat();
  const float edge_width = edge_width_prop_->getFloat();

  // --- Draw variable nodes ---
  for (const auto & node : msg->nodes) {
    using N = factor_graph_msgs::msg::FactorGraphNode;

    if (node.type != N::POSE) continue;
    if (!show_pose_nodes_prop_->getBool()) continue;

    auto shape = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
    shape->setPosition(id_pos.at(node.id));
    shape->setScale(Ogre::Vector3(node_size, node_size, node_size));
    shape->setColor(Ogre::ColourValue(0.25f, 0.60f, 1.00f, 1.0f));
    node_visuals_.push_back(std::move(shape));
  }

  // --- Draw loop closure edges ---
  using E = factor_graph_msgs::msg::FactorGraphEdge;

  for (const auto & edge : msg->edges) {
    if (edge.type != E::LOOP_CLOSURE && edge.type != E::REJECTED_LOOP_CLOSURE) continue;
    if (edge.type == E::LOOP_CLOSURE     && !show_loop_closure_prop_->getBool()) continue;
    if (edge.type == E::REJECTED_LOOP_CLOSURE && !show_rejected_prop_->getBool()) continue;

    auto it_from = id_pos.find(edge.id_from);
    auto it_to   = id_pos.find(edge.id_to);
    if (it_from == id_pos.end() || it_to == id_pos.end()) continue;

    const Ogre::ColourValue color = (edge.type == E::LOOP_CLOSURE)
      ? Ogre::ColourValue(0.20f, 1.00f, 0.30f, 1.0f)   // green
      : Ogre::ColourValue(1.00f, 0.20f, 0.20f, 0.8f);  // red

    auto line = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
    line->setMaxPointsPerLine(2);
    line->setLineWidth(edge_width);
    line->setColor(color.r, color.g, color.b, color.a);
    line->addPoint(it_from->second);
    line->addPoint(it_to->second);
    edge_visuals_.push_back(std::move(line));
  }

  // --- Draw acceptance rings around the query node of each accepted LC ---
  if (show_loop_closure_prop_->getBool()) {
    std::unordered_set<uint64_t> lc_query_ids;
    for (const auto & edge : msg->edges) {
      if (edge.type == E::LOOP_CLOSURE) lc_query_ids.insert(edge.id_to);
    }

    constexpr int   kN = 33;   // 32 segments + closing point
    constexpr float kR = 0.75f;
    constexpr float kZ = 0.5f;

    for (uint64_t nid : lc_query_ids) {
      auto it = id_pos.find(nid);
      if (it == id_pos.end()) continue;
      const Ogre::Vector3 & c = it->second;

      auto ring = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
      ring->setMaxPointsPerLine(kN);
      ring->setLineWidth(edge_width * 2.0f);
      ring->setColor(0.20f, 1.00f, 0.30f, 1.0f);
      for (int i = 0; i < kN; ++i) {
        const float angle = static_cast<float>(2.0 * M_PI * i / (kN - 1));
        ring->addPoint(Ogre::Vector3(
          c.x + kR * std::cos(angle),
          c.y + kR * std::sin(angle),
          c.z + kZ));
      }
      edge_visuals_.push_back(std::move(ring));
    }
  }
}

}  // namespace factor_graph_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  factor_graph_rviz_plugin::FactorGraphDisplay,
  rviz_common::Display)
