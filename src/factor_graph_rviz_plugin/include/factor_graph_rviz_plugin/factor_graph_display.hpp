#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include <factor_graph_msgs/msg/factor_graph.hpp>

namespace factor_graph_rviz_plugin
{

class FactorGraphDisplay
  : public rviz_common::MessageFilterDisplay<factor_graph_msgs::msg::FactorGraph>
{
  Q_OBJECT

public:
  FactorGraphDisplay();
  ~FactorGraphDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void processMessage(
    factor_graph_msgs::msg::FactorGraph::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateVisuals();

private:
  void clearVisuals();

  factor_graph_msgs::msg::FactorGraph::ConstSharedPtr last_msg_;

  rviz_common::properties::FloatProperty * node_size_prop_;
  rviz_common::properties::FloatProperty * edge_width_prop_;
  rviz_common::properties::BoolProperty  * show_pose_nodes_prop_;
  rviz_common::properties::BoolProperty  * show_loop_closure_prop_;
  rviz_common::properties::BoolProperty  * show_rejected_prop_;

  std::vector<std::unique_ptr<rviz_rendering::Shape>>         node_visuals_;
  std::vector<std::unique_ptr<rviz_rendering::BillboardLine>> edge_visuals_;
};

}  // namespace factor_graph_rviz_plugin
