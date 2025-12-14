#pragma once

#include <deque>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include <factor_graph_interfaces/msg/factor_graph.hpp>
#include <factor_graph_interfaces/srv/get_point_cloud.hpp>

#include <rviz_factor_graph_plugins/plugins/point_color_settings.hpp>

namespace rviz_factor_graph_plugins {

class Lines;
struct PoseNode;

class FactorGraphVisual {
public:
  using FactorGraph = factor_graph_interfaces::msg::FactorGraph;
  using GetPointCloud = factor_graph_interfaces::srv::GetPointCloud;

  FactorGraphVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~FactorGraphVisual();

  void reset();
  void update();

  void setVisibility(bool show_factors, bool show_axes, bool show_points);
  void setFactorColor(const Ogre::ColourValue& factor_color);

  void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  void setMessage(const FactorGraph::ConstSharedPtr& graph_msg);

  void setAxesShape(float length, float radius);
  void setPointStyle(float size, float alpha, rviz_rendering::PointCloud::RenderMode mode);
  void setColorSettings(const std::shared_ptr<PointColorSettings>& color_settings);

  void setPointsLoadingParams(int max_load_count, int max_requests);
  void setGetPointCloudService(std::shared_ptr<rclcpp::Node> node,
                               rclcpp::Client<GetPointCloud>::SharedPtr service);

private:
  struct TextLabelHandle {
    std::shared_ptr<rviz_rendering::MovableText> text;
    Ogre::SceneNode* node = nullptr;  // Node, an dem der Text hängt
  };

  void clearPoseLabels();
  void clearLandmarks();
  void clearLandmarkLabels();

private:
  std::unordered_map<std::uint64_t, std::shared_ptr<PoseNode>> pose_nodes;
  std::shared_ptr<Lines> factor_lines;

  // Landmarks (Shapes)
  std::vector<std::shared_ptr<rviz_rendering::Shape>> landmarks_;

  // Labels
  std::unordered_map<std::uint64_t, TextLabelHandle> pose_labels_;
  std::vector<TextLabelHandle> landmark_labels_;

  bool show_axes = true;
  float axes_length = 0.5f;
  float axes_radius = 0.02f;

  bool show_points = true;
  float point_size = 0.02f;
  float point_alpha = 1.0f;
  rviz_rendering::PointCloud::RenderMode point_style = rviz_rendering::PointCloud::RM_FLAT_SQUARES;

  std::shared_ptr<PointColorSettings> color_settings;

  bool show_landmarks_ = true;
  float landmark_scale_ = 0.20f;

  Ogre::SceneNode* frame_node_ = nullptr;
  Ogre::SceneManager* scene_manager_ = nullptr;

  rclcpp::Client<GetPointCloud>::SharedPtr get_point_cloud;

  int max_requests = 5;
  int max_load_count = 15;

  std::uint64_t loading_counter = 0;
  FactorGraph::ConstSharedPtr last_graph_msg;

  std::deque<std::uint64_t> load_priority_queue;
  std::deque<std::pair<std::shared_future<GetPointCloud::Response::SharedPtr>,
                       rclcpp::Client<GetPointCloud>::SharedPtr>>
      get_point_cloud_results;
};

}  // namespace rviz_factor_graph_plugins
