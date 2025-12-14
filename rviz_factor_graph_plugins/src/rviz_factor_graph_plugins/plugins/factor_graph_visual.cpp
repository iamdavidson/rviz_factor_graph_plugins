#include <rviz_factor_graph_plugins/plugins/factor_graph_visual.hpp>

#include <cmath>
#include <iostream>
#include <map>

#include <rviz_factor_graph_plugins/common/rviz_lines.hpp>
#include <rviz_factor_graph_plugins/plugins/pose_node.hpp>

namespace rviz_factor_graph_plugins {

FactorGraphVisual::FactorGraphVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  factor_lines.reset(new Lines(scene_manager, frame_node_));

  max_requests = 5;
  max_load_count = 15;

  show_axes = true;
  axes_length = 0.5f;
  axes_radius = 0.02f;

  show_points = true;
  point_style = rviz_rendering::PointCloud::RM_FLAT_SQUARES;
  color_settings.reset(new PointColorSettings);

  loading_counter = 0;
}

FactorGraphVisual::~FactorGraphVisual() {
  reset();
  frame_node_->detachAllObjects();
  scene_manager_->destroySceneNode(frame_node_);
}

void FactorGraphVisual::clearPoseLabels() {
  // Pose-Labels hängen an PoseNodes -> Nodes sauber zerstören
  for (auto& [key, handle] : pose_labels_) {
    if (handle.node) {
      handle.node->detachAllObjects();
      scene_manager_->destroySceneNode(handle.node);
      handle.node = nullptr;
    }
    handle.text.reset();
  }
  pose_labels_.clear();
}

void FactorGraphVisual::clearLandmarkLabels() {
  // Landmark-Labels hängen an frame_node_ -> Nodes sauber zerstören
  for (auto& handle : landmark_labels_) {
    if (handle.node) {
      handle.node->detachAllObjects();
      scene_manager_->destroySceneNode(handle.node);
      handle.node = nullptr;
    }
    handle.text.reset();
  }
  landmark_labels_.clear();
}

void FactorGraphVisual::clearLandmarks() {
  // Shapes freigeben
  for (auto& lm : landmarks_) lm.reset();
  landmarks_.clear();

  clearLandmarkLabels();
}

void FactorGraphVisual::reset() {
  last_graph_msg.reset();

  // Labels & Landmarks zuerst (weil PoseNodes ggf. Parent-Nodes sind)
  clearPoseLabels();
  clearLandmarks();

  pose_nodes.clear();
  factor_lines->clear();
}

void FactorGraphVisual::update() {
  if (!get_point_cloud || !get_point_cloud->wait_for_service(std::chrono::nanoseconds(1)) ||
      !last_graph_msg || last_graph_msg->poses.empty()) {
    return;
  }

  // Get asynchronously fetched points
  while (!get_point_cloud_results.empty()) {
    auto& result = get_point_cloud_results.front().first;

    if (result.wait_for(std::chrono::nanoseconds(1)) != std::future_status::ready) {
      break;
    }

    const auto response = result.get();
    get_point_cloud_results.pop_front();

    const char chr = (response->key >> 56);
    const size_t index = ((response->key << 8) >> 8);

    if (response->success) {
      auto node = pose_nodes.find(response->key);
      if (node != pose_nodes.end()) {
        node->second->setPointCloud(response->points, color_settings);
        node->second->setPointStyle(point_size, point_alpha, point_style);
      } else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_factor_graph_plugins"),
                           "Node for the fetched points is not found. key=" << response->key
                                                                            << " symbol=" << chr
                                                                            << index);
      }
    } else {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_factor_graph_plugins"),
                         "Failed to retrieve points for key=" << response->key
                                                              << " symbol=" << chr << index);
    }
  }

  // Issue asynchronous points fetching calls
  for (int i = 0; i < max_load_count && (int)get_point_cloud_results.size() < max_requests; i++) {
    auto req = std::make_shared<GetPointCloud::Request>();

    if (!load_priority_queue.empty()) {
      req->key = load_priority_queue.front();
      load_priority_queue.pop_front();
    } else {
      const auto& poses = last_graph_msg->poses;
      const auto& pose = poses[(loading_counter++) % poses.size()];

      if (pose.type != factor_graph_interfaces::msg::PoseWithID::POINTS) {
        continue;
      }
      req->key = pose.key;
    }

    auto found = pose_nodes.find(req->key);
    if (found == pose_nodes.end()) {
      continue;
    }

    if (found->second->points && !found->second->recoloringRequired(*color_settings)) {
      continue;  // already has points
    }

    get_point_cloud_results.emplace_back(get_point_cloud->async_send_request(req), get_point_cloud);
  }
}

void FactorGraphVisual::setVisibility(bool show_factors, bool show_axes, bool show_points) {
  factor_lines->setVisible(show_factors);
  this->show_axes = show_axes;
  this->show_points = show_points;

  for (auto& node : pose_nodes) {
    node.second->setVisibility(show_axes, show_points);
  }

  for (auto& shp : landmarks_) {
    if (shp) shp->getRootNode()->setVisible(show_landmarks_);
  }

  // Labels sinnvoll mitskalieren: Pose-Labels an show_axes, Landmark-Labels an show_landmarks_
  for (auto& [key, handle] : pose_labels_) {
    if (handle.node) handle.node->setVisible(show_axes);
  }
  for (auto& handle : landmark_labels_) {
    if (handle.node) handle.node->setVisible(show_landmarks_);
  }
}

void FactorGraphVisual::setFactorColor(const Ogre::ColourValue& factor_color) {
  factor_lines->setColor(factor_color);
}

void FactorGraphVisual::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);
}

void FactorGraphVisual::setMessage(const FactorGraph::ConstSharedPtr& graph_msg) {
  last_graph_msg = graph_msg;

  // === Pose-Nodes aktualisieren oder neu anlegen ===
  for (size_t i = 0; i < graph_msg->poses.size(); i++) {
    const std::uint64_t key = graph_msg->poses[i].key;
    const size_t index = ((key << 8) >> 8);

    const bool has_points =
        (graph_msg->poses[i].type == factor_graph_interfaces::msg::PoseWithID::POINTS);

    const auto& trans = graph_msg->poses[i].pose.position;
    const auto& quat = graph_msg->poses[i].pose.orientation;

    const Ogre::Vector3 pos(trans.x, trans.y, trans.z);
    const Ogre::Quaternion ori(quat.w, quat.x, quat.y, quat.z);

    auto it = pose_nodes.find(key);
    if (it == pose_nodes.end()) {
      it = pose_nodes.emplace_hint(it, key, new PoseNode(scene_manager_, frame_node_));
      it->second->setAxesShape(axes_length, axes_radius);
      it->second->setVisibility(show_axes, show_points);
      if (has_points) {
        load_priority_queue.emplace_back(key);
      }
    }

    it->second->setPose(pos, ori);

    // === Pose-Label: als Child vom PoseNode-SceneNode (wandert automatisch mit) ===
    auto lbl_it = pose_labels_.find(key);
    if (lbl_it == pose_labels_.end()) {
      auto text = std::make_shared<rviz_rendering::MovableText>(std::to_string(index));
      text->setCharacterHeight(0.18f);
      text->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f));
      text->setTextAlignment(rviz_rendering::MovableText::H_CENTER,
                             rviz_rendering::MovableText::V_ABOVE);

      // Always-on-top Workaround
      {
        Ogre::MaterialPtr mat = text->getMaterial();
        if (!mat.isNull()) {
          mat->setDepthCheckEnabled(false);
          mat->setDepthWriteEnabled(false);
          mat->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        }
      }

      // >>> WICHTIG: PoseNode muss getSceneNode() anbieten! <<<
      Ogre::SceneNode* pose_scene_node = it->second->getSceneNode();  // ggf. anpassen!
      Ogre::SceneNode* label_node =
          pose_scene_node->createChildSceneNode(Ogre::Vector3(0, 0, 0.5f));
      label_node->attachObject(text.get());
      label_node->setInheritOrientation(false);
      label_node->setVisible(show_axes);

      pose_labels_[key] = TextLabelHandle{text, label_node};
    } else {
      lbl_it->second.text->setCaption(std::to_string(index));
      if (lbl_it->second.node) lbl_it->second.node->setVisible(show_axes);
    }
  }

  // === Linien vorbereiten ===
  std::vector<Ogre::Vector3> factor_points;
  factor_points.reserve(graph_msg->binary_factors.size() * 2);

  // Map Landmark-Key -> Pose-Key(s)
  std::multimap<uint64_t, uint64_t> landmark_links;

  // === Pose–Pose-Edges und Landmark-Links erkennen ===
  for (const auto& factor : graph_msg->binary_factors) {
    const uint64_t key1 = factor.keys[0];
    const uint64_t key2 = factor.keys[1];

    const bool is_pose1 = (pose_nodes.find(key1) != pose_nodes.end());
    const bool is_pose2 = (pose_nodes.find(key2) != pose_nodes.end());

    if (is_pose1 && is_pose2) {
      factor_points.emplace_back(pose_nodes.at(key1)->getPosition());
      factor_points.emplace_back(pose_nodes.at(key2)->getPosition());
    } else if (is_pose1 && !is_pose2) {
      landmark_links.emplace(key2, key1);
    } else if (!is_pose1 && is_pose2) {
      landmark_links.emplace(key1, key2);
    }
  }

  // === Landmarken neu zeichnen (inkl. Labels + Edges) ===
  clearLandmarks();

  std::vector<Ogre::Vector3> landmark_edges;

  if (show_landmarks_) {
    for (const auto& point : graph_msg->points) {
      const std::uint64_t key = point.key;
      const size_t index = ((key << 8) >> 8);

      Ogre::Vector3 landmark_pos(point.point.x, point.point.y, point.point.z + 0.1f);

      auto shp = std::make_shared<rviz_rendering::Shape>(
          rviz_rendering::Shape::Sphere, scene_manager_, frame_node_);
      shp->setScale(Ogre::Vector3(0.40f, 0.40f, 0.40f));
      shp->setColor(1.0f, 1.0f, 0.0f, 1.0f);
      shp->setPosition(landmark_pos);
      shp->getRootNode()->setVisible(true);
      landmarks_.push_back(shp);

      // Landmark-Label
      auto text = std::make_shared<rviz_rendering::MovableText>(std::to_string(index));
      text->setCharacterHeight(0.18f);
      text->setColor(Ogre::ColourValue(1.0f, 1.0f, 0.0f));
      text->setTextAlignment(rviz_rendering::MovableText::H_CENTER,
                             rviz_rendering::MovableText::V_ABOVE);

      {
        Ogre::MaterialPtr mat = text->getMaterial();
        if (!mat.isNull()) {
          mat->setDepthCheckEnabled(false);
          mat->setDepthWriteEnabled(false);
          mat->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        }
      }

      Ogre::SceneNode* label_node =
          frame_node_->createChildSceneNode(landmark_pos + Ogre::Vector3(0, 0, 0.5f));
      label_node->attachObject(text.get());
      label_node->setInheritOrientation(false);
      label_node->setVisible(show_landmarks_);

      landmark_labels_.push_back(TextLabelHandle{text, label_node});

      // Kanten zu Posen
      auto range = landmark_links.equal_range(point.key);
      for (auto it = range.first; it != range.second; ++it) {
        uint64_t pose_key = it->second;
        auto pose_it = pose_nodes.find(pose_key);
        if (pose_it != pose_nodes.end()) {
          landmark_edges.emplace_back(pose_it->second->getPosition());
          landmark_edges.emplace_back(landmark_pos);
        }
      }
    }
  }

  // === Pose-Pose + Pose-Landmark zusammenführen ===
  std::vector<Ogre::Vector3> all_lines;
  all_lines.reserve(factor_points.size() + landmark_edges.size());
  all_lines.insert(all_lines.end(), factor_points.begin(), factor_points.end());
  all_lines.insert(all_lines.end(), landmark_edges.begin(), landmark_edges.end());

  factor_lines->setPoints(all_lines, false);

  RCLCPP_DEBUG(rclcpp::get_logger("rviz_factor_graph_plugins"),
               "Rendered %zu poses, %zu factors, %zu landmarks (%zu landmark edges)",
               graph_msg->poses.size(), graph_msg->binary_factors.size(), graph_msg->points.size(),
               landmark_edges.size() / 2);
}

void FactorGraphVisual::setAxesShape(float length, float radius) {
  axes_length = length;
  axes_radius = radius;

  for (auto& node : pose_nodes) {
    node.second->setAxesShape(axes_length, axes_radius);
  }
}

void FactorGraphVisual::setPointStyle(float size, float alpha,
                                      rviz_rendering::PointCloud::RenderMode mode) {
  point_size = size;
  point_alpha = alpha;
  point_style = mode;

  for (auto& node : pose_nodes) {
    node.second->setPointStyle(point_size, point_alpha, point_style);
  }
}

void FactorGraphVisual::setColorSettings(const std::shared_ptr<PointColorSettings>& new_settings) {
  for (auto& node : pose_nodes) {
    const auto& settings = node.second->color_settings;

    bool needs_recoloring = false;
    needs_recoloring |= (settings.mode != new_settings->mode);
    needs_recoloring |= (settings.axis != new_settings->axis);
    needs_recoloring |= (settings.colormap != new_settings->colormap);
    needs_recoloring |= std::abs(settings.color.r - new_settings->color.r) > 1e-3;
    needs_recoloring |= std::abs(settings.color.g - new_settings->color.g) > 1e-3;
    needs_recoloring |= std::abs(settings.color.b - new_settings->color.b) > 1e-3;

    if (needs_recoloring) {
      node.second->points.reset();
    }
  }

  color_settings = new_settings;
}

void FactorGraphVisual::setPointsLoadingParams(int max_load_count, int max_requests) {
  this->max_requests = max_requests;
  this->max_load_count = max_load_count;
}

void FactorGraphVisual::setGetPointCloudService(std::shared_ptr<rclcpp::Node> /*node*/,
                                                rclcpp::Client<GetPointCloud>::SharedPtr service) {
  get_point_cloud = service;
}

}  // namespace rviz_factor_graph_plugins
