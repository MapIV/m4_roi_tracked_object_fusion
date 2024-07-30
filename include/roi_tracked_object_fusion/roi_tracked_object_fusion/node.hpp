// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROI_TRACKED_OBJECT_FUSION__ROI_TRACKED_OBJECT_FUSION__NODE_HPP_
#define ROI_TRACKED_OBJECT_FUSION__ROI_TRACKED_OBJECT_FUSION__NODE_HPP_

#include "roi_tracked_object_fusion/fusion_node.hpp"
#include "tier4_autoware_utils/ros/debug_publisher.hpp"

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"

#include <map>
#include <memory>
#include <vector>

namespace roi_tracked_object_fusion
{

using sensor_msgs::msg::RegionOfInterest;

class RoiTrackedObjectFusionNode : public FusionNode<TrackedObjects, TrackedObject>
{
public:
  explicit RoiTrackedObjectFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(TrackedObjects & output_msg) override;

  void fuseOnSingleImage(
    const TrackedObjects & input_object_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, TrackedObjects & output_object_msg) override;

  std::map<std::size_t, DetectedObjectWithFeature> generateDetectedObjectRoIs(
    const TrackedObjects & input_object_msg, const double image_width, const double image_height,
    const Eigen::Affine3d & object2camera_affine, const Eigen::Matrix4d & camera_projection);

  void fuseObjectsOnImage(
    const TrackedObjects & input_object_msg,
    const std::vector<DetectedObjectWithFeature> & image_rois,
    const std::map<std::size_t, DetectedObjectWithFeature> & object_roi_map);

  void publish(const TrackedObjects & output_msg) override;

  bool out_of_scope(const TrackedObject & obj);

private:
  struct
  {
    std::vector<double> passthrough_lower_bound_probability_thresholds{};
    std::vector<double> trust_distances{};
    double min_iou_threshold{};
    bool use_roi_probability{};
    double roi_probability_threshold{};
    Eigen::MatrixXi can_assign_matrix;
  } fusion_params_;

  std::map<int64_t, std::vector<bool>> passthrough_object_flags_map_, fused_object_flags_map_,
    ignored_object_flags_map_;
};

}  // namespace roi_tracked_object_fusion

#endif  // ROI_TRACKED_OBJECT_FUSION__ROI_TRACKED_OBJECT_FUSION__NODE_HPP_
