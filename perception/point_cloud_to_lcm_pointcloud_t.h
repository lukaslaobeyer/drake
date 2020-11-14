#pragma once

#include <string>
#include <vector>

#include "bot_core/pointcloud_t.hpp"

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// A PointCloudToLcmPointcloudT takes as input a PointCloud, and outputs an
/// AbstractValue containing a `Value<bot_core::pointcloud_t>` LCM message
/// that can then be sent to other processes that subscribe to it using
/// LcmPublisherSystem, such as drake-visualizer.
class PointCloudToLcmPointcloudT : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointCloudToLcmPointcloudT)

  /// An %PointCloudToLcmPointcloudT constructor.
  PointCloudToLcmPointcloudT();

  /// An %PointCloudToLcmPointcloudT constructor specifying a fixed camera_pose
  /// transform for this %PointCloudToLcmPointcloudT.
  PointCloudToLcmPointcloudT(const Eigen::Isometry3d& camera_pose);

  /// Returns the input port containing the camera_pose transform.
  const systems::InputPort<double>& camera_pose_input_port() const;

  /// Returns the input port containing the point cloud.
  const systems::InputPort<double>& point_cloud_input_port() const;

  /// Returns the abstract valued output port that contains a
  /// `Value<bot_core::pointcloud_t>`.
  const systems::OutputPort<double>& pointcloud_t_msg_output_port() const;
 private:
  void CalcLcmPointcloudT(const systems::Context<double>& context,
                          bot_core::pointcloud_t* msg) const;

  int camera_pose_input_port_index_{-1};
  int point_cloud_input_port_index_{-1};
  int pointcloud_t_msg_output_port_index_{-1};

  bool fixed_camera_pose_{false};
  Eigen::Isometry3d camera_pose_;
};

}  // namespace perception
}  // namespace drake
