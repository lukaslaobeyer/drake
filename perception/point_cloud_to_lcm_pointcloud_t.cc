#include "drake/perception/point_cloud_to_lcm_pointcloud_t.h"

#include <memory>
#include <string>
#include <vector>

#include "bot_core/pointcloud_t.hpp"
#include "drake/perception/point_cloud.h"

using std::string;
using bot_core::pointcloud_t;
using drake::perception::PointCloud;

namespace drake {
namespace perception {

PointCloudToLcmPointcloudT::PointCloudToLcmPointcloudT() {
  camera_pose_input_port_index_ =
      DeclareAbstractInputPort("camera_pose", Value<Eigen::Isometry3d>())
          .get_index();
  point_cloud_input_port_index_ =
      DeclareAbstractInputPort("point_cloud", Value<PointCloud>())
          .get_index();
  pointcloud_t_msg_output_port_index_ =
      DeclareAbstractOutputPort(&PointCloudToLcmPointcloudT::CalcLcmPointcloudT)
          .get_index();
}

PointCloudToLcmPointcloudT::PointCloudToLcmPointcloudT(const Eigen::Isometry3d& camera_pose)
    : PointCloudToLcmPointcloudT() {
    fixed_camera_pose_ = true;
    camera_pose_ = camera_pose;
}

const systems::InputPort<double>& PointCloudToLcmPointcloudT::camera_pose_input_port()
    const {
  return this->get_input_port(camera_pose_input_port_index_);
}

const systems::InputPort<double>& PointCloudToLcmPointcloudT::point_cloud_input_port()
    const {
  return this->get_input_port(point_cloud_input_port_index_);
}

const systems::OutputPort<double>& PointCloudToLcmPointcloudT::pointcloud_t_msg_output_port()
    const {
  return System<double>::get_output_port(pointcloud_t_msg_output_port_index_);
}

void PointCloudToLcmPointcloudT::CalcLcmPointcloudT(
    const systems::Context<double>& context, pointcloud_t* msg) const {
  const auto& camera_pose = fixed_camera_pose_
      ? camera_pose_
      : this->camera_pose_input_port().
          template Eval<AbstractValue>(context).get_value<Eigen::Isometry3d>();

  const auto& cloud = this->point_cloud_input_port().
      template Eval<AbstractValue>(context).get_value<PointCloud>();

  msg->frame_id = "world";
  msg->n_points = cloud.size();
  msg->points.resize(msg->n_points);
  // See: director.drakevisualizer, DrakeVisualier.onPointCloud
  msg->n_channels = 3;
  msg->channel_names = {"r", "g", "b"};
  msg->channels.resize(3, std::vector<float>(msg->n_points));
  for (int i = 0; i < msg->n_points; ++i) {
    const auto& point = cloud.rgb(i);
    msg->channels[0][i] = point[0] / 255.0;
    msg->channels[1][i] = point[1] / 255.0;
    msg->channels[2][i] = point[2] / 255.0;
    const Eigen::Vector3d xyz = cloud.xyz(i).cast<double>();
    Eigen::Vector3f pt_W =
        (camera_pose * xyz).cast<float>();
    msg->points[i] = {pt_W[0], pt_W[1], pt_W[2]};
  }
}

}  // namespace perception
}  // namespace drake
