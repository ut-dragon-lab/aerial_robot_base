#include <tf2_ros/static_transform_broadcaster.h>
#include <urdf/model.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_kdl/tf2_kdl.hpp>
#include <vector>

using namespace std::chrono_literals;
using std::string;

class RotorTfPublisher : public rclcpp::Node {
 public:
  RotorTfPublisher(const KDL::Tree& tree, const urdf::Model& model)
      : Node("rotor_tf_publisher"), tree_(tree), model_(model) {
    // parameters
    this->declare_parameter<string>("rotor_joint_name", "rotor");
    this->declare_parameter<string>("tf_prefix", "");
    this->get_parameter("rotor_joint_name", rotor_joint_name_);
    this->get_parameter("tf_prefix", tf_prefix_);

    // collect all the rotor‐attached segments
    addChildren(tree_.getRootSegment()->second);

    // schedule one‐shot timer to broadcast static TFs after 100ms
    timer_ = this->create_wall_timer(100ms, [this]() {
      broadcastStaticTransforms();
      timer_->cancel();
    });

    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
  }

 private:
  /// pair of segment + its root/tip frame names
  struct SegmentPair {
    KDL::Segment segment;
    string root, tip;
  };

  /// recurse the KDL::Tree, collect segments whose joint name contains rotor_joint_name_
  void addChildren(const KDL::TreeElement& elem) {
    const auto& seg = GetTreeElementSegment(elem);
    const string root = seg.getName();

    for (const auto& child : GetTreeElementChildren(elem)) {
      const auto& child_seg = GetTreeElementSegment(child->second);
      if (child_seg.getJoint().getName().find(rotor_joint_name_) != string::npos) {
        segments_rotor_.emplace(child_seg.getJoint().getName(), SegmentPair{child_seg, root, child_seg.getName()});
      }
      addChildren(child->second);
    }
  }

  /// build and send all static transforms
  void broadcastStaticTransforms() {
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.reserve(segments_rotor_.size());
    auto now = this->get_clock()->now();
    for (auto& kv : segments_rotor_) {
      const auto& sp = kv.second;
      auto msg = tf2::kdlToTransform(sp.segment.pose(0));
      msg.header.stamp = now;

      // prefix が空文字でなければ "prefix/root" 形式に
      if (tf_prefix_.empty()) {
        msg.header.frame_id = sp.root;
        msg.child_frame_id = sp.tip;
      } else {
        msg.header.frame_id = tf_prefix_ + "/" + sp.root;
        msg.child_frame_id = tf_prefix_ + "/" + sp.tip;
      }
      tfs.push_back(msg);
    }
    static_broadcaster_->sendTransform(tfs);
    RCLCPP_INFO(this->get_logger(), "Published %zu static rotor TF(s)", tfs.size());
  }

  // members
  KDL::Tree tree_;
  urdf::Model model_;
  string rotor_joint_name_;
  string tf_prefix_;
  std::map<string, SegmentPair> segments_rotor_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 1) パラメータ専用の小ノードを立ち上げて URDF を取得
  auto param_node = std::make_shared<rclcpp::Node>(
      "rotor_tf_publisher_param",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  std::string urdf_xml;
  param_node->declare_parameter<std::string>("robot_description", "");
  param_node->get_parameter("robot_description", urdf_xml);

  // 2) URDF モデル初期化
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    RCLCPP_ERROR(rclcpp::get_logger("rotor_tf_publisher"), "Failed to parse 'robot_description' URDF");
    return 1;
  }

  // 3) KDL ツリー生成
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    RCLCPP_ERROR(rclcpp::get_logger("rotor_tf_publisher"), "Failed to extract KDL tree from URDF");
    return 1;
  }

  // 4) 実ノードを起動
  auto node = std::make_shared<RotorTfPublisher>(tree, model);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
