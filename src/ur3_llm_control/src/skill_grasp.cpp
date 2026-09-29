#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mutex>
#include <condition_variable>
#include <chrono>

// Simulation-only ideal suction grasp. A fixed joint is created only when the
// cup is physically close to the requested cube. No object teleportation.
namespace ur3_llm_control {
class SkillGrasp : public gazebo::ModelPlugin {
  using Switch = std_srvs::srv::SetBool;
  gazebo::physics::ModelPtr robot_;
  gazebo::physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  std::vector<rclcpp::Service<Switch>::SharedPtr> services_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::string held_, requested_, message_;
  bool pending_{false}, attach_{false}, ok_{false};
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    robot_ = model; node_ = gazebo_ros::Node::Get(sdf);
    for (const std::string name : {"red_cube", "yellow_cube", "blue_cube"}) {
      services_.push_back(node_->create_service<Switch>(name,
        [this, name](Switch::Request::SharedPtr req, Switch::Response::SharedPtr res) {
          std::unique_lock<std::mutex> lock(mutex_);
          if (pending_) { res->success = false; res->message = "BUSY"; return; }
          requested_ = name; attach_ = req->data; pending_ = true;
          if (!cv_.wait_for(lock, std::chrono::seconds(3), [this]{return !pending_;})) {
            pending_ = false; res->success = false; res->message = "SIMULATION_TIMEOUT"; return;
          }
          res->success = ok_; res->message = message_;
        }));
    }
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &){Update();});
  }
  void Update() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) return;
    ok_ = false;
    auto object = robot_->GetWorld()->ModelByName(requested_);
    auto cup = robot_->GetLink("suction_tip");
    if (!object || !cup) { message_ = "MISSING_MODEL_OR_CUP"; }
    else if (attach_) {
      if (!held_.empty()) message_ = "GRIPPER_OCCUPIED";
      else if (cup->WorldPose().Pos().Distance(object->WorldPose().Pos()) > 0.035)
        message_ = "OBJECT_TOO_FAR";
      else {
        joint_ = robot_->GetWorld()->Physics()->CreateJoint("fixed", robot_);
        joint_->Load(cup, object->GetLink("link"), ignition::math::Pose3d());
        joint_->Init(); held_ = requested_; ok_ = true; message_ = "ATTACHED";
      }
    } else {
      if (held_ != requested_ || !joint_) message_ = "NOT_HELD";
      else { joint_->Detach(); joint_.reset(); held_.clear(); ok_ = true; message_ = "DETACHED"; }
    }
    pending_ = false; cv_.notify_all();
  }
};
GZ_REGISTER_MODEL_PLUGIN(SkillGrasp)
}
