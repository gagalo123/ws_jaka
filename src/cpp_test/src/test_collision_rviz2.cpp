#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/collision.hpp"
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
using namespace pinocchio;
class Test_Collision_Rviz2 : public rclcpp::Node {
public:
  Test_Collision_Rviz2() : Node("test_collision_rviz2") {
    // 订阅 /robot_description 话题，接收 URDF 字符串
    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", 10,
        std::bind(&Test_Collision_Rviz2::urdf_callback, this,
                  std::placeholders::_1));
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
    RCLCPP_INFO(this->get_logger(), "Test_Collision_Rviz2_Node 节点已启动。");
    // while (!urdf_loaded_) {
    //   RCLCPP_INFO(this->get_logger(), "等待 URDF 数据...");
    //   std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
  }
  void wait_for_urdf() {
    rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                       promise_.get_future());
  }
  std::string get_urdf_string() const { return urdf_string_; }
  void publish_joint_states(const Model &model) {
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    auto q0 = pinocchio::randomConfiguration(model);
    std::cout << "model.names.size()=" << model.names.size() << std::endl;
    std::cout << "q0.size()=" << q0.size() << std::endl;
    msg->header.stamp = this->now();
    msg->name = model.names;
    msg->name.erase(msg->name.begin()); // 移除第一个元素 "UNNAMED"
    msg->position.resize(msg->name.size(), 0.0);
    for (int i = 0; i < (int)msg->name.size(); ++i) {
      msg->position[i] = q0[i];
    }
    joint_state_pub_->publish(*msg);
  }

private:
  std::promise<void> promise_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  std::string urdf_string_;
  bool urdf_loaded_ = false;
  void urdf_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到 URDF 字符串，长度: %zu",
                msg->data.size());
    if (!urdf_loaded_) {
      urdf_loaded_ = true;
      promise_.set_value();
      urdf_string_ = msg->data;
    }

    // 在这里可以使用 pinocchio 或其他库解析 URDF 字符串
    // ...
    RCLCPP_INFO(this->get_logger(), "URDF 解析完成。");
    // 取消订阅，避免重复处理
  }
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};
std::mutex mutex_urdf_loaded;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Test_Collision_Rviz2>();

  node->wait_for_urdf();
  std::string urdf_string = node->get_urdf_string();
  std::istringstream urdf_stream(urdf_string);
  RCLCPP_INFO(rclcpp::get_logger("test_collision_rviz2"),
              "URDF 字符串长度: %zu", urdf_string.size());

  Model model;

  pinocchio::urdf::buildModelFromXML(urdf_string, model);
  RCLCPP_INFO(rclcpp::get_logger("test_collision_rviz2"),
              "模型加载完成，关节数: %d", model.njoints);
  // for (auto &joint : model.joints) {
  //   std::cout << "Joint Name: " << joint << ", Type: " << joint.shortname()
  //             << std::endl;
  // }

  Data data(model);
  for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints;
       ++joint_id)
    std::cout << std::left << std::setw(12) << joint_id << std::setw(24)
              << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose() << std::endl;

  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, urdf_stream, pinocchio::COLLISION,
                             geom_model);

  geom_model.addAllCollisionPairs();
  geom_model.removeCollisionPair(CollisionPair(7, 8));
  geom_model.removeCollisionPair(CollisionPair(15, 16));
  GeometryData geom_data(geom_model);

  auto q0 = pinocchio::neutral(model);
  pinocchio::forwardKinematics(model, data, q0);
  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);
  pinocchio::computeCollisions(geom_model, geom_data);

  for (int T = 0; T < 100; T) {
    q0 = pinocchio::randomConfiguration(model);
    pinocchio::forwardKinematics(model, data, q0);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);
    pinocchio::computeCollisions(geom_model, geom_data);

    node->publish_joint_states(model);
    // std::cout << "Step " << T << ":\n";
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k) {
      const CollisionPair &cp = geom_model.collisionPairs[k];
      const hpp::fcl::CollisionResult &cr = geom_data.collisionResults[k];
      if (cr.isCollision()) {
        // std::cout << "  Collision detected between objects "
        //           << geom_model.geometryObjects[cp.first].name << " and "
        //           << geom_model.geometryObjects[cp.second].name << std::endl;
        // std::cout << "Press Enter to continue...";
        // std::cin.get(); // 等待用户按下 Enter
      }
    }
    rclcpp::spin_some(node);
  }
  // rclcpp::executors::MultiThreadedExecutor
  // executor(rclcpp::ExecutorOptions(),
  //                                                   4); // 4个线程
  // executor.add_node(node);
  // std::thread spin_thread([&executor]() {
  //   RCLCPP_INFO(rclcpp::get_logger("main"), "Starting executor spin");
  //   executor.spin();
  //   RCLCPP_INFO(rclcpp::get_logger("main"), "Executor spin stopped");
  // });

  // RCLCPP_INFO(rclcpp::get_logger("test_collision_rviz2"),
  //             "test_collision_rviz2 结束");
  // std::cout << "test_collision_rviz2 结束" << std::endl;

  // // 停止 executor 并等待 spin 线程结束
  // executor.cancel();  // 停止 spin
  // spin_thread.join(); // 等待 spin 线程结束
  rclcpp::shutdown();
  return 0;
}