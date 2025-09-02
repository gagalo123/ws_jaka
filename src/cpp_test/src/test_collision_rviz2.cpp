#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
class Test_Collision_Rviz2 : public rclcpp::Node {
public:
  Test_Collision_Rviz2() : Node("test_collision_rviz2") {
    // 订阅 /robot_description 话题，接收 URDF 字符串
    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", 10,
        std::bind(&Test_Collision_Rviz2::urdf_callback, this,
                  std::placeholders::_1));
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
};
std::mutex mutex_urdf_loaded;
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Test_Collision_Rviz2>();

  node->wait_for_urdf();
  std::string urdf_string = node->get_urdf_string();
  RCLCPP_INFO(rclcpp::get_logger("test_collision_rviz2"),
              "URDF 字符串长度: %zu", urdf_string.size());
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