#include"Dyn_Manip.cpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<pickPlace>();
  executor.add_node(node);
  executor.spin();
  return 0;
}