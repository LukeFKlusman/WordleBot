#include "interaction_execution/main_window.hpp"

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = rclcpp::Node::make_shared("interaction_execution");

  MainWindow window(node);
  window.show();

  const auto exit_code = app.exec();
  rclcpp::shutdown();
  return exit_code;
}
