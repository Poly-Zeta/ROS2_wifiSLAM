#include "rclcpp/rclcpp.hpp"
#include "uart_wifi_analyzer/uart_wifi_analyzer.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartWifiAnalyzer>();
    node->run();  // UART受信ループ開始
    rclcpp::shutdown();
    return 0;
}
