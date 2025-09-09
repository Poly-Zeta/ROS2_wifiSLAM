#ifndef I2C_SENSORS_DRIVER_HPP_
#define I2C_SENSORS_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/int32.hpp>
#include "i2c_sensors_driver/msg/esp32_time_sync.hpp"

// #include <iostream>  
// #include <iomanip>
// #include <cstdint>
// #include <chrono>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <linux/i2c.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <vector>
// #include <utility>
// #include <memory>

// #define dataLength 88

class I2CsensorsDriver : public rclcpp::Node{
    public:
        I2CsensorsDriver(const std::string & node_name, const rclcpp::NodeOptions & options);
    
    private:
        static constexpr int dataLength = 88;

        void initializeI2C();
        void onTimer();  
        // void dataPublish();
        void read_block_data_noResister(int fd,uint8_t address, unsigned char * readBuf, uint8_t byteCount );
        
        int fd_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock::SharedPtr clock_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
        
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr humAndPress_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr l_wheel_count_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr r_wheel_count_pub_;

        rclcpp::Publisher<i2c_sensors_driver::msg::Esp32TimeSync>::SharedPtr time_sync_pub_;

        // float dataBuff_[dataLength/4-1];
};

#endif  // I2C_SENSORS_DRIVER_HPP_