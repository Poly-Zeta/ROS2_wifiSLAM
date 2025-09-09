#include "i2c_sensors_driver/i2c_sensors_driver.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include <rclcpp/timer.hpp>
// #include "std_msgs/msg/string.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/magnetic_field.hpp"
// #include "sensor_msgs/msg/temperature.hpp"

#include <iostream>  
#include <iomanip>
#include <cstdint>
#include <chrono>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <utility>
#include <memory>

#define deviceAddr 0x55

I2CsensorsDriver::I2CsensorsDriver(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options){
    using std::placeholders::_1;
    using std::chrono_literals::operator""ms;

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("i2cReceive/imu", rclcpp::QoS{100});
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("i2cReceive/mag", rclcpp::QoS{100});
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("i2cReceive/temp", rclcpp::QoS{10});
    humAndPress_pub_ = create_publisher<std_msgs::msg::String>("i2cReceive/humAndPress", rclcpp::QoS{10});
    l_wheel_count_pub_ = create_publisher<std_msgs::msg::Int32>("i2cReceive/l_wheel_count", rclcpp::QoS{10});
    r_wheel_count_pub_ = create_publisher<std_msgs::msg::Int32>("i2cReceive/r_wheel_count", rclcpp::QoS{10});
    time_sync_pub_ = create_publisher<i2c_sensors_driver::msg::Esp32TimeSync>("i2cReceive/esp32_time_sync", rclcpp::QoS{100});

    auto on_timer_ = std::bind(&I2CsensorsDriver::onTimer, this);
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_)>>(
        this->get_clock(), 10ms, std::move(on_timer_), this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);

    initializeI2C();
}

void I2CsensorsDriver::initializeI2C(){

    fd_=open("/dev/i2c-1",O_RDWR);
    
    if (fd_ == -1){
        printf("ERROR : No device!!");
    }

}

void I2CsensorsDriver::onTimer(){
    
    unsigned char readBuf[dataLength];
    uint32_t clk=0;
    float dataBuff[dataLength/4-1];

    read_block_data_noResister(fd_,deviceAddr, readBuf, sizeof(readBuf));

    //88Byte読みだして，4Byteごとに処理する

    //先頭4つはクロックのulong    
    clk=(readBuf[3]<<24)|(readBuf[2]<<16)|(readBuf[1]<< 8)|(readBuf[0]<< 0);
    //std::cout << clk <<"\n";

    //残りは4Byteごとにfloat
    for(int i=1;i<dataLength/4;i++){
        char bf[4];
        bf[0]=readBuf[4*i+0];
        bf[1]=readBuf[4*i+1];
        bf[2]=readBuf[4*i+2];
        bf[3]=readBuf[4*i+3];
        float *p=(float*)bf;
        dataBuff[i-1]=*p;
        //std::cout << dataBuff[i-1]<<"\n";
    }
    // dataPublish();

    //ROS2側に出力する

    i2c_sensors_driver::msg::Esp32TimeSync sync_msg;
    sync_msg.header.stamp = now();       // ROS受信時刻
    sync_msg.esp_stamp_ms = clk;         // ESP32のmillis()
    time_sync_pub_->publish(sync_msg);

    //IMU
    sensor_msgs::msg::Imu msg_imu;
    // rclcpp::Time t = this->now();
    msg_imu.header.stamp =now();
    msg_imu.header.frame_id = "imu_frame";
    msg_imu.angular_velocity.x = dataBuff[0];
    msg_imu.angular_velocity.y = dataBuff[1];
    msg_imu.angular_velocity.z = dataBuff[2];
    msg_imu.linear_acceleration.x = dataBuff[9];
    msg_imu.linear_acceleration.y = dataBuff[10];
    msg_imu.linear_acceleration.z = dataBuff[11];
    msg_imu.orientation.x=dataBuff[13];
    msg_imu.orientation.y=dataBuff[14];
    msg_imu.orientation.z=dataBuff[15];
    msg_imu.orientation.w=dataBuff[12];
    imu_pub_->publish(msg_imu);
    // gyro_.clear();
    // accel_.clear();

    //地磁気
    sensor_msgs::msg::MagneticField msg_mag;
    msg_mag.header.stamp =now();
    msg_mag.header.frame_id = "mag_frame";
    msg_mag.magnetic_field.x =dataBuff[3];
    msg_mag.magnetic_field.y =dataBuff[4];
    msg_mag.magnetic_field.z =dataBuff[5];
    mag_pub_->publish(msg_mag);

    //パルスカウンタ
    // 左車輪カウンタ
    std_msgs::msg::Int32 msg_l;
    msg_l.data = static_cast<int32_t>(dataBuff[19]);
    l_wheel_count_pub_->publish(msg_l);

    // 右車輪カウンタ
    std_msgs::msg::Int32 msg_r;
    msg_r.data = static_cast<int32_t>(dataBuff[20]);
    r_wheel_count_pub_->publish(msg_r);

    //温度
    sensor_msgs::msg::Temperature msg_temp;
    msg_temp.header.stamp = now();
    msg_temp.header.frame_id = "temp_frame";
    msg_temp.temperature =dataBuff[16];
    temp_pub_->publish(msg_temp);

    //湿度，気圧(string)
    std_msgs::msg::String msg_humAndPress;
    // msg_humAndPress.header.stamp = clock->now();
    // msg_humAndPress.header.frame_id = 'humAndPress_frame';
    msg_humAndPress.data=std::to_string(dataBuff[17])+","+std::to_string(dataBuff[18]);
    humAndPress_pub_->publish(msg_humAndPress);

    return;
}

// void I2CsensorsDriver::dataPublish(){

// }

void I2CsensorsDriver::read_block_data_noResister(int fd_,uint8_t address, unsigned char * readBuf, uint8_t byteCount ){
    struct i2c_msg i2c_msg[1];
    struct i2c_rdwr_ioctl_data packets;
    // unsigned char readData[byteCount];

    i2c_msg[0].addr =  address;    // device address
    i2c_msg[0].flags = I2C_M_RD;   // read
    i2c_msg[0].len = byteCount;
    i2c_msg[0].buf = &readBuf[0];

    packets.msgs = i2c_msg;
    packets.nmsgs = 1;        // データ読み出しでmsgは1

    int ret = ioctl(fd_, I2C_RDWR, &packets);

    // for (int i = 0; i <byteCount; i++){
    //     readBuf[i] =  readData[i];
    // }

    return;
}
