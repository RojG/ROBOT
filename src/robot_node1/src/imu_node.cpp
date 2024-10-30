// sensor_imu/src/imu_node.cpp
#include "mpu6500.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <fcntl.h>    // For O_RDWR
#include <unistd.h>   // For open, close

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), mpu6500(i2c_fd) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
        i2c_fd = open("/dev/i2c-1", O_RDWR);
        
        if (i2c_fd < 0 || mpu6500.init(ACCEL_SENS_4G, GYRO_SENS_500DPS) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6500.");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ImuNode::publish_imu_data, this));
    }

private:
    void publish_imu_data() {
        int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
        sensor_msgs::msg::Imu imu_msg;

        if (mpu6500.read_accel(accel_x, accel_y, accel_z) == 0 &&
            mpu6500.read_gyro(gyro_x, gyro_y, gyro_z) == 0) {

            imu_msg.header.stamp = this->get_clock()->now();
            imu_msg.header.frame_id = "imu_link";

            imu_msg.linear_acceleration.x = accel_x * ACCEL_SCALE;
            imu_msg.linear_acceleration.y = accel_y * ACCEL_SCALE;
            imu_msg.linear_acceleration.z = accel_z * ACCEL_SCALE;

            imu_msg.angular_velocity.x = gyro_x * GYRO_SCALE;
            imu_msg.angular_velocity.y = gyro_y * GYRO_SCALE;
            imu_msg.angular_velocity.z = gyro_z * GYRO_SCALE;

            publisher_->publish(imu_msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    MPU6500 mpu6500;
    int i2c_fd;
    static constexpr double ACCEL_SCALE = 9.81 / 32768.0;  // 加速度轉換比例
    static constexpr double GYRO_SCALE = 3.14159 / (180.0 * 32768.0);  // 角速度轉換比例
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
