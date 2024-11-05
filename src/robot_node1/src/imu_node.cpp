// sensor_imu/src/imu_node.cpp
#include "mpu6500.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <fcntl.h>    // For O_RDWR
#include <unistd.h>   // For open, close

class ImuNode : public rclcpp::Node {
public:
    ImuNode() 
    : Node("imu_node"), 
      mpu6500(open("/dev/i2c-1", O_RDWR), ACCEL_SENS_4G, GYRO_SENS_500DPS) // 設定加速度與陀螺儀靈敏度
    {
        if (!mpu6500.initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6500.");
            throw std::runtime_error("Failed to initialize MPU6500.");
        }
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&ImuNode::publish_imu_data, this)
        );
    }

    ~ImuNode() {}

private:
    MPU6500 mpu6500;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 發佈 IMU 數據
    void publish_imu_data() {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;

        if (mpu6500.read_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)) {
            auto msg = sensor_msgs::msg::Imu();

            msg.header.stamp = this->get_clock()->now();

            msg.header.frame_id = "imu_link";

            msg.linear_acceleration.x = accel_x;
            msg.linear_acceleration.y = accel_y;
            msg.linear_acceleration.z = accel_z;

            msg.angular_velocity.x = gyro_x;
            msg.angular_velocity.y = gyro_y;
            msg.angular_velocity.z = gyro_z;

            msg.linear_acceleration_covariance[0] = 0.02;
            msg.linear_acceleration_covariance[4] = 0.02;
            msg.linear_acceleration_covariance[8] = 0.02;

            msg.angular_velocity_covariance[0] = 0.01;
            msg.angular_velocity_covariance[4] = 0.01;
            msg.angular_velocity_covariance[8] = 0.01;

            imu_publisher_->publish(msg);

            //RCLCPP_INFO(this->get_logger(), "Published IMU data with timestamp: %d", msg.header.stamp.nanosec);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data.");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}