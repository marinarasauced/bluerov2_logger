#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int16.hpp>

#include <rosbag2_cpp/writer.hpp>

class BlueROV2BagRecorder : public rclcpp::Node
{
public:
    BlueROV2BagRecorder()
    : Node("bluerov2_bag_recorder")
    {
        std::string bag_filename = generate_bag_filename();
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open(bag_filename);

        // Define topics
        std::vector<std::string> topics = {
            "bluerov2/manual_control",
            "bluerov2/override_rc",
            "bluerov2/pressure",
            "bluerov2/temperature",
            "bluerov2/magnetic_field",
            "bluerov2/heading",
            "bluerov2/imu",
            "bluerov2/camera"
        };

        // Subscribe to each topic and record messages
        for (const auto &topic : topics) {
            std::string type = get_topic_type(topic);

            auto callback = [this, topic, type](const std::shared_ptr<rclcpp::SerializedMessage> msg) {                rclcpp::Time time_stamp = this->now();
                writer_->write(*msg, topic, type, time_stamp);
            };

            if (topic == "bluerov2/manual_control") {
                sub_to_manual_control_ = create_subscription<mavros_msgs::msg::ManualControl>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/override_rc") {
                sub_to_override_rc_ = create_subscription<mavros_msgs::msg::OverrideRCIn>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/pressure") {
                sub_to_pressure_ = create_subscription<sensor_msgs::msg::FluidPressure>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/temperature") {
                sub_to_temperature_ = create_subscription<sensor_msgs::msg::Temperature>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/magnetic_field") {
                sub_to_magnetic_field_ = create_subscription<sensor_msgs::msg::MagneticField>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/heading") {
                sub_to_heading_ = create_subscription<std_msgs::msg::Int16>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/imu") {
                sub_to_imu_ = create_subscription<sensor_msgs::msg::Imu>(
                    topic, 10, callback);
            } else if (topic == "bluerov2/camera") {
                sub_to_camera_ = create_subscription<sensor_msgs::msg::Image>(
                    topic, 10, callback);
            }
        }
    }

private:
    std::string generate_bag_filename() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        std::ostringstream oss;
        oss << "bag_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".db3";
        return oss.str();
    }

    std::string get_topic_type(const std::string &topic_name) {
        if (topic_name == "bluerov2/manual_control") return "mavros_msgs/msg/ManualControl";
        if (topic_name == "bluerov2/override_rc") return "mavros_msgs/msg/OverrideRCIn";
        if (topic_name == "bluerov2/pressure") return "sensor_msgs/msg/Pressure";
        if (topic_name == "bluerov2/temperature") return "sensor_msgs/msg/Temperature";
        if (topic_name == "bluerov2/magnetic_field") return "sensor_msgs/msg/MagneticField";
        if (topic_name == "bluerov2/heading") return "std_msgs/msg/Int16";
        if (topic_name == "bluerov2/imu") return "sensor_msgs/msg/Imu";
        if (topic_name == "bluerov2/camera") return "sensor_msgs/msg/Image";

        // Return empty string for unknown topics
        return "";
    }

    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr sub_to_manual_control_;
    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_to_override_rc_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_to_pressure_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr sub_to_temperature_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_to_magnetic_field_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_to_heading_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_to_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_to_camera_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlueROV2BagRecorder>());
    rclcpp::shutdown();
    return 0;
}
