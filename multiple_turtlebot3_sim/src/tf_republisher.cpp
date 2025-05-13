#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TFRepublisher : public rclcpp::Node
{
public:
    TFRepublisher(const std::string & input_topic, const std::string & output_topic)
    : Node("tf_republisher")
    {
        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(output_topic, 10);
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            input_topic, 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                tf_pub_->publish(*msg);
            }
        );
    }

private:
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::string input_topic = "/robot1/tf";  // Varsayılan giriş topiği
    std::string output_topic = "/tf";        // Varsayılan çıkış topiği

    if (argc > 1) {
        input_topic = argv[1];
    }

    rclcpp::spin(std::make_shared<TFRepublisher>(input_topic, output_topic));
    rclcpp::shutdown();
    return 0;
}
