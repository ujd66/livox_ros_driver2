#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs::msg;
using namespace livox_ros_driver2::msg;
using namespace message_filters;

class SyncNode : public rclcpp::Node {
public:
    SyncNode() : Node("sync_node"), sync_(MySyncPolicy(10), image_sub_, lidar_sub_) {
        // 声明参数
        this->declare_parameter("image_topic", "/camera/color/image_raw");
        this->declare_parameter("lidar_topic", "/livox/lidar");
        this->declare_parameter("synced_image_topic", "/synced_image");
        this->declare_parameter("synced_lidar_topic", "/synced_lidar");
        this->declare_parameter("publish_rate", 10.0);

        // 获取参数
        auto image_topic = this->get_parameter("image_topic").as_string();
        auto lidar_topic = this->get_parameter("lidar_topic").as_string();
        auto synced_image_topic = this->get_parameter("synced_image_topic").as_string();
        auto synced_lidar_topic = this->get_parameter("synced_lidar_topic").as_string();
        auto publish_rate = this->get_parameter("publish_rate").as_double();

        // 初始化订阅者
        image_sub_.subscribe(this, image_topic);
        lidar_sub_.subscribe(this, lidar_topic);

        // 初始化发布者
        image_pub_ = this->create_publisher<Image>(synced_image_topic, 10);
        lidar_pub_ = this->create_publisher<CustomMsg>(synced_lidar_topic, 10);

        // 注册同步回调
        sync_.registerCallback(std::bind(&SyncNode::callback, this, 
                                        std::placeholders::_1, std::placeholders::_2));

        // 设置定时器
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(period, std::bind(&SyncNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Sync node initialized with:");
        RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Lidar topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate);
    }

private:
    // 订阅者
    Subscriber<Image> image_sub_;
    Subscriber<CustomMsg> lidar_sub_;
    
    // 同步器
    typedef sync_policies::ApproximateTime<Image, CustomMsg> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync_;
    
    // 发布者
    rclcpp::Publisher<Image>::SharedPtr image_pub_;
    rclcpp::Publisher<CustomMsg>::SharedPtr lidar_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 存储最新的同步消息
    Image::SharedPtr last_image_msg_;
    CustomMsg::SharedPtr last_lidar_msg_;
    bool new_image_received_ = false;
    bool new_lidar_received_ = false;

    void callback(const Image::ConstSharedPtr& img_msg, const CustomMsg::ConstSharedPtr& lidar_msg) {
        // 存储最新的同步消息
        last_image_msg_ = std::make_shared<Image>(*img_msg);
        last_lidar_msg_ = std::make_shared<CustomMsg>(*lidar_msg);
        new_image_received_ = true;
        new_lidar_received_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Synchronized messages received");
    }

    void timerCallback() {
        if (new_image_received_ && new_lidar_received_) {
            // 发布同步后的消息
            image_pub_->publish(*last_image_msg_);
            lidar_pub_->publish(*last_lidar_msg_);
            
            new_image_received_ = false;
            new_lidar_received_ = false;
            
            RCLCPP_DEBUG(this->get_logger(), "Published synchronized messages");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SyncNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting sync node...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}