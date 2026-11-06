#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <vector>
#include <map>
#include <algorithm>

class CmdVelMux : public rclcpp::Node
{
public:
    CmdVelMux() : Node("cmd_vel_mux")
    {
        this->declare_parameter("deadband", 0.05);
        this->declare_parameter("timeout_ms", 500);

        // Порядок topics задаёт приоритет: первый источник побеждает.
        std::vector<std::string> default_inputs = {
            "/teleop_cmd_vel",
            "/llm_cmd_vel",
            "/person_following_cmd_vel"
        };

        this->declare_parameter("inputs", default_inputs);

        deadband_ = this->get_parameter("deadband").as_double();
        timeout_ = std::chrono::milliseconds(this->get_parameter("timeout_ms").as_int());

        auto input_config = this->get_parameter("inputs").as_string_array();

        for (size_t i = 0; i < input_config.size(); i++)
        {
            std::string topic = input_config[i];
            int priority = static_cast<int>(i);  // Index is priority (0 = highest)

            InputSource source;
            source.topic = topic;
            source.priority = priority;
            source.active = false;
            source.received = false;
            source.last_activity = this->now();
            source.cmd_vel = geometry_msgs::msg::Twist();

            source.subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                topic, 10,
                [this, topic](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    this->cmdVelCallback(topic, msg);
                });

            input_sources_[topic] = source;

            RCLCPP_INFO(this->get_logger(), "Added input: %s with priority %d",
                        topic.c_str(), priority);
        }

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);

        current_active_source_ = "none";

        RCLCPP_INFO(this->get_logger(), "CmdVelMux node initialized with %zu input sources",
                    input_sources_.size());
        RCLCPP_INFO(this->get_logger(), "Deadband: %f", deadband_);
        RCLCPP_INFO(this->get_logger(), "Timeout: %ld ms", timeout_.count());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CmdVelMux::timerCallback, this));
    }

private:
    struct InputSource
    {
        std::string topic;
        int priority;
        bool active;
        bool received;
        rclcpp::Time last_activity;
        geometry_msgs::msg::Twist cmd_vel;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
    };

    void cmdVelCallback(const std::string& topic, const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto& source = input_sources_[topic];
        source.cmd_vel = *msg;
        source.received = true;

        // Нулевые сообщения джойстика не должны отнимать управление у других источников.
        bool is_joystick = (topic == "/teleop_cmd_vel" || topic == "/joystick_cmd_vel");

        if (is_joystick)
        {
            bool activity_detected = (std::abs(msg->linear.x) > deadband_ ||
                                     std::abs(msg->linear.y) > deadband_ ||
                                     std::abs(msg->angular.z) > deadband_);

            if (activity_detected)
            {
                source.last_activity = this->now();
                if (!source.active)
                {
                    source.active = true;
                    RCLCPP_INFO(this->get_logger(), "Input %s activated (deadband exceeded)", topic.c_str());
                }
            }
        }
        else
        {
            // Автономные источники сами отвечают за публикацию нулевой команды.
            source.last_activity = this->now();
            if (!source.active)
            {
                source.active = true;
                RCLCPP_INFO(this->get_logger(), "Input %s activated", topic.c_str());
            }
        }
    }

    void timerCallback()
    {
        auto now = this->now();

        for (auto& [topic, source] : input_sources_)
        {
            if (source.active)
            {
                auto time_since_activity = now - source.last_activity;
                if (time_since_activity > rclcpp::Duration(timeout_))
                {
                    source.active = false;
                    RCLCPP_INFO(this->get_logger(), "Input %s timed out", topic.c_str());
                }
            }
        }

        // Выбираем свежий источник с наименьшим номером приоритета.
        InputSource* best_source = nullptr;
        int best_priority = std::numeric_limits<int>::max();

        for (auto& [topic, source] : input_sources_)
        {
            if (source.active && source.received)
            {
                if (source.priority < best_priority)
                {
                    best_priority = source.priority;
                    best_source = &source;
                }
            }
        }

        geometry_msgs::msg::Twist output_cmd_vel;
        std::string active_source;

        if (best_source)
        {
            output_cmd_vel = best_source->cmd_vel;
            active_source = best_source->topic;
        }
        else
        {
            output_cmd_vel = geometry_msgs::msg::Twist();
            active_source = "none";
        }

        bool should_publish = (active_source != "none") || (active_source != current_active_source_);

        if (should_publish)
        {
            cmd_vel_pub_->publish(output_cmd_vel);
        }

        if (active_source != current_active_source_)
        {
            current_active_source_ = active_source;

            std_msgs::msg::String status_msg;
            status_msg.data = active_source;
            status_pub_->publish(status_msg);

            RCLCPP_INFO(this->get_logger(), "Active source: %s", active_source.c_str());
        }
    }

    std::map<std::string, InputSource> input_sources_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double deadband_;
    std::chrono::milliseconds timeout_;

    std::string current_active_source_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelMux>());
    rclcpp::shutdown();
    return 0;
}