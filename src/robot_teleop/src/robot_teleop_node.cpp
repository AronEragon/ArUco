/*
 *
 *
 *
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop()
        : Node("keyboard_teleop")
    {
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        const char *device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
        // добавление клавиатуры sudo chmod 777 /dev/input/by-path/platform-i8042-serio-0-event-kbd



        fd = open(device, O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Не удалось открыть устройство ");
            return;
        }

        rc = libevdev_new_from_fd(fd, &dev);
        if (rc < 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Failed to init libevdev");
            return;
        }

        timer = this->create_wall_timer(10ms,
                                        std::bind(&KeyboardTeleop::timer_callback,
                                                  this));

        RCLCPP_INFO(this->get_logger(), "Используйте WASD для управления");
    }
    ~KeyboardTeleop()
    {
        libevdev_free(dev);
        close(fd);
    }
    void timer_callback()
    {
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            if (ev.type == EV_KEY) {
                switch (ev.value) {
                case 0:
                    if (ev.code == 17 or ev.code == 31 or ev.code == 103 or ev.code == 108) {
                        msg.linear.x = 0;
                    }
                    if (ev.code == 30 or ev.code == 32 or ev.code == 105 or ev.code == 106) {
                        msg.angular.z = 0;
                    }
                    break;
                case 1:
                    if (ev.code == 17 or ev.code == 103) {
                        msg.linear.x = 0.1;
                    }
                    if (ev.code == 31 or ev.code == 108) {
                        msg.linear.x = -0.1;
                    }
                    if (ev.code == 30 or ev.code == 105) {
                        msg.angular.z = 0.1;
                    }
                    if (ev.code == 32 or ev.code == 106) {
                        msg.angular.z = -0.1;
                    }
                    break;
                case 2:
                    break;
                }
            }
        }

        cmd_pub->publish(msg);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer;
    int fd = -1;
    struct libevdev *dev = NULL;
    int rc;
    geometry_msgs::msg::Twist msg;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}
