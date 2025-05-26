#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class LabImageSender : public rclcpp::Node{
public:
    LabImageSender():
        Node("lab_image_sender_node"),
        counter{0}
    {
        pub = image_transport::create_publisher(this,"/camera");
        image_1 = cv::imread("/home/eragon/Изображения/Снимки экрана/singlemarkerssource.jpg"); //
        image_2 = cv::imread("/home/eragon/Изображения/Снимки экрана/singlemarkersoriginal.jpg");
        timer = this->create_wall_timer(2s, std::bind(&LabImageSender::timer_callback, this));
    }
private:
    int counter;
    image_transport::Publisher pub;
    cv::Mat image_1, image_2;
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback(){
        std_msgs::msg::Header H;
        H.frame_id = "/camera_optical";
        sensor_msgs::msg::Image::SharedPtr msg;
        msg = cv_bridge::CvImage(H, "bgr8", image_2).toImageMsg();


        pub.publish(msg);
        counter++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LabImageSender>());
    rclcpp::shutdown();
    return 0;
}
