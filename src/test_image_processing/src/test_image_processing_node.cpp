#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class TestImageSender : public rclcpp::Node{
public:
    TestImageSender():
        Node("test_image_sender_node")
    {
        pub = image_transport::create_publisher(this,"/image");
        image = cv::imread("/home/ivan/BIIS/road_signs.jpg");
        cv::Mat image_gray;
        cv::cvtColor(image, image_gray,cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(image_gray,image_gray,cv::Size(9,9),2,2);
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(image_gray,circles,CV_HOUGH_GRADIENT,1,image_gray.rows/8,200,100,0,0);

        for (int i = 0; i < circles.size();i++){
            cv::circle(image,
                       cv::Point(circles[i][0],circles[i][1]),
                       circles[i][2],
                       cv::Scalar(0,255,0),
                       3);
        }

        timer = this->create_wall_timer(500ms, std::bind(&TestImageSender::timer_callback, this));
    }
private:
    image_transport::Publisher pub;
    cv::Mat image;
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback(){
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestImageSender>());
    rclcpp::shutdown();
    return 0;
}
