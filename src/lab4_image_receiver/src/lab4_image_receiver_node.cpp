#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

class ArUcoDetector : public rclcpp::Node {
public:
    ArUcoDetector() : Node("aruco_detector_node") {
        sub = image_transport::create_subscription(this, "/camera", std::bind(&ArUcoDetector::image_callback, this, std::placeholders::_1), "raw");
        pub = image_transport::create_publisher(this, "/image_with_markers");

        // Инициализация словаря ArUco
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        // Параметры камеры (замените на реальные значения)
        camera_matrix = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
        dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Коэффициенты искажения

        // Размер маркера в метрах (например, 0.05 м = 5 см)
        marker_size = 0.05;
    }

private:
    cv::Mat image;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    cv::Mat camera_matrix; // Матрица внутренних параметров камеры
    cv::Mat dist_coeffs;   // Коэффициенты искажения
    double marker_size;    // Размер стороны маркера в метрах

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            image = cv_ptr->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Обнаружение ArUco-маркеров
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            // Рисуем рамки вокруг маркеров
            cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

            for (size_t i = 0; i < marker_ids.size(); ++i) {
                int marker_id = marker_ids[i];
                std::vector<cv::Point2f> corners = marker_corners[i];

                // Подготовка 3D-координат углов маркера
                std::vector<cv::Point3f> object_points = {
                    {-marker_size / 2, marker_size / 2, 0},  // Верхний левый угол
                    {marker_size / 2, marker_size / 2, 0},  // Верхний правый угол
                    {marker_size / 2, -marker_size / 2, 0}, // Нижний правый угол
                    {-marker_size / 2, -marker_size / 2, 0} // Нижний левый угол
                };

                // Вычисление позы камеры относительно маркера
                cv::Vec3d rvec, tvec;
                cv::solvePnP(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec);

                // Рисуем оси координат на изображении
                cv::drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_size / 2);

                // Выводим информацию о маркере в лог
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Detected ArUco marker ID: " << marker_id
                                                                << " Translation: (" << tvec[0] << ", " << tvec[1] << ", " << tvec[2]
                                                                << ") Rotation: (" << rvec[0] << ", " << rvec[1] << ", " << rvec[2] << ")");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "No ArUco markers detected.");
        }

        //Публикуем изображение с рамками и осями
        cv_ptr->image = image;
        pub.publish(cv_ptr->toImageMsg());

        // Отображаем изображение с найденными маркерами (опционально)
        // cv::imshow("ArUco Markers", image);
        // cv::waitKey(1);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArUcoDetector>());
    rclcpp::shutdown();
    return 0;
}
