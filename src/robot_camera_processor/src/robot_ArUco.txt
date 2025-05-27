// рабочий не правильно сделана метка
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class RobotCameraProcessor : public rclcpp::Node {
public:
    RobotCameraProcessor() : Node("robot_camera_processor") {
        // Инициализируем TransformBroadcaster через make_unique
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Параметры узла
        this->declare_parameter<double>("marker_size", 0.05);
        this->declare_parameter<std::string>("camera_topic", "/textured_camera/image_raw");
        this->declare_parameter<std::string>("output_topic", "/processed_image");
        this->declare_parameter<std::string>("pose_topic", "/marker_pose");
        this->declare_parameter<std::string>("camera_frame", "camera_link");
        this->declare_parameter<std::string>("marker_frame_prefix", "marker_");

        // Получение параметров
        marker_size_ = this->get_parameter("marker_size").as_double();
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        marker_frame_prefix_ = this->get_parameter("marker_frame_prefix").as_string();

        // Подписка на изображения с камеры
        sub_ = image_transport::create_subscription(
            this,
            camera_topic,
            std::bind(&RobotCameraProcessor::image_callback, this, std::placeholders::_1),
            "raw",
            rmw_qos_profile_sensor_data
            );

        // Публикация обработанных изображений
        pub_image_ = image_transport::create_publisher(this, output_topic);

        // Публикация позиций маркеров
        pub_markers_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

        // Инициализация словаря ArUco маркеров (6x6, 250 вариантов)
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        // Параметры камеры (можно загружать из файла)
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                              600, 0, 320,   // fx, 0, cx
                          0, 600, 240,   // 0, fy, cy
                          0, 0, 1);      // 0, 0, 1

        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);  // Коэффициенты дисторсии

        RCLCPP_INFO(this->get_logger(), "Узел обработки изображений инициализирован");
        RCLCPP_INFO(this->get_logger(), "Размер маркера: %.3f м", marker_size_);
    }

private:
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_markers_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double marker_size_;
    std::string camera_frame_;
    std::string marker_frame_prefix_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Конвертация ROS сообщения в OpenCV изображение
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            // Если изображение не в BGR, конвертируем
            if (msg->encoding != "bgr8") {
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
            }
            cv::Mat image = cv_ptr->image;

            // Обнаружение ArUco маркеров
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids);

            if (!marker_ids.empty()) {
                // Рисуем обнаруженные маркеры
                cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

                // Оценка позы маркеров
                std::vector<cv::Vec3d> rvecs, tvecs;

                // Вариант 1: для всех маркеров сразу (быстрее)
                cv::aruco::estimatePoseSingleMarkers(
                    marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

                // Обработка каждого маркера
                for (size_t i = 0; i < marker_ids.size(); ++i) {
                    // Вариант 2: индивидуальная оценка (точнее для отдельных маркеров)
                    std::vector<cv::Point3f> object_points = {
                        {-marker_size_/2, marker_size_/2, 0},   // Верхний левый
                        {marker_size_/2, marker_size_/2, 0},    // Верхний правый
                        {marker_size_/2, -marker_size_/2, 0},   // Нижний правый
                        {-marker_size_/2, -marker_size_/2, 0}   // Нижний левый
                    };
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(object_points, marker_corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);

                    // Используем результаты индивидуальной оценки
                    rvecs[i] = rvec;
                    tvecs[i] = tvec;

                    // Рисуем оси координат маркера
                    cv::drawFrameAxes(
                        image, camera_matrix_, dist_coeffs_,
                        rvecs[i], tvecs[i], marker_size_ * 0.5f);

                    // Публикация позиции маркера
                    publish_marker_pose(marker_ids[i], rvecs[i], tvecs[i]);

                    // Публикация трансформации в tf2
                    publish_marker_tf(marker_ids[i], rvecs[i], tvecs[i]);
                }
            }

            // Добавляем информационный текст
            cv::putText(image,
                        "Камера робота | Обнаружено маркеров: " + std::to_string(marker_ids.size()),
                        cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.8,
                        cv::Scalar(0, 255, 0),
                        2);

            // Публикация обработанного изображения
            cv_ptr->image = image;
            pub_image_.publish(cv_ptr->toImageMsg());

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка cv_bridge: %s", e.what());
        } catch (cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка OpenCV: %s", e.what());
        }
    }

    void publish_marker_pose(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = camera_frame_;

        // Позиция маркера
        pose_msg.pose.position.x = tvec[0];
        pose_msg.pose.position.y = tvec[1];
        pose_msg.pose.position.z = tvec[2];

        // Конвертация вращения в кватернион
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tf2::Matrix3x3 tf_rotation(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
        tf2::Quaternion q;
        tf_rotation.getRotation(q);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Публикация
        pub_markers_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(),
                    "Маркер ID: %d | Позиция: (%.3f, %.3f, %.3f) м | Ориентация: (%.3f, %.3f, %.3f, %.3f)",
                    marker_id,
                    tvec[0], tvec[1], tvec[2],
                    q.x(), q.y(), q.z(), q.w());
    }

    void publish_marker_tf(int marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Заполняем заголовок трансформации
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = camera_frame_;
        transform_stamped.child_frame_id = marker_frame_prefix_ + std::to_string(marker_id);

        // Позиция маркера
        transform_stamped.transform.translation.x = tvec[0];
        transform_stamped.transform.translation.y = tvec[1];
        transform_stamped.transform.translation.z = tvec[2];

        // Конвертация вращения в кватернион
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tf2::Matrix3x3 tf_rotation(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
        tf2::Quaternion q;
        tf_rotation.getRotation(q);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Публикация трансформации
        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotCameraProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
