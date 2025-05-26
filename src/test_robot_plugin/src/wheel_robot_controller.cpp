#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>

using namespace std::chrono_literals;

namespace gazebo_plugins{
class WheelVelocityController : public gazebo::ModelPlugin{
public:
    WheelVelocityController() : ModelPlugin() {}
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override{
        node = gazebo_ros::Node::Get(sdf);                                         //Инициализация ROS2 узла
        joint_right = model->GetJoint("right_wheel_joint");
        joint_left = model->GetJoint("left_wheel_joint");
        if (!joint_right || !joint_left){
            RCLCPP_ERROR(node->get_logger(),"Wheel joints not found!");
            return;
        }
        cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&WheelVelocityController::cmd_vel_callback,this,std::placeholders::_1));
        joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
        timer = node->create_wall_timer(100ms,std::bind(&WheelVelocityController::timer_callback,this));
    }

private:
    gazebo_ros::Node::SharedPtr node;                                              //ROS2 узел
    gazebo::physics::JointPtr joint_right, joint_left;                             //Сочлениения колес
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;        //Подписчик на команды скорости
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;    //Публикатор для публикации углов сочленений
    rclcpp::TimerBase::SharedPtr timer;                                            //Таймер для периодической публикации

    void cmd_vel_callback(const geometry_msgs::msg::Twist msg){

        double linear_velocity = msg.linear.x; //Линейная скорость (м/с)
        double angular_velocity = msg.angular.z; //Угловая скорость (рад/с)

        double wheel_base = 0.26; //Растояние между колесами
        double wheel_radius = 0.05; //Радиус колес

        double v_right = (linear_velocity + (angular_velocity * wheel_base / 2.0)) / wheel_radius;
        double v_left = (linear_velocity - (angular_velocity * wheel_base / 2.0)) / wheel_radius;

        if (joint_right){
            joint_right->SetParam("fmax", 0, 100.0); //Максимальный крутящий момент
            joint_right->SetParam("vel", 0, v_right); //Установка скорости вращения правого колеса
        }

        if (joint_left){
            joint_left->SetParam("fmax", 0, 100.0); //Максимальный крутящий момент
            joint_left->SetParam("vel", 0, v_left); //Установка скорости врощения левого колеса
        }

    }

    void timer_callback() {
        if (joint_right && joint_left){

            sensor_msgs::msg::JointState joint_state_msg;

            joint_state_msg.name.push_back("right_wheel_joint");
            joint_state_msg.name.push_back("left_wheel_joint");

            joint_state_msg.position.push_back(joint_right->Position(0));
            joint_state_msg.position.push_back(joint_left->Position(0));

            joint_state_msg.velocity.push_back(joint_right->GetVelocity(0));
            joint_state_msg.velocity.push_back(joint_left->GetVelocity(0));

            joint_states_pub->publish(joint_state_msg);

        }
    }

};
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_plugins::WheelVelocityController) // Регистрация плагина
