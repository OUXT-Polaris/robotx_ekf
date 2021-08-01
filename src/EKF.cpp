
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "EKF.hpp"
#include "geometry_msgs/msg/PoseWithCovarianceStamped"
#include "sensor_msgs/msg/Imu"
#include "geometry_msgs/msg/PoseStamped"
using std::placeholders::_1;


//get_sensordata_node
//get a_imu,w_imu,p_gps
class PubSub : public rclcpp::Node
{
  public:
    PubSub_EKF()
    : Node("PubSub_EKF"), count_(0)
    {
      GPSsubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "GPS_Topic", 10, std::bind(&PubSub_EKF::GPStopic_callback, this, _1));

      IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "IMU_Topic", 10, std::bind(&PubSub_EKF::IMUtopic_callback, this, _1));

      Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Pub_Pose", 10);

      timer_ = this->create_wall_timer(
        500ms, std::bind(&PubSub::timer_callback, this));

    }

  private:
    Eigen::vector10d x(10);
    Eigen::vector10d y(10);
    Eigen::vector6d u(6);
    
    
    void GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
    {
      y(0) = msg -> pose.pose.position.x;
      y(1) = msg -> pose.pose.position.y;
      y(2) = msg -> pose.pose.position.z;
        
    }

    void IMUtopic_callback(const sensor_msgs::msg::Imu msg) const
    {
      u(0) = msg-> angular_velocity.x;
      u(1) = msg-> angular_velocity.y;
      u(2) = msg-> angular_velocity.z;

      u(3) = msg-> linear_acceleration.x;
      u(4) = msg-> linear_acceleration.y;
      u(5) = msg-> linear_acceleration.z;

    }

    void timer_callback()
    {
      auto message = geometry_msgs::msg::PoseStamped();
      message.pose.position.x = x(0);
      message.pose.position.y = x(1);
      message.pose.position.y = x(2);

      Posepublisher_->publish(message);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr GPSsubscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Posepublisher_;
    size_t count_;
};


//EKF update

EKF::EKF(double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P) 
    : A(A), B(B), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt),
    I(n, n), x(n), x_hat(n)
{}

void EKF::init(const Eigen::VectorXd& x0) {
  x = x0;
  P = P0;
}


void EKF::update(const Eigen::VectorXd& y, Eigen::VectorXd& u, double& t){
    //予測ステップ
    x_hat = A * x + B* u;
    P = A*P*A.transpose() + B*Q*B.transpose();
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    //フィルタリングステップ
    x_hat += K * (y - C*x_hat);
    P = (I - K*C)*P;
    x = x_hat;
    t += dt;
}

