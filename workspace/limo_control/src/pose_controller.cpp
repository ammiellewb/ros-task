#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class PoseController : public rclcpp::Node {
public:
    PoseController() : Node("pose_controller") {
        // Parameter declarations with type specifications
        this->declare_parameter<double>("goal_x", 5.0);
        this->declare_parameter<double>("goal_y", 3.0);
        this->declare_parameter<double>("goal_theta", 1.57);
        this->declare_parameter<double>("max_linear_speed", 0.5);
        this->declare_parameter<double>("max_angular_speed", 1.0);
        this->declare_parameter<double>("position_tolerance", 0.05);
        this->declare_parameter<double>("orientation_tolerance", 0.1);
        
        // Publishers and Subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PoseController::odomCallback, this, std::placeholders::_1));
        
        // Control gains
        k_rho_ = 0.5;
        k_alpha_ = 1.0;
        k_beta_ = -0.3;
        
        RCLCPP_INFO(this->get_logger(), "Pose controller initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get current pose
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = 2 * atan2(msg->pose.pose.orientation.z, 
                                msg->pose.pose.orientation.w);
        
        // Get parameters with explicit type
        double xg = this->get_parameter("goal_x").as_double();
        double yg = this->get_parameter("goal_y").as_double();
        double thetag = this->get_parameter("goal_theta").as_double();
        double max_linear = this->get_parameter("max_linear_speed").as_double();
        double max_angular = this->get_parameter("max_angular_speed").as_double();
        double pos_tol = this->get_parameter("position_tolerance").as_double();
        double ang_tol = this->get_parameter("orientation_tolerance").as_double();
        
        // Compute errors
        double dx = xg - x;
        double dy = yg - y;
        double rho = sqrt(dx*dx + dy*dy);
        double alpha = atan2(dy, dx) - theta;
        double beta = thetag - theta - alpha;
        
        // Normalize angles
        alpha = atan2(sin(alpha), cos(alpha));
        beta = atan2(sin(beta), cos(beta));
        
        // Control law
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        if (rho > pos_tol || fabs(thetag - theta) > ang_tol) {
            cmd_vel.linear.x = std::min(k_rho_ * rho, max_linear);
            cmd_vel.angular.z = std::clamp(
                k_alpha_ * alpha + k_beta_ * beta, 
                -max_angular, max_angular);
        }
        
        cmd_vel_pub_->publish(cmd_vel);
        RCLCPP_DEBUG(this->get_logger(), "Control: v=%.2f, ω=%.2f", 
                     cmd_vel.linear.x, cmd_vel.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double k_rho_, k_alpha_, k_beta_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseController>());
    rclcpp::shutdown();
    return 0;
}