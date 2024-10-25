#ifndef MOVEIT_SERVO_UR_HPP
#define MOVEIT_SERVO_UR_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include <string>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}
class moveit_servo_ur : public rclcpp::Node
{
public:
    moveit_servo_ur();

private:

    int state = 0;
    int count = 0;

    bool switched_controllers_flag = false;
    bool actived_servo_flag = false;   
    bool goal_reached = false; //Nazar added
    bool goal_reached_2 = false;
    bool goal_reached_3 = false;
    bool gipper_open = true;

    size_t count_;
    
    void wait_for_services();
    void routine_callback();

    void request_activate_servo();
    void request_switch_controllers();

    void client_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    void client_switch_controller_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future);
    // Nazar added
    void bottlePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void gripperCallback(std_msgs::msg::Bool::SharedPtr msg);
    void flip_call_back();
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_servo_trigger_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_switch_controller_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_routine_;
    rclcpp::TimerBase::SharedPtr flip_timer_rout_;
    //Nazar added    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bottle_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grip_state_sub_;    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grip_command_pub;
};

#endif // MOVEIT_SERVO_UR_HPP
