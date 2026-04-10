#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FakeStaticTFPublisher : public rclcpp::Node
{
public:
    FakeStaticTFPublisher()
    : Node("fake_static_tf_publisher")
    {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        publishTransforms();

        RCLCPP_INFO(this->get_logger(), "Fake static TFs published");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

    geometry_msgs::msg::TransformStamped makeTF(
        const std::string& parent,
        const std::string& child)
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = this->now();
        tf.header.frame_id = parent;
        tf.child_frame_id = child;

        // Identidad (sin offset)
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;

        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        return tf;
    }

    void publishTransforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> tfs;

        // Base → ruedas
        tfs.push_back(makeTF("base_footprint", "wheel0_link"));
        tfs.push_back(makeTF("base_footprint", "wheel1_link"));
        tfs.push_back(makeTF("base_footprint", "wheel2_link"));

        // Base → grippers (dedos)
        tfs.push_back(makeTF("base_footprint", "left_gripper_L_link"));
        tfs.push_back(makeTF("base_footprint", "left_gripper_R_link"));
        tfs.push_back(makeTF("base_footprint", "right_gripper_L_link"));
        tfs.push_back(makeTF("base_footprint", "right_gripper_R_link"));

        broadcaster_->sendTransform(tfs);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeStaticTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
