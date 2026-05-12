# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose


class TruthConverterNode(Node):
    """
    Converts ground truth data from HoloOcean to odometry messages.

    Optionally publishes the map->base_link transform.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("truth_converter_node")

        self.declare_parameter("input_topic", "DynamicsSensorOdom")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("com_frame", "com_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_tf", False)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.com_frame = (
            self.get_parameter("com_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(
            Odometry, output_topic, qos_profile_system_default
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, qos_profile_system_default
        )

        self.get_logger().info(
            f"Truth converter started. Listening on {input_topic} "
            f"and publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry) -> None:
        """
        Process ground truth Odometry data.

        :param msg: Odometry message from DynamicsSensorOdom.
        """
        p_com_in_holo = PoseStamped()
        p_com_in_holo.header = msg.header
        p_com_in_holo.pose = msg.pose.pose

        try:
            p_com_in_map = self.tf_buffer.transform(
                p_com_in_holo,
                self.map_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            self.get_logger().error(
                f"Could not transform pose from {msg.header.frame_id} to {self.map_frame}",
                throttle_duration_sec=1.0,
            )
            return

        try:
            com_T_base_tf = self.tf_buffer.lookup_transform(
                self.com_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            self.get_logger().error(
                f"Could not find transform from {self.com_frame} to {self.base_frame}",
                throttle_duration_sec=1.0,
            )
            return

        map_T_com_tf = TransformStamped()
        map_T_com_tf.header = p_com_in_map.header
        map_T_com_tf.child_frame_id = self.com_frame
        map_T_com_tf.transform.translation.x = p_com_in_map.pose.position.x
        map_T_com_tf.transform.translation.y = p_com_in_map.pose.position.y
        map_T_com_tf.transform.translation.z = p_com_in_map.pose.position.z
        map_T_com_tf.transform.rotation = p_com_in_map.pose.orientation

        p_base_in_com = PoseStamped()
        p_base_in_com.pose.position.x = com_T_base_tf.transform.translation.x
        p_base_in_com.pose.position.y = com_T_base_tf.transform.translation.y
        p_base_in_com.pose.position.z = com_T_base_tf.transform.translation.z
        p_base_in_com.pose.orientation = com_T_base_tf.transform.rotation

        p_base_in_map = do_transform_pose(p_base_in_com.pose, map_T_com_tf)

        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = p_base_in_map
        odom_msg.pose.covariance = msg.pose.covariance
        odom_msg.twist.covariance = msg.twist.covariance

        self.publisher.publish(odom_msg)

        if self.get_parameter("publish_tf").get_parameter_value().bool_value:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = p_base_in_map.position.x
            t.transform.translation.y = p_base_in_map.position.y
            t.transform.translation.z = p_base_in_map.position.z
            t.transform.rotation = p_base_in_map.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    truth_converter_node = TruthConverterNode()
    try:
        rclpy.spin(truth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        truth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
