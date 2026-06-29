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

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVLDR
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation

_Q_NED_ENU = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]).inv()


class DvlOdomConverterNode(Node):
    """
    Converts HoloOcean ground truth odometry to a simulated DVL A50 DVLDR message.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("dvl_odom_converter_node")

        self.declare_parameter("input_topic", "DynamicsSensorOdom")
        self.declare_parameter("output_topic", "dvl/position")
        self.declare_parameter("com_frame", "com_link")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("map_frame", "map")
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.com_frame = (
            self.get_parameter("com_frame").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(
            DVLDR, output_topic, qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, qos_profile_system_default
        )

        self.get_logger().info(
            f"DVL odom converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry) -> None:
        """
        Process ground truth Odometry and publish a simulated DVLDR message.

        :param msg: Odometry message from DynamicsSensorOdom (COM in HoloOcean frame).
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
        except Exception as ex:
            self.get_logger().warn(
                f"Could not transform {self.map_frame} to {msg.header.frame_id}: {ex}",
                throttle_duration_sec=1.0,
            )
            return

        try:
            com_T_dvl_tf = self.tf_buffer.lookup_transform(
                self.com_frame, self.dvl_frame, rclpy.time.Time()
            )
        except Exception as ex:
            self.get_logger().warn(
                f"Could not transform {self.com_frame} to {self.dvl_frame}: {ex}",
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

        p_dvl_in_com = PoseStamped()
        p_dvl_in_com.pose.position.x = com_T_dvl_tf.transform.translation.x
        p_dvl_in_com.pose.position.y = com_T_dvl_tf.transform.translation.y
        p_dvl_in_com.pose.position.z = com_T_dvl_tf.transform.translation.z
        p_dvl_in_com.pose.orientation = com_T_dvl_tf.transform.rotation

        p_dvl_in_map = do_transform_pose(p_dvl_in_com.pose, map_T_com_tf)

        # Convert ENU -> NED
        x_ned = p_dvl_in_map.position.y
        y_ned = p_dvl_in_map.position.x
        z_ned = -p_dvl_in_map.position.z

        q = p_dvl_in_map.orientation
        q_enu_b = Rotation.from_quat([q.x, q.y, q.z, q.w])
        q_ned_b = _Q_NED_ENU * q_enu_b
        roll, pitch, yaw = q_ned_b.as_euler("xyz", degrees=True)

        dvl_msg = DVLDR()
        dvl_msg.header.stamp = msg.header.stamp
        dvl_msg.header.frame_id = self.dvl_frame
        dvl_msg.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dvl_msg.position.x = x_ned
        dvl_msg.position.y = y_ned
        dvl_msg.position.z = z_ned
        dvl_msg.roll = roll
        dvl_msg.pitch = pitch
        dvl_msg.yaw = yaw
        dvl_msg.status = 0

        self.publisher.publish(dvl_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dvl_odom_converter_node = DvlOdomConverterNode()
    try:
        rclpy.spin(dvl_odom_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_odom_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
