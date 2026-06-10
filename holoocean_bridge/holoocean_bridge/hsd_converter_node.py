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
from coug_interfaces.msg import ControlSetpoint
from holoocean_interfaces.msg import DesiredCommand
from std_msgs.msg import Header


class HsdConverterNode(Node):
    """
    Converts a ControlSetpoint message into HoloOcean desired command messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("hsd_converter_node")

        self.declare_parameter("hsd_topic", "cmd_hsd")
        self.declare_parameter("output_heading_topic", "/heading")
        self.declare_parameter("output_speed_topic", "/speed")
        self.declare_parameter("output_depth_topic", "/depth")
        self.declare_parameter("agent_name", "auv0")

        self.hsd_topic = (
            self.get_parameter("hsd_topic").get_parameter_value().string_value
        )
        self.output_heading_topic = (
            self.get_parameter("output_heading_topic")
            .get_parameter_value()
            .string_value
        )
        self.output_speed_topic = (
            self.get_parameter("output_speed_topic").get_parameter_value().string_value
        )
        self.output_depth_topic = (
            self.get_parameter("output_depth_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )

        self.output_heading_pub = self.create_publisher(
            DesiredCommand, self.output_heading_topic, qos_profile_system_default
        )
        self.output_speed_pub = self.create_publisher(
            DesiredCommand, self.output_speed_topic, qos_profile_system_default
        )
        self.output_depth_pub = self.create_publisher(
            DesiredCommand, self.output_depth_topic, qos_profile_system_default
        )

        self.hsd_sub = self.create_subscription(
            ControlSetpoint,
            self.hsd_topic,
            self.hsd_callback,
            qos_profile_system_default,
        )

        self.get_logger().info(
            f"HSD converter started. Listening on {self.hsd_topic} and publishing "
            f"on {self.output_heading_topic}, {self.output_speed_topic}, and "
            f"{self.output_depth_topic}."
        )

    def create_command_msg(self, value: float) -> DesiredCommand:
        """
        Create a DesiredCommand message.

        :param value: The value (heading, speed, or depth) to put in the message.
        :return: Populated DesiredCommand message.
        """
        msg = DesiredCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.agent_name
        msg.data = float(value)
        return msg

    def hsd_callback(self, msg: ControlSetpoint) -> None:
        """
        Process a ControlSetpoint message and publish heading, speed, and depth separately.

        :param msg: ControlSetpoint message containing heading, speed, and depth.
        """
        self.output_heading_pub.publish(self.create_command_msg(msg.heading))
        self.output_speed_pub.publish(
            self.create_command_msg(max(0.0, min(1525.0, msg.speed)))
        )
        self.output_depth_pub.publish(self.create_command_msg(max(-msg.depth, 0.0)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HsdConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
