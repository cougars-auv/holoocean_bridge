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
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TwistStamped
from holoocean_interfaces.msg import AgentCommand


class CmdVelConverterNode(Node):
    """
    Converts ROS 2 cmd_vel commands to HoloOcean agent command messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_converter_node")

        self.declare_parameter("input_topic", "cmd_vel_out")
        self.declare_parameter("output_topic", "/command/agent")
        self.declare_parameter("agent_name", "auv0")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            TwistStamped,
            input_topic,
            self.listener_callback,
            qos_profile_system_default,
        )
        self.publisher = self.create_publisher(
            AgentCommand, output_topic, qos_profile_system_default
        )

        # HoloOcean BlueROV2 (BlueROV2.h)
        linear_drag = 11.5  # mass(11.5) × SetLinearDamping(1.0)
        angular_damping = 0.225  # Iz(0.3) × SetAngularDamping(0.75)
        self.thruster_limit = 28.75  # BR_MAX_THRUST
        thruster_x = 0.1562
        thruster_y = 0.0988

        # Force/torque to hold steady state against drag at unit velocity
        self.h_scale = linear_drag / (4.0 * math.sqrt(0.5))
        self.v_scale = linear_drag / 4.0
        self.a_scale = angular_damping / (
            4.0 * (thruster_x - thruster_y) * math.sqrt(0.5)
        )

        self.get_logger().info("Initialization complete.")

    def listener_callback(self, msg: TwistStamped) -> None:
        """
        Process cmd_vel (TwistStamped) messages.

        :param msg: TwistStamped message containing linear and angular velocities.
        """
        agent_cmd = AgentCommand()
        agent_cmd.header.stamp = self.get_clock().now().to_msg()
        agent_cmd.header.frame_id = self.agent_name

        # IMPORTANT! Assuming no quadratic drag
        fwd = msg.twist.linear.x * self.h_scale
        lat = msg.twist.linear.y * self.h_scale
        vert = msg.twist.linear.z * self.v_scale
        roll = msg.twist.angular.x * self.a_scale
        pitch = msg.twist.angular.y * self.a_scale
        yaw = msg.twist.angular.z * self.a_scale

        cmd_0 = vert - pitch - roll
        cmd_1 = vert - pitch + roll
        cmd_2 = vert + pitch + roll
        cmd_3 = vert + pitch - roll
        cmd_4 = fwd + lat + yaw
        cmd_5 = fwd - lat - yaw
        cmd_6 = fwd + lat - yaw
        cmd_7 = fwd - lat + yaw

        raw_cmds = [cmd_0, cmd_1, cmd_2, cmd_3, cmd_4, cmd_5, cmd_6, cmd_7]

        max_req = max([abs(x) for x in raw_cmds])
        if max_req > self.thruster_limit:
            scale_factor = self.thruster_limit / max_req
            final_cmds = [x * scale_factor for x in raw_cmds]
        else:
            final_cmds = raw_cmds

        agent_cmd.command = final_cmds
        self.publisher.publish(agent_cmd)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    cmd_vel_converter = CmdVelConverterNode()
    try:
        rclpy.spin(cmd_vel_converter)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
