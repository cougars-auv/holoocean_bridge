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

import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure


class PressureConverterNode(Node):
    """
    Converts depth odometry from HoloOcean to fluid pressure messages and adds noise.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("pressure_converter_node")

        self.declare_parameter("input_topic", "DepthSensor")
        self.declare_parameter("output_topic", "pressure")
        self.declare_parameter("depth_frame", "depth_link")
        self.declare_parameter("water_density", 997.0)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("atmospheric_pressure", 101325.0)
        self.declare_parameter("noise_sigma", 195.61)
        self.declare_parameter("add_noise", True)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.depth_frame = (
            self.get_parameter("depth_frame").get_parameter_value().string_value
        )
        self.water_density = (
            self.get_parameter("water_density").get_parameter_value().double_value
        )
        self.gravity = self.get_parameter("gravity").get_parameter_value().double_value
        self.atmospheric_pressure = (
            self.get_parameter("atmospheric_pressure")
            .get_parameter_value()
            .double_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, qos_profile_system_default
        )
        self.publisher = self.create_publisher(
            FluidPressure, output_topic, qos_profile_system_default
        )

        self.get_logger().info(
            f"Pressure converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry) -> None:
        """
        Process depth odometry and publish fluid pressure.

        :param msg: Odometry message containing depth in pose.pose.position.z.
        """
        depth = msg.pose.pose.position.z

        # pressure [Pa] = depth [m] * rho [kg/m^3] * g [m/s^2] + atmospheric_pressure [Pa]
        rho_g = self.water_density * self.gravity
        pressure = depth * rho_g + self.atmospheric_pressure

        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = msg.header.stamp
        pressure_msg.header.frame_id = self.depth_frame
        pressure_msg.fluid_pressure = pressure
        pressure_msg.variance = self.noise_sigma * self.noise_sigma

        if self.add_noise:
            pressure_msg.fluid_pressure += random.gauss(0, self.noise_sigma)

        self.publisher.publish(pressure_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    pressure_converter_node = PressureConverterNode()
    try:
        rclpy.spin(pressure_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        pressure_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
