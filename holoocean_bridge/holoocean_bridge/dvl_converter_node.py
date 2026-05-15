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
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from geometry_msgs.msg import TwistWithCovarianceStamped
from dvl_msgs.msg import DVL, DVLBeam


class DvlConverterNode(Node):
    """
    Converts DVL data from HoloOcean to Waterlinked DVL messages and adds noise.

    Uses the Janus configuration for the DVL noise model like HoloOcean.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("dvl_converter_node")

        self.declare_parameter("input_topic", "DVLSensorVelocity")
        self.declare_parameter("output_topic", "dvl/data")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("noise_sigmas", [0.02, 0.02, 0.02])
        self.declare_parameter("add_noise", True)

        self.declare_parameter("beam_elevation_deg", [22.5, 22.5, 22.5, 22.5])
        self.declare_parameter("beam_azimuth_deg", [135.0, 225.0, 315.0, 45.0])

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
        )
        self.noise_sigmas = (
            self.get_parameter("noise_sigmas").get_parameter_value().double_array_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )

        self.beam_geometry = self.beam_geometry_matrix(
            self.get_parameter("beam_elevation_deg")
            .get_parameter_value()
            .double_array_value,
            self.get_parameter("beam_azimuth_deg")
            .get_parameter_value()
            .double_array_value,
        )

        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            input_topic,
            self.listener_callback,
            qos_profile_system_default,
        )
        self.publisher = self.create_publisher(
            DVL, output_topic, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"DVL converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def beam_geometry_matrix(
        self, beam_tilt_angles: list[float], beam_azimuth_angles: list[float]
    ) -> np.ndarray:
        """Return the (Nx3) projection matrix H for a N-beam DVL.

        Each row is the unit vector for one beam, so that:
            beam_velocity_i = H[i] @ [vx, vy, vz]

        Parameters
        ----------
        beam_tilt_angles : list of (tilt_deg) per beam
        beam_azimuth_angles : list of (az_deg) per beam
        """
        # Check to make sure the input lists are the same length
        if len(beam_tilt_angles) != len(beam_azimuth_angles):
            raise ValueError(
                "Beam tilt and azimuth angle lists must be the same length."
            )

        rows = []
        for az_deg, tilt_deg in zip(beam_azimuth_angles, beam_tilt_angles):
            az = np.deg2rad(az_deg)
            tilt = np.deg2rad(tilt_deg)
            rows.append(
                [
                    np.sin(tilt) * np.cos(az),  # bx
                    np.sin(tilt) * np.sin(az),  # by
                    np.cos(tilt),  # bz  (positive: beam points toward +Z / seafloor)
                ]
            )
        return np.array(rows)

    def construct_beam_velocities(self, velocity: np.ndarray) -> np.ndarray:
        """
        Compute the velocity along each DVL beam given the 3D velocity vector.
        returns list of DVLBeam Messages.
        """
        beam_velocities = self.beam_geometry @ velocity
        return [DVLBeam(velocity=vel) for vel in beam_velocities]

    def listener_callback(self, msg: TwistWithCovarianceStamped) -> None:
        """
        Process DVL sensor data (TwistWithCovarianceStamped).

        :param msg: TwistWithCovarianceStamped message containing DVL data.
        """
        msg.header.frame_id = self.dvl_frame

        dvl_msg = DVL()
        dvl_msg.header = msg.header
        dvl_msg.header.frame_id = self.dvl_frame

        # TODO noise on each beam instead of velocity components
        if self.add_noise:
            noise_x = random.gauss(0, self.noise_sigmas[0])
            noise_y = random.gauss(0, self.noise_sigmas[1])
            noise_z = random.gauss(0, self.noise_sigmas[2])
        else:
            noise_x = 0.0
            noise_y = 0.0
            noise_z = 0.0

        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_z = msg.twist.twist.linear.z

        # Beam velocities
        dvl_msg.beams = self.construct_beam_velocities(np.array([vel_x, vel_y, vel_z]))

        # TODO reconstruct velocity measurements from beam velocities
        dvl_msg.velocity.x = vel_x + noise_x
        dvl_msg.velocity.y = vel_y + noise_y
        dvl_msg.velocity.z = vel_z + noise_z

        dvl_msg.velocity_valid = True

        # Convert nanoseconds to microseconds
        dvl_msg.time_of_validity = int(
            msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3
        )

        dvl_msg.covariance = [0.0] * 9
        dvl_msg.covariance[0] = self.noise_sigmas[0] ** 2
        dvl_msg.covariance[4] = self.noise_sigmas[1] ** 2
        dvl_msg.covariance[8] = self.noise_sigmas[2] ** 2

        self.publisher.publish(dvl_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dvl_converter_node = DvlConverterNode()
    try:
        rclpy.spin(dvl_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
