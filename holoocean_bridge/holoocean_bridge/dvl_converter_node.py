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
from holoocean_interfaces.msg import DVLSensorRange


class DvlConverterNode(Node):
    """
    Converts DVL data from HoloOcean to Waterlinked DVL messages and adds noise.

    Uses the Janus configuration for the DVL noise model like HoloOcean.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("dvl_converter_node")

        self.declare_parameter("vel_topic", "DVLSensorVelocity")
        self.declare_parameter("range_topic", "DVLSensorRange")
        self.declare_parameter("output_topic", "dvl/data")
        self.declare_parameter("dvl_frame", "dvl_link")
        # TODO: Change the noise sigmas to be per beam
        self.declare_parameter("noise_sigmas", [0.02, 0.02, 0.02])
        self.declare_parameter("add_noise", False)
        self.declare_parameter("beam_elevation_deg", [22.5, 22.5, 22.5, 22.5])
        self.declare_parameter("beam_azimuth_deg", [135.0, 225.0, 315.0, 45.0])

        vel_topic = self.get_parameter("vel_topic").get_parameter_value().string_value
        range_topic = (
            self.get_parameter("range_topic").get_parameter_value().string_value
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
        self.beam_tilt_deg = self.get_parameter("beam_elevation_deg").value
        self.num_beams = len(self.beam_tilt_deg)
        self.beam_azimuth_deg = self.get_parameter("beam_azimuth_deg").value
        self.beam_geometry = self.beam_geometry_matrix(
            self.beam_tilt_deg,
            self.beam_azimuth_deg,
        )

        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            vel_topic,
            self.listener_callback,
            qos_profile_system_default,
        )
        self.range_subscription = self.create_subscription(
            DVLSensorRange,
            range_topic,
            self.range_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            DVL, output_topic, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"DVL converter started. Listening on {vel_topic} and {range_topic} and publishing on {output_topic}."
        )
        self.range = [-1.0] * self.num_beams

    def beam_geometry_matrix(
        self, beam_tilt_angles: list[float], beam_azimuth_angles: list[float]
    ) -> np.ndarray:
        """
        Return the (Nx3) projection matrix H for a N-beam DVL.

        Each row is the unit vector for one beam, so that:
            beam_velocity_i = H[i] @ [vx, vy, vz]

        Parameters
        beam_tilt_angles : list of (tilt_deg) per beam
        beam_azimuth_angles : list of (az_deg) per beam
        """
        # Check to make sure the input lists are the same length
        if self.num_beams != len(beam_azimuth_angles):
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

    def construct_beam_data(self, velocity: np.ndarray) -> list[DVLBeam]:
        """
        Compute the data for each DVL beam given the 3D velocity vector.
        returns list of DVLBeam Messages.
        """
        beam_velocities = self.beam_geometry @ velocity
        beams = []
        for i, vel in enumerate(beam_velocities):
            beam = DVLBeam()
            if self.add_noise:
                noise = random.gauss(
                    0, self.noise_sigmas[0]
                )  # TODO: different noise per beam?
                beam_velocities[i] += noise
            beam.id = i
            beam.velocity = vel
            # TODO: Will change this whe we update simulator
            beam.distance = self.range[i]
            beam.valid = (
                self.range[i] > 0.0
            )  # TODO: better validity check when we update simulator
            beams.append(beam)

        return beams

    def estimate_altitude_from_beams(
        self,
        beams: list[DVLBeam],
    ) -> float:
        """
        Estimate vertical altitude from per-beam slant-range distances.

        For each beam at phi_deg from vertical:
            vertical_i = slant_range_i * cos(phi_i)

        Parameters:
        beams : list[DVLBeam]  list of DVL beam messages

        Returns:
        estimated vertical altitude [m]
        """
        phi_cos = np.array(
            [np.cos(np.deg2rad(tilt)) for tilt in self.beam_tilt_deg]
        )  # (N,)
        beam_dist = np.array([beam.distance for beam in beams])  # (N,)
        vertical = beam_dist * phi_cos  # (N,)

        return np.nanmean(vertical)

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
        dvl_msg.beams = self.construct_beam_data(np.array([vel_x, vel_y, vel_z]))
        dvl_msg.altitude = self.estimate_altitude_from_beams(
            dvl_msg.beams,
        )
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

    def range_callback(self, msg: DVLSensorRange) -> None:
        """
        Process DVL range data (DVLSensorRange).

        :param msg: DVLSensorRange message containing DVL range data.
        """
        # Convert msg to float
        self.range = [float(r) for r in msg.range]


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
