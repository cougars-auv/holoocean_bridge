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
import random
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from scipy.spatial.transform import Rotation


class ImuConverterNode(Node):
    """
    Converts IMU + AHRS data from HoloOcean to standard IMU messages and adds noise.

    Also models IMU bias as a random walk.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("imu_converter_node")

        self.declare_parameter("imu_input_topic", "IMUSensor")
        self.declare_parameter("ahrs_input_topic", "RotationSensor")
        self.declare_parameter("output_topic", "imu/data")
        self.declare_parameter("bias_topic", "imu/bias")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("accel_noise_sigmas", [0.0078, 0.0078, 0.0078])
        self.declare_parameter("gyro_noise_sigmas", [0.0012, 0.0012, 0.0012])
        self.declare_parameter("ahrs_noise_sigmas", [0.00349, 0.00349, 0.01745])
        self.declare_parameter("add_noise", True)
        self.declare_parameter("add_bias", True)
        self.declare_parameter("accel_bias_rw_sigmas", [1.05e-5, 1.05e-5, 1.05e-5])
        self.declare_parameter("gyro_bias_rw_sigmas", [3.91e-6, 3.91e-6, 3.91e-6])

        imu_input_topic = (
            self.get_parameter("imu_input_topic").get_parameter_value().string_value
        )
        ahrs_input_topic = (
            self.get_parameter("ahrs_input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        bias_topic = self.get_parameter("bias_topic").get_parameter_value().string_value
        self.imu_frame = (
            self.get_parameter("imu_frame").get_parameter_value().string_value
        )
        self.accel_noise_sigmas = (
            self.get_parameter("accel_noise_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.gyro_noise_sigmas = (
            self.get_parameter("gyro_noise_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.ahrs_noise_sigmas = (
            self.get_parameter("ahrs_noise_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )
        self.add_bias = self.get_parameter("add_bias").get_parameter_value().bool_value
        self.accel_bias_rw_sigmas = (
            self.get_parameter("accel_bias_rw_sigmas")
            .get_parameter_value()
            .double_array_value
        )
        self.gyro_bias_rw_sigmas = (
            self.get_parameter("gyro_bias_rw_sigmas")
            .get_parameter_value()
            .double_array_value
        )

        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.last_stamp = None

        self.imu_sub = message_filters.Subscriber(
            self, Imu, imu_input_topic, qos_profile=qos_profile_system_default
        )
        self.ahrs_sub = message_filters.Subscriber(
            self,
            Vector3Stamped,
            ahrs_input_topic,
            qos_profile=qos_profile_system_default,
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.ahrs_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(
            Imu, output_topic, qos_profile_system_default
        )
        self.bias_publisher = self.create_publisher(
            TwistWithCovarianceStamped, bias_topic, qos_profile_system_default
        )

        self.get_logger().info(
            f"IMU converter started. Listening on {imu_input_topic} + {ahrs_input_topic}, "
            f"publishing on {output_topic} and {bias_topic}."
        )

    def sync_callback(self, imu_msg: Imu, ahrs_msg: Vector3Stamped) -> None:
        """
        Process synchronized IMU and AHRS data.

        :param imu_msg: Imu message containing raw accelerometer and gyroscope data.
        :param ahrs_msg: Vector3Stamped message containing fused Euler angles in degrees.
        """
        roll_rad = math.radians(ahrs_msg.vector.x)
        pitch_rad = math.radians(ahrs_msg.vector.y)
        yaw_rad = math.radians(ahrs_msg.vector.z)

        if self.add_noise:
            roll_rad += random.gauss(0, self.ahrs_noise_sigmas[0])
            pitch_rad += random.gauss(0, self.ahrs_noise_sigmas[1])
            yaw_rad += random.gauss(0, self.ahrs_noise_sigmas[2])

        q = Rotation.from_euler("xyz", [roll_rad, pitch_rad, yaw_rad]).as_quat()

        imu_msg.header.frame_id = self.imu_frame

        if self.add_bias:
            current_stamp = (
                imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            )
            if self.last_stamp is not None:
                dt = current_stamp - self.last_stamp
                if dt > 0.0:
                    sqrt_dt = math.sqrt(dt)
                    for i in range(3):
                        self.accel_bias[i] += random.gauss(
                            0, self.accel_bias_rw_sigmas[i] * sqrt_dt
                        )
                        self.gyro_bias[i] += random.gauss(
                            0, self.gyro_bias_rw_sigmas[i] * sqrt_dt
                        )
            self.last_stamp = current_stamp

            imu_msg.linear_acceleration.x += self.accel_bias[0]
            imu_msg.linear_acceleration.y += self.accel_bias[1]
            imu_msg.linear_acceleration.z += self.accel_bias[2]

            imu_msg.angular_velocity.x += self.gyro_bias[0]
            imu_msg.angular_velocity.y += self.gyro_bias[1]
            imu_msg.angular_velocity.z += self.gyro_bias[2]

        if self.add_noise:
            imu_msg.linear_acceleration.x += random.gauss(0, self.accel_noise_sigmas[0])
            imu_msg.linear_acceleration.y += random.gauss(0, self.accel_noise_sigmas[1])
            imu_msg.linear_acceleration.z += random.gauss(0, self.accel_noise_sigmas[2])

            imu_msg.angular_velocity.x += random.gauss(0, self.gyro_noise_sigmas[0])
            imu_msg.angular_velocity.y += random.gauss(0, self.gyro_noise_sigmas[1])
            imu_msg.angular_velocity.z += random.gauss(0, self.gyro_noise_sigmas[2])

        imu_msg.linear_acceleration_covariance[0] = self.accel_noise_sigmas[0] ** 2
        imu_msg.linear_acceleration_covariance[4] = self.accel_noise_sigmas[1] ** 2
        imu_msg.linear_acceleration_covariance[8] = self.accel_noise_sigmas[2] ** 2

        imu_msg.angular_velocity_covariance[0] = self.gyro_noise_sigmas[0] ** 2
        imu_msg.angular_velocity_covariance[4] = self.gyro_noise_sigmas[1] ** 2
        imu_msg.angular_velocity_covariance[8] = self.gyro_noise_sigmas[2] ** 2

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.orientation_covariance[0] = self.ahrs_noise_sigmas[0] ** 2
        imu_msg.orientation_covariance[4] = self.ahrs_noise_sigmas[1] ** 2
        imu_msg.orientation_covariance[8] = self.ahrs_noise_sigmas[2] ** 2

        self.publisher.publish(imu_msg)

        bias_msg = TwistWithCovarianceStamped()
        bias_msg.header = imu_msg.header
        bias_msg.twist.twist.linear.x = self.accel_bias[0]
        bias_msg.twist.twist.linear.y = self.accel_bias[1]
        bias_msg.twist.twist.linear.z = self.accel_bias[2]
        bias_msg.twist.twist.angular.x = self.gyro_bias[0]
        bias_msg.twist.twist.angular.y = self.gyro_bias[1]
        bias_msg.twist.twist.angular.z = self.gyro_bias[2]

        self.bias_publisher.publish(bias_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    imu_converter_node = ImuConverterNode()
    try:
        rclpy.spin(imu_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
