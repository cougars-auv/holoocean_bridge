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
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Imu, MagneticField
from scipy.spatial.transform import Rotation
from seatrac_interfaces.msg import ModemStatus

_CID_STATUS = 16  # 0x10
_Q_NED_ENU = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]).inv()


class ModemStatusConverterNode(Node):
    """
    Combines processed IMU and magnetometer data into seatrac_interfaces/ModemStatus messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("modem_status_converter_node")

        self.declare_parameter("imu_input_topic", "imu/data")
        self.declare_parameter("mag_input_topic", "imu/mag")
        self.declare_parameter("output_topic", "modem/status")

        imu_input_topic = self.get_parameter("imu_input_topic").value
        mag_input_topic = self.get_parameter("mag_input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.start_time = self.get_clock().now()

        self.imu_sub = message_filters.Subscriber(
            self, Imu, imu_input_topic, qos_profile=qos_profile_system_default
        )
        self.mag_sub = message_filters.Subscriber(
            self, MagneticField, mag_input_topic, qos_profile=qos_profile_system_default
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.mag_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(
            ModemStatus, output_topic, qos_profile_system_default
        )

        self.get_logger().info(
            f"Modem status converter started. Listening on {imu_input_topic} and "
            f"{mag_input_topic}, publishing on {output_topic}."
        )

    def sync_callback(self, imu_msg: Imu, mag_msg: MagneticField) -> None:
        """
        Process synchronized IMU and magnetometer data.

        :param imu_msg: Imu message from imu_converter (with noise, bias, and fused orientation).
        :param mag_msg: MagneticField message from mag_converter (with noise).
        """
        modem_status_msg = ModemStatus()
        modem_status_msg.header = imu_msg.header

        modem_status_msg.msg_id = _CID_STATUS
        elapsed_ns = (self.get_clock().now() - self.start_time).nanoseconds
        modem_status_msg.timestamp = elapsed_ns // 1_000_000  # ms since start

        # Convert ENU -> NED
        q = imu_msg.orientation
        q_enu_b = Rotation.from_quat([q.x, q.y, q.z, q.w])
        q_ned_b = _Q_NED_ENU * q_enu_b
        yaw_ned, pitch_ned, roll_ned = q_ned_b.as_euler("zyx", degrees=True)

        modem_status_msg.includes_local_attitude = True
        modem_status_msg.attitude_yaw = max(-32768, min(32767, int(yaw_ned * 10)))
        modem_status_msg.attitude_pitch = max(-32768, min(32767, int(pitch_ned * 10)))
        modem_status_msg.attitude_roll = max(-32768, min(32767, int(roll_ned * 10)))

        modem_status_msg.includes_comp_ahrs = True
        modem_status_msg.acc_x = imu_msg.linear_acceleration.x
        modem_status_msg.acc_y = imu_msg.linear_acceleration.y
        modem_status_msg.acc_z = imu_msg.linear_acceleration.z
        modem_status_msg.gyro_x = imu_msg.angular_velocity.x
        modem_status_msg.gyro_y = imu_msg.angular_velocity.y
        modem_status_msg.gyro_z = imu_msg.angular_velocity.z
        modem_status_msg.mag_x = mag_msg.magnetic_field.x
        modem_status_msg.mag_y = mag_msg.magnetic_field.y
        modem_status_msg.mag_z = mag_msg.magnetic_field.z

        self.publisher.publish(modem_status_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    modem_status_converter_node = ModemStatusConverterNode()
    try:
        rclpy.spin(modem_status_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        modem_status_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
