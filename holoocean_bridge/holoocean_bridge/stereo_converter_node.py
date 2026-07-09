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
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Image, CameraInfo
import message_filters


class StereoConverterNode(Node):
    """
    ROS 2 node that converts HoloOcean Image messages to Image and CameraInfo messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("stereo_converter_node")

        self.declare_parameter("front_input_topic", "RGBCameraFront")
        self.declare_parameter("back_input_topic", "RGBCameraBack")
        self.declare_parameter("front_output_topic", "stereo/front/image_raw")
        self.declare_parameter("back_output_topic", "stereo/back/image_raw")
        self.declare_parameter("front_stereo_info_topic", "stereo/front/camera_info")
        self.declare_parameter("back_stereo_info_topic", "stereo/back/camera_info")
        self.declare_parameter("front_stereo_frame", "front_stereo_link")
        self.declare_parameter("back_stereo_frame", "back_stereo_link")

        self.front_input_topic = self.get_parameter("front_input_topic").value
        self.back_input_topic = self.get_parameter("back_input_topic").value
        self.front_output_topic = self.get_parameter("front_output_topic").value
        self.back_output_topic = self.get_parameter("back_output_topic").value
        self.front_stereo_info_topic = self.get_parameter(
            "front_stereo_info_topic"
        ).value
        self.back_stereo_info_topic = self.get_parameter("back_stereo_info_topic").value
        self.front_stereo_frame = self.get_parameter("front_stereo_frame").value
        self.back_stereo_frame = self.get_parameter("back_stereo_frame").value

        self.front_pub = self.create_publisher(
            Image, self.front_output_topic, qos_profile_sensor_data
        )
        self.back_pub = self.create_publisher(
            Image, self.back_output_topic, qos_profile_sensor_data
        )
        self.front_info_pub = self.create_publisher(
            CameraInfo, self.front_stereo_info_topic, qos_profile_sensor_data
        )
        self.back_info_pub = self.create_publisher(
            CameraInfo, self.back_stereo_info_topic, qos_profile_sensor_data
        )

        self.front_sub = message_filters.Subscriber(
            self, Image, self.front_input_topic, qos_profile=qos_profile_system_default
        )
        self.back_sub = message_filters.Subscriber(
            self, Image, self.back_input_topic, qos_profile=qos_profile_system_default
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.front_sub, self.back_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Initialization complete.")

    def create_camera_info(self, image_msg: Image) -> CameraInfo:
        """
        Generate an identity CameraInfo message.

        :param image_msg: Image whose header and dimensions seed the CameraInfo.
        :return: CameraInfo with a pinhole intrinsics guess and identity rectification.
        """
        info = CameraInfo()
        info.header = image_msg.header
        info.height = image_msg.height
        info.width = image_msg.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = info.width / 2.0
        fy = fx
        cx = info.width / 2.0
        cy = info.height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return info

    def sync_callback(self, front_msg: Image, back_msg: Image) -> None:
        """
        Synchronize and publish the stereo image pair and their corresponding CameraInfo messages.

        :param front_msg: The front camera image message.
        :param back_msg: The back camera image message.
        """
        back_msg.header.stamp = front_msg.header.stamp

        front_msg.header.frame_id = self.front_stereo_frame
        back_msg.header.frame_id = self.back_stereo_frame

        self.front_pub.publish(front_msg)
        self.back_pub.publish(back_msg)

        front_info_msg = self.create_camera_info(front_msg)
        self.front_info_pub.publish(front_info_msg)

        back_info_msg = self.create_camera_info(back_msg)
        self.back_info_pub.publish(back_info_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    stereo_converter_node = StereoConverterNode()
    try:
        rclpy.spin(stereo_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        stereo_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
