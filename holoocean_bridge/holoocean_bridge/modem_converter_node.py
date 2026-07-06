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
from holoocean_interfaces.msg import AcousticBeaconSensor, AcousticBeaconSend
from seatrac_interfaces.msg import ModemRec, ModemSend, ModemCmdUpdate
from nav_msgs.msg import Odometry

from holoocean_bridge.utils import seatrac_enums as seatrac


class ModemConverterNode(Node):
    """
    Bridges HoloOcean acoustic beacon data to/from seatrac_interfaces messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("modem_converter_node")

        self.declare_parameter("beacon_rec_topic", "AcousticBeaconSensor")
        self.declare_parameter("beacon_send_topic", "/acoustic_beacon_send")
        self.declare_parameter("modem_rec_topic", "modem_rec")
        self.declare_parameter("modem_send_topic", "modem_send")
        self.declare_parameter("modem_cmd_update_topic", "modem_cmd_update")
        self.declare_parameter("depth_topic", "modem/depth/odometry")
        self.declare_parameter("beacon_id", 1)
        self.declare_parameter("modem_frame", "modem_link")
        self.declare_parameter("tick_period_sec", 0.1)
        self.declare_parameter("send_delay_sec", 0.4)
        self.declare_parameter("resp_delay_sec", 0.0)
        self.declare_parameter("resp_timeout_sec", 4.0)

        beacon_rec_topic = (
            self.get_parameter("beacon_rec_topic").get_parameter_value().string_value
        )
        beacon_send_topic = (
            self.get_parameter("beacon_send_topic").get_parameter_value().string_value
        )
        modem_rec_topic = (
            self.get_parameter("modem_rec_topic").get_parameter_value().string_value
        )
        modem_send_topic = (
            self.get_parameter("modem_send_topic").get_parameter_value().string_value
        )
        modem_cmd_update_topic = (
            self.get_parameter("modem_cmd_update_topic")
            .get_parameter_value()
            .string_value
        )
        depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self.beacon_id = (
            self.get_parameter("beacon_id").get_parameter_value().integer_value
        )
        self.modem_frame = (
            self.get_parameter("modem_frame").get_parameter_value().string_value
        )
        self.tick_period_sec = (
            self.get_parameter("tick_period_sec").get_parameter_value().double_value
        )
        self.send_delay_sec = (
            self.get_parameter("send_delay_sec").get_parameter_value().double_value
        )
        self.resp_delay_sec = (
            self.get_parameter("resp_delay_sec").get_parameter_value().double_value
        )
        self.resp_timeout_sec = (
            self.get_parameter("resp_timeout_sec").get_parameter_value().double_value
        )

        self.send_delay_ticks = max(
            1, round(self.send_delay_sec / self.tick_period_sec)
        )
        self.resp_delay_ticks = max(
            0, round(self.resp_delay_sec / self.tick_period_sec)
        )
        self.resp_timeout_ticks = max(
            1, round(self.resp_timeout_sec / self.tick_period_sec)
        )

        self.send_queue = []
        self.pending_auto_responses = []
        self.pending_resp_target = None
        self.send_delay_ticker = 0
        self.pending_resp_ticker = 0
        self.dat_queue = {}

        self.agent_depth = 0.0

        self.beacon_rec_sub = self.create_subscription(
            AcousticBeaconSensor,
            beacon_rec_topic,
            self.beacon_callback,
            qos_profile_system_default,
        )
        self.modem_send_sub = self.create_subscription(
            ModemSend,
            modem_send_topic,
            self.modem_send_callback,
            qos_profile_system_default,
        )
        self.depth_sub = self.create_subscription(
            Odometry,
            depth_topic,
            self.depth_callback,
            qos_profile_system_default,
        )

        self.modem_rec_pub = self.create_publisher(
            ModemRec, modem_rec_topic, qos_profile_system_default
        )
        self.beacon_send_pub = self.create_publisher(
            AcousticBeaconSend, beacon_send_topic, qos_profile_system_default
        )
        self.modem_cmd_update_pub = self.create_publisher(
            ModemCmdUpdate, modem_cmd_update_topic, qos_profile_system_default
        )

        self.tick_timer = self.create_timer(self.tick_period_sec, self.tick_callback)

        self.get_logger().info(
            f"Modem converter started. Listening on {beacon_rec_topic} and {modem_send_topic}, "
            f"publishing on {modem_rec_topic}, {beacon_send_topic}, and {modem_cmd_update_topic}."
        )

    def depth_callback(self, msg: Odometry) -> None:
        """
        Update the agent's depth.

        :param msg: Odometry message containing the agent's depth.
        """
        self.agent_depth = -msg.pose.pose.position.z

    def beacon_callback(self, msg: AcousticBeaconSensor) -> None:
        """
        Process an incoming HoloOcean beacon message.

        :param msg: Incoming acoustic beacon message from HoloOcean.
        """
        self.publish_modem_rec(msg)

        # The real beacon firmware answers REQ messages with the queued data
        if msg.msg_type in seatrac.REQ_TO_RESP and msg.to_beacon == self.beacon_id:
            self.queue_auto_response(msg)

        # A RESP from the queried beacon frees the channel
        if (
            msg.msg_type in seatrac.RESP_TYPES
            and msg.from_beacon == self.pending_resp_target
        ):
            self.pending_resp_target = None
            self.pending_resp_ticker = 0
            self.attempt_send()

    def publish_modem_rec(self, msg: AcousticBeaconSensor) -> None:
        """
        Convert an incoming beacon message to a ModemRec and publish it.

        :param msg: Incoming acoustic beacon message from HoloOcean.
        """
        modem_rec = ModemRec()
        modem_rec.header.stamp = msg.header.stamp
        modem_rec.header.frame_id = self.modem_frame
        modem_rec.msg_id = seatrac.CID_DAT_RECEIVE

        modem_rec.local_flag = msg.to_beacon in (self.beacon_id, 0)
        modem_rec.dest_id = msg.to_beacon & 0xFF
        modem_rec.src_id = msg.from_beacon & 0xFF

        modem_rec.depth_local = seatrac.clamp_int16(self.agent_depth * 10.0)

        modem_rec.includes_usbl = msg.msg_type in seatrac.HAS_USBL
        if modem_rec.includes_usbl:
            modem_rec.usbl_azimuth = seatrac.clamp_int16(
                math.degrees(msg.azimuth) * 10.0
            )
            modem_rec.usbl_elevation = seatrac.clamp_int16(
                math.degrees(msg.elevation) * 10.0
            )
            modem_rec.usbl_channels = 4

        modem_rec.includes_range = msg.msg_type in seatrac.HAS_RANGE
        if modem_rec.includes_range:
            modem_rec.range_dist = seatrac.clamp_uint16(msg.range * 10.0)

        modem_rec.includes_position = msg.msg_type in seatrac.HAS_Z
        if modem_rec.includes_position:
            # TODO: Fix RESPX remote depth reading in HoloOcean (not populated)
            modem_rec.position_enhanced = False
            z = self.agent_depth - msg.range * math.sin(msg.elevation)
            modem_rec.position_depth = seatrac.clamp_int16(z * 10.0)

        payload = list(msg.msg_data[:30])
        modem_rec.packet_len = len(payload)
        modem_rec.packet_data = payload + [0] * (30 - len(payload))

        self.modem_rec_pub.publish(modem_rec)

    def queue_auto_response(self, msg: AcousticBeaconSensor) -> None:
        """
        Answer a REQ message with the matching RESP type, after a response delay.

        :param msg: Incoming REQ beacon message to answer.
        """
        # Consume any payload staged for the requester (or for all beacons)
        queued = self.dat_queue.pop(int(msg.from_beacon), None) or self.dat_queue.pop(
            0, None
        )

        resp = AcousticBeaconSend()
        resp.header.stamp = self.get_clock().now().to_msg()
        resp.header.frame_id = self.modem_frame
        resp.from_beacon = self.beacon_id
        resp.to_beacon = int(msg.from_beacon)
        resp.msg_type = seatrac.REQ_TO_RESP[msg.msg_type]
        resp.msg_data = queued or []

        if self.resp_delay_ticks <= 0:
            self.send_queue.append((resp, False))
        else:
            self.pending_auto_responses.append([resp, self.resp_delay_ticks])
        self.attempt_send()

    def modem_send_callback(self, msg: ModemSend) -> None:
        """
        Process an outgoing ModemSend command.

        :param msg: Outgoing modem command.
        """
        if msg.msg_id == seatrac.CID_DAT_QUEUE_SET:
            self.set_dat_queue(msg)
            return

        if msg.msg_id != seatrac.CID_DAT_SEND:
            self.get_logger().warning(
                f"Unsupported send CID 0x{msg.msg_id:02X}; only CID_DAT_SEND "
                f"(0x{seatrac.CID_DAT_SEND:02X}) and CID_DAT_QUEUE_SET "
                f"(0x{seatrac.CID_DAT_QUEUE_SET:02X}) are simulated. Dropping message."
            )
            return

        beacon_send = AcousticBeaconSend()
        beacon_send.header = msg.header
        beacon_send.from_beacon = self.beacon_id
        beacon_send.to_beacon = int(msg.dest_id)
        beacon_send.msg_type = seatrac.AMSGTYPE_TO_MSG_TYPE.get(msg.msg_type, "OWAY")
        beacon_send.msg_data = list(msg.packet_data[: msg.packet_len])

        self.send_queue.append((beacon_send, True))
        self.attempt_send()

    def set_dat_queue(self, msg: ModemSend) -> None:
        """
        Stage RESP payload data for the next REQ from dest_id (or from any
        beacon, if dest_id is 0). An empty payload clears the staged data.

        :param msg: CID_DAT_QUEUE_SET modem command with the payload to stage.
        """
        payload = list(msg.packet_data[: msg.packet_len])
        if payload:
            self.dat_queue[int(msg.dest_id)] = payload
        else:
            self.dat_queue.pop(int(msg.dest_id), None)

        self.publish_cmd_update(seatrac.CID_DAT_QUEUE_SET, msg.dest_id)

    def tick_callback(self) -> None:
        """
        Refresh the send queue once per tick, timing out stale REQs.
        """
        # A REQ that never gets a RESP eventually times out and frees the channel
        if self.pending_resp_target is not None:
            self.pending_resp_ticker += 1
            if self.pending_resp_ticker >= self.resp_timeout_ticks:
                self.publish_cmd_update(
                    seatrac.CID_DAT_ERROR,
                    self.pending_resp_target,
                    seatrac.CST_XCVR_RESP_TIMEOUT,
                )
                self.pending_resp_target = None
                self.pending_resp_ticker = 0

        self.release_auto_responses()
        self.attempt_send()

        if self.send_delay_ticker > 0:
            self.send_delay_ticker -= 1

    def release_auto_responses(self) -> None:
        """
        Queue staged auto-responses whose response delay has elapsed.
        """
        ready = []
        for item in self.pending_auto_responses:
            item[1] -= 1
            if item[1] <= 0:
                ready.append(item)

        for item in ready:
            self.pending_auto_responses.remove(item)
            self.send_queue.append((item[0], False))

    def attempt_send(self) -> None:
        """
        Transmit the next queued send, or reject it with CST_XCVR_BUSY if the channel is in use.
        """
        if not self.send_queue or self.send_delay_ticker > 0:
            return

        beacon_send, is_command = self.send_queue[0]

        # The channel is held while a prior REQ awaits its RESP
        if self.pending_resp_target is not None:
            if is_command:
                self.publish_cmd_update(
                    seatrac.CID_DAT_SEND, beacon_send.to_beacon, seatrac.CST_XCVR_BUSY
                )
            self.send_delay_ticker = self.send_delay_ticks
            return

        self.send_queue.pop(0)
        self.beacon_send_pub.publish(beacon_send)

        if is_command:
            # Transmission always succeeds in sim
            self.publish_cmd_update(seatrac.CID_DAT_SEND, beacon_send.to_beacon)

        # A REQ holds the channel until its RESP arrives (or times out)
        if beacon_send.msg_type in seatrac.REQ_TO_RESP:
            self.pending_resp_target = int(beacon_send.to_beacon)
            self.pending_resp_ticker = 0

        self.send_delay_ticker = self.send_delay_ticks

    def publish_cmd_update(
        self, msg_id: int, target_id: int, status: int = seatrac.CST_OK
    ) -> None:
        """
        Publish a ModemCmdUpdate for a modem command.

        :param msg_id: CID of the acknowledged command.
        :param target_id: Beacon ID the command was addressed to.
        :param status: Command status code (CST_E).
        """
        cmd_update = ModemCmdUpdate()
        cmd_update.header.stamp = self.get_clock().now().to_msg()
        cmd_update.msg_id = msg_id
        cmd_update.command_status_code = status
        cmd_update.target_id = target_id & 0xFF
        cmd_update.queue_size = len(self.send_queue)
        cmd_update.time_sent = cmd_update.header.stamp

        self.modem_cmd_update_pub.publish(cmd_update)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    modem_converter_node = ModemConverterNode()
    try:
        rclpy.spin(modem_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        modem_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
