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

_CID_NAV_QUERY_SEND = 0x50
_CID_PING_REQ = 0x41
_CID_PING_RESP = 0x42
_CID_DAT_REC = 0x61
_CID_NAV_QUERY_REQ = 0x51
_CID_NAV_QUERY_RESP = 0x52
_CST_OK = 0x00

_AMSGTYPE_TO_MSG_TYPE = {
    0: "OWAY",
    1: "OWAYU",
    2: "MSG_REQ",
    3: "MSG_RESP",
    4: "MSG_REQU",
    5: "MSG_RESPU",
    6: "MSG_REQX",
    7: "MSG_RESPX",
}

_MSG_TYPE_TO_REC_CID = {
    "OWAY": _CID_DAT_REC,
    "OWAYU": _CID_DAT_REC,
    "MSG_REQ": _CID_PING_REQ,
    "MSG_RESP": _CID_PING_RESP,
    "MSG_REQU": _CID_PING_REQ,
    "MSG_RESPU": _CID_PING_RESP,
    "MSG_REQX": _CID_NAV_QUERY_REQ,
    "MSG_RESPX": _CID_NAV_QUERY_RESP,
}

_HAS_USBL = {"OWAYU", "MSG_REQU", "MSG_RESPU", "MSG_REQX", "MSG_RESPX"}
_HAS_RANGE = {"MSG_RESPU", "MSG_RESPX"}
_HAS_Z = {"MSG_REQX", "MSG_RESPX"}

_RAD_TO_DEG = 180.0 / math.pi
_INT16_MAX = 32767
_INT16_MIN = -32768
_UINT16_MAX = 65535


def _clamp_int16(v: float) -> int:
    return max(_INT16_MIN, min(_INT16_MAX, int(round(v))))


def _clamp_uint16(v: float) -> int:
    return max(0, min(_UINT16_MAX, int(round(v))))


class ModemConverterNode(Node):
    """
    Bridges HoloOcean acoustic beacon data to/from seatrac_interfaces messages.

    :author: Nelson Durrant
    :date: June 2026
    """

    def __init__(self) -> None:
        super().__init__("modem_converter_node")

        self.declare_parameter("beacon_rec_topic", "AcousticBeaconSensor")
        self.declare_parameter("beacon_send_topic", "/acoustic_beacon_send")
        self.declare_parameter("modem_rec_topic", "modem_rec")
        self.declare_parameter("modem_send_topic", "modem_send")
        self.declare_parameter("modem_cmd_update_topic", "modem_cmd_update")
        self.declare_parameter("beacon_id", 1)
        self.declare_parameter("modem_frame", "modem_link")

        beacon_rec_topic = self.get_parameter("beacon_rec_topic").value
        beacon_send_topic = self.get_parameter("beacon_send_topic").value
        modem_rec_topic = self.get_parameter("modem_rec_topic").value
        modem_send_topic = self.get_parameter("modem_send_topic").value
        modem_cmd_update_topic = self.get_parameter("modem_cmd_update_topic").value
        self.beacon_id = self.get_parameter("beacon_id").value
        self.modem_frame = self.get_parameter("modem_frame").value

        self._send_queue = []
        self._waiting_for_ack = False

        self.beacon_rec_sub = self.create_subscription(
            AcousticBeaconSensor,
            beacon_rec_topic,
            self._beacon_callback,
            qos_profile_system_default,
        )
        self.modem_send_sub = self.create_subscription(
            ModemSend,
            modem_send_topic,
            self._modem_send_callback,
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

        self.get_logger().info(
            f"Modem converter started. Listening on {beacon_rec_topic} and {modem_send_topic}, "
            f"publishing on {modem_rec_topic}, {beacon_send_topic}, and {modem_cmd_update_topic}."
        )

    def _beacon_callback(self, msg: AcousticBeaconSensor) -> None:
        """
        Convert an incoming HoloOcean beacon to a ModemRec and drain the send queue.

        :param msg: Incoming acoustic beacon message from HoloOcean.
        """
        rec_cid = _MSG_TYPE_TO_REC_CID.get(msg.msg_type, _CID_DAT_REC)

        modem_rec = ModemRec()
        modem_rec.header.stamp = msg.header.stamp
        modem_rec.header.frame_id = self.modem_frame
        modem_rec.msg_id = rec_cid

        modem_rec.local_flag = msg.to_beacon == self.beacon_id
        modem_rec.dest_id = msg.to_beacon & 0xFF
        modem_rec.src_id = msg.from_beacon & 0xFF

        modem_rec.includes_usbl = msg.msg_type in _HAS_USBL
        if modem_rec.includes_usbl:
            modem_rec.usbl_azimuth = _clamp_int16(msg.azimuth * _RAD_TO_DEG * 10.0)
            modem_rec.usbl_elevation = _clamp_int16(msg.elevation * _RAD_TO_DEG * 10.0)
            modem_rec.usbl_channels = 4

        modem_rec.includes_range = msg.msg_type in _HAS_RANGE
        if modem_rec.includes_range:
            modem_rec.range_dist = _clamp_uint16(msg.range * 10.0)

        modem_rec.includes_position = msg.msg_type in _HAS_Z
        if modem_rec.includes_position:
            modem_rec.position_depth = _clamp_int16(msg.z * 10.0)

        payload = list(msg.msg_data[:30])
        modem_rec.packet_len = len(payload)
        modem_rec.packet_data = payload + [0] * (30 - len(payload))

        self.modem_rec_pub.publish(modem_rec)

        self._waiting_for_ack = False
        self._drain_send_queue()

    def _modem_send_callback(self, msg: ModemSend) -> None:
        """
        Queue an outgoing ModemSend and send immediately if the channel is free.

        :param msg: Outgoing modem message to queue.
        """
        self._send_queue.append(msg)
        if not self._waiting_for_ack:
            self._drain_send_queue()

    def _drain_send_queue(self) -> None:
        if not self._send_queue:
            return

        msg = self._send_queue.pop(0)

        beacon_send = AcousticBeaconSend()
        beacon_send.header = msg.header
        beacon_send.from_beacon = self.beacon_id
        beacon_send.to_beacon = int(msg.dest_id)
        if msg.msg_id == _CID_NAV_QUERY_SEND:
            beacon_send.msg_type = "MSG_REQX"
        else:
            beacon_send.msg_type = _AMSGTYPE_TO_MSG_TYPE.get(msg.msg_type, "OWAY")
        beacon_send.msg_data = list(msg.packet_data[: msg.packet_len])

        self.beacon_send_pub.publish(beacon_send)

        cmd_update = ModemCmdUpdate()
        cmd_update.header.stamp = self.get_clock().now().to_msg()
        cmd_update.msg_id = msg.msg_id
        cmd_update.command_status_code = _CST_OK
        cmd_update.target_id = msg.dest_id
        cmd_update.queue_size = len(self._send_queue)
        cmd_update.time_sent = msg.header.stamp

        self.modem_cmd_update_pub.publish(cmd_update)

        self._waiting_for_ack = True


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
