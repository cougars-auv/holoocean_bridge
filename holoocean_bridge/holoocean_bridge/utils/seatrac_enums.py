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

# AMSGTYPE_E enum values to HoloOcean msg_type strings
AMSGTYPE_TO_MSG_TYPE = {
    0: "OWAY",
    1: "OWAYU",
    2: "MSG_REQ",
    3: "MSG_RESP",
    4: "MSG_REQU",
    5: "MSG_RESPU",
    6: "MSG_REQX",
    7: "MSG_RESPX",
}

# Command identification codes (CID_E)
CID_STATUS = 0x10
CID_DAT_SEND = 0x60
CID_DAT_RECEIVE = 0x61
CID_DAT_ERROR = 0x63
CID_DAT_QUEUE_SET = 0x64

# Command status codes (CST_E)
CST_OK = 0x00
CST_XCVR_BUSY = 0x30
CST_XCVR_RESP_TIMEOUT = 0x34

# REQ message types and the RESP types that answer them
REQ_TO_RESP = {
    "MSG_REQ": "MSG_RESP",
    "MSG_REQU": "MSG_RESPU",
    "MSG_REQX": "MSG_RESPX",
}
RESP_TYPES = set(REQ_TO_RESP.values())

# Message types that carry USBL angles, range, and depth fields
HAS_USBL = {"OWAYU", "MSG_REQU", "MSG_RESPU", "MSG_REQX", "MSG_RESPX"}
HAS_RANGE = {"MSG_RESP", "MSG_RESPU", "MSG_RESPX"}
HAS_Z = {"MSG_RESPU", "MSG_RESPX"}


def clamp_int16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


def clamp_uint16(v: float) -> int:
    return max(0, min(65535, int(round(v))))
