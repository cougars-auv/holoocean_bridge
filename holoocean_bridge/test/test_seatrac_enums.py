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

from holoocean_bridge.utils import seatrac_enums as se


def test_clamp_int16_in_range():
    assert se.clamp_int16(0) == 0
    assert se.clamp_int16(100.4) == 100
    assert se.clamp_int16(-100.6) == -101


def test_clamp_int16_saturates():
    assert se.clamp_int16(40000) == 32767
    assert se.clamp_int16(-40000) == -32768


def test_clamp_uint16_in_range():
    assert se.clamp_uint16(0) == 0
    assert se.clamp_uint16(1234.5) == 1234


def test_clamp_uint16_saturates():
    assert se.clamp_uint16(-5) == 0
    assert se.clamp_uint16(100000) == 65535


def test_req_resp_tables_are_consistent():
    # Every REQ maps to a RESP that is recorded in RESP_TYPES
    for req, resp in se.REQ_TO_RESP.items():
        assert req.startswith("MSG_REQ")
        assert resp in se.RESP_TYPES


def test_msg_type_lookup():
    assert se.AMSGTYPE_TO_MSG_TYPE[0] == "OWAY"
    assert se.AMSGTYPE_TO_MSG_TYPE[5] == "MSG_RESPU"
    # USBL-bearing messages are a subset of the known msg types
    known = set(se.AMSGTYPE_TO_MSG_TYPE.values())
    assert se.HAS_USBL <= known
    assert se.HAS_Z <= se.HAS_USBL
