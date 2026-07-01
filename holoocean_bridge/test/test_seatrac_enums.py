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

"""
Unit tests for utils/seatrac_enums.py.

:author: Nelson Durrant (w Claude Opus 4.8)
:date: May 2026
"""

from holoocean_bridge.utils import seatrac_enums as seatrac


def test_amsgtype_table() -> None:
    """
    Verify AMSGTYPE_E codes map to a contiguous 0..7 block of msg types.
    """
    assert seatrac.AMSGTYPE_TO_MSG_TYPE[0] == "OWAY"
    assert seatrac.AMSGTYPE_TO_MSG_TYPE[7] == "MSG_RESPX"
    assert sorted(seatrac.AMSGTYPE_TO_MSG_TYPE) == list(range(8))


def test_req_resp_consistency() -> None:
    """
    Verify every request maps to a response and the two sets are disjoint.
    """
    assert seatrac.RESP_TYPES == set(seatrac.REQ_TO_RESP.values())
    assert set(seatrac.REQ_TO_RESP).isdisjoint(seatrac.RESP_TYPES)


def test_field_flag_sets_reference_known_types() -> None:
    """
    Verify the USBL, range, and depth flag sets only hold known msg types.
    """
    known = set(seatrac.AMSGTYPE_TO_MSG_TYPE.values())
    for flags in (seatrac.HAS_USBL, seatrac.HAS_RANGE, seatrac.HAS_Z):
        assert flags <= known


def test_clamp_int16() -> None:
    """
    Verify clamp_int16 saturates to the signed 16-bit range and rounds.
    """
    assert seatrac.clamp_int16(0.0) == 0
    assert seatrac.clamp_int16(40000) == 32767
    assert seatrac.clamp_int16(-40000) == -32768
    assert seatrac.clamp_int16(1.5) == 2
    assert seatrac.clamp_int16(-1.5) == -2


def test_clamp_uint16() -> None:
    """
    Verify clamp_uint16 saturates to the unsigned 16-bit range and rounds.
    """
    assert seatrac.clamp_uint16(0.0) == 0
    assert seatrac.clamp_uint16(-5) == 0
    assert seatrac.clamp_uint16(100000) == 65535
    assert seatrac.clamp_uint16(2.5) == 2
