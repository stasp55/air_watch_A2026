"""Тесты протокола обмена. Без ROS и без брокера."""

import json

import pytest

from td_link.protocol import (
    DuplicateFilter,
    Envelope,
    ProtocolError,
    cmd_topic,
    evt_topic,
    make_command,
    make_event,
)


def test_command_roundtrip():
    sent = make_command("dispatcher", "rover-01", "goto_cell", {"cell": "A3"})
    got = Envelope.decode(sent.encode())
    assert got.type == "cmd"
    assert got.name == "goto_cell"
    assert got.data == {"cell": "A3"}
    assert got.msg_id == sent.msg_id
    assert got.src == "dispatcher" and got.dst == "rover-01"


def test_event_carries_correlation_id():
    cmd = make_command("dispatcher", "drone-01", "find_object", {"kind": "TARGET"})
    evt = make_event("drone-01", "dispatcher", "result", {"success": True}, corr_id=cmd.msg_id)
    assert Envelope.decode(evt.encode()).corr_id == cmd.msg_id


def test_cyrillic_survives_roundtrip():
    evt = make_event("rover-01", "dispatcher", "status", {"message": "еду в A3"})
    assert Envelope.decode(evt.encode()).data["message"] == "еду в A3"


@pytest.mark.parametrize(
    "payload",
    [
        b"\xff\xfe not utf8",
        "not json at all",
        json.dumps([1, 2, 3]),
        json.dumps({"type": "cmd"}),                      # нет name
        json.dumps({"type": "wat", "name": "x"}),         # неизвестный тип
        json.dumps({"type": "cmd", "name": "x", "data": 5}),  # data не объект
        json.dumps({"v": 99, "type": "cmd", "name": "x"}),    # чужая версия
    ],
)
def test_broken_payloads_raise_protocol_error(payload):
    with pytest.raises(ProtocolError):
        Envelope.decode(payload)


def test_missing_optional_fields_get_defaults():
    env = Envelope.decode(json.dumps({"type": "evt", "name": "status"}))
    assert env.msg_id
    assert env.ts > 0
    assert env.data == {}


def test_topics():
    assert cmd_topic("fleet/v1/robots", "rover-01") == "fleet/v1/robots/rover-01/cmd"
    assert evt_topic("fleet/v1/robots/", "drone-01") == "fleet/v1/robots/drone-01/evt"


def test_duplicate_filter():
    flt = DuplicateFilter(capacity=3)
    assert flt.is_duplicate("a") is False
    assert flt.is_duplicate("a") is True
    for key in "bcd":
        flt.is_duplicate(key)
    assert flt.is_duplicate("a") is False, "самый старый id должен вытесняться"
