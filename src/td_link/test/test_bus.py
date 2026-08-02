"""Тесты транспорта: сопоставление тем и локальная шина."""

import pytest

from td_link.bus import LoopbackBus, topic_matches


@pytest.mark.parametrize(
    "pattern,topic,expected",
    [
        ("td/v1/agents/rover-01/cmd", "td/v1/agents/rover-01/cmd", True),
        ("td/v1/agents/rover-01/cmd", "td/v1/agents/drone-01/cmd", False),
        ("td/v1/agents/+/evt", "td/v1/agents/rover-01/evt", True),
        ("td/v1/agents/+/evt", "td/v1/agents/rover-01/cmd", False),
        ("td/v1/agents/#", "td/v1/agents/rover-01/evt", True),
        ("td/v1/agents/+", "td/v1/agents/rover-01/evt", False),
    ],
)
def test_topic_matches(pattern, topic, expected):
    assert topic_matches(pattern, topic) is expected


def test_loopback_delivers_to_matching_subscribers():
    bus = LoopbackBus()
    received = []
    bus.subscribe("td/v1/agents/+/evt", lambda t, p: received.append((t, p)))
    bus.subscribe("td/v1/agents/rover-01/cmd", lambda t, p: received.append((t, p)))

    bus.publish("td/v1/agents/rover-01/evt", b"one")
    bus.publish("td/v1/agents/rover-01/cmd", b"two")
    bus.publish("td/v1/agents/rover-01/other", b"three")

    assert [payload for _, payload in received] == [b"one", b"two"]
