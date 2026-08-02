"""Тесты адаптера управления дроном (на заглушке)."""

import pytest

from td_drone_executor.drone_api import MockDrone, create_drone_api


def test_mock_starts_on_the_ground():
    drone = MockDrone()
    assert drone.position() == (0.0, 0.0, 0.0)


def test_flight_before_takeoff_is_refused():
    drone = MockDrone()
    ok, message = drone.go_to(1.0, 1.0, 1.5, None, 0.5, wait=True)
    assert ok is False
    assert "не в воздухе" in message


def test_takeoff_then_move_then_land():
    drone = MockDrone()
    assert drone.takeoff(1.6, 0.5)[0] is True
    assert drone.position()[2] == pytest.approx(1.6)

    assert drone.go_to(2.0, 3.0, 1.6, None, 0.5, wait=True)[0] is True
    assert drone.position() == pytest.approx((2.0, 3.0, 1.6))

    assert drone.land(10.0)[0] is True
    assert drone.position()[2] == pytest.approx(0.0)
    assert drone.log == ["takeoff 1.60", "go_to 2.00 3.00 1.60", "land"]


def test_factory_returns_mock():
    assert isinstance(create_drone_api("mock"), MockDrone)


def test_factory_rejects_unknown_backend():
    with pytest.raises(ValueError, match="backend"):
        create_drone_api("no_such_backend")
