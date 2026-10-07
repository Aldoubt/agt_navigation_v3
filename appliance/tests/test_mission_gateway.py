import pytest
from test_contracts import route_data

from agt_field.contracts import ContractError
from agt_field.gateway import Framer, Watchdog, encode
from agt_field.mission import Mission


class Clock:
    value = 0

    def __call__(self):
        return self.value


def setup():
    clock = Clock()
    sent = []
    cancelled = []
    m = Mission(lambda p, e: sent.append((p, e)), cancelled.append, clock)
    binding = dict(map_bundle_id="a", map_version="1", bundle_sha256="a" * 64)
    m.bind(binding)
    m.load(route_data(binding))
    m.readiness(True, True)
    return m, clock, sent, cancelled


def test_success_and_dwell():
    m, c, s, _ = setup()
    m.start()
    for d in [5, 20, 0]:
        assert m.state == "NAVIGATING"
        m.result(m.epoch, True)
        assert m.state == "DWELLING"
        c.value += max(0, d - 0.1)
        m.tick()
        if d:
            assert m.state == "DWELLING"
            c.value += 0.1
            m.tick()
    assert m.state == "COMPLETED"
    assert len(s) == 3


def test_pause_navigation_ack_barrier():
    m, _, s, c = setup()
    m.start()
    old = m.epoch
    m.pause()
    assert c == [old]
    m.result(old, True)
    assert m.state == "PAUSED"
    with pytest.raises(ContractError):
        m.resume()
    m.cancelled(old)
    m.resume()
    assert m.epoch > old
    m.result(old, True)
    assert m.state == "NAVIGATING"
    assert len(s) == 2


def test_pause_dwell_freezes_time():
    m, c, _, _ = setup()
    m.start()
    m.result(m.epoch, True)
    c.value = 2
    m.pause()
    c.value = 100
    m.resume()
    m.tick()
    assert m.remaining == 3
    c.value = 103
    m.tick()
    assert m.index == 1


@pytest.mark.parametrize("fault", ["localization", "gateway", "nav", "cancel"])
def test_faults(fault):
    m, _, _, c = setup()
    m.start()
    old = m.epoch
    if fault == "localization":
        m.readiness(False, True)
    if fault == "gateway":
        m.readiness(True, False)
    if fault == "nav":
        m.result(old, False)
    if fault == "cancel":
        m.stop()
    assert m.state == ("CANCELLED" if fault == "cancel" else "ERROR")
    m.result(old, True)
    m.tick()
    assert m.index == 0
    if fault != "nav":
        assert c == [old]


def test_start_gate_and_map_switch():
    m, _, _, _ = setup()
    m.readiness(False, True)
    with pytest.raises(ContractError):
        m.start()
    m.readiness(True, True)
    m.start()
    with pytest.raises(ContractError):
        m.bind({})


def test_gateway_framing_and_watchdog():
    c = Clock()
    w = Watchdog(0.15, c)
    f = Framer()
    msg = dict(type="cmd_vel", session="one", sequence=1, linear=0.1, angular=0.2)
    raw = encode(msg)
    assert f.feed(raw[:5]) == []
    assert f.feed(raw[5:]) == [msg]
    w.connect("one")
    w.estop = False
    w.command("one", 1, 0.1, 0.2)
    assert w.output() == (0.1, 0.2)
    c.value = 0.16
    assert w.output() == (0, 0)
    w.disconnect()
    assert w.output() == (0, 0)
    w.connect("two")
    w.estop = False
    with pytest.raises(ContractError):
        w.command("one", 2, 0.1, 0)
    w.command("two", 1, 0.1, 0)
    assert w.output() == (0.1, 0)
    w.estop = True
    assert w.output() == (0, 0)
    w.estop = False
    assert w.output() == (0, 0)


def test_gateway_invalid_and_sequence():
    w = Watchdog(0.1)
    w.connect("one")
    with pytest.raises(ContractError):
        w.command("one", 1, float("nan"), 0)
    w.command("one", 1, 0, 0)
    with pytest.raises(ContractError):
        w.command("one", 1, 0.1, 0)
    with pytest.raises(ContractError):
        Framer().feed(b"x" * 65537)
