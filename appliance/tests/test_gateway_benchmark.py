import json
import socket
import threading
import time
import statistics
from agt_field.gateway import Framer, encode, Watchdog


def test_50hz_socket_transfer_and_disconnect():
    sender, receiver = socket.socketpair()
    receiver.settimeout(0.5)
    received = []
    errors = []
    w = Watchdog(0.12)
    w.connect("benchmark")
    w.estop = False

    def consume():
        f = Framer()
        try:
            while True:
                raw = receiver.recv(16384)
                if not raw:
                    break
                for row in f.feed(raw):
                    now = time.monotonic()
                    received.append((now, row["sent"]))
                    w.command(row["session"], row["sequence"], row["linear"], row["angular"])
                    receiver.sendall(
                        encode(dict(type="odom", sequence=row["sequence"], stamp=row["sent"]))
                    )
        except Exception as exc:
            errors.append(str(exc))
        finally:
            w.disconnect()

    thread = threading.Thread(target=consume)
    thread.start()
    f = Framer()
    acks = []
    start = time.monotonic()
    for i in range(50):
        target = start + i * 0.02
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        sender.sendall(
            encode(
                dict(
                    type="cmd_vel",
                    session="benchmark",
                    sequence=i,
                    linear=0.1,
                    angular=0,
                    sent=time.monotonic(),
                )
            )
        )
        acks.extend(f.feed(sender.recv(65536)))
    sender.close()
    thread.join(timeout=2)
    receiver.close()
    assert not errors and len(received) == 50 and len(acks) == 50
    rate = 49 / (received[-1][0] - received[0][0])
    assert 40 <= rate <= 60
    assert w.output() == (0, 0)
    latencies = [(rx, tx) for rx, tx in received]
    ms = [(rx - tx) * 1000 for rx, tx in latencies]
    result = dict(
        mode="MOCK_SOCKET_ONLY",
        samples=50,
        measured_hz=rate,
        one_way_p50_ms=statistics.median(ms),
        one_way_max_ms=max(ms),
        disconnect_zero=True,
    )
    print(json.dumps(result))
