from unittest.mock import Mock
import signal

import agt_global_relocalization.global_relocalization as relocalization_module
from rclpy.signals import SignalHandlerOptions


def test_main_handles_sigint_after_rclpy_context_is_already_shutdown(monkeypatch):
    node = Mock()
    init = Mock()
    monkeypatch.setattr(relocalization_module.rclpy, 'init', init)
    set_signal = Mock()
    monkeypatch.setattr(relocalization_module.signal, 'signal', set_signal)
    monkeypatch.setattr(relocalization_module, 'GlobalRelocalization', lambda: node)
    monkeypatch.setattr(
        relocalization_module.rclpy, 'spin',
        lambda _node: (_ for _ in ()).throw(KeyboardInterrupt()))
    monkeypatch.setattr(relocalization_module.rclpy, 'ok', lambda: False)
    shutdown = Mock()
    monkeypatch.setattr(relocalization_module.rclpy, 'shutdown', shutdown)

    relocalization_module.main()

    node.destroy_node.assert_called_once_with()
    init.assert_called_once_with(
        args=None, signal_handler_options=SignalHandlerOptions.NO)
    assert [call.args[0] for call in set_signal.call_args_list] == [
        signal.SIGINT, signal.SIGTERM]
    shutdown.assert_not_called()
