import json
import threading
from types import SimpleNamespace
from urllib.request import urlopen
from unittest.mock import Mock

import pytest

import control_panel
from control_panel import ControlPanelHandler, IdleTimeoutHTTPServer


@pytest.fixture
def panel_server():
    servers = []

    def make(timeout=120, handler=ControlPanelHandler):
        clock = [0.0]

        class Handler(handler):
            app = SimpleNamespace(get_state_dict=lambda: {"ready": True})

        server = IdleTimeoutHTTPServer(('127.0.0.1', 0), Handler,
                                       idle_timeout_seconds=timeout, clock=lambda: clock[0])
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
        thread.start()
        servers.append((server, thread))
        return server, thread, clock

    yield make
    for server, thread in servers:
        if thread.is_alive():
            server.shutdown()
        thread.join(1)
        server.server_close()


def get(server, path='/api/state'):
    with urlopen(f'http://127.0.0.1:{server.server_port}{path}', timeout=2) as response:
        return json.load(response)


def test_monitoring_does_not_keep_idle_panel_alive(panel_server):
    server, thread, clock = panel_server()
    clock[0] = 119
    assert get(server, '/api/panel/lifecycle')['idle_seconds'] == 119
    clock[0] = 121
    thread.join(1)
    assert not thread.is_alive()


def test_requests_from_any_browser_renew_idle_deadline(panel_server):
    server, thread, clock = panel_server()
    clock[0] = 100
    assert get(server)['ready'] is True
    clock[0] = 200
    assert get(server)['ready'] is True
    clock[0] = 319
    assert get(server, '/api/panel/lifecycle')['shutting_down'] is False
    clock[0] = 321
    thread.join(1)
    assert not thread.is_alive()


def test_active_request_is_not_interrupted_by_idle_expiry(panel_server):
    started, release = threading.Event(), threading.Event()

    class HoldingHandler(ControlPanelHandler):
        def do_GET(self):
            started.set()
            release.wait(2)
            self._send_json({'done': True})

    server, thread, clock = panel_server(handler=HoldingHandler)
    responses = []
    client = threading.Thread(target=lambda: responses.append(get(server)))
    client.start()
    assert started.wait(1)
    clock[0] = 1000
    server.service_actions()
    assert not server.lifecycle_status()['shutting_down']
    release.set()
    client.join(1)
    assert responses == [{'done': True}]
    clock[0] = 1121
    thread.join(1)
    assert not thread.is_alive()


def test_manual_launch_timeout_zero_does_not_expire(panel_server):
    server, thread, clock = panel_server(timeout=0)
    clock[0] = 100000
    server.service_actions()
    assert thread.is_alive()
    assert not server.lifecycle_status()['shutting_down']


def test_main_runs_hardware_cleanup_after_managed_server_exits(monkeypatch):
    app = Mock()
    app.vision = None
    server = Mock()
    constructor = Mock(return_value=server)
    monkeypatch.setattr(control_panel, 'ControlPanelApp', Mock(return_value=app))
    monkeypatch.setattr(control_panel, 'IdleTimeoutHTTPServer', constructor)
    monkeypatch.setattr(control_panel.sys, 'argv', ['control_panel.py', '--no-runtime', '--no-vision', '--idle-timeout-seconds', '120'])
    control_panel.main()
    assert constructor.call_args.kwargs['idle_timeout_seconds'] == 120
    app.control_shutdown.assert_called_once()
    app.stop_recording.assert_called_once()
    app.calib_stop_stream.assert_called_once()
    app.birdeye_stop.assert_called_once()
    server.server_close.assert_called_once()


def test_shutdown_stops_chassis_and_dribbler():
    app = control_panel.ControlPanelApp.__new__(control_panel.ControlPanelApp)
    app._control_lock = threading.Lock()
    app._control_enabled = True
    app._control_last_motion_ts = 1.0
    app._dribble_active = True
    app._control_stop = Mock()
    peripheral = SimpleNamespace(Dribble=Mock())
    app._get_chassis_and_peripheral = lambda: (None, peripheral)
    app.control_shutdown()
    app._control_stop.assert_called_once()
    peripheral.Dribble.assert_called_once_with(False)
    assert not app._control_enabled
    assert not app._dribble_active
