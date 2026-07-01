import importlib.util
import sys
import time
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
LUNAPRE_ROOT = ROOT / 'LunaPre'


def _install_reasondata_stub(monkeypatch, lidar_type='s3'):
    reasondata_stub = types.ModuleType('ReasonData')

    class DummySettings:
        class RoboInfo:
            LidarType = lidar_type

        class Debug:
            Beta = False
            FullLog = False

        class Ports:
            LidarID = 0
            LidarPort = '/dev/sllidar'

    class DummyLogger:
        def info(self, *args, **kwargs):
            return None

        def warning(self, *args, **kwargs):
            return None

        def error(self, *args, **kwargs):
            return None

        def debug(self, *args, **kwargs):
            return None

    reasondata_stub.Settings = DummySettings()
    reasondata_stub.logger = DummyLogger()
    reasondata_stub.LOG_FILE = 'dummy.log'
    reasondata_stub.DATA_DIR = '/tmp'
    reasondata_stub.ROS_ENV = '/tmp/setup.bash'
    reasondata_stub.Preference = lambda: type('Pref', (), {
        'read': lambda self, section, key: {
            'Ports': {'LidarID': 77, 'LidarPort': '/dev/sllidar'},
            'RoboInfo': {'LidarType': lidar_type},
        }[section][key]
    })()

    monkeypatch.setitem(sys.modules, 'ReasonData', reasondata_stub)


def _install_lunapre_package(monkeypatch):
    package = types.ModuleType('LunaPre')
    package.__path__ = [str(LUNAPRE_ROOT)]
    monkeypatch.setitem(sys.modules, 'LunaPre', package)


def _load_lunapre_module(monkeypatch, module_name, lidar_type='s3'):
    _install_reasondata_stub(monkeypatch, lidar_type=lidar_type)
    _install_lunapre_package(monkeypatch)
    path = LUNAPRE_ROOT / f'{module_name}.py'
    spec = importlib.util.spec_from_file_location(f'LunaPre.{module_name}', path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[f'LunaPre.{module_name}'] = module
    spec.loader.exec_module(module)
    return module


def test_ros2_driver_uses_s3_launch_when_configured(monkeypatch):
    driver_module = _load_lunapre_module(monkeypatch, 'lidar_driver_ros2', lidar_type='s3')
    captured = {}

    class DummyProcess:
        def __init__(self):
            self.pid = 1234

        def poll(self):
            return None

        def wait(self, timeout=None):
            return 0

    def fake_popen(args, **kwargs):
        captured['cmd'] = args[2]
        return DummyProcess()

    monkeypatch.setattr(driver_module, 'ROS_AVAILABLE', True)
    monkeypatch.setattr(driver_module, 'ROS_ENV', '/tmp/setup.bash')
    monkeypatch.setattr(driver_module.subprocess, 'Popen', fake_popen)

    driver = driver_module.ROS2LidarDriver(bashrc='/tmp/setup.bash')
    result = driver.start_sllidar_driver(enforce_settings=True, bashrc='/tmp/setup.bash')

    assert result is not None
    assert 'sllidar_s3_launch.py' in captured['cmd']
    assert 'serial_port:=/dev/sllidar' in captured['cmd']


def test_rplidar_driver_uses_queue(monkeypatch):
    driver_module = _load_lunapre_module(monkeypatch, 'lidar_driver_rplidar', lidar_type='s1')

    class FakeRPLidar:
        instances = []

        def __init__(self, port, baudrate, timeout):
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self._iter = iter([
                (False, 0, 10.0, 12.0),
                (True, 0, 20.0, 24.0),
            ])
            FakeRPLidar.instances.append(self)

        def iter_measurments(self):
            return self._iter

        def clear_input(self):
            return None

        def stop(self):
            return None

        def disconnect(self):
            return None

    fake_module = types.ModuleType('rplidar')
    fake_module.RPLidar = FakeRPLidar
    fake_module.RPLidarException = Exception
    monkeypatch.setitem(sys.modules, 'rplidar', fake_module)
    monkeypatch.setenv('LIDAR_PORT', '/dev/ttyUSB1')
    monkeypatch.setenv('LIDAR_BAUDRATE', '256000')
    monkeypatch.setenv('LIDAR_ANGLE_OFFSET_DEG', '0')
    monkeypatch.setenv('LIDAR_SCAN_MIN_POINTS', '1')

    scan_queue = __import__('queue').Queue(maxsize=3)
    driver = driver_module.RPLidarDriver(lidar_type='s1')
    driver.run_forever(scan_queue)

    assert FakeRPLidar.instances[0].port == '/dev/ttyUSB1'
    assert FakeRPLidar.instances[0].baudrate == 256000
    assert not scan_queue.empty()
    batch = scan_queue.get()
    assert batch[0] == (10.0, 0.012)


def test_rplidar_model_defaults(monkeypatch):
    driver_module = _load_lunapre_module(monkeypatch, 'lidar_driver_rplidar')

    s1 = driver_module.RPLidarDriver(lidar_type='s1')
    s2 = driver_module.RPLidarDriver(lidar_type='s2')
    s3 = driver_module.RPLidarDriver(lidar_type='s3')

    assert s1.port == '/dev/sllidar'
    assert s1.baudrate == 256000
    assert s1.scan_type == 'normal'
    assert s2.baudrate == 1000000
    assert s2.scan_type == 'normal'
    assert s3.baudrate == 1000000
    assert s3.scan_type == 'normal'


def test_sensor_facade_starts_injected_driver(monkeypatch):
    sensor = _load_lunapre_module(monkeypatch, 'sensor')

    class FakeDriver:
        def __init__(self):
            self.started = False
            self.stopped = False

        def start(self, scan_queue):
            self.started = True
            scan_queue.put(tuple([
                (0.0, 1.0),
                (1.0, 1.0),
                (90.0, 1.0),
                (91.0, 1.0),
                (180.0, 1.0),
                (181.0, 1.0),
            ]))
            return True

        def stop(self):
            self.stopped = True

    fake_driver = FakeDriver()
    lidar = sensor.Lidar(lambda: 0, driver=fake_driver, auto_start=True)
    assert fake_driver.started is True
    assert lidar.disabled is False
    lidar.stop()
    assert fake_driver.stopped is True


def test_sensor_facade_accepts_numpy_scan_frames(monkeypatch):
    np = pytest.importorskip("numpy")
    sensor = _load_lunapre_module(monkeypatch, 'sensor')

    assert sensor._scan_frame_is_empty(None) is True
    assert sensor._scan_frame_is_empty(np.empty((0, 2), dtype=np.float32)) is True
    assert sensor._scan_frame_is_empty(np.array([[0.0, 1.0]], dtype=np.float32)) is False

    class FakeDriver:
        def start(self, scan_queue):
            scan_queue.put(np.array([
                [0.0, 1.0],
                [1.0, 1.0],
                [90.0, 1.0],
                [91.0, 1.0],
                [180.0, 1.0],
                [181.0, 1.0],
            ], dtype=np.float32))
            return True

        def stop(self):
            return None

    lidar = sensor.Lidar(lambda: 0, driver=FakeDriver(), auto_start=True)
    deadline = time.time() + 1.0
    while time.time() < deadline and not lidar.dirLidarQueue.empty():
        time.sleep(0.01)
    lidar.stop()
    assert lidar.disabled is False


def test_sensor_backend_comes_from_config_not_environment(monkeypatch):
    sensor = _load_lunapre_module(monkeypatch, 'sensor')

    monkeypatch.setenv('LIDAR_BACKEND', 'ros2')
    sensor.Settings.RoboInfo.LidarBackend = 'rplidar'
    assert sensor._resolve_backend(None) == 'rplidar'

    sensor.Settings.RoboInfo.LidarBackend = 'ros2'
    assert sensor._resolve_backend(None) == 'ros2'
    assert sensor._resolve_backend('direct') == 'rplidar'
