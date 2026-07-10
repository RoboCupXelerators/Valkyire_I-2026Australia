"""Camera lifecycle / cleanup guarantees for VisionPreUntil."""

from __future__ import annotations

import threading
import time

import vision_utils.PreProcess as preprocess


class _FakeCam:
    def __init__(self, block_read: bool = False, open_ok: bool = True):
        self.released = False
        self.open_ok = open_ok
        self.block_read = block_read
        self.read_started = threading.Event()
        self._release_event = threading.Event()
        self.read_count = 0

    def isOpened(self):
        return self.open_ok and not self.released

    def read(self):
        self.read_started.set()
        self.read_count += 1
        if self.block_read:
            # Simulate a blocked V4L2 read until release() is called.
            while not self._release_event.is_set():
                time.sleep(0.01)
            return False, None
        if self.released or not self.open_ok:
            return False, None
        return True, object()

    def release(self):
        self.released = True
        self.open_ok = False
        self._release_event.set()


def test_close_releases_cameras_before_join(monkeypatch):
    released_before_join = {}

    class BlockingThread:
        def __init__(self, *args, **kwargs):
            self._alive = True
            self.daemon = kwargs.get("daemon", False)

        def start(self):
            self._alive = True

        def is_alive(self):
            return self._alive

        def join(self, timeout=None):
            # Capture whether cameras were already released when join happens.
            released_before_join["value"] = all(cam.released for cam in vision.Cams) if vision.Cams else True
            # Also check snapshot path used after release clears Cams.
            released_before_join["cams_cleared"] = vision.Cams == []
            self._alive = False

    vision = preprocess.VisionPreUntil.__new__(preprocess.VisionPreUntil)
    vision._stop_event = threading.Event()
    vision._close_lock = threading.RLock()
    vision._closed = False
    vision._cams_lock = threading.RLock()
    vision.cfg = type("C", (), {"Debug": type("D", (), {"FullLog": False})()})()
    vision.logger = type("L", (), {"warning": lambda *a, **k: None, "error": lambda *a, **k: None, "debug": lambda *a, **k: None, "info": lambda *a, **k: None})()
    vision.CamPorts = [0, 2]
    vision.StopRecord = 0
    vision.Frames = [None, None, None, None]
    vision.Videos = []
    vision.BallPos = [0, 0]
    vision.VideoRecordThread = None
    vision._recorder = None
    cams = [_FakeCam(), _FakeCam()]
    vision.Cams = cams
    vision.ReadCamsThread = BlockingThread()
    vision.ReadCamTF = 0
    vision.PreProcessTF = 0
    vision.InferTF = 0
    vision.PostProcessTF = 0
    vision.PreProcessMs = 0.0
    vision.InferMs = 0.0
    vision.InferBatchMs = 0.0
    vision.InferBatchSize = 0
    vision.PostProcessMs = 0.0

    preprocess.VisionPreUntil._register_instance(vision)
    assert vision in preprocess.VisionPreUntil._active_instances

    vision.close(join_timeout=0.2)

    assert released_before_join.get("cams_cleared") is True
    assert all(cam.released for cam in cams)
    assert vision.Cams == []
    assert vision not in preprocess.VisionPreUntil._active_instances
    # second close is a no-op
    vision.close(join_timeout=0.1)


def test_readcams_exits_after_cameras_released():
    vision = preprocess.VisionPreUntil.__new__(preprocess.VisionPreUntil)
    vision._stop_event = threading.Event()
    vision._close_lock = threading.RLock()
    vision._closed = False
    vision._cams_lock = threading.RLock()
    vision.cfg = type("C", (), {"Debug": type("D", (), {"FullLog": False})()})()
    vision.logger = type(
        "L",
        (),
        {
            "warning": lambda *a, **k: None,
            "error": lambda *a, **k: None,
            "debug": lambda *a, **k: None,
            "info": lambda *a, **k: None,
        },
    )()
    vision.CamPorts = [0]
    vision.Frames = [None, None, None, None]
    vision.ReadCamTF = 0
    cam = _FakeCam(block_read=True)
    vision.Cams = [cam]

    thread = threading.Thread(target=vision.ReadCams, daemon=True)
    vision.ReadCamsThread = thread
    thread.start()

    assert cam.read_started.wait(timeout=1.0)
    vision.close(join_timeout=1.0)

    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert cam.released


def test_cleanup_all_instances_closes_registered():
    vision = preprocess.VisionPreUntil.__new__(preprocess.VisionPreUntil)
    vision._stop_event = threading.Event()
    vision._close_lock = threading.RLock()
    vision._closed = False
    vision._cams_lock = threading.RLock()
    vision.cfg = type("C", (), {"Debug": type("D", (), {"FullLog": False})()})()
    vision.logger = type("L", (), {"warning": lambda *a, **k: None, "error": lambda *a, **k: None, "debug": lambda *a, **k: None, "info": lambda *a, **k: None})()
    vision.CamPorts = [0]
    vision.Frames = [None, None, None, None]
    vision.Videos = []
    vision.ReadCamsThread = None
    vision.VideoRecordThread = None
    vision._recorder = None
    cam = _FakeCam()
    vision.Cams = [cam]

    preprocess.VisionPreUntil._register_instance(vision)
    preprocess.VisionPreUntil._cleanup_all_instances(join_timeout=0.1)

    assert cam.released
    assert vision.Cams == []
    assert vision not in preprocess.VisionPreUntil._active_instances
