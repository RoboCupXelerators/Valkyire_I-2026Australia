import queue
import sys
import threading
import types

import cv2
import numpy as np
import pytest

from control_panel import ControlPanelApp


def _load_arisu_intelligence():
    # Hailo is optional in the generic test environment.  Coordinate-space
    # calculation is pure Python, so a minimal import shim is sufficient here.
    module = types.ModuleType("hailo_platform")
    module.VDevice = object
    module.HailoSchedulingAlgorithm = types.SimpleNamespace(ROUND_ROBIN=object())
    previous = sys.modules.get("hailo_platform")
    sys.modules["hailo_platform"] = module
    try:
        from vision_utils.ArisuIntelligence import ArisuIntelligence

        return ArisuIntelligence
    finally:
        if previous is None:
            sys.modules.pop("hailo_platform", None)
        else:
            sys.modules["hailo_platform"] = previous


def test_hailo_bbox_coordinate_space_matches_integer_letterbox_resize():
    arisu = _load_arisu_intelligence()
    frame = np.zeros((721, 1281, 3), dtype=np.uint8)

    space = arisu._bbox_coordinate_space(frame)

    assert space == {
        "source_width": 1281,
        "source_height": 721,
        "input_width": 640,
        "input_height": 640,
        "resized_width": 640,
        "resized_height": 360,
        "pad_x": 0,
        "pad_y": 140,
    }


def test_hailo_goal_overlay_relocates_a_line_false_positive_to_yellow_goal():
    arisu = _load_arisu_intelligence()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # A yellow goal above the black field line that Hailo incorrectly boxed.
    frame[100:141, 180:381] = (0, 255, 255)
    raw_hailo_bbox = (180, 320, 380, 390, 0.65)

    refined = arisu._refine_goal_bbox_for_overlay(frame, 2, raw_hailo_bbox)
    x1, y1, x2, y2 = arisu._bbox_640_to_frame(refined, frame.shape)

    assert (x1, x2) == (180, 381)
    assert (y1, y2) == (100, 141)
    assert refined[-1] == pytest.approx(0.65)


def test_hailo_ball_overlay_relocates_a_line_false_positive_to_orange_ball():
    arisu = _load_arisu_intelligence()
    frame = np.full((480, 640, 3), (0, 170, 0), dtype=np.uint8)
    cv2.circle(frame, (190, 130), 20, (0, 140, 255), -1)
    raw_hailo_bbox = (170, 360, 211, 401, 0.61)

    refined = arisu._refine_ball_bbox_for_overlay(frame, raw_hailo_bbox)
    x1, y1, x2, y2 = arisu._bbox_640_to_frame(refined, frame.shape)

    assert (x1, y1, x2, y2) == (170, 110, 211, 151)
    assert refined[-1] == pytest.approx(0.61)


def test_hailo_ball_overlay_rejects_an_orange_hand_shaped_region():
    arisu = _load_arisu_intelligence()
    frame = np.full((480, 640, 3), (0, 170, 0), dtype=np.uint8)
    frame[90:150, 170:300] = (0, 140, 255)

    refined = arisu._refine_ball_bbox_for_overlay(frame, (170, 280, 300, 380, 0.80))

    assert refined is None


def test_hailo_chassis_overlay_relocates_a_line_false_positive_to_metal_body():
    arisu = _load_arisu_intelligence()
    frame = np.full((480, 640, 3), (0, 170, 0), dtype=np.uint8)
    frame[250:370, 220:380] = (20, 20, 20)  # black field line false-positive
    frame[75:195, 220:380] = (160, 160, 160)  # metallic chassis body
    raw_hailo_bbox = (220, 330, 380, 450, 0.55)

    refined = arisu._refine_chassis_bbox_for_overlay(frame, raw_hailo_bbox)
    x1, y1, x2, y2 = arisu._bbox_640_to_frame(refined, frame.shape)

    assert (x1, x2) == (220, 380)
    assert 70 <= y1 <= 90
    assert 190 <= y2 <= 210
    assert refined[-1] == pytest.approx(0.55)


def test_hailo_sparse_batch_keeps_the_source_camera_slot():
    arisu = _load_arisu_intelligence()

    class OutputBindings:
        def __init__(self, output):
            self._output = output

        def output(self):
            return self

        def get_buffer(self):
            return self._output

    class ConfiguredModel:
        def run(self, bindings, timeout_ms):
            assert len(bindings) == 1
            assert timeout_ms == 2000

    class OneBatchQueue:
        def __init__(self, batch):
            self._batch = batch
            self._consumed = False

        def get(self, timeout=None):
            if self._consumed:
                raise StopIteration
            self._consumed = True
            return self._batch

    output = [
        np.array([[0.20, 0.10, 0.40, 0.30, 0.90]], dtype=np.float32),
        np.empty((0, 5), dtype=np.float32),
        # Hailo's candidate is on a lower field line, while the actual
        # yellow goal is visibly above it in source_frame below.
        np.array([[0.50, 0.28125, 0.609375, 0.59375, 0.65]], dtype=np.float32),
        np.empty((0, 5), dtype=np.float32),
    ]
    coordinate_space = {
        "source_width": 640,
        "source_height": 480,
        "input_width": 640,
        "input_height": 640,
        "resized_width": 640,
        "resized_height": 480,
        "pad_x": 0,
        "pad_y": 80,
    }
    inst = arisu.__new__(arisu)
    inst._stop_event = threading.Event()
    inst._model_ready = threading.Event()
    inst._model_ready.set()
    inst._model_reset_event = threading.Event()
    inst._configured_model_lock = threading.Lock()
    inst._configured_model_generation = 1
    source_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(source_frame, (120, 20), 15, (0, 140, 255), -1)
    source_frame[100:141, 180:381] = (0, 255, 255)
    configured_model = ConfiguredModel()
    inst.YOLOQueue = OneBatchQueue(
        (
            configured_model,
            1,
            [(2, OutputBindings(output), coordinate_space, source_frame)],
        ),
    )
    inst.ConfiguredInferModel = configured_model
    inst.InferTF = 0
    inst.InferBatchSize = 0
    inst.InferBatchMs = 0.0
    inst.InferMs = 0.0
    inst._warned = True
    inst.ChassisQueue = queue.Queue(maxsize=1)
    inst.BallQueue = queue.Queue(maxsize=1)
    inst.GoalQueue = queue.Queue(maxsize=1)
    inst.BallBBoxQueue = queue.Queue(maxsize=1)
    inst.ChassisBBoxQueue = queue.Queue(maxsize=1)
    inst.GoalBBoxQueue = queue.Queue(maxsize=1)
    inst._bbox_coordinate_spaces = [None, None, None, None]
    inst._bbox_coordinate_spaces_lock = threading.Lock()
    inst._overlay_frames = [None, None, None, None]
    inst._overlay_frames_lock = threading.Lock()

    with pytest.raises(StopIteration):
        inst.ModelInfer()

    ball_bboxes = inst.BallBBoxQueue.get_nowait()
    assert ball_bboxes[0][-1] == 0.0
    assert ball_bboxes[2] == [2, 105, 85, 136, 116, pytest.approx(0.90)]
    # Pixel2CM receives this same corrected bbox through BallQueue.
    assert inst.BallQueue.get_nowait() == [[2, 120, 36, 31, 31, pytest.approx(0.90)]]
    goal_bboxes = inst.GoalBBoxQueue.get_nowait()
    assert goal_bboxes[2] == [2, 2, 180, 180, 381, 221, pytest.approx(0.65)]
    # Robot control still observes the unmodified Hailo candidate.
    assert inst.GoalQueue.get_nowait() == [[2, 2, 280, 310, 200, 70, pytest.approx(0.65)]]
    assert inst.GetBBoxCoordinateSpaces()[2] == coordinate_space
    assert inst.GetOverlayFrame(2) is source_frame


def test_hailo_invalidation_unpublishes_the_broken_configured_model():
    arisu = _load_arisu_intelligence()
    inst = arisu.__new__(arisu)
    broken_model = object()
    inst.ConfiguredInferModel = broken_model
    inst._configured_model_generation = 3
    inst._configured_model_lock = threading.Lock()
    inst._model_ready = threading.Event()
    inst._model_ready.set()
    inst._model_reset_event = threading.Event()

    inst._request_model_reinit(broken_model)

    assert inst.ConfiguredInferModel is None
    assert not inst._model_ready.is_set()
    assert inst._model_reset_event.is_set()


def test_hailo_stale_invalidation_does_not_reset_a_new_model_generation():
    arisu = _load_arisu_intelligence()
    inst = arisu.__new__(arisu)
    stale_model = object()
    current_model = object()
    inst.ConfiguredInferModel = current_model
    inst._configured_model_generation = 4
    inst._configured_model_lock = threading.Lock()
    inst._model_ready = threading.Event()
    inst._model_ready.set()
    inst._model_reset_event = threading.Event()

    inst._request_model_reinit(stale_model)

    assert inst.ConfiguredInferModel is current_model
    assert inst._model_ready.is_set()
    assert not inst._model_reset_event.is_set()


def test_control_panel_uses_hailo_inference_frame_for_overlay_jpeg():
    class Vision:
        CamPorts = [0, 2, 4, 6]

        def __init__(self):
            self.Frames = [np.full((16, 16, 3), (0, 0, 255), dtype=np.uint8)]
            self.Frames.extend([None, None, None])
            self.overlay_frame = np.full((16, 16, 3), (0, 255, 0), dtype=np.uint8)

        def GetOverlayFrame(self, camera_index):
            return self.overlay_frame if camera_index == 0 else None

    app = ControlPanelApp.__new__(ControlPanelApp)
    app.vision = Vision()
    app.jpeg_quality = 100

    jpeg, _ = app.get_cam_jpeg(0)

    assert jpeg is not None
    # JPEG compression can move values slightly, but green proves the snapshot
    # was selected instead of the newer live red frame.
    import cv2

    decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert int(decoded[0, 0, 1]) > 200
    assert int(decoded[0, 0, 2]) < 30
