/**
 * Xelerator Control Panel - Main Application
 */

// Configuration
const CONFIG = {
  apiBase: "/api",
  updateInterval: 100, // ms
  visionApiBase: "", // Will be set from vision dashboard
};

// State
const state = {
  currentPage: "overview",
  currentCam: 0,
  isRecording: false,
  recordingSessionId: null,
  recordingFrameCount: 0,
  replaySessionId: null,
  replayFrames: [],
  replayCurrentFrame: 0,
  replayIsPlaying: false,
  replayLoop: false,
  replaySpeed: 1.0,
  fieldWidth: 260, // cm
  fieldHeight: 200, // cm
  config: null, // Configuration data
  localIP: "--",
  isOnline: false, // 系统在线状态
  lastOnlineTime: 0, // 最后在线时间
};

const RADAR_MIN_RANGE_CM = 25;
const RADAR_MAX_RANGE_CM = 1000;
const RADAR_RANGE_STEP_CM = 25;
const radarZoomState = {
  manualRangeCm: null,
  autoRangeCm: 100,
  lastData: null,
  pointCloud: [],
};

// DOM Elements
const elements = {};

// Initialize on DOM ready
document.addEventListener("DOMContentLoaded", init);

function init() {
  cacheElements();
  setupNavigation();
  setupRecordingControls();
  setupReplayControls();
  setupVisionControls();
  setupRadarControls();
  setupSettingsTabs();
  setupConfigPanel();
  setupOTAControls();
  loadConfig();
  startPolling();
  loadSessions();
  initFieldCanvas();

  // 初始化时设置为离线状态，直到成功连接到后端
  updateOnlineStatus(false);

  // 初始化键盘控制页面
  setupControlPage();
}

function cacheElements() {
  // Navigation
  elements.navItems = document.querySelectorAll(".nav-item");
  elements.pages = document.querySelectorAll(".page");

  // Overview
  elements.uptime = document.getElementById("uptime");
  elements.robotPose = document.getElementById("robot-pose");
  elements.lidarPointCount = document.getElementById("lidar-point-count");
  elements.hasBall = document.getElementById("has-ball");
  elements.ballX = document.getElementById("ball-x");
  elements.ballY = document.getElementById("ball-y");
  elements.recIndicator = document.getElementById("rec-indicator");
  elements.recStatus = document.getElementById("rec-status");
  elements.recFrames = document.getElementById("rec-frames");
  elements.quickRecordBtn = document.getElementById("quick-record-btn");
  elements.fps = document.getElementById("fps");
  elements.latency = document.getElementById("latency");

  // Recording
  elements.sessionId = document.getElementById("session-id");
  elements.recordFreq = document.getElementById("record-freq");
  elements.sessionDesc = document.getElementById("session-desc");
  elements.startRecordBtn = document.getElementById("start-record-btn");
  elements.stopRecordBtn = document.getElementById("stop-record-btn");
  elements.sessionsTbody = document.getElementById("sessions-tbody");

  // Replay
  elements.replaySessionSelect = document.getElementById(
    "replay-session-select",
  );
  elements.replayPlay = document.getElementById("replay-play");
  elements.replayPrev = document.getElementById("replay-prev");
  elements.replayNext = document.getElementById("replay-next");
  elements.replayLoop = document.getElementById("replay-loop");
  elements.replaySpeed = document.getElementById("replay-speed");
  elements.speedValue = document.getElementById("speed-value");
  elements.replayProgress = document.getElementById("replay-progress");
  elements.currentTime = document.getElementById("current-time");
  elements.totalTime = document.getElementById("total-time");
  elements.fieldCanvas = document.getElementById("field-canvas");
  elements.frameNum = document.getElementById("frame-num");
  elements.frameTs = document.getElementById("frame-ts");
  elements.framePos = document.getElementById("frame-pos");
  elements.frameBall = document.getElementById("frame-ball");

  // Vision
  elements.cameraFeed = document.getElementById("camera-feed");
  elements.bboxOverlay = document.getElementById("bbox-overlay");
  elements.detectionList = document.getElementById("detection-list");
  elements.camBtns = document.querySelectorAll(".cam-btn");
}

// Navigation
function setupNavigation() {
  elements.navItems.forEach((item) => {
    item.addEventListener("click", () => {
      const page = item.dataset.page;
      navigateTo(page);
    });
  });
}

function navigateTo(page) {
  const previousPage = state.currentPage;
  state.currentPage = page;

  elements.navItems.forEach((item) => {
    item.classList.toggle("active", item.dataset.page === page);
  });

  elements.pages.forEach((p) => {
    p.classList.toggle("active", p.id === `page-${page}`);
  });

  // Page-specific cleanup when leaving
  if (previousPage === "vision") {
    // 离开视觉页时停止摄像头流
    stopVisionFeeds();
  } else if (previousPage === "tools") {
    // 离开工具页时停止所有工具
    stopAllTools();
  }

  // Page-specific initialization
  if (page === "vision") {
    // 进入视觉页时启动摄像头流
    startVisionFeeds();
  } else if (page === "replay") {
    loadReplaySessions();
  } else if (page === "tools") {
    // 进入工具页时初始化
    if (!toolsState.initialized) {
      setupToolsPage();
      toolsState.initialized = true;
    }
  }
}

// Vision Controls - Four camera grid
function setupVisionControls() {
  // Initialize camera feeds - will be started when navigating to vision page
}

function setupRadarControls() {
  document
    .getElementById("radar-zoom-out")
    ?.addEventListener("click", () => changeRadarZoom(1.25));
  document
    .getElementById("radar-zoom-in")
    ?.addEventListener("click", () => changeRadarZoom(0.8));
  document
    .getElementById("radar-zoom-auto")
    ?.addEventListener("click", resetRadarZoom);
  updateRadarZoomControls();
}

function changeRadarZoom(factor) {
  const currentRange = radarZoomState.manualRangeCm ?? radarZoomState.autoRangeCm;
  const nextRange = Math.round(
    (currentRange * factor) / RADAR_RANGE_STEP_CM,
  ) * RADAR_RANGE_STEP_CM;
  radarZoomState.manualRangeCm = Math.max(
    RADAR_MIN_RANGE_CM,
    Math.min(RADAR_MAX_RANGE_CM, nextRange),
  );
  updateRadarZoomControls();
  if (radarZoomState.lastData) {
    drawRadar(radarZoomState.lastData);
  }
}

function resetRadarZoom() {
  radarZoomState.manualRangeCm = null;
  updateRadarZoomControls();
  if (radarZoomState.lastData) {
    drawRadar(radarZoomState.lastData);
  }
}

function updateRadarZoomControls() {
  const label = document.getElementById("radar-zoom-label");
  const zoomOut = document.getElementById("radar-zoom-out");
  const zoomIn = document.getElementById("radar-zoom-in");
  const autoButton = document.getElementById("radar-zoom-auto");
  const manualRange = radarZoomState.manualRangeCm;

  if (label) {
    label.textContent = manualRange === null
      ? `自动 ±${radarZoomState.autoRangeCm}cm`
      : `手动 ±${manualRange}cm`;
  }
  if (zoomOut) zoomOut.disabled = manualRange >= RADAR_MAX_RANGE_CM;
  if (zoomIn) zoomIn.disabled = manualRange !== null && manualRange <= RADAR_MIN_RANGE_CM;
  if (autoButton) autoButton.disabled = manualRange === null;
}

// Camera indices will be loaded from config
// CAM_INDICES: 显示位置对应的摄像头端口号 [Back, Left, Front, Right]
// CAM_PORTS: Vision 模块的 CamPorts 顺序 [Back, Left, Front, Right]（默认）
let CAM_INDICES = [0, 2, 4, 6]; // 显示位置对应的摄像头端口号
let CAM_PORTS = [0, 2, 4, 6]; // Vision 模块的 CamPorts 顺序

// Camera latency tracking (server-side processing time)
let camLatencies = {};

// 初始化摄像头索引（从配置加载）
function initCameraIndices() {
  // 新配置结构使用 Vision.CamPorts
  if (state.config && state.config.Vision && state.config.Vision.CamPorts) {
    // CamPorts 是 [Back, Left, Front, Right] 的顺序
    const camPorts = state.config.Vision.CamPorts;
    CAM_PORTS = camPorts;
    // 显示顺序: Back(后), Left(左), Front(前), Right(右)
    CAM_INDICES = [
      camPorts[0] ?? 0, // Back
      camPorts[1] ?? 2, // Left
      camPorts[2] ?? 4, // Front
      camPorts[3] ?? 6, // Right
    ];
    console.log("[Vision] CamPorts from config:", CAM_PORTS);
    console.log(
      "[Vision] Display indices (Back,Left,Front,Right):",
      CAM_INDICES,
    );
  }
  // 更新 camLatencies 的键
  camLatencies = {};
  CAM_INDICES.forEach((idx) => {
    camLatencies[idx] = [];
  });
}

// Vision feed intervals
let visionFeedInterval = null;
let visionStateInterval = null;
let lidarPointCloudInterval = null;
let visionStateRequestInFlight = false;
let lidarPointCloudRequestInFlight = false;
let overviewRequestInFlight = false;
const cameraRequestsInFlight = new Set();

function startVisionFeeds() {
  // 如果已经在运行，先停止
  stopVisionFeeds();

  // Refresh all four camera feeds periodically with latency tracking
  function refresh() {
    const t = Date.now();
    for (let i = 0; i < 4; i++) {
      const camIdx = CAM_INDICES[i];
      const img = document.getElementById("cam" + i);
      if (img && !cameraRequestsInFlight.has(camIdx)) {
        cameraRequestsInFlight.add(camIdx);
        // Use fetch to get response headers with processing time
        fetch(`/api/cam/${camIdx}.jpg?size=640&t=${t}`)
          .then((response) => {
            if (!response.ok) {
              throw new Error(
                `Camera ${camIdx} unavailable: ${response.status}`,
              );
            }
            // Read processing time from response header
            const processingTimeHeader = response.headers.get(
              "X-Processing-Time-Ms",
            );
            if (processingTimeHeader) {
              const processingTime = parseFloat(processingTimeHeader);
              camLatencies[camIdx].push(processingTime);
              if (camLatencies[camIdx].length > 10) {
                camLatencies[camIdx].shift();
              }
            }
            return response.blob();
          })
          .then((blob) => {
            const url = URL.createObjectURL(blob);
            img.onload = function () {
              URL.revokeObjectURL(url);
              drawAllOverlays();
            };
            img.src = url;
            updateLatencyDisplay();
          })
          .catch((e) => {
            // Clear latency on error
            camLatencies[camIdx] = [];
            updateLatencyDisplay();
          })
          .finally(() => {
            cameraRequestsInFlight.delete(camIdx);
          });
      }
    }
  }
  refresh();
  visionFeedInterval = setInterval(refresh, 120); // ~8 FPS

  // Start state polling for radar and info
  visionStateInterval = setInterval(updateVisionState, 100);
  updateLidarPointCloud();
  lidarPointCloudInterval = setInterval(updateLidarPointCloud, 200);
}

// 停止视觉页面的摄像头流
function stopVisionFeeds() {
  if (visionFeedInterval) {
    clearInterval(visionFeedInterval);
    visionFeedInterval = null;
  }
  if (visionStateInterval) {
    clearInterval(visionStateInterval);
    visionStateInterval = null;
  }
  if (lidarPointCloudInterval) {
    clearInterval(lidarPointCloudInterval);
    lidarPointCloudInterval = null;
  }
  radarZoomState.pointCloud = [];
  console.log("[Vision] Stopped vision feeds");
}

// 停止所有工具
function stopAllTools() {
  // 停止标定工具
  if (toolsState.calibRunning) {
    stopCalibStream();
  }
  if (calibPreviewInterval) {
    clearInterval(calibPreviewInterval);
    calibPreviewInterval = null;
  }

  // 停止鸟瞰图
  if (toolsState.birdeyeRunning) {
    stopBirdeye();
  }
  if (birdeyePreviewInterval) {
    clearInterval(birdeyePreviewInterval);
    birdeyePreviewInterval = null;
  }

  console.log("[Tools] Stopped all tools");
}

function updateLatencyDisplay() {
  let totalAvg = 0;
  let validCams = 0;

  for (let i = 0; i < 4; i++) {
    const camIdx = CAM_INDICES[i];
    const el = document.getElementById("lat-cam" + i);
    if (!el) continue;

    const samples = camLatencies[camIdx];
    if (samples.length > 0) {
      const avg = samples.reduce((a, b) => a + b, 0) / samples.length;
      el.textContent = avg.toFixed(0) + " ms";

      // Color coding based on latency
      el.classList.remove("latency-good", "latency-medium", "latency-bad");
      if (avg < 100) {
        el.classList.add("latency-good");
      } else if (avg < 200) {
        el.classList.add("latency-medium");
      } else {
        el.classList.add("latency-bad");
      }

      totalAvg += avg;
      validCams++;
    } else {
      el.textContent = "-- ms";
      el.classList.remove("latency-good", "latency-medium", "latency-bad");
    }
  }

  // Update average
  const avgEl = document.getElementById("lat-avg");
  if (avgEl) {
    if (validCams > 0) {
      const avg = totalAvg / validCams;
      avgEl.textContent = avg.toFixed(0) + " ms";
      avgEl.classList.remove("latency-good", "latency-medium", "latency-bad");
      if (avg < 100) {
        avgEl.classList.add("latency-good");
      } else if (avg < 200) {
        avgEl.classList.add("latency-medium");
      } else {
        avgEl.classList.add("latency-bad");
      }
    } else {
      avgEl.textContent = "-- ms";
      avgEl.classList.remove("latency-good", "latency-medium", "latency-bad");
    }
  }
}

// Vision state for bbox drawing
let visionState = {
  ball_bboxes_640: [null, null, null, null],
  chassis_bboxes_640: [null, null, null, null],
  goal_bboxes_640: [null, null, null, null],
  bbox_coordinate_spaces: [null, null, null, null],
  perf: {
    preprocess_tf: 0,
    inference_tf: 0,
    postprocess_tf: 0,
    camera_tf: 0,
  },
};

// Performance metrics calculation state
let perfState = {
  lastTs: 0,
  lastPreprocess: 0,
  lastInference: 0,
  lastPostprocess: 0,
  lastCamera: 0,
  // Smoothed values
  preprocessMs: 0,
  inferenceMs: 0,
  postprocessMs: 0,
  preprocessPeriodMs: 0,
  inferencePeriodMs: 0,
  postprocessPeriodMs: 0,
  cameraFps: 0,
};

function updateVisionState() {
  if (visionStateRequestInFlight) return;
  visionStateRequestInFlight = true;
  fetch("/api/state")
    .then((r) => {
      if (!r.ok) throw new Error(`State request failed: ${r.status}`);
      return r.json();
    })
    .then((data) => {
      // Store bboxes for drawing
      visionState.ball_bboxes_640 = data.ball_bboxes_640 || [
        null,
        null,
        null,
        null,
      ];
      visionState.chassis_bboxes_640 = data.chassis_bboxes_640 || [
        null,
        null,
        null,
        null,
      ];
      visionState.goal_bboxes_640 = data.goal_bboxes_640 || [
        null,
        null,
        null,
        null,
      ];
      visionState.bbox_coordinate_spaces = data.bbox_coordinate_spaces || [
        null,
        null,
        null,
        null,
      ];

      // Store performance metrics
      if (data.perf) {
        visionState.perf = data.perf;
      }

      // Update ball position
      const ballPosEl = document.getElementById("ball-pos");
      if (ballPosEl) {
        ballPosEl.textContent = formatBallPositionText(data.ball_pos_cm);
        ballPosEl.title = `Raw: ${formatBallPositionText(data.ball_pos_raw_cm)}`;
      }
      updateVisionDetails(data);
      drawAllOverlays();

      // Update vision status
      const statusEl = document.getElementById("vision-status");
      if (statusEl) {
        const backend =
          data.vision_status?.detect_backend ||
          data.vision_status?.type ||
          "--";
        statusEl.textContent = `状态: 在线 | 后端: ${backend} | 运行时间: ${formatDuration(data.uptime_s)}`;
      }

      updateLidarPanel(data.lidar_status);

      // Update radar
      drawRadar(data);
    })
    .catch((e) => {
      const statusEl = document.getElementById("vision-status");
      if (statusEl) {
        statusEl.textContent = "状态: 离线";
      }
    })
    .finally(() => {
      visionStateRequestInFlight = false;
    });
}

function updateLidarPointCloud() {
  if (lidarPointCloudRequestInFlight) return;
  lidarPointCloudRequestInFlight = true;
  fetch("/api/lidar/point-cloud")
    .then((response) => {
      if (!response.ok) throw new Error(`Point cloud request failed: ${response.status}`);
      return response.json();
    })
    .then((data) => {
      radarZoomState.pointCloud = Array.isArray(data.point_cloud_cm)
        ? data.point_cloud_cm
        : [];
      if (radarZoomState.lastData) {
        drawRadar(radarZoomState.lastData);
      }
    })
    .catch(() => {
      radarZoomState.pointCloud = [];
    })
    .finally(() => {
      lidarPointCloudRequestInFlight = false;
    });
}

function updateLidarPanel(lidarStatus) {
  const stateEl = document.getElementById("lidar-state");
  const distsEl = document.getElementById("lidar-dists");
  const scanPointCountEl = document.getElementById("lidar-scan-point-count");
  const packetPointCountEl = document.getElementById("lidar-packet-point-count");
  const normalizedPointCountEl = document.getElementById("lidar-normalized-point-count");
  if (!stateEl && !distsEl && !scanPointCountEl) return;

  const status = lidarStatus || {};
  const stateText = status.state || "unknown";
  const msg = status.message ? ` - ${status.message}` : "";
  if (stateEl) {
    stateEl.textContent = `${stateText}${msg}`;
    stateEl.className = "";
    if (stateText === "ok") {
      stateEl.classList.add("status-good");
    } else if (stateText === "no_data" || stateText === "not_initialized") {
      stateEl.classList.add("status-warn");
    } else {
      stateEl.classList.add("status-bad");
    }
  }

  if (distsEl) {
    const dists = Array.isArray(status.dist_mm) ? status.dist_mm : [];
    distsEl.textContent =
      dists.length === 4 ? dists.map((v) => `${v}mm`).join(" / ") : "--";
    distsEl.title = `backend=${status.backend || "--"}, driver=${status.driver || "--"}, running=${!!status.running}`;
  }

  const scanPointCount = Number(status.scan_point_count || 0);
  const packetPointCount = Number(status.packet_point_count || 0);
  const normalizedPointCount = Number(status.point_count || 0);
  if (scanPointCountEl) {
    scanPointCountEl.textContent = scanPointCount > 0 ? `${scanPointCount} 点` : "--";
  }
  if (packetPointCountEl) {
    packetPointCountEl.textContent = packetPointCount > 0 ? `${packetPointCount} 点` : "--";
  }
  if (normalizedPointCountEl) {
    normalizedPointCountEl.textContent = normalizedPointCount > 0
      ? `${normalizedPointCount} 点`
      : "--";
  }
}

// Map a detector-space bbox back to the displayed image.  Hailo publishes the
// exact transform from its inference batch; Jetson keeps the 640x640 fallback.
function mapBbox640ToImage(bbox, srcW, srcH, dstW, dstH, coordinateSpace = null) {
  if (!bbox || bbox.length < 6) return null;

  try {
    const prefix = bbox.length >= 7 ? bbox.slice(0, 2) : [bbox[0]];
    const x1 = bbox.length >= 7 ? bbox[2] : bbox[1];
    const y1 = bbox.length >= 7 ? bbox[3] : bbox[2];
    const x2 = bbox.length >= 7 ? bbox[4] : bbox[3];
    const y2 = bbox.length >= 7 ? bbox[5] : bbox[4];
    const conf = bbox.length >= 7 ? bbox[6] : bbox[5];

    srcW = Math.max(1, srcW);
    srcH = Math.max(1, srcH);
    dstW = Math.max(1, dstW);
    dstH = Math.max(1, dstH);

    const isFinitePositive = (value) => Number.isFinite(Number(value)) && Number(value) > 0;
    const space = coordinateSpace && typeof coordinateSpace === "object"
      ? coordinateSpace
      : {};
    const detectorW = isFinitePositive(space.input_width)
      ? Number(space.input_width)
      : 640;
    const detectorH = isFinitePositive(space.input_height)
      ? Number(space.input_height)
      : 640;
    const sourceW = isFinitePositive(space.source_width)
      ? Number(space.source_width)
      : srcW;
    const sourceH = isFinitePositive(space.source_height)
      ? Number(space.source_height)
      : srcH;
    const letterboxScale = Math.min(detectorW / sourceW, detectorH / sourceH);
    const resizedW = isFinitePositive(space.resized_width)
      ? Number(space.resized_width)
      : Math.floor(sourceW * letterboxScale);
    const resizedH = isFinitePositive(space.resized_height)
      ? Number(space.resized_height)
      : Math.floor(sourceH * letterboxScale);
    const padX = Number.isFinite(Number(space.pad_x))
      ? Number(space.pad_x)
      : Math.floor((detectorW - resizedW) / 2);
    const padY = Number.isFinite(Number(space.pad_y))
      ? Number(space.pad_y)
      : Math.floor((detectorH - resizedH) / 2);
    const toSourceX = (value) => (value - padX) * sourceW / resizedW;
    const toSourceY = (value) => (value - padY) * sourceH / resizedH;
    const clampX = (value) => Math.max(0, Math.min(sourceW, value));
    const clampY = (value) => Math.max(0, Math.min(sourceH, value));

    const sourceX1 = clampX(toSourceX(x1));
    const sourceY1 = clampY(toSourceY(y1));
    const sourceX2 = clampX(toSourceX(x2));
    const sourceY2 = clampY(toSourceY(y2));
    const displayScaleX = dstW / sourceW;
    const displayScaleY = dstH / sourceH;

    const dx1 = Math.round(sourceX1 * displayScaleX);
    const dy1 = Math.round(sourceY1 * displayScaleY);
    const dx2 = Math.round(sourceX2 * displayScaleX);
    const dy2 = Math.round(sourceY2 * displayScaleY);

    if (bbox.length >= 7) {
      return [prefix[0], prefix[1], dx1, dy1, dx2, dy2, conf];
    }
    return [prefix[0], dx1, dy1, dx2, dy2, conf];
  } catch (e) {
    return null;
  }
}

// Draw overlays for all cameras
function drawAllOverlays() {
  for (let i = 0; i < 4; i++) {
    const img = document.getElementById("cam" + i);
    const canvas = document.getElementById("ov" + i);
    if (!img || !canvas) continue;

    // The canvas covers the wrapper; the image itself may occupy only part of it
    // because object-fit: contain adds letterbox space around the image.
    const wrapper = img.parentElement;
    const wrapperW = wrapper ? wrapper.clientWidth : 640;
    const wrapperH = wrapper ? wrapper.clientHeight : 640;

    // Set canvas to match wrapper size (so overlay covers full area)
    canvas.width = wrapperW;
    canvas.height = wrapperH;

    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Match the browser's object-fit: contain layout exactly.
    const imgNaturalW = img.naturalWidth || 640;
    const imgNaturalH = img.naturalHeight || 480;

    // object-fit: contain means: scale = min(wrapperW/imgW, wrapperH/imgH)
    // Then the image is centered with padding
    const displayScale = Math.min(
      wrapperW / imgNaturalW,
      wrapperH / imgNaturalH,
    );
    const displayedW = imgNaturalW * displayScale;
    const displayedH = imgNaturalH * displayScale;
    const offsetX = (wrapperW - displayedW) / 2;
    const offsetY = (wrapperH - displayedH) / 2;

    const dstW = displayedW;
    const dstH = displayedH;

    // CAM_INDICES[i] 是显示位置 i 对应的摄像头端口号
    // CAM_PORTS 是 Vision 模块的 CamPorts 顺序，bbox 按 CAM_PORTS 顺序存储
    // 需要将摄像头端口号转换为 Frames 索引
    const camIdx = CAM_INDICES[i]; // 显示位置 i 对应的摄像头端口号
    const frameIdx = CAM_PORTS.indexOf(camIdx); // 在 CamPorts 中的位置，即 Frames 索引

    if (frameIdx < 0 || frameIdx >= 4) {
      continue;
    }

    // Bboxes are in detector space. Map them to the image area, then apply the
    // image's object-fit offset inside the wrapper.
    const ballBbox = visionState.ball_bboxes_640[frameIdx];
    if (ballBbox) {
      const mapped = mapBbox640ToImage(
        ballBbox,
        imgNaturalW,
        imgNaturalH,
        dstW,
        dstH,
        visionState.bbox_coordinate_spaces[frameIdx],
      );
      drawBboxOnCanvas(canvas, mapped, "#f72585", "ball", offsetX, offsetY);
    }

    // Draw chassis bbox
    const chassisBbox = visionState.chassis_bboxes_640[frameIdx];
    if (chassisBbox) {
      const mapped = mapBbox640ToImage(
        chassisBbox,
        imgNaturalW,
        imgNaturalH,
        dstW,
        dstH,
        visionState.bbox_coordinate_spaces[frameIdx],
      );
      drawBboxOnCanvas(canvas, mapped, "#4cc9f0", "chassis", offsetX, offsetY);
    }

    // Draw goal bbox
    const goalBbox = visionState.goal_bboxes_640[frameIdx];
    if (goalBbox && goalBbox.length >= 7) {
      const clsId = goalBbox[1];
      let color, label;
      if (clsId === 1) {
        color = "#3b82f6";
        label = "blue_goal";
      } else if (clsId === 2) {
        color = "#facc15";
        label = "yellow_goal";
      } else {
        color = "#a78bfa";
        label = "goal";
      }
      const mapped = mapBbox640ToImage(
        goalBbox,
        imgNaturalW,
        imgNaturalH,
        dstW,
        dstH,
        visionState.bbox_coordinate_spaces[frameIdx],
      );
      drawBboxOnCanvas(canvas, mapped, color, label, offsetX, offsetY);
    }
  }
}

// Draw bbox on canvas overlay with offset for object-fit: contain
function drawBboxOnCanvas(
  canvas,
  bbox,
  color,
  label,
  offsetX = 0,
  offsetY = 0,
) {
  if (!bbox || bbox.length < 6) return;

  const ctx = canvas.getContext("2d");

  // Get coordinates based on bbox format
  const x1_i = bbox.length >= 7 ? 2 : 1;
  const y1_i = bbox.length >= 7 ? 3 : 2;
  const x2_i = bbox.length >= 7 ? 4 : 3;
  const y2_i = bbox.length >= 7 ? 5 : 4;
  const conf_i = bbox.length >= 7 ? 6 : 5;

  const x1 = Math.round(bbox[x1_i] + offsetX);
  const y1 = Math.round(bbox[y1_i] + offsetY);
  const x2 = Math.round(bbox[x2_i] + offsetX);
  const y2 = Math.round(bbox[y2_i] + offsetY);
  const conf = bbox[conf_i];

  // Draw rectangle
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);

  // Draw label
  ctx.fillStyle = color;
  ctx.font = "12px sans-serif";
  const text = `${label} ${conf.toFixed(2)}`;
  const textY = Math.max(15, y1 - 5);
  ctx.fillText(text, x1, textY);
}

function drawRadar(data) {
  const canvas = document.getElementById("radar");
  if (!canvas) return;

  radarZoomState.lastData = data;

  const ctx = canvas.getContext("2d");
  const w = canvas.width,
    h = canvas.height;

  // Clear
  ctx.fillStyle = "#0c1430";
  ctx.fillRect(0, 0, w, h);

  const cx = w / 2,
    cy = h / 2;

  // 动态计算范围：根据可见目标的位置自动调整
  let maxDist = 100; // 最小范围 100cm
  const freeBall = getFreeBallCoords(data.ball_pos_cm);
  if (freeBall) {
    maxDist = expandRadarRange(maxDist, freeBall, 1.5);
  }
  if (Array.isArray(data.chassis_list_cm)) {
    for (const chassis of data.chassis_list_cm) {
      maxDist = expandRadarRange(maxDist, chassis, 1.35);
    }
  }
  maxDist = expandRadarRange(maxDist, data.blue_goal_pos_cm, 1.25);
  maxDist = expandRadarRange(maxDist, data.yellow_goal_pos_cm, 1.25);
  const lidarPoints = getLidarPointCloud(data);
  for (const point of lidarPoints) {
    maxDist = expandRadarRange(maxDist, point, 1.0);
  }

  // 机器人视角里目标可能在整场任意位置，不能只按半场尺寸裁剪
  const fieldMax = Math.hypot(
    state.fieldWidth || 260,
    state.fieldHeight || 200,
  );
  maxDist = Math.min(maxDist, fieldMax);

  // 确保动态范围是合理的整数（50cm 的倍数）
  maxDist = Math.ceil(maxDist / 50) * 50;
  maxDist = Math.max(100, Math.min(300, maxDist)); // 限制在 100-300cm
  radarZoomState.autoRangeCm = maxDist;
  updateRadarZoomControls();

  const rangeCm = radarZoomState.manualRangeCm ?? maxDist;
  const scale = (Math.min(w, h) * 0.4) / rangeCm;

  // Grid circles
  ctx.strokeStyle = "rgba(76,201,240,0.18)";
  ctx.lineWidth = 1;
  const gridStep = rangeCm <= 150 ? 25 : 50; // 小范围时使用 25cm 步长
  for (let r = gridStep; r <= rangeCm; r += gridStep) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();

    // 在网格圆上标注距离
    ctx.fillStyle = "rgba(159,176,214,0.5)";
    ctx.font = "10px sans-serif";
    ctx.fillText(`${r}cm`, cx + r * scale + 4, cy - 4);
  }

  // Axes
  ctx.strokeStyle = "rgba(233,238,252,0.15)";
  ctx.beginPath();
  ctx.moveTo(0, cy);
  ctx.lineTo(w, cy);
  ctx.moveTo(cx, 0);
  ctx.lineTo(cx, h);
  ctx.stroke();

  // 方向标注
  ctx.fillStyle = "rgba(159,176,214,0.7)";
  ctx.font = "12px sans-serif";
  ctx.fillText("前", cx + 4, 20);
  ctx.fillText("后", cx + 4, h - 10);
  ctx.fillText("左", 10, cy + 4);
  ctx.fillText("右", w - 20, cy + 4);

  // Lidar points share the radar's coordinate system and scale.
  drawLidarPoints(ctx, lidarPoints, cx, cy, scale, w, h);

  // Ego robot at center
  ctx.fillStyle = "rgba(58,238,159,0.85)";
  ctx.beginPath();
  ctx.arc(cx, cy, 8, 0, Math.PI * 2);
  ctx.fill();

  // 机器人标注
  ctx.fillStyle = "rgba(58,238,159,0.9)";
  ctx.font = "10px sans-serif";
  ctx.fillText("ROBOT", cx + 12, cy - 4);

  // Ball
  if (freeBall) {
    const bx = freeBall.x;
    const by = freeBall.y;
    const px = cx - by * scale; // Y left is positive
    const py = cy - bx * scale; // X forward is positive

    // 检查球是否在可视范围内
    if (px >= 0 && px <= w && py >= 0 && py <= h) {
      ctx.fillStyle = "rgba(247,37,133,0.95)";
      ctx.beginPath();
      ctx.arc(px, py, 6, 0, Math.PI * 2);
      ctx.fill();

      // 球的标注和距离
      const ballDist = Math.sqrt(bx * bx + by * by);
      ctx.fillStyle = "rgba(247,37,133,0.9)";
      ctx.font = "10px sans-serif";
      ctx.fillText(`BALL (${ballDist.toFixed(0)}cm)`, px + 10, py - 6);
    } else {
      // 球在范围外，显示指示箭头
      const angle = Math.atan2(-bx, -by);
      const arrowDist = Math.min(w, h) * 0.38;
      const arrowX = cx + Math.cos(angle) * arrowDist;
      const arrowY = cy + Math.sin(angle) * arrowDist;

      ctx.fillStyle = "rgba(247,37,133,0.7)";
      ctx.font = "10px sans-serif";
      ctx.fillText("BALL (超出范围)", arrowX - 40, arrowY);
    }
  }

  // Range label
  ctx.fillStyle = "rgba(159,176,214,0.85)";
  ctx.font = "10px sans-serif";
  ctx.fillText(`范围: ±${rangeCm}cm`, 8, 14);

  // Goal observations: bbox gives a bearing band, position gives an estimated bar.
  drawGoalObservations(ctx, data, cx, cy, scale, rangeCm);

  // Draw chassis detections
  // chassis_list_cm is a list of [x, y] positions in robot frame
  if (data.chassis_list_cm && data.chassis_list_cm.length > 0) {
    for (const chassis of data.chassis_list_cm) {
      if (chassis && chassis.length >= 2) {
        const chx = chassis[0],
          chy = chassis[1];
        if (chx !== 0 || chy !== 0) {
          // Convert to canvas coordinates (same as ball)
          // X forward is positive -> up on canvas
          // Y left is positive -> left on canvas
          const px = cx - chy * scale; // Y left is positive
          const py = cy - chx * scale; // X forward is positive

          if (px >= 0 && px <= w && py >= 0 && py <= h) {
            // Draw chassis as a rectangle
            ctx.fillStyle = "rgba(76,201,240,0.85)";
            ctx.fillRect(px - 10, py - 8, 20, 16);

            // Chassis label
            ctx.fillStyle = "rgba(76,201,240,0.9)";
            ctx.font = "9px sans-serif";
            const chassisDist = Math.sqrt(chx * chx + chy * chy);
            ctx.fillText(
              `CHASSIS (${chassisDist.toFixed(0)}cm)`,
              px + 12,
              py - 2,
            );
          }
        }
      }
    }
  }

}

function getLidarPointCloud(data) {
  const rawPoints = radarZoomState.pointCloud.length > 0
    ? radarZoomState.pointCloud
    : data?.lidar_status?.point_cloud_cm;
  if (!Array.isArray(rawPoints)) return [];
  return rawPoints.map(normalizeRadarPoint).filter(Boolean);
}

function drawLidarPoints(ctx, points, cx, cy, scale, width, height) {
  if (!points.length) return;

  ctx.fillStyle = "rgba(76,201,240,0.75)";
  ctx.beginPath();
  for (const point of points) {
    const { x, y } = point;
    const canvasPoint = robotPointToRadarCanvas(x, y, cx, cy, scale);
    if (
      canvasPoint.x < 0 ||
      canvasPoint.x > width ||
      canvasPoint.y < 0 ||
      canvasPoint.y > height
    ) {
      continue;
    }
    ctx.moveTo(canvasPoint.x + 2, canvasPoint.y);
    ctx.arc(canvasPoint.x, canvasPoint.y, 2, 0, Math.PI * 2);
  }
  ctx.fill();
}

// Recording Controls
function setupRecordingControls() {
  elements.startRecordBtn.addEventListener("click", startRecording);
  elements.stopRecordBtn.addEventListener("click", stopRecording);
  elements.quickRecordBtn.addEventListener("click", quickToggleRecording);
}

async function startRecording() {
  const sessionId = elements.sessionId.value || null;
  const freq = parseInt(elements.recordFreq.value) || 5;
  const desc = elements.sessionDesc.value || "";

  try {
    const response = await fetch("/api/recording/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sessionId, freq, description: desc }),
    });

    const data = await response.json();

    if (data.success) {
      state.isRecording = true;
      state.recordingSessionId = data.sessionId;
      state.recordingFrameCount = 0;
      updateRecordingUI(true);
    } else {
      alert("启动记录失败: " + data.error);
    }
  } catch (e) {
    console.error("Failed to start recording:", e);
    alert("启动记录失败");
  }
}

async function stopRecording() {
  try {
    const response = await fetch("/api/recording/stop", {
      method: "POST",
    });

    const data = await response.json();

    if (data.success) {
      state.isRecording = false;
      updateRecordingUI(false);
      loadSessions();
    }
  } catch (e) {
    console.error("Failed to stop recording:", e);
  }
}

function quickToggleRecording() {
  if (state.isRecording) {
    stopRecording();
  } else {
    startRecording();
  }
}

function updateRecordingUI(isRecording) {
  elements.startRecordBtn.disabled = isRecording;
  elements.stopRecordBtn.disabled = !isRecording;
  elements.quickRecordBtn.textContent = isRecording
    ? "⏹ 停止记录"
    : "▶ 开始记录";
  elements.quickRecordBtn.classList.toggle("btn-danger", isRecording);
  elements.quickRecordBtn.classList.toggle("btn-primary", !isRecording);

  elements.recIndicator.classList.toggle("active", isRecording);
  elements.recStatus.textContent = isRecording
    ? `记录中: ${state.recordingSessionId}`
    : "未记录";
}

// Sessions
async function loadSessions() {
  try {
    const response = await fetch("/api/sessions");
    const sessions = await response.json();
    renderSessionsTable(sessions);
    updateReplaySessionSelect(sessions);
  } catch (e) {
    console.error("Failed to load sessions:", e);
  }
}

function renderSessionsTable(sessions) {
  if (!sessions || sessions.length === 0) {
    elements.sessionsTbody.innerHTML =
      '<tr><td colspan="6" style="text-align:center;color:var(--text-muted)">暂无记录</td></tr>';
    return;
  }

  elements.sessionsTbody.innerHTML = sessions
    .map(
      (s) => `
        <tr>
            <td><code>${s.session_id}</code></td>
            <td>${formatDateTime(s.start_time)}</td>
            <td>${formatDuration(s.duration)}</td>
            <td>${s.frame_count}</td>
            <td>${s.description || "-"}</td>
            <td>
                <button class="btn btn-secondary" onclick="replaySession('${s.session_id}')">▶ 回放</button>
                <button class="btn btn-secondary" onclick="exportSession('${s.session_id}')">导出</button>
                <button class="btn btn-danger" onclick="deleteSession('${s.session_id}')">删除</button>
            </td>
        </tr>
    `,
    )
    .join("");
}

function updateReplaySessionSelect(sessions) {
  const select = elements.replaySessionSelect;
  select.innerHTML =
    '<option value="">-- 选择会话 --</option>' +
    sessions
      .map(
        (s) =>
          `<option value="${s.session_id}">${s.session_id} (${s.frame_count} 帧)</option>`,
      )
      .join("");
}

// Replay Controls
function setupReplayControls() {
  elements.replaySessionSelect.addEventListener("change", (e) => {
    loadReplayData(e.target.value);
  });

  elements.replayPlay.addEventListener("click", toggleReplayPlay);
  elements.replayPrev.addEventListener("click", () => stepReplay(-1));
  elements.replayNext.addEventListener("click", () => stepReplay(1));
  elements.replayLoop.addEventListener("click", toggleReplayLoop);

  elements.replaySpeed.addEventListener("input", (e) => {
    state.replaySpeed = parseFloat(e.target.value);
    elements.speedValue.textContent = state.replaySpeed.toFixed(1) + "x";
  });

  elements.replayProgress.addEventListener("input", (e) => {
    const frameIdx = Math.floor(
      (e.target.value / 100) * (state.replayFrames.length - 1),
    );
    state.replayCurrentFrame = frameIdx;
    renderReplayFrame();
  });
}

async function loadReplayData(sessionId) {
  if (!sessionId) {
    state.replayFrames = [];
    state.replayCurrentFrame = 0;
    renderReplayFrame();
    return;
  }

  try {
    const response = await fetch(`/api/sessions/${sessionId}/data`);
    const data = await response.json();

    state.replayFrames = data.positions || [];
    state.replayCurrentFrame = 0;
    state.replaySessionId = sessionId;

    // Update total time
    if (state.replayFrames.length > 0) {
      const duration =
        state.replayFrames[state.replayFrames.length - 1].tick -
        state.replayFrames[0].tick;
      elements.totalTime.textContent = formatTime(duration);
    }

    renderReplayFrame();
  } catch (e) {
    console.error("Failed to load replay data:", e);
  }
}

function toggleReplayPlay() {
  if (state.replayIsPlaying) {
    state.replayIsPlaying = false;
    elements.replayPlay.textContent = "▶";
  } else {
    state.replayIsPlaying = true;
    elements.replayPlay.textContent = "⏸";
    playReplayLoop();
  }
}

function playReplayLoop() {
  if (!state.replayIsPlaying || state.replayFrames.length === 0) return;

  stepReplay(1);

  if (state.replayCurrentFrame >= state.replayFrames.length - 1) {
    if (state.replayLoop) {
      state.replayCurrentFrame = 0;
    } else {
      state.replayIsPlaying = false;
      elements.replayPlay.textContent = "▶";
      return;
    }
  }

  // Calculate delay based on frame timestamps and speed
  const currentFrame = state.replayFrames[state.replayCurrentFrame];
  const nextFrame = state.replayFrames[state.replayCurrentFrame + 1];
  let delay = 50; // Default 50ms

  if (nextFrame && currentFrame) {
    delay = ((nextFrame.tick - currentFrame.tick) * 1000) / state.replaySpeed;
    delay = Math.max(10, Math.min(delay, 500)); // Clamp between 10-500ms
  }

  setTimeout(playReplayLoop, delay);
}

function stepReplay(delta) {
  const newFrame = state.replayCurrentFrame + delta;

  if (newFrame < 0) {
    state.replayCurrentFrame = 0;
  } else if (newFrame >= state.replayFrames.length) {
    if (state.replayLoop) {
      state.replayCurrentFrame = 0;
    } else {
      state.replayCurrentFrame = state.replayFrames.length - 1;
    }
  } else {
    state.replayCurrentFrame = newFrame;
  }

  renderReplayFrame();
}

function toggleReplayLoop() {
  state.replayLoop = !state.replayLoop;
  elements.replayLoop.classList.toggle("btn-primary", state.replayLoop);
}

function renderReplayFrame() {
  const frame = state.replayFrames[state.replayCurrentFrame];

  if (!frame) {
    clearFieldCanvas();
    elements.frameNum.textContent = "--";
    elements.frameTs.textContent = "--";
    elements.framePos.textContent = "--";
    elements.frameBall.textContent = "--";
    elements.currentTime.textContent = "00:00";
    elements.replayProgress.value = 0;
    return;
  }

  // Update frame info
  elements.frameNum.textContent = `${state.replayCurrentFrame + 1} / ${state.replayFrames.length}`;
  elements.frameTs.textContent = new Date(
    frame.tick * 1000,
  ).toLocaleTimeString();
  elements.framePos.textContent = `(${frame.x_cm.toFixed(1)}, ${frame.y_cm.toFixed(1)}) @ ${frame.yaw_deg.toFixed(1)}°`;
  elements.frameBall.textContent = `(${frame.ball_x_cm.toFixed(1)}, ${frame.ball_y_cm.toFixed(1)})`;

  // Update progress
  const progress =
    (state.replayCurrentFrame / Math.max(1, state.replayFrames.length - 1)) *
    100;
  elements.replayProgress.value = progress;

  // Update time display
  const elapsed = frame.tick - state.replayFrames[0].tick;
  elements.currentTime.textContent = formatTime(elapsed);

  // Render field visualization
  drawField(frame);
}

// Field Canvas
function initFieldCanvas() {
  const canvas = elements.fieldCanvas;
  const ctx = canvas.getContext("2d");

  // Set canvas size
  canvas.width = 520;
  canvas.height = 400;

  clearFieldCanvas();
}

function clearFieldCanvas() {
  const canvas = elements.fieldCanvas;
  const ctx = canvas.getContext("2d");

  // Green field
  ctx.fillStyle = "#1a472a";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Field lines
  ctx.strokeStyle = "rgba(255, 255, 255, 0.3)";
  ctx.lineWidth = 2;

  // Center circle
  ctx.beginPath();
  ctx.arc(canvas.width / 2, canvas.height / 2, 50, 0, Math.PI * 2);
  ctx.stroke();

  // Center line
  ctx.beginPath();
  ctx.moveTo(canvas.width / 2, 0);
  ctx.lineTo(canvas.width / 2, canvas.height);
  ctx.stroke();

  // Border
  ctx.strokeRect(10, 10, canvas.width - 20, canvas.height - 20);
}

function drawField(frame) {
  const canvas = elements.fieldCanvas;
  const ctx = canvas.getContext("2d");

  // Clear
  clearFieldCanvas();

  // Scale factors (field is 260x200 cm, canvas is 520x400)
  const scaleX = (canvas.width - 40) / state.fieldWidth;
  const scaleY = (canvas.height - 40) / state.fieldHeight;

  const toCanvasX = (x) => canvas.width / 2 + x * scaleX;
  const toCanvasY = (y) => canvas.height / 2 - y * scaleY; // Y is inverted

  // Draw ball
  if (frame.ball_x_cm !== 0 || frame.ball_y_cm !== 0) {
    const ballX = toCanvasX(frame.x_cm + frame.ball_x_cm);
    const ballY = toCanvasY(frame.y_cm + frame.ball_y_cm);

    ctx.fillStyle = "#ff6b6b";
    ctx.beginPath();
    ctx.arc(ballX, ballY, 8, 0, Math.PI * 2);
    ctx.fill();
  }

  // Draw robot
  const robotX = toCanvasX(frame.x_cm);
  const robotY = toCanvasY(frame.y_cm);

  // Robot body
  ctx.fillStyle = frame.has_ball ? "#22c55e" : "#3b82f6";
  ctx.beginPath();
  ctx.arc(robotX, robotY, 15, 0, Math.PI * 2);
  ctx.fill();

  // Direction indicator
  const yawRad = (frame.yaw_deg * Math.PI) / 180;
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(robotX + Math.cos(yawRad) * 20, robotY - Math.sin(yawRad) * 20);
  ctx.stroke();

  // Robot ID
  ctx.fillStyle = "#ffffff";
  ctx.font = "bold 12px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(frame.robot_id, robotX, robotY);
}

// Polling
function startPolling() {
  setInterval(updateOverview, CONFIG.updateInterval);
}

// 更新系统在线状态指示器
function updateOnlineStatus(isOnline) {
  state.isOnline = isOnline;
  if (isOnline) {
    state.lastOnlineTime = Date.now();
  }

  // 使用 id 直接获取元素
  const dot = document.getElementById("status-dot");
  const text = document.getElementById("status-text");

  console.log(
    "[Status] Updating online status:",
    isOnline,
    "dot:",
    dot,
    "text:",
    text,
  );

  if (dot) {
    dot.classList.remove("online", "offline");
    dot.classList.add(isOnline ? "online" : "offline");
    console.log("[Status] Dot classes:", dot.className);
  } else {
    console.warn("[Status] status-dot element not found");
  }

  if (text) {
    text.textContent = isOnline ? "系统在线" : "系统离线";
    console.log("[Status] Text content:", text.textContent);
  } else {
    console.warn("[Status] status-text element not found");
  }
}

async function updateOverview() {
  // The vision page owns the live state poll while its camera/radar streams run.
  // Avoid a second /api/state request competing with those camera requests.
  if (state.currentPage === "vision") return;
  if (overviewRequestInFlight) return;
  overviewRequestInFlight = true;

  try {
    // Fetch vision state
    const visionResponse = await fetch("/api/state");
    const visionData = await visionResponse.json();

    // 更新系统在线状态
    updateOnlineStatus(true);

    const poseValid = visionData.robot_pose_valid !== false;
    const headingAvailable = visionData.robot_heading_available === true;
    let poseX = poseValid ? toFiniteNumber(visionData.robot_x_cm) : NaN;
    let poseY = poseValid ? toFiniteNumber(visionData.robot_y_cm) : NaN;
    const poseYaw = poseValid
      ? toFiniteNumber(visionData.robot_yaw_deg)
      : headingAvailable
        ? toFiniteNumber(visionData.robot_heading_candidate_deg)
        : NaN;
    if (
      poseValid &&
      (!Number.isFinite(poseX) || !Number.isFinite(poseY)) &&
      visionData.chassis_list_cm &&
      visionData.chassis_list_cm.length > 0
    ) {
      // Fallback: chassis_list_cm is a list of detections, use first one
      const firstChassis = visionData.chassis_list_cm[0];
      if (firstChassis && firstChassis.length >= 2) {
        poseX = toFiniteNumber(firstChassis[0]);
        poseY = toFiniteNumber(firstChassis[1]);
      }
    }

    if (elements.robotPose) {
      elements.robotPose.textContent = formatPose(poseX, poseY, poseYaw);
      if (headingAvailable && !visionData.robot_heading_axis_mapping_verified) {
        elements.robotPose.title = "候选航向：IMU 轴映射与机器人方向尚未确认";
      } else {
        elements.robotPose.title = "";
      }
    }

    if (elements.lidarPointCount) {
      const lidarStatus = visionData.lidar_status || {};
      const pointCount = toFiniteNumber(
        lidarStatus.scan_point_count ?? lidarStatus.raw_point_count ?? 0,
      );
      elements.lidarPointCount.textContent = Number.isFinite(pointCount)
        ? `${pointCount} 点/圈`
        : "-- 点/圈";
      elements.lidarPointCount.style.color =
        lidarStatus.state === "ok" ? "#22c55e" : "var(--text-muted)";
      elements.lidarPointCount.title = `packet=${lidarStatus.packet_point_count ?? "--"}, near=${lidarStatus.point_count ?? "--"}, state=${lidarStatus.state || "--"}`;
    }

    // Update has_ball status
    if (visionData.has_ball !== undefined) {
      elements.hasBall.textContent = visionData.has_ball ? "是" : "否";
      elements.hasBall.style.color = visionData.has_ball
        ? "#22c55e"
        : "var(--text-muted)";
    }

    updateBallOverview(visionData.ball_pos_cm, visionData.ball_pos_raw_cm);

    // Update uptime
    if (visionData.uptime_s) {
      elements.uptime.textContent = formatDuration(visionData.uptime_s);
    }

    // Update performance metrics - calculate from frame counters
    if (visionData.perf) {
      const now = Date.now();
      const dt = (now - perfState.lastTs) / 1000; // seconds

      // Only update every 1 second to reduce fluctuation
      if (dt >= 1.0 && perfState.lastTs > 0) {
        const preprocessTf = visionData.perf.preprocess_tf || 0;
        const inferenceTf = visionData.perf.inference_tf || 0;
        const postprocessTf = visionData.perf.postprocess_tf || 0;
        const cameraTf = visionData.perf.camera_tf || 0;

        // Calculate FPS from frame counter delta
        const preprocessFps = (preprocessTf - perfState.lastPreprocess) / dt;
        const inferenceFps = (inferenceTf - perfState.lastInference) / dt;
        const postprocessFps = (postprocessTf - perfState.lastPostprocess) / dt;
        const cameraFps = (cameraTf - perfState.lastCamera) / dt;

        // Convert FPS to latency (ms), with smoothing
        const alpha = 0.4; // Smoothing factor
        if (preprocessFps > 0.1) {
          const ms = 1000 / preprocessFps;
          perfState.preprocessPeriodMs =
            alpha * ms + (1 - alpha) * perfState.preprocessPeriodMs;
        }
        if (inferenceFps > 0.1) {
          const ms = 1000 / inferenceFps;
          perfState.inferencePeriodMs =
            alpha * ms + (1 - alpha) * perfState.inferencePeriodMs;
        }
        if (postprocessFps > 0.1) {
          const ms = 1000 / postprocessFps;
          perfState.postprocessPeriodMs =
            alpha * ms + (1 - alpha) * perfState.postprocessPeriodMs;
        }
        if (typeof visionData.perf.preprocess_ms === "number") {
          perfState.preprocessMs =
            perfState.preprocessMs > 0
              ? alpha * visionData.perf.preprocess_ms +
                (1 - alpha) * perfState.preprocessMs
              : visionData.perf.preprocess_ms;
        } else if (preprocessFps > 0.1) {
          perfState.preprocessMs = perfState.preprocessPeriodMs;
        }
        if (typeof visionData.perf.inference_ms === "number") {
          perfState.inferenceMs =
            perfState.inferenceMs > 0
              ? alpha * visionData.perf.inference_ms +
                (1 - alpha) * perfState.inferenceMs
              : visionData.perf.inference_ms;
        } else if (inferenceFps > 0.1) {
          perfState.inferenceMs = perfState.inferencePeriodMs;
        }
        if (typeof visionData.perf.postprocess_ms === "number") {
          perfState.postprocessMs =
            perfState.postprocessMs > 0
              ? alpha * visionData.perf.postprocess_ms +
                (1 - alpha) * perfState.postprocessMs
              : visionData.perf.postprocess_ms;
        } else if (postprocessFps > 0.1) {
          perfState.postprocessMs = perfState.postprocessPeriodMs;
        }
        if (cameraFps > 0) {
          perfState.cameraFps =
            alpha * cameraFps + (1 - alpha) * perfState.cameraFps;
        }

        // Update last values
        perfState.lastPreprocess = preprocessTf;
        perfState.lastInference = inferenceTf;
        perfState.lastPostprocess = postprocessTf;
        perfState.lastCamera = cameraTf;
        perfState.lastTs = now;
      } else if (perfState.lastTs === 0) {
        // Initialize
        perfState.lastTs = now;
        perfState.lastPreprocess = visionData.perf.preprocess_tf || 0;
        perfState.lastInference = visionData.perf.inference_tf || 0;
        perfState.lastPostprocess = visionData.perf.postprocess_tf || 0;
        perfState.lastCamera = visionData.perf.camera_tf || 0;
      }

      // Update display
      const perfPreprocessEl = document.getElementById("perf-preprocess");
      const perfInferenceEl = document.getElementById("perf-inference");
      const perfPostprocessEl = document.getElementById("perf-postprocess");
      const perfCameraEl = document.getElementById("perf-camera");
      const perfBlobEl = document.getElementById("perf-blob");
      const perfOutputHzEl = document.getElementById("perf-output-hz");
      const perfTrackerHzEl = document.getElementById("perf-tracker-hz");
      const perfConfidenceEl = document.getElementById("perf-confidence");

      if (perfPreprocessEl && perfState.preprocessMs > 0)
        perfPreprocessEl.textContent =
          Math.round(perfState.preprocessMs) + " ms";
      if (perfInferenceEl && perfState.inferenceMs > 0)
        perfInferenceEl.textContent = Math.round(perfState.inferenceMs) + " ms";
      if (perfPostprocessEl && perfState.postprocessMs > 0)
        perfPostprocessEl.textContent =
          Math.round(perfState.postprocessMs) + " ms";
      if (perfPreprocessEl && perfState.preprocessPeriodMs > 0)
        perfPreprocessEl.title = `线程周期约 ${Math.round(perfState.preprocessPeriodMs)} ms`;
      if (perfInferenceEl && perfState.inferencePeriodMs > 0) {
        const batchMs = visionData.perf.inference_batch_ms;
        const batchSize = visionData.perf.inference_batch_size || 0;
        const batchText =
          typeof batchMs === "number"
            ? `batch总耗时 ${Math.round(batchMs)} ms / ${batchSize || "?"} 路`
            : "batch总耗时 --";
        perfInferenceEl.title = `${batchText}；线程周期约 ${Math.round(perfState.inferencePeriodMs)} ms`;
      }
      if (perfPostprocessEl && perfState.postprocessPeriodMs > 0)
        perfPostprocessEl.title = `线程周期约 ${Math.round(perfState.postprocessPeriodMs)} ms`;
      if (perfCameraEl && perfState.cameraFps > 0)
        perfCameraEl.textContent = Math.round(perfState.cameraFps) + " FPS";
      if (perfBlobEl && typeof visionData.perf.blob_ms === "number")
        perfBlobEl.textContent = visionData.perf.blob_ms.toFixed(1) + " ms";
      if (perfOutputHzEl && typeof visionData.perf.output_hz === "number")
        perfOutputHzEl.textContent =
          visionData.perf.output_hz.toFixed(1) + " Hz";
      if (perfTrackerHzEl && typeof visionData.perf.tracker_hz === "number")
        perfTrackerHzEl.textContent =
          visionData.perf.tracker_hz.toFixed(1) + " Hz";
      if (
        perfConfidenceEl &&
        typeof visionData.perf.vision_confidence === "number"
      )
        perfConfidenceEl.textContent =
          visionData.perf.vision_confidence.toFixed(2);
      if (perfInferenceEl && visionData.perf.yolo_backend) {
        const yoloHz =
          typeof visionData.perf.yolo_hz === "number"
            ? visionData.perf.yolo_hz.toFixed(1)
            : "--";
        const yoloError = visionData.perf.yolo_error
          ? `；YOLO错误: ${visionData.perf.yolo_error}`
          : "";
        perfInferenceEl.title = `${perfInferenceEl.title || ""}；YOLO ${visionData.perf.yolo_backend} ${yoloHz} Hz${yoloError}`;
      }
    }

    // Fetch recording state
    try {
      const recResponse = await fetch("/api/recording/status");
      const recData = await recResponse.json();

      state.isRecording = recData.is_recording;
      state.recordingSessionId = recData.session_id;
      state.recordingFrameCount = recData.frame_count || 0;

      updateRecordingUI(state.isRecording);
      elements.recFrames.textContent = state.recordingFrameCount + " 帧";
      elements.recStatus.textContent = state.isRecording
        ? `记录中: ${state.recordingSessionId}`
        : "未记录";
    } catch (e) {
      // Recording API not available
    }
  } catch (e) {
    console.error("Polling error:", e);
    // 更新系统在线状态为离线
    updateOnlineStatus(false);
  } finally {
    overviewRequestInFlight = false;
  }
}

// Helper functions
function formatDateTime(timestamp) {
  if (!timestamp) return "--";
  return new Date(timestamp * 1000).toLocaleString("zh-CN");
}

function formatDuration(seconds) {
  if (!seconds) return "0:00";
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${mins}:${secs.toString().padStart(2, "0")}`;
}

function formatTime(seconds) {
  if (!seconds) return "00:00";
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
}

function toFiniteNumber(value) {
  const num = Number(value);
  return Number.isFinite(num) ? num : NaN;
}

function getBallObservation(ballPos) {
  if (!Array.isArray(ballPos) || ballPos.length < 2) {
    return { state: "lost" };
  }
  const x = toFiniteNumber(ballPos[0]);
  const y = toFiniteNumber(ballPos[1]);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return { state: "lost" };
  }
  if (x === 0xfff && y === 0xfff) {
    return { state: "lost" };
  }
  if (x === 0xddd && y === 0xddd) {
    return { state: "held" };
  }
  return { state: "free", x, y };
}

function getFreeBallCoords(ballPos) {
  const observation = getBallObservation(ballPos);
  return observation.state === "free" ? observation : null;
}

const RADAR_GOAL_CAMERA_FOV_DEG = 95;
const RADAR_GOAL_WIDTH_CM = 100;
const RADAR_CAMERA_BASE_BEARING_DEG = {
  0: 180, // back
  1: 90, // left
  2: 0, // front
  3: -90, // right
};
const RADAR_GOAL_STYLES = {
  1: {
    label: "BLUE GOAL",
    shortLabel: "BLUE",
    rgb: "59,130,246",
    stroke: "#3b82f6",
  },
  2: {
    label: "YELLOW GOAL",
    shortLabel: "YELLOW",
    rgb: "250,204,21",
    stroke: "#facc15",
  },
};

function normalizeRadarPoint(point) {
  if (!point) return null;
  let x;
  let y;
  if (Array.isArray(point)) {
    if (point.length < 2) return null;
    x = toFiniteNumber(point[0]);
    y = toFiniteNumber(point[1]);
  } else {
    x = toFiniteNumber(point.x);
    y = toFiniteNumber(point.y);
  }
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  if (x === 0 && y === 0) return null;
  return { x, y };
}

function expandRadarRange(current, point, margin = 1.35) {
  const coords = normalizeRadarPoint(point);
  if (!coords) return current;
  return Math.max(current, Math.hypot(coords.x, coords.y) * margin);
}

function robotPointToRadarCanvas(x, y, cx, cy, scale) {
  return {
    x: cx - y * scale,
    y: cy - x * scale,
  };
}

function bearingDegToRadarCanvas(cx, cy, radiusPx, bearingDeg) {
  const rad = (bearingDeg * Math.PI) / 180;
  return {
    x: cx - Math.sin(rad) * radiusPx,
    y: cy - Math.cos(rad) * radiusPx,
  };
}

function clampCanvasTextPoint(ctx, x, y, width = 86) {
  return {
    x: Math.max(6, Math.min(ctx.canvas.width - width, x)),
    y: Math.max(14, Math.min(ctx.canvas.height - 8, y)),
  };
}

function getRadarGoalStyle(cls) {
  return (
    RADAR_GOAL_STYLES[cls] || {
      label: "GOAL",
      shortLabel: "GOAL",
      rgb: "167,139,250",
      stroke: "#a78bfa",
    }
  );
}

function getGoalBboxBearingSector(bbox) {
  if (!Array.isArray(bbox) || bbox.length < 7) return null;
  const cam = toFiniteNumber(bbox[0]);
  const cls = Math.round(toFiniteNumber(bbox[1]));
  const x1 = toFiniteNumber(bbox[2]);
  const x2 = toFiniteNumber(bbox[4]);
  const conf = toFiniteNumber(bbox[6]);
  if (!Number.isFinite(cam) || !Number.isFinite(x1) || !Number.isFinite(x2))
    return null;
  if (!RADAR_GOAL_STYLES[cls]) return null;

  const baseBearing = RADAR_CAMERA_BASE_BEARING_DEG[Math.round(cam)];
  if (!Number.isFinite(baseBearing)) return null;

  const leftX = Math.max(0, Math.min(640, Math.min(x1, x2)));
  const rightX = Math.max(0, Math.min(640, Math.max(x1, x2)));
  const bearingForX = (x) =>
    baseBearing + ((320 - x) / 640) * RADAR_GOAL_CAMERA_FOV_DEG;
  const bearingA = bearingForX(leftX);
  const bearingB = bearingForX(rightX);
  const startDeg = Math.min(bearingA, bearingB);
  const endDeg = Math.max(bearingA, bearingB);

  return {
    cls,
    conf: Number.isFinite(conf) ? conf : 0,
    startDeg,
    endDeg,
    centerDeg: bearingForX((leftX + rightX) / 2),
    spanDeg: Math.max(4, Math.abs(endDeg - startDeg)),
    style: getRadarGoalStyle(cls),
  };
}

function collectGoalBearingSectors(data) {
  const goalBboxes = Array.isArray(data.goal_bboxes_640)
    ? data.goal_bboxes_640
    : visionState.goal_bboxes_640;
  if (!Array.isArray(goalBboxes)) return [];

  const sectors = [];
  for (const bbox of goalBboxes) {
    const sector = getGoalBboxBearingSector(bbox);
    if (sector) sectors.push(sector);
  }
  return sectors;
}

function drawGoalBearingSector(ctx, sector, cx, cy, radiusPx) {
  const steps = Math.max(2, Math.ceil(sector.spanDeg / 8));
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  for (let i = 0; i <= steps; i++) {
    const bearing =
      sector.startDeg + (sector.endDeg - sector.startDeg) * (i / steps);
    const p = bearingDegToRadarCanvas(cx, cy, radiusPx, bearing);
    ctx.lineTo(p.x, p.y);
  }
  ctx.closePath();
  ctx.fillStyle = `rgba(${sector.style.rgb},0.10)`;
  ctx.fill();
  ctx.strokeStyle = `rgba(${sector.style.rgb},0.38)`;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 5]);
  ctx.stroke();
  ctx.setLineDash([]);

  const labelPoint = bearingDegToRadarCanvas(
    cx,
    cy,
    radiusPx * 0.78,
    sector.centerDeg,
  );
  const label = `${sector.style.shortLabel} ${sector.conf.toFixed(2)}`;
  const textPoint = clampCanvasTextPoint(
    ctx,
    labelPoint.x + 4,
    labelPoint.y,
    78,
  );
  ctx.fillStyle = `rgba(${sector.style.rgb},0.82)`;
  ctx.font = "10px sans-serif";
  ctx.fillText(label, textPoint.x, textPoint.y);
  ctx.restore();
}

function estimateGoalBarWidthPx(distCm, scale, sector) {
  let widthCm = RADAR_GOAL_WIDTH_CM;
  if (sector && distCm > 1) {
    const spanRad =
      (Math.min(110, Math.max(8, sector.spanDeg)) * Math.PI) / 180;
    widthCm = 2 * distCm * Math.tan(spanRad / 2);
  }
  return Math.max(28, Math.min(90, widthCm * scale));
}

function drawGoalPositionBar(
  ctx,
  goalPos,
  cls,
  sector,
  cx,
  cy,
  scale,
  rangeCm,
) {
  const point = normalizeRadarPoint(goalPos);
  if (!point) return;

  const style = getRadarGoalStyle(cls);
  const distCm = Math.hypot(point.x, point.y);
  if (distCm <= 1) return;

  const clamped = distCm > rangeCm;
  const drawDistCm = clamped ? rangeCm * 0.94 : distCm;
  const drawX = (point.x / distCm) * drawDistCm;
  const drawY = (point.y / distCm) * drawDistCm;
  const center = robotPointToRadarCanvas(drawX, drawY, cx, cy, scale);

  const radialX = center.x - cx;
  const radialY = center.y - cy;
  const radialLen = Math.max(1, Math.hypot(radialX, radialY));
  const tangentX = -radialY / radialLen;
  const tangentY = radialX / radialLen;
  const halfWidth = estimateGoalBarWidthPx(distCm, scale, sector) / 2;

  ctx.save();
  ctx.strokeStyle = style.stroke;
  ctx.lineWidth = clamped ? 4 : 6;
  ctx.lineCap = "round";
  if (clamped) ctx.setLineDash([7, 5]);
  ctx.beginPath();
  ctx.moveTo(center.x - tangentX * halfWidth, center.y - tangentY * halfWidth);
  ctx.lineTo(center.x + tangentX * halfWidth, center.y + tangentY * halfWidth);
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.fillStyle = `rgba(${style.rgb},0.95)`;
  ctx.beginPath();
  ctx.arc(center.x, center.y, 3.5, 0, Math.PI * 2);
  ctx.fill();

  const distanceText = clamped
    ? `>${rangeCm.toFixed(0)}cm`
    : `${distCm.toFixed(0)}cm`;
  const labelPoint = clampCanvasTextPoint(ctx, center.x + 8, center.y - 8, 118);
  ctx.font = "10px sans-serif";
  ctx.fillText(`${style.label} ${distanceText}`, labelPoint.x, labelPoint.y);
  ctx.restore();
}

function drawGoalObservations(ctx, data, cx, cy, scale, rangeCm) {
  const sectors = collectGoalBearingSectors(data);
  const bestSectorByClass = {};
  const radiusPx = rangeCm * scale * 0.98;

  for (const sector of sectors) {
    drawGoalBearingSector(ctx, sector, cx, cy, radiusPx);
    if (
      !bestSectorByClass[sector.cls] ||
      sector.conf > bestSectorByClass[sector.cls].conf
    ) {
      bestSectorByClass[sector.cls] = sector;
    }
  }

  drawGoalPositionBar(
    ctx,
    data.blue_goal_pos_cm,
    1,
    bestSectorByClass[1],
    cx,
    cy,
    scale,
    rangeCm,
  );
  drawGoalPositionBar(
    ctx,
    data.yellow_goal_pos_cm,
    2,
    bestSectorByClass[2],
    cx,
    cy,
    scale,
    rangeCm,
  );
}

function formatBallPositionText(ballPos) {
  const observation = getBallObservation(ballPos);
  if (observation.state === "lost") return "未发现";
  if (observation.state === "held") return "持球中";
  return `(${observation.x.toFixed(1)}, ${observation.y.toFixed(1)}) cm`;
}

function bestBboxByConfidence(bboxes, confIndex = 5) {
  if (!Array.isArray(bboxes)) return null;
  let best = null;
  for (const bbox of bboxes) {
    if (!Array.isArray(bbox) || bbox.length <= confIndex) continue;
    const conf = toFiniteNumber(bbox[confIndex]);
    if (!Number.isFinite(conf) || conf <= 0) continue;
    if (!best || conf > best.conf) {
      best = { bbox, conf };
    }
  }
  return best ? best.bbox : null;
}

function formatBboxText(bbox, hasClass = false) {
  if (!Array.isArray(bbox)) return "--";
  const cam = Math.round(toFiniteNumber(bbox[0]));
  const clsOffset = hasClass ? 1 : 0;
  const cls = hasClass ? Math.round(toFiniteNumber(bbox[1])) : null;
  const x1 = toFiniteNumber(bbox[1 + clsOffset]);
  const y1 = toFiniteNumber(bbox[2 + clsOffset]);
  const x2 = toFiniteNumber(bbox[3 + clsOffset]);
  const y2 = toFiniteNumber(bbox[4 + clsOffset]);
  const conf = toFiniteNumber(bbox[5 + clsOffset]);
  if (![cam, x1, y1, x2, y2, conf].every(Number.isFinite)) return "--";
  const clsText = hasClass ? ` cls=${cls}` : "";
  return `cam${cam}${clsText} [${Math.round(x1)},${Math.round(y1)}]-[${Math.round(x2)},${Math.round(y2)}] ${conf.toFixed(2)}`;
}

function formatBallPixelPoint(point) {
  if (!Array.isArray(point) || point.length < 3) return "--";
  const cam = toFiniteNumber(point[0]);
  const cx = toFiniteNumber(point[1]);
  const bottomY = toFiniteNumber(point[2]);
  if (![cam, cx, bottomY].every(Number.isFinite)) return "--";
  return `cam${Math.round(cam)} cx=${Math.round(cx)} bottom=${Math.round(bottomY)}`;
}

function formatGoalPositionText(goalPos) {
  const point = normalizeRadarPoint(goalPos);
  if (!point) return "--";
  let confText = "";
  if (Array.isArray(goalPos) && goalPos.length >= 3) {
    const conf = toFiniteNumber(goalPos[2]);
    if (Number.isFinite(conf)) confText = ` ${conf.toFixed(2)}`;
  }
  return `(${point.x.toFixed(1)}, ${point.y.toFixed(1)}) cm${confText}`;
}

function updateVisionDetails(data) {
  const ballRawEl = document.getElementById("ball-pos-raw");
  if (ballRawEl) {
    ballRawEl.textContent = formatBallPositionText(data.ball_pos_raw_cm);
  }

  const ballPixelEl = document.getElementById("ball-pixel-point");
  if (ballPixelEl) {
    ballPixelEl.textContent = formatBallPixelPoint(data.ball_pixel_point);
  }

  const ballBboxEl = document.getElementById("ball-bbox");
  if (ballBboxEl) {
    ballBboxEl.textContent = formatBboxText(
      bestBboxByConfidence(data.ball_bboxes_640, 5),
    );
  }

  const blueGoalEl = document.getElementById("blue-goal");
  if (blueGoalEl) {
    blueGoalEl.textContent = formatGoalPositionText(data.blue_goal_pos_cm);
  }

  const yellowGoalEl = document.getElementById("yellow-goal");
  if (yellowGoalEl) {
    yellowGoalEl.textContent = formatGoalPositionText(data.yellow_goal_pos_cm);
  }
}

function formatCm(value) {
  const num = toFiniteNumber(value);
  return Number.isFinite(num) ? `${num.toFixed(1)} cm` : "-- cm";
}

function formatPose(x, y, yaw) {
  const hasX = Number.isFinite(x);
  const hasY = Number.isFinite(y);
  const hasYaw = Number.isFinite(yaw);
  if (!hasX && !hasY && !hasYaw) return "--";
  const xyText =
    hasX && hasY ? `(${x.toFixed(1)}, ${y.toFixed(1)}) cm` : "(--, --) cm";
  const yawText = hasYaw ? `${yaw.toFixed(1)}°` : "--°";
  return `${xyText} @ ${yawText}`;
}

function updateBallOverview(ballPos, rawBallPos = null) {
  if (!elements.ballX || !elements.ballY) return;
  const observation = getBallObservation(ballPos);
  const rawText = rawBallPos
    ? `Raw: ${formatBallPositionText(rawBallPos)}`
    : "";
  elements.ballX.title = rawText;
  elements.ballY.title = rawText;
  if (observation.state === "lost") {
    elements.ballX.textContent = "未发现";
    elements.ballY.textContent = "未发现";
    return;
  }
  if (observation.state === "held") {
    elements.ballX.textContent = "持球中";
    elements.ballY.textContent = "持球中";
    return;
  }

  elements.ballX.textContent = formatCm(observation.x);
  elements.ballY.textContent = formatCm(observation.y);
}

// ==================== 工具页面 ====================

// 工具状态
const toolsState = {
  calibRunning: false,
  calibSamples: 0,
  calibFound: false,
  calibHasResult: false,
  birdeyeRunning: false,
  calibFiles: [],
};

// 初始化工具页面
function setupToolsPage() {
  // 工具标签页切换
  document.querySelectorAll(".tools-tabs .tab-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const tabId = btn.dataset.toolTab;

      // 更新标签按钮状态
      document
        .querySelectorAll(".tools-tabs .tab-btn")
        .forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");

      // 更新标签内容
      document
        .querySelectorAll(".tool-tab-content")
        .forEach((content) => content.classList.remove("active"));
      document.getElementById(`tool-tab-${tabId}`).classList.add("active");

      // 标签页初始化
      if (tabId === "files") {
        loadCalibFiles();
      } else if (tabId === "birdeye") {
        loadBirdeyeCalibFiles();
      }
    });
  });

  // 标定工具按钮
  document
    .getElementById("calib-scan-cams")
    ?.addEventListener("click", scanCalibCameras);
  document
    .getElementById("calib-start-btn")
    ?.addEventListener("click", startCalibStream);
  document
    .getElementById("calib-stop-btn")
    ?.addEventListener("click", stopCalibStream);
  document
    .getElementById("calib-capture-btn")
    ?.addEventListener("click", captureCalibSample);
  document
    .getElementById("calib-clear-btn")
    ?.addEventListener("click", clearCalibSamples);
  document
    .getElementById("calib-calibrate-btn")
    ?.addEventListener("click", calibrateCamera);
  document
    .getElementById("calib-save-btn")
    ?.addEventListener("click", saveCalibration);

  // 鸟瞰图按钮
  document
    .getElementById("birdeye-start-btn")
    ?.addEventListener("click", startBirdeye);
  document
    .getElementById("birdeye-stop-btn")
    ?.addEventListener("click", stopBirdeye);

  // 标定文件刷新
  document
    .getElementById("refresh-calib-files-btn")
    ?.addEventListener("click", loadCalibFiles);

  // 初始化
  scanCalibCameras();
  loadCalibFiles();
}

// 扫描摄像头
async function scanCalibCameras() {
  try {
    const response = await fetch("/api/calib/cameras");
    const data = await response.json();

    const select = document.getElementById("calib-camera");
    const birdeyeSelect = document.getElementById("birdeye-camera");

    if (select && data.cameras) {
      select.innerHTML = data.cameras
        .map((cam) => `<option value="${cam}">摄像头 ${cam}</option>`)
        .join("");
    }

    if (birdeyeSelect && data.cameras) {
      birdeyeSelect.innerHTML = data.cameras
        .map((cam) => `<option value="${cam}">摄像头 ${cam}</option>`)
        .join("");
    }
  } catch (e) {
    console.error("Failed to scan cameras:", e);
  }
}

// 启动标定流
async function startCalibStream() {
  const camIndex = parseInt(
    document.getElementById("calib-camera")?.value || "0",
  );
  const resolution =
    document.getElementById("calib-resolution")?.value || "640x480";
  const [width, height] = resolution.split("x").map(Number);
  const boardW = parseInt(
    document.getElementById("calib-board-w")?.value || "9",
  );
  const boardH = parseInt(
    document.getElementById("calib-board-h")?.value || "9",
  );
  const squareMm = parseFloat(
    document.getElementById("calib-square-mm")?.value || "15",
  );
  const originXCm = parseFloat(
    document.getElementById("calib-origin-x-cm")?.value || "6.0",
  );
  const originYCm = parseFloat(
    document.getElementById("calib-origin-y-cm")?.value || "17.5",
  );

  try {
    const response = await fetch("/api/calib/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        cam_index: camIndex,
        width,
        height,
        board_w: boardW,
        board_h: boardH,
        square_mm: squareMm,
        origin_x_cm: originXCm,
        origin_y_cm: originYCm,
      }),
    });

    const data = await response.json();

    if (data.success) {
      toolsState.calibRunning = true;
      updateCalibUI();
      startCalibPreview();
    } else {
      alert("启动失败: " + data.error);
    }
  } catch (e) {
    console.error("Failed to start calibration:", e);
    alert("启动失败");
  }
}

// 停止标定流
async function stopCalibStream() {
  try {
    await fetch("/api/calib/stop", { method: "POST" });
    toolsState.calibRunning = false;
    updateCalibUI();
  } catch (e) {
    console.error("Failed to stop calibration:", e);
  }
}

// 标定预览循环
let calibPreviewInterval = null;

function escapeCalibHtml(value) {
  return String(value ?? "").replace(
    /[&<>"']/g,
    (ch) =>
      ({
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;",
        '"': "&quot;",
        "'": "&#39;",
      })[ch],
  );
}

function fmtCalibCm(value) {
  const num = Number(value);
  return Number.isFinite(num) ? num.toFixed(1) : "--";
}

function renderCalibResult(data) {
  const resultEl = document.getElementById("calib-result");
  if (!resultEl) return;

  const rmse = Number(data?.rmse);
  const diagnostic = data?.diagnostic || {};
  const extent = diagnostic.extent_cm || null;
  const lines = [];

  if (data?.message) {
    lines.push(`<span class="success">${escapeCalibHtml(data.message)}</span>`);
  } else if (Number.isFinite(rmse)) {
    lines.push(
      `<span class="success">标定成功! RMSE = ${rmse.toFixed(3)} px</span>`,
    );
  }

  if (extent) {
    lines.push(
      `<span class="detail">可见标定平面: ` +
        `X ${fmtCalibCm(extent.x_min)}..${fmtCalibCm(extent.x_max)} cm, ` +
        `Y ${fmtCalibCm(extent.y_min)}..${fmtCalibCm(extent.y_max)} cm</span>`,
    );
  }

  if (diagnostic.warning) {
    lines.push(
      `<span class="warning">${escapeCalibHtml(diagnostic.warning)}</span>`,
    );
  } else if (diagnostic.error) {
    lines.push(
      `<span class="warning">自检失败: ${escapeCalibHtml(diagnostic.error)}</span>`,
    );
  }

  if (lines.length) {
    resultEl.innerHTML = lines.join("<br>");
  }
}

function startCalibPreview() {
  if (calibPreviewInterval) {
    clearInterval(calibPreviewInterval);
  }

  calibPreviewInterval = setInterval(async () => {
    if (!toolsState.calibRunning) {
      clearInterval(calibPreviewInterval);
      return;
    }

    // 更新预览图像
    const img = document.getElementById("calib-preview");
    if (img) {
      img.src = `/api/calib/frame?t=${Date.now()}`;
    }

    // 更新状态
    try {
      const response = await fetch("/api/calib/status");
      const status = await response.json();

      toolsState.calibSamples = status.samples || 0;
      toolsState.calibFound = status.found || false;
      toolsState.calibHasResult = status.has_result || false;

      document.getElementById("calib-samples").textContent =
        toolsState.calibSamples;
      document.getElementById("calib-found").textContent = toolsState.calibFound
        ? "已检测"
        : "未检测";
      document.getElementById("calib-status-text").textContent =
        status.status || "空闲";

      // 更新按钮状态
      document.getElementById("calib-capture-btn").disabled =
        !toolsState.calibFound;
      document.getElementById("calib-calibrate-btn").disabled =
        toolsState.calibSamples < 3;
      document.getElementById("calib-save-btn").disabled =
        !toolsState.calibHasResult;

      if (Number.isFinite(Number(status.rmse))) {
        renderCalibResult(status);
      }
    } catch (e) {
      console.error("Failed to get calibration status:", e);
    }
  }, 100);
}

// 更新标定 UI
function updateCalibUI() {
  document.getElementById("calib-start-btn").disabled = toolsState.calibRunning;
  document.getElementById("calib-stop-btn").disabled = !toolsState.calibRunning;
  document.getElementById("calib-capture-btn").disabled =
    !toolsState.calibRunning || !toolsState.calibFound;
  document.getElementById("calib-clear-btn").disabled =
    toolsState.calibSamples === 0;
  document.getElementById("calib-calibrate-btn").disabled =
    toolsState.calibSamples < 3;
  document.getElementById("calib-save-btn").disabled =
    !toolsState.calibHasResult;

  if (!toolsState.calibRunning) {
    document.getElementById("calib-status-text").textContent = "已停止";
  }
}

// 采集标定样本
async function captureCalibSample() {
  try {
    const response = await fetch("/api/calib/capture", { method: "POST" });
    const data = await response.json();

    if (data.success) {
      // 短暂高亮提示
      const samplesEl = document.getElementById("calib-samples");
      samplesEl.style.color = "#22c55e";
      setTimeout(() => (samplesEl.style.color = ""), 500);
    } else {
      console.warn("Capture failed:", data.error);
    }
  } catch (e) {
    console.error("Failed to capture sample:", e);
  }
}

// 清除标定样本
async function clearCalibSamples() {
  try {
    await fetch("/api/calib/clear", { method: "POST" });
    toolsState.calibSamples = 0;
    toolsState.calibHasResult = false;
    updateCalibUI();
    document.getElementById("calib-result").innerHTML = "";
  } catch (e) {
    console.error("Failed to clear samples:", e);
  }
}

// 执行标定
async function calibrateCamera() {
  const resultEl = document.getElementById("calib-result");
  resultEl.innerHTML = "正在标定...";

  try {
    const response = await fetch("/api/calib/calibrate", { method: "POST" });
    const data = await response.json();

    if (data.success) {
      toolsState.calibHasResult = true;
      renderCalibResult(data);
      updateCalibUI();
    } else {
      resultEl.innerHTML = `<span class="error">${data.error}</span>`;
    }
  } catch (e) {
    resultEl.innerHTML = `<span class="error">标定失败: ${e.message}</span>`;
  }
}

// 保存标定结果
async function saveCalibration() {
  const camIndex = parseInt(
    document.getElementById("calib-save-index")?.value || "0",
  );

  try {
    const response = await fetch("/api/calib/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cam_index: camIndex }),
    });

    const data = await response.json();

    if (data.success) {
      alert(data.message);
      loadCalibFiles();
    } else {
      alert("保存失败: " + data.error);
    }
  } catch (e) {
    console.error("Failed to save calibration:", e);
    alert("保存失败");
  }
}

// 加载标定文件列表
async function loadCalibFiles() {
  try {
    const response = await fetch("/api/calibration-files");
    const data = await response.json();

    toolsState.calibFiles = data.files || [];

    const tbody = document.getElementById("calib-files-tbody");
    if (tbody) {
      if (toolsState.calibFiles.length === 0) {
        tbody.innerHTML =
          '<tr><td colspan="7" style="text-align:center;color:var(--text-muted)">暂无标定文件</td></tr>';
      } else {
        tbody.innerHTML = toolsState.calibFiles
          .map(
            (f) => `
                    <tr>
                        <td><code>${f.name}</code></td>
                        <td>${f.camera_index ?? "--"}</td>
                        <td>${f.board_size ? `${f.board_size[0]}x${f.board_size[1]}` : "--"}</td>
                        <td>${f.square_size_mm ? `${f.square_size_mm} mm` : "--"}</td>
                        <td>${f.origin_offset_cm ? `${f.origin_offset_cm[0]} / ${f.origin_offset_cm[1]} cm` : "--"}</td>
                        <td>${f.rmse ? `${f.rmse.toFixed(3)} px` : "--"}</td>
                        <td>${f.size || "--"}</td>
                    </tr>
                `,
          )
          .join("");
      }
    }
  } catch (e) {
    console.error("Failed to load calibration files:", e);
  }
}

// 加载鸟瞰图标定文件选项
async function loadBirdeyeCalibFiles() {
  try {
    const response = await fetch("/api/calibration-files");
    const data = await response.json();

    const select = document.getElementById("birdeye-calib-file");
    if (select && data.files) {
      select.innerHTML =
        '<option value="">-- 选择标定文件 --</option>' +
        data.files
          .map(
            (f) =>
              `<option value="${f.path}">${f.name} (CAM ${f.camera_index ?? "?"})</option>`,
          )
          .join("");
    }
  } catch (e) {
    console.error("Failed to load birdeye calibration files:", e);
  }
}

// 启动鸟瞰图预览
async function startBirdeye() {
  const camIndex = parseInt(
    document.getElementById("birdeye-camera")?.value || "0",
  );
  const resolution =
    document.getElementById("birdeye-resolution")?.value || "640x480";
  const [width, height] = resolution.split("x").map(Number);
  const calibFile = document.getElementById("birdeye-calib-file")?.value || "";
  const ppm = parseFloat(document.getElementById("birdeye-ppm")?.value || "4");
  const gridMm = parseFloat(
    document.getElementById("birdeye-grid")?.value || "10",
  );
  const scale = parseFloat(
    document.getElementById("birdeye-scale")?.value || "0.4",
  );
  const mjpg = document.getElementById("birdeye-mjpg")?.checked || false;

  if (!calibFile) {
    alert("请选择标定文件");
    return;
  }

  try {
    const response = await fetch("/api/birdeye/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        cam_index: camIndex,
        calib_file: calibFile,
        width,
        height,
        pixels_per_mm: ppm,
        grid_mm: gridMm,
        preview_scale: scale,
        use_mjpg: mjpg,
      }),
    });

    const data = await response.json();

    if (data.success) {
      toolsState.birdeyeRunning = true;
      updateBirdeyeUI();
      startBirdeyePreview();
    } else {
      alert("启动失败: " + data.error);
    }
  } catch (e) {
    console.error("Failed to start birdeye:", e);
    alert("启动失败");
  }
}

// 停止鸟瞰图预览
async function stopBirdeye() {
  try {
    await fetch("/api/birdeye/stop", { method: "POST" });
    toolsState.birdeyeRunning = false;
    updateBirdeyeUI();
  } catch (e) {
    console.error("Failed to stop birdeye:", e);
  }
}

// 鸟瞰图预览循环
let birdeyePreviewInterval = null;

function startBirdeyePreview() {
  if (birdeyePreviewInterval) {
    clearInterval(birdeyePreviewInterval);
  }

  birdeyePreviewInterval = setInterval(async () => {
    if (!toolsState.birdeyeRunning) {
      clearInterval(birdeyePreviewInterval);
      return;
    }

    try {
      const response = await fetch("/api/birdeye/frame");
      const data = await response.json();

      if (data.raw) {
        document.getElementById("birdeye-raw").src =
          "data:image/jpeg;base64," + data.raw;
      }
      if (data.bird) {
        document.getElementById("birdeye-bird").src =
          "data:image/jpeg;base64," + data.bird;
      }
    } catch (e) {
      console.error("Failed to get birdeye frame:", e);
    }
  }, 100);
}

// 更新鸟瞰图 UI
function updateBirdeyeUI() {
  document.getElementById("birdeye-start-btn").disabled =
    toolsState.birdeyeRunning;
  document.getElementById("birdeye-stop-btn").disabled =
    !toolsState.birdeyeRunning;
  document.getElementById("birdeye-status-text").textContent =
    toolsState.birdeyeRunning ? "状态: 运行中" : "状态: 空闲";
}

// 在 DOMContentLoaded 后初始化工具页面
document.addEventListener("DOMContentLoaded", function () {
  // 等待原始 init 完成后再初始化工具页面
  setTimeout(setupToolsPage, 100);
});

// Tools Page Functions (for external tool windows)
const TOOL_PORTS = {
  calibration: 7863,
  birdeye: 7864,
};

const TOOL_NAMES = {
  calibration: "摄像头标定",
  birdeye: "鸟瞰图预览",
};

// Check tool status
function checkToolStatus() {
  for (const [tool, port] of Object.entries(TOOL_PORTS)) {
    const statusEl = document.getElementById(`${tool}-status`);
    if (!statusEl) continue;

    fetch(`http://localhost:${port}/`, { mode: "no-cors", timeout: 2000 })
      .then(() => {
        statusEl.textContent = "在线";
        statusEl.classList.add("online");
        statusEl.classList.remove("offline");
      })
      .catch(() => {
        statusEl.textContent = "离线";
        statusEl.classList.add("offline");
        statusEl.classList.remove("online");
      });
  }
}

// Open tool in new window
window.openTool = function (tool) {
  const port = TOOL_PORTS[tool];
  if (port) {
    window.open(`http://localhost:${port}/`, "_blank");
  }
};

// Open tool embedded
window.openToolEmbed = function (tool) {
  const port = TOOL_PORTS[tool];
  const name = TOOL_NAMES[tool];
  if (!port) return;

  const container = document.getElementById("tool-embed-container");
  const iframe = document.getElementById("tool-iframe");
  const title = document.getElementById("embed-title");

  if (container && iframe && title) {
    title.textContent = name;
    iframe.src = `http://localhost:${port}/`;
    container.style.display = "block";
  }
};

// Close embedded tool
window.closeToolEmbed = function () {
  const container = document.getElementById("tool-embed-container");
  const iframe = document.getElementById("tool-iframe");

  if (container && iframe) {
    iframe.src = "";
    container.style.display = "none";
  }
};

// Refresh calibration files
window.refreshCalibFiles = function () {
  const listEl = document.getElementById("calib-files-list");
  if (!listEl) return;

  listEl.innerHTML = '<div class="loading">加载中...</div>';

  fetch("/api/calibration-files")
    .then((r) => r.json())
    .then((data) => {
      if (!data.files || data.files.length === 0) {
        listEl.innerHTML = '<div class="loading">暂无标定文件</div>';
        return;
      }

      listEl.innerHTML = data.files
        .map(
          (f) => `
                <div class="file-item">
                    <span class="file-name">${f.name}</span>
                    <span class="file-info">${f.camera_index !== undefined ? `摄像头 ${f.camera_index}` : ""} | ${f.size || "--"}</span>
                </div>
            `,
        )
        .join("");
    })
    .catch((e) => {
      listEl.innerHTML = '<div class="loading">加载失败</div>';
    });
};

// Global functions for inline handlers
window.replaySession = function (sessionId) {
  navigateTo("replay");
  elements.replaySessionSelect.value = sessionId;
  loadReplayData(sessionId);
};

window.exportSession = async function (sessionId) {
  try {
    const response = await fetch(`/api/sessions/${sessionId}/export`);
    const blob = await response.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `replay_${sessionId}.json`;
    a.click();
    URL.revokeObjectURL(url);
  } catch (e) {
    alert("导出失败: " + e.message);
  }
};

window.deleteSession = async function (sessionId) {
  if (!confirm(`确定要删除会话 ${sessionId} 吗？`)) return;

  try {
    const response = await fetch(`/api/sessions/${sessionId}`, {
      method: "DELETE",
    });
    const data = await response.json();

    if (data.success) {
      loadSessions();
    } else {
      alert("删除失败: " + data.error);
    }
  } catch (e) {
    alert("删除失败: " + e.message);
  }
};

// Settings Tabs
function setupSettingsTabs() {
  const tabBtns = document.querySelectorAll(".tab-btn");
  tabBtns.forEach((btn) => {
    btn.addEventListener("click", () => {
      const tabId = btn.dataset.tab;
      switchSettingsTab(tabId);
    });
  });
}

function switchSettingsTab(tabId) {
  // Update button states
  document.querySelectorAll(".tab-btn").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.tab === tabId);
  });

  // Update content visibility
  document.querySelectorAll(".tab-content").forEach((content) => {
    content.classList.toggle("active", content.id === `tab-${tabId}`);
  });
}

// Configuration Panel
function setupConfigPanel() {
  const saveBtn = document.getElementById("save-config-btn");
  if (saveBtn) {
    saveBtn.addEventListener("click", saveConfig);
  }
}

async function loadConfig() {
  try {
    // Load config template from backend API (auto-generated from config.py)
    const templateResponse = await fetch("/api/config/schema");
    if (templateResponse.ok) {
      state.configTemplate = await templateResponse.json();
      console.log(
        "[Config] Template loaded from backend:",
        state.configTemplate,
      );
    } else {
      console.warn("[Config] Failed to load template from backend");
      state.configTemplate = null;
    }

    // Load config
    const configResponse = await fetch("/api/config");
    const config = await configResponse.json();
    state.config = config;

    // 初始化摄像头索引（从配置加载）
    initCameraIndices();

    // Load dynamic options for dropdowns
    await loadDynamicOptions();

    // Render config form from template (or use legacy fallback)
    if (state.configTemplate && state.configTemplate.sections) {
      renderConfigFormFromTemplate(config, state.configTemplate);
    } else {
      populateConfigForm(config);
    }
    updateSystemInfoBar(config);
  } catch (e) {
    console.error("Failed to load config:", e);
  }

  // Also load system info from dedicated endpoint
  try {
    const sysResponse = await fetch("/api/system-info");
    const sysInfo = await sysResponse.json();

    // Update local IP from system info
    const localIPEl = document.getElementById("local-ip");
    if (localIPEl && sysInfo.local_ip && sysInfo.local_ip !== "--") {
      localIPEl.textContent = sysInfo.local_ip;
    }
  } catch (e) {
    console.error("Failed to load system info:", e);
  }
}

// Dynamic form rendering from template
function renderConfigFormFromTemplate(config, template) {
  console.log("[Config] Rendering form from template");

  const sections = template.sections;
  if (!sections) return;

  // Group sections by tab
  const tabSections = {};
  for (const [sectionKey, sectionInfo] of Object.entries(sections)) {
    const tab = sectionInfo.tab || "data";
    if (!tabSections[tab]) tabSections[tab] = [];
    tabSections[tab].push({ key: sectionKey, info: sectionInfo });
  }

  // Render each tab
  for (const [tabId, sectionList] of Object.entries(tabSections)) {
    const tabContent = document.getElementById(`tab-${tabId}`);
    if (!tabContent) continue;

    // Build HTML for all sections in this tab
    let html = "";
    for (const { key: sectionKey, info: sectionInfo } of sectionList) {
      html += renderSection(sectionKey, sectionInfo, config);
    }
    tabContent.innerHTML = html;
  }

  // Update system info bar
  updateSystemInfoBar(config);
}

function renderSection(sectionKey, sectionInfo, config) {
  const sectionData = config[sectionKey] || {};
  const icon = sectionInfo.icon || "⚙️";
  const label = sectionInfo.label || sectionKey;

  let html = `<div class="config-section" data-section="${sectionKey}">
        <h4>${icon} ${label}</h4>
        <div class="config-fields">`;

  // Sort fields by order
  const fields = sectionInfo.fields || {};
  const sortedFields = Object.entries(fields).sort(
    (a, b) => (a[1].order || 0) - (b[1].order || 0),
  );

  // Group fields by their "group" property
  const groups = {};
  const ungrouped = [];

  for (const [fieldKey, fieldInfo] of sortedFields) {
    if (fieldInfo.group) {
      if (!groups[fieldInfo.group]) {
        groups[fieldInfo.group] = {
          label: fieldInfo.group_label || fieldInfo.group,
          fields: [],
        };
      }
      groups[fieldInfo.group].fields.push({ key: fieldKey, info: fieldInfo });
    } else {
      ungrouped.push({ key: fieldKey, info: fieldInfo });
    }
  }

  // Render ungrouped fields first
  for (const { key: fieldKey, info: fieldInfo } of ungrouped) {
    const value = sectionData[fieldKey];
    html += renderField(sectionKey, fieldKey, fieldInfo, value);
  }

  // Render grouped fields in horizontal layout
  for (const [groupKey, groupInfo] of Object.entries(groups)) {
    html += `<div class="config-field-group">
            <div class="config-group-label">${groupInfo.label}</div>
            <div class="config-group-fields">`;

    for (const { key: fieldKey, info: fieldInfo } of groupInfo.fields) {
      const value = sectionData[fieldKey];
      html += renderField(sectionKey, fieldKey, fieldInfo, value, true);
    }

    html += "</div></div>";
  }

  html += "</div></div>";
  return html;
}

function renderField(sectionKey, fieldKey, fieldInfo, value) {
  const id = `cfg-${sectionKey}-${fieldKey}`;
  const label = fieldInfo.label || fieldKey;
  const type = fieldInfo.type || "string";
  const description = fieldInfo.description || "";

  let fieldHtml = `<div class="config-field" data-field="${fieldKey}" data-type="${type}">
        <label for="${id}">${label}</label>`;

  if (description) {
    fieldHtml += `<small class="field-desc">${description}</small>`;
  }

  switch (type) {
    case "bool":
      const checked = value === true ? "checked" : "";
      fieldHtml += `<input type="checkbox" id="${id}" ${checked}>`;
      break;

    case "int":
      const intVal = value !== undefined ? value : fieldInfo.default || 0;
      const intMin =
        fieldInfo.min !== undefined ? `min="${fieldInfo.min}"` : "";
      const intMax =
        fieldInfo.max !== undefined ? `max="${fieldInfo.max}"` : "";
      fieldHtml += `<input type="number" id="${id}" value="${intVal}" ${intMin} ${intMax} step="1">`;
      break;

    case "float":
      const floatVal = value !== undefined ? value : fieldInfo.default || 0;
      const floatMin =
        fieldInfo.min !== undefined ? `min="${fieldInfo.min}"` : "";
      const floatMax =
        fieldInfo.max !== undefined ? `max="${fieldInfo.max}"` : "";
      const step = fieldInfo.step ? `step="${fieldInfo.step}"` : 'step="0.01"';
      fieldHtml += `<input type="number" id="${id}" value="${floatVal}" ${floatMin} ${floatMax} ${step}>`;
      break;

    case "select":
      const selectVal = value !== undefined ? value : fieldInfo.default || "";
      fieldHtml += `<select id="${id}">`;
      for (const opt of fieldInfo.options || []) {
        const optLabel = fieldInfo.option_labels?.[opt] || opt;
        const selected = opt === selectVal ? "selected" : "";
        fieldHtml += `<option value="${opt}" ${selected}>${optLabel}</option>`;
      }
      fieldHtml += "</select>";
      break;

    case "string":
      const strVal =
        value !== undefined && value !== null ? value : fieldInfo.default || "";
      const placeholder = fieldInfo.placeholder
        ? `placeholder="${fieldInfo.placeholder}"`
        : "";
      fieldHtml += `<input type="text" id="${id}" value="${strVal}" ${placeholder}>`;
      break;

    case "list[int]":
    case "list[float]":
      const arrValue = Array.isArray(value) ? value : fieldInfo.default || [];
      const itemLabels = fieldInfo.item_labels || [];
      const isFloat = type === "list[float]";
      const arrStep = isFloat ? 'step="0.01"' : 'step="1"';

      fieldHtml += '<div class="list-inputs">';
      for (let i = 0; i < arrValue.length; i++) {
        const itemLabel = itemLabels[i] || `#${i + 1}`;
        const itemVal = arrValue[i] !== undefined ? arrValue[i] : 0;
        fieldHtml += `<div class="list-item">
                    <span class="list-item-label">${itemLabel}</span>
                    <input type="number" id="${id}-${i}" value="${itemVal}" ${arrStep}>
                </div>`;
      }
      fieldHtml += "</div>";
      break;

    case "object":
      // Recursively render nested fields
      const nestedData = value || {};
      const nestedFields = fieldInfo.fields || {};
      const sortedNestedFields = Object.entries(nestedFields).sort(
        (a, b) => (a[1].order || 0) - (b[1].order || 0),
      );

      fieldHtml += '<div class="nested-fields">';
      for (const [nestedKey, nestedInfo] of sortedNestedFields) {
        const nestedValue = nestedData[nestedKey];
        fieldHtml += renderField(
          sectionKey,
          `${fieldKey}.${nestedKey}`,
          nestedInfo,
          nestedValue,
        );
      }
      fieldHtml += "</div>";
      break;

    default:
      const defVal = value !== undefined ? value : "";
      fieldHtml += `<input type="text" id="${id}" value="${defVal}">`;
  }

  fieldHtml += "</div>";
  return fieldHtml;
}

async function loadDynamicOptions() {
  try {
    const response = await fetch("/api/dynamic-options");
    const options = await response.json();

    // Get current config values for models
    const hailoModel = state.config?.VisionVals?.HailoModelName;
    const tunaModel = state.config?.VisionVals?.TunaModelName;
    const tunaEngine = state.config?.VisionVals?.TunaEnginePath;

    // Populate Hailo models dropdown with config value
    populateSelectOptions(
      "vision-hailo-model",
      options.hailo_models || [],
      "-- 选择模型 --",
      hailoModel,
    );

    // Populate Tuna models dropdown with config value
    populateSelectOptions(
      "vision-tuna-model",
      options.tuna_models || [],
      "-- 选择模型 --",
      tunaModel,
    );

    // Populate Tuna engines dropdown with config value
    populateSelectOptions(
      "vision-tuna-engine",
      options.tuna_engines || [],
      "-- 选择引擎 --",
      tunaEngine,
    );

    console.log("[Config] Dynamic options loaded:", options);
  } catch (e) {
    console.error("Failed to load dynamic options:", e);
  }
}

function populateSelectOptions(selectId, options, defaultLabel, configValue) {
  const select = document.getElementById(selectId);
  if (!select) return;

  // Clear existing options
  select.innerHTML = "";

  // Add default option
  const defaultOpt = document.createElement("option");
  defaultOpt.value = "";
  defaultOpt.textContent = defaultLabel;
  select.appendChild(defaultOpt);

  // Add options from list
  options.forEach((opt) => {
    const option = document.createElement("option");
    option.value = opt;
    option.textContent = opt;
    select.appendChild(option);
  });

  // Try to match config value
  if (configValue) {
    // Extract filename from path (e.g., "/ArisuModel_v1.2s.hef" -> "ArisuModel_v1.2s.hef")
    const filename = configValue.split("/").pop();

    // First try exact match
    if (options.includes(configValue)) {
      select.value = configValue;
    }
    // Then try filename match
    else if (options.includes(filename)) {
      select.value = filename;
    }
    // If no match, add config value as custom option
    else if (configValue && configValue.trim()) {
      const customOpt = document.createElement("option");
      customOpt.value = configValue;
      customOpt.textContent = configValue + " (配置)";
      select.appendChild(customOpt);
      select.value = configValue;
    }
  }
}

function populateConfigForm(config) {
  if (!config) {
    console.warn("[Config] No config data to populate");
    return;
  }

  console.log("[Config] Populating form with config:", config);

  // Robot info
  setFieldValue("robot-number", config.RoboInfo?.number);
  setFieldValue("robot-type", config.RoboInfo?.type);
  setFieldValue("robot-bit", config.RoboInfo?.Bit); // Note: JSON uses "Bit" (capital B)
  setFieldValue("robot-rec", config.RoboInfo?.Rec); // Note: JSON uses "Rec" (capital R)

  console.log("[Config] Robot info loaded:", {
    number: config.RoboInfo?.number,
    type: config.RoboInfo?.type,
    Bit: config.RoboInfo?.Bit,
    Rec: config.RoboInfo?.Rec,
  });

  // Ports
  setFieldValue("lidar-id", config.Ports?.LidarID);
  setFieldValue("iceloong-port", config.Ports?.IceLoongPort);
  setFieldValue("motor-lf", config.Ports?.LeftFront);
  setFieldValue("motor-lb", config.Ports?.LeftBack);
  setFieldValue("motor-rb", config.Ports?.RightBack);
  setFieldValue("motor-rf", config.Ports?.RightFront);
  setFieldValue("motor-dribble", config.Ports?.Dribble);
  setFieldValue("motor-magnet", config.Ports?.ElecMagnet);

  // Simulator
  setFieldValue("sim-unity-ip", config.Sim?.UnityIP);
  setFieldValue("sim-robot-id", config.Sim?.RobotId);
  setFieldValue("sim-cmd-port", config.Sim?.CmdPort);
  setFieldValue("sim-status-port", config.Sim?.StatusPort);
  setFieldValue("sim-wheel-v", config.Sim?.WheelToV);
  setFieldValue("sim-wheel-wz", config.Sim?.WheelToWz);
  setFieldValue("sim-kick", config.Sim?.KickStrength);
  setFieldValue("sim-dribble", config.Sim?.DribbleStrengthScale);
  setFieldValue("sim-move-scale", config.Sim?.PlanarMoveSpeedScale);
  setFieldValue("sim-unity-cm", config.Sim?.UnityToCm);
  setFieldValue("sim-yaw-offset", config.Sim?.YawOffsetDeg);
  setFieldValue("sim-yaw-zero-start", config.Sim?.YawZeroAtStart, "checkbox");
  setFieldValue("sim-port-base", config.Sim?.CmdPortBase);
  setFieldValue("sim-port-stride", config.Sim?.PortStride);
  setFieldValue("sim-multi-mode", config.Sim?.MultiRobotMode);

  // Vision
  setFieldValue("cam-front", config.VisionVals?.Front);
  setFieldValue("cam-right", config.VisionVals?.Right);
  setFieldValue("cam-back", config.VisionVals?.Back);
  setFieldValue("cam-left", config.VisionVals?.Left);
  setFieldValue("cam-expose", config.VisionVals?.ExposeVal);
  setFieldValue(
    "cam-auto-expose",
    config.VisionVals?.AutoExpose === 1,
    "checkbox",
  );
  setFieldValue("vision-hailo-model", config.VisionVals?.HailoModelName);
  setFieldValue("vision-tuna-model", config.VisionVals?.TunaModelName);
  setFieldValue("vision-tuna-engine", config.VisionVals?.TunaEnginePath);
  setFieldValue("obj-ball-dia", config.VisionVals?.BallDiameterCM);
  setFieldValue("obj-chassis-w", config.VisionVals?.ChassisWidthCM);
  setFieldValue("obj-chassis-h", config.VisionVals?.ChassisHeightCM);
  setFieldValue("obj-blue-goal-w", config.VisionVals?.BlueGoalWidthCM);
  setFieldValue("obj-blue-goal-h", config.VisionVals?.BlueGoalHeightCM);
  setFieldValue("obj-yellow-goal-w", config.VisionVals?.YellowGoalWidthCM);
  setFieldValue("obj-yellow-goal-h", config.VisionVals?.YellowGoalHeightCM);
  setFieldValue(
    "vision-bbox-dist",
    config.VisionVals?.EnableBBoxDistanceEstimate,
    "checkbox",
  );

  // Network
  setFieldValue("trans-type", config.Transimission?.Type);
  setFieldValue("trans-method", config.Transimission?.Method);
  setFieldValue("wifi-self-ip", config.WIFI?.SelfIP);
  setFieldValue("wifi-remote-ip", config.WIFI?.RemoteIP);
  setFieldValue("wifi-port", config.WIFI?.Port);
  setFieldValue("wifi-ssid", config.WIFI?.SSID);
  setFieldValue("wifi-pwd", config.WIFI?.PWD);

  // Data/Recording
  setFieldValue("rec-record", config.VisionVals?.Record, "checkbox");
  setFieldValue("rec-dir", config.VisionVals?.RecordDir);
  setFieldValue("rec-width", config.VisionVals?.RecordWidth);
  setFieldValue("rec-height", config.VisionVals?.RecordHeight);
  setFieldValue("rec-fps", config.VisionVals?.RecordFPS);
  setFieldValue("rec-segment", config.VisionVals?.RecordSegmentSeconds);
  setFieldValue("rec-codec", config.VisionVals?.RecordCodec);
  setFieldValue("rec-ext", config.VisionVals?.RecordExt);

  // Field
  setFieldValue("field-short", config.Bounds?.Short);
  setFieldValue("field-long", config.Bounds?.Long);
  setFieldValue("field-restrict-x", config.Bounds?.restricted_area_x);
  setFieldValue("field-restrict-y", config.Bounds?.restricted_area_y);

  // Home
  const home = config.CheckPoints?.Home || [0, 0, 0];
  setFieldValue("home-x", home[0]);
  setFieldValue("home-y", home[1]);
  setFieldValue("home-theta", home[2]);

  // Debug
  setFieldValue("debug-full-log", config.Debug?.FullLog, "checkbox");
  setFieldValue("debug-beta", config.Debug?.Beta, "checkbox");

  // Motion recording (for replay)
  setFieldValue("debug-record-motion", config.Debug?.RecordMotion, "checkbox");
  setFieldValue("debug-record-interval", config.Debug?.RecordInterval);
  setFieldValue("debug-record-session", config.Debug?.RecordSessionId);
}

function setFieldValue(id, value, type = "input") {
  const el = document.getElementById(id);
  if (!el) {
    console.warn("[Config] Element not found:", id);
    return;
  }

  if (type === "checkbox") {
    el.checked = Boolean(value);
  } else if (el.tagName === "SELECT") {
    // For select elements, find and select the matching option
    const strValue = String(value ?? "");
    const options = el.options;
    let found = false;

    for (let i = 0; i < options.length; i++) {
      if (options[i].value === strValue) {
        el.selectedIndex = i;
        found = true;
        console.log("[Config] Set select", id, "to", strValue);
        break;
      }
    }

    if (!found && strValue) {
      console.warn("[Config] Option not found for", id, "value:", strValue);
    }
  } else {
    // For input elements, set the value
    if (value !== null && value !== undefined) {
      el.value = value;
    }
  }
}

function getFieldValue(id, type = "input") {
  const el = document.getElementById(id);
  if (!el) return null;

  if (type === "checkbox") {
    return el.checked;
  } else if (el.type === "number") {
    return parseFloat(el.value) || 0;
  } else {
    return el.value;
  }
}

function updateSystemInfoBar(config) {
  // Local IP
  const localIPEl = document.getElementById("local-ip");
  if (localIPEl) {
    localIPEl.textContent = config.WIFI?.SelfIP || "--";
  }

  // Unity IP
  const unityIPEl = document.getElementById("unity-ip-display");
  if (unityIPEl) {
    unityIPEl.textContent = config.Sim?.UnityIP || "--";
  }

  // Robot info
  const robotInfoEl = document.getElementById("robot-info");
  if (robotInfoEl && config.RoboInfo) {
    robotInfoEl.textContent = `#${config.RoboInfo.number} ${config.RoboInfo.type}`;
  }

  // Connection mode
  const connModeEl = document.getElementById("connection-mode");
  if (connModeEl && config.Transimission) {
    connModeEl.textContent = `${config.Transimission.Type} / ${config.Transimission.Method}`;
  }

  // Recognition type
  const recTypeEl = document.getElementById("rec-type-display");
  if (recTypeEl && config.RoboInfo?.Rec) {
    const recNames = {
      AI: "AI (Arisu)",
      TV: "TV (Tuna)",
      BB: "BB (Blob)",
      XV: "XV (Xelerator)",
    };
    recTypeEl.textContent =
      recNames[config.RoboInfo.Rec] || config.RoboInfo.Rec;
  }
}

// OTA Functions
function setupOTAControls() {
  const startBtn = document.getElementById("ota-start-btn");
  const refreshBtn = document.getElementById("ota-refresh-btn");

  if (startBtn) {
    startBtn.addEventListener("click", startOTAUpdate);
  }

  if (refreshBtn) {
    refreshBtn.addEventListener("click", () => {
      loadDynamicOptions();
      document.getElementById("ota-log").value = "模型列表已刷新";
    });
  }
}

async function startOTAUpdate() {
  const btn = document.getElementById("ota-start-btn");
  const logEl = document.getElementById("ota-log");

  if (btn) {
    btn.disabled = true;
    btn.textContent = "⏳ 更新中...";
  }

  if (logEl) {
    logEl.value = "正在启动 OTA 更新...\n";
  }

  try {
    const response = await fetch("/api/ota/start", {
      method: "POST",
    });
    const data = await response.json();

    if (data.success) {
      // Start polling for log updates
      pollOTALog();
    } else {
      if (logEl) {
        logEl.value += "启动失败: " + (data.error || "未知错误");
      }
    }
  } catch (e) {
    if (logEl) {
      logEl.value += "请求失败: " + e.message;
    }
  }
}

function pollOTALog() {
  const logEl = document.getElementById("ota-log");
  const btn = document.getElementById("ota-start-btn");

  let attempts = 0;
  const maxAttempts = 300; // 5 minutes at 1 second intervals

  const poll = async () => {
    try {
      const response = await fetch("/api/ota/log");
      const data = await response.json();

      if (logEl) {
        logEl.value = data.log;
        // Scroll to bottom
        logEl.scrollTop = logEl.scrollHeight;
      }

      // Check if OTA is complete
      if (
        data.log &&
        (data.log.includes("OTA 更新完成") ||
          data.log.includes("OTA 更新失败") ||
          data.log.includes("OTA 更新出错"))
      ) {
        if (btn) {
          btn.disabled = false;
          btn.textContent = "🔄 开始 OTA 更新";
        }
        // Refresh model options
        loadDynamicOptions();
        return;
      }

      // Continue polling
      attempts++;
      if (attempts < maxAttempts) {
        setTimeout(poll, 1000);
      } else {
        if (logEl) {
          logEl.value += "\n\n日志轮询超时";
        }
        if (btn) {
          btn.disabled = false;
          btn.textContent = "🔄 开始 OTA 更新";
        }
      }
    } catch (e) {
      console.error("OTA log poll error:", e);
      attempts++;
      if (attempts < maxAttempts) {
        setTimeout(poll, 2000); // Retry with longer interval
      }
    }
  };

  setTimeout(poll, 500);
}

// ==================== 键盘控制页面 ====================

// 控制状态
const controlState = {
  enabled: false,
  speed: 0.1,
  turnSpeed: 50,
  dribbleActive: false,
  runtimeInitialized: false,
  chassisAvailable: false,
  peripheralAvailable: false,
  profile: "normal",
  telemetryPending: false,
  leasePending: false,
  currentDirection: null,
  keysPressed: new Set(),
  mouseControlEnabled: false,
  mouseCaptured: false,
  mouseSensitivity: 12,
  mouseAccumX: 0,
  mouseAccumY: 0,
  lastMouseMoveTime: 0,
  mouseTurnWz: 0,
  mouseTurnResetTimer: null,
};

// 初始化键盘控制页面
function setupControlPage() {
  // 启用/禁用按钮
  const enableBtn = document.getElementById("control-enable-btn");
  if (enableBtn) {
    enableBtn.addEventListener("click", toggleControlEnable);
  }

  const profileSelect = document.getElementById("control-profile");
  if (profileSelect) {
    profileSelect.addEventListener("change", updateControlProfile);
  }

  setupReverseHoldButton("reverse-forward-btn", "w");
  setupReverseHoldButton("reverse-backward-btn", "s");

  // 速度滑块
  const speedSlider = document.getElementById("control-speed");
  const speedValue = document.getElementById("control-speed-value");
  if (speedSlider && speedValue) {
    speedSlider.addEventListener("input", (e) => {
      controlState.speed = parseFloat(e.target.value);
      speedValue.textContent = controlState.speed.toFixed(2);
      updateControlSpeed();
    });
  }

  // 转向速度滑块
  const turnSpeedSlider = document.getElementById("control-turn-speed");
  const turnSpeedValue = document.getElementById("control-turn-speed-value");
  if (turnSpeedSlider && turnSpeedValue) {
    turnSpeedSlider.addEventListener("input", (e) => {
      controlState.turnSpeed = parseInt(e.target.value);
      turnSpeedValue.textContent = controlState.turnSpeed;
      updateControlSpeed();
    });
  }

  // 键盘事件
  document.addEventListener("keydown", handleKeyDown);
  document.addEventListener("keyup", handleKeyUp);

  // 鼠标控制
  setupMouseControl();

  // 定期更新控制状态
  setInterval(updateControlStatus, 500);
  setInterval(updateControlTelemetry, 500);

  // The backend owns the 30 ms C-board command heartbeat. The browser only
  // renews its lightweight ownership lease, avoiding overlapping serial
  // control requests while a WASD key is held.
  setInterval(() => {
    if (
      controlState.enabled &&
      state.currentPage === "control" &&
      !document.hidden
    ) {
      renewControlLease();
    }
  }, 100);

  window.addEventListener("blur", releaseControlKeys);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) disableControlForSafety();
  });
  window.addEventListener("pagehide", disableControlForSafety);
}

function setupReverseHoldButton(id, key) {
  const button = document.getElementById(id);
  if (!button) return;
  const start = (event) => {
    event.preventDefault();
    if (!controlState.enabled) return;
    if (button.setPointerCapture && event.pointerId !== undefined) {
      button.setPointerCapture(event.pointerId);
    }
    controlState.keysPressed.delete(key === "w" ? "s" : "w");
    controlState.keysPressed.add(key);
    sendCombinedControlVector();
  };
  const stop = (event) => {
    event.preventDefault();
    controlState.keysPressed.delete(key);
    if (controlState.enabled) sendCombinedControlVector();
  };
  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointercancel", stop);
  button.addEventListener("lostpointercapture", stop);
}

async function renewControlLease() {
  if (controlState.leasePending) return;
  controlState.leasePending = true;
  try {
    const response = await fetch("/api/control/lease", { method: "POST" });
    const data = await response.json();
    if (!data.success && controlState.enabled) {
      controlState.enabled = false;
      updateControlUI();
      setControlStatusMessage("状态: 控制心跳超时，已安全停机", true);
    }
  } catch (error) {
    console.error("Failed to renew control lease:", error);
  } finally {
    controlState.leasePending = false;
  }
}

async function updateControlProfile() {
  const select = document.getElementById("control-profile");
  if (!select || controlState.enabled) return;
  try {
    const response = await fetch("/api/control/profile", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ profile: select.value }),
    });
    const data = await response.json();
    if (!data.success) throw new Error(data.error || "模式切换失败");
    controlState.profile = data.profile;
  } catch (error) {
    select.value = controlState.profile;
    setControlStatusMessage(`状态: ${error.message}`, true);
  }
}

function displayMetric(value, suffix = " ms") {
  return value === null || value === undefined ? "not_measured" : `${value}${suffix}`;
}

function displayImuVector(values) {
  if (!Array.isArray(values) || values.length !== 3) return "--";
  return values.map((value) => value === null ? "--" : Number(value).toFixed(3)).join(" / ");
}

function updateControlImu(imu) {
  const fields = {
    "control-imu-status": !imu?.available ? `不可用${imu?.error ? `: ${imu.error}` : ""}` : (imu.healthy ? "健康" : "异常"),
    "control-imu-source": imu?.available ? imu.source : "--",
    "control-imu-age": imu?.available && imu.age_ms !== null ? `${Number(imu.age_ms).toFixed(1)} ms` : "--",
    "control-imu-attitude": imu?.available ? displayImuVector([
      imu.attitude_deg?.roll,
      imu.attitude_deg?.pitch,
      imu.attitude_deg?.yaw,
    ]) : "--",
    "control-imu-accel": imu?.available ? displayImuVector(imu.accel_mps2) : "--",
    "control-imu-gyro": imu?.available ? displayImuVector(imu.gyro_rad_s) : "--",
    "control-imu-temperature": imu?.available && imu.temperature_c !== null ? `${Number(imu.temperature_c).toFixed(1)} °C` : "--",
    "control-imu-flags": imu?.available ? `0x${Number(imu.flags).toString(16)}` : "--",
  };
  for (const [id, value] of Object.entries(fields)) {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  }
}

async function updateControlTelemetry() {
  if (state.currentPage !== "control" || controlState.telemetryPending) return;
  controlState.telemetryPending = true;
  try {
    const response = await fetch("/api/control/telemetry");
    const data = await response.json();
    if (!data.success) throw new Error(data.error || "反馈不可用");
    const tbody = document.getElementById("control-motor-telemetry");
    if (tbody) {
      tbody.replaceChildren(...data.wheels.map((wheel) => {
        const row = document.createElement("tr");
        const values = [
          wheel.wheel,
          Number(wheel.target_velocity_rad_s).toFixed(2),
          wheel.measured_velocity_rad_s === null ? "--" : Number(wheel.measured_velocity_rad_s).toFixed(2),
          wheel.velocity_error_rad_s === null ? "--" : Number(wheel.velocity_error_rad_s).toFixed(2),
          !wheel.torque_feedback_supported ? "unsupported" :
            (wheel.measured_torque_nm === null ? "--" : Number(wheel.measured_torque_nm).toFixed(2)),
          wheel.motor_temperature_c === null || wheel.motor_temperature_c === undefined ? "--" : Number(wheel.motor_temperature_c).toFixed(0),
          ["空闲", "制动", "过零", "反向加速", "完成"][wheel.reverse_state] || `状态${wheel.reverse_state}`,
          ["加速", "减速", "反向制动", "速度饱和"][wheel.limiter_state] || `限幅${wheel.limiter_state}`,
        ];
        for (const value of values) {
          const cell = document.createElement("td");
          cell.textContent = value;
          row.appendChild(cell);
        }
        return row;
      }));
    }
    const event = data.latest_reverse_event;
    document.getElementById("reverse-control-period").textContent = `${data.control_period_ms} ms`;
    document.getElementById("reverse-brake-latency").textContent = event ? displayMetric(event.command_to_brake_latency_ms) : "--";
    document.getElementById("reverse-zero-time").textContent = event ? displayMetric(event.brake_to_zero_cross_ms) : "--";
    document.getElementById("reverse-response-time").textContent = event ? displayMetric(event.zero_cross_to_reverse_response_ms) : "--";
    document.getElementById("reverse-peak-torque").textContent = event ? displayMetric(event.peak_measured_torque_nm, " Nm") : "--";
    document.getElementById("reverse-fault").textContent = data.runtime_fault ? `0x${data.runtime_fault.toString(16)}` : "无";
    updateControlImu(data.imu);
  } catch (error) {
    const tbody = document.getElementById("control-motor-telemetry");
    if (tbody) {
      const row = document.createElement("tr");
      const cell = document.createElement("td");
      cell.colSpan = 8;
      cell.textContent = error.message;
      row.appendChild(cell);
      tbody.replaceChildren(row);
    }
    updateControlImu({ available: false, error: error.message });
  } finally {
    controlState.telemetryPending = false;
  }
}

function releaseControlKeys() {
  controlState.keysPressed.clear();
  controlState.mouseTurnWz = 0;
  if (controlState.enabled) sendControlMoveVector(0, 0, 0);
  updateDirectionDisplayFromVector(0, 0, 0);
  updateDirectionDisplay();
}

function disableControlForSafety() {
  if (!controlState.enabled) return;
  releaseControlKeys();
  controlState.enabled = false;
  fetch("/api/control/enable", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ enabled: false }),
    keepalive: true,
  }).catch(() => {});
}

function setupMouseControl() {
  const captureArea = document.getElementById("mouse-capture-area");
  const mouseControlCheckbox = document.getElementById("mouse-control-enabled");
  const sensitivitySlider = document.getElementById("mouse-sensitivity");
  const sensitivityValue = document.getElementById("mouse-sensitivity-value");

  if (sensitivitySlider && sensitivityValue) {
    sensitivitySlider.addEventListener("input", (e) => {
      controlState.mouseSensitivity = parseInt(e.target.value);
      sensitivityValue.textContent = controlState.mouseSensitivity;
      updateMouseCaptureUI();
    });
  }

  if (mouseControlCheckbox) {
    mouseControlCheckbox.addEventListener("change", (e) => {
      controlState.mouseControlEnabled = e.target.checked;
      if (!controlState.mouseControlEnabled && controlState.mouseCaptured) {
        exitPointerLock();
      }
      updateMouseCaptureUI();
    });
  }

  if (captureArea) {
    captureArea.addEventListener("click", () => {
      if (
        controlState.enabled &&
        controlState.mouseControlEnabled &&
        !controlState.mouseCaptured
      ) {
        requestPointerLock();
      }
    });
  }

  document.addEventListener("pointerlockchange", handlePointerLockChange);
  document.addEventListener("mousemove", handleMouseMove);
}

function requestPointerLock() {
  const captureArea = document.getElementById("mouse-capture-area");
  if (!captureArea) return;
  captureArea.requestPointerLock =
    captureArea.requestPointerLock || captureArea.mozRequestPointerLock;
  captureArea.requestPointerLock();
}

function exitPointerLock() {
  document.exitPointerLock =
    document.exitPointerLock || document.mozExitPointerLock;
  if (document.exitPointerLock) {
    document.exitPointerLock();
  }
}

function handlePointerLockChange() {
  const captureArea = document.getElementById("mouse-capture-area");
  const locked =
    document.pointerLockElement === captureArea ||
    document.mozPointerLockElement === captureArea;

  controlState.mouseCaptured = locked;
  if (!locked) {
    controlState.mouseAccumX = 0;
    controlState.mouseAccumY = 0;
    controlState.lastMouseMoveTime = 0;
    controlState.mouseTurnWz = 0;
    if (controlState.mouseTurnResetTimer) {
      clearTimeout(controlState.mouseTurnResetTimer);
      controlState.mouseTurnResetTimer = null;
    }
    if (controlState.enabled) {
      sendCombinedControlVector();
    }
  }

  updateMouseCaptureUI();
}

function handleMouseMove(e) {
  if (
    state.currentPage !== "control" ||
    !controlState.enabled ||
    !controlState.mouseCaptured
  )
    return;

  const movementX = e.movementX || e.mozMovementX || 0;
  const movementY = e.movementY || e.mozMovementY || 0;
  const now = performance.now();
  const deltaMs =
    controlState.lastMouseMoveTime > 0
      ? Math.max(1, now - controlState.lastMouseMoveTime)
      : 16;
  controlState.lastMouseMoveTime = now;

  controlState.mouseAccumX += movementX;
  controlState.mouseAccumY += movementY;

  const posEl = document.getElementById("mouse-position");
  if (posEl) {
    posEl.textContent = `X: ${controlState.mouseAccumX}, Y: ${controlState.mouseAccumY}`;
  }

  const sensitivity = controlState.mouseSensitivity;
  const maxTurnSpeed = controlState.turnSpeed;

  if (Math.abs(movementX) >= 1) {
    const movementSpeed = Math.abs(movementX) / deltaMs;
    const direction = movementX > 0 ? -1 : 1;
    const normalizedMagnitude = Math.min(
      1,
      (Math.abs(movementX) * (0.55 + sensitivity * 0.18)) / 18 +
        movementSpeed * (0.35 + sensitivity * 0.05),
    );
    const boostedTurnSpeed = maxTurnSpeed * (0.2 + normalizedMagnitude * 0.8);
    controlState.mouseTurnWz = direction * boostedTurnSpeed;
  }

  if (controlState.mouseTurnResetTimer) {
    clearTimeout(controlState.mouseTurnResetTimer);
  }
  controlState.mouseTurnResetTimer = setTimeout(() => {
    controlState.mouseTurnWz = 0;
    sendCombinedControlVector();
  }, 120);

  sendCombinedControlVector();
}

function updateMouseCaptureUI() {
  const captureArea = document.getElementById("mouse-capture-area");
  const hintEl = document.getElementById("mouse-hint");
  const statusEl = document.getElementById("mouse-capture-status");

  if (captureArea) {
    captureArea.classList.toggle("captured", controlState.mouseCaptured);
  }

  if (hintEl) {
    if (controlState.mouseCaptured) {
      hintEl.textContent = `鼠标左右移动可连续调节转向速度（当前灵敏度 ${controlState.mouseSensitivity}），按 ESC 退出控制`;
    } else if (controlState.enabled && controlState.mouseControlEnabled) {
      hintEl.textContent = `点击此区域进入鼠标转向控制（当前灵敏度 ${controlState.mouseSensitivity}）`;
    } else {
      hintEl.textContent = "请先启用键盘控制与鼠标控制";
    }
  }

  if (statusEl) {
    statusEl.textContent = controlState.mouseCaptured ? "已捕获" : "未捕获";
    statusEl.classList.toggle("active", controlState.mouseCaptured);
    statusEl.classList.toggle("inactive", !controlState.mouseCaptured);
  }
}

function setControlStatusMessage(message, isError = false) {
  const statusEl = document.getElementById("control-status");
  if (!statusEl) return;
  statusEl.textContent = message;
  statusEl.style.color = isError ? "#ef4444" : "";
}

// 切换控制启用状态
async function toggleControlEnable() {
  const btn = document.getElementById("control-enable-btn");

  try {
    const response = await fetch("/api/control/enable", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: !controlState.enabled }),
    });

    const data = await response.json();

    if (data.success) {
      controlState.enabled = data.enabled;
      updateControlUI();
    } else {
      setControlStatusMessage("状态: " + (data.error || "启用失败"), true);
      alert("操作失败: " + data.error);
    }
  } catch (e) {
    console.error("Failed to toggle control:", e);
    alert("操作失败");
  }
}

// 更新控制速度
async function updateControlSpeed() {
  try {
    await fetch("/api/control/speed", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        speed: controlState.speed,
        turn_speed: controlState.turnSpeed,
      }),
    });
  } catch (e) {
    console.error("Failed to update speed:", e);
  }
}

// 处理键盘按下
function handleKeyDown(e) {
  // 只在控制页面且启用时响应
  if (state.currentPage !== "control" || !controlState.enabled) return;

  const key = e.key.toLowerCase();

  // 防止重复触发
  if (controlState.keysPressed.has(key)) return;
  controlState.keysPressed.add(key);

  // 高亮按键
  highlightKey(key, true);

  // 处理特殊按键
  if (key === " ") {
    e.preventDefault();
    sendControlKick();
    return;
  }
  if (key === "r") {
    toggleDribble();
    return;
  }

  if (key === "escape" && controlState.mouseCaptured) {
    exitPointerLock();
    return;
  }

  // 移动键：更新组合方向
  if (["w", "s", "a", "d"].includes(key)) {
    sendCombinedControlVector();
  }
}

// 处理键盘释放
function handleKeyUp(e) {
  const key = e.key.toLowerCase();
  controlState.keysPressed.delete(key);

  // 取消高亮
  highlightKey(key, false);

  // 移动键：更新组合方向
  if (["w", "s", "a", "d"].includes(key)) {
    sendCombinedControlVector();
  }
}

// 发送键盘 + 鼠标的组合控制向量
function sendCombinedControlVector() {
  const keys = controlState.keysPressed;
  const speed = controlState.speed;

  let vx = 0,
    vy = 0,
    wz = 0;

  // W/S: 前后 (X方向)
  if (keys.has("w")) vx += speed;
  if (keys.has("s")) vx -= speed;

  // A/D: 左右 (Y方向)
  if (keys.has("a")) vy += speed;
  if (keys.has("d")) vy -= speed;

  // Keep diagonal translation at the selected resultant speed instead of
  // making W+A/W+D sqrt(2) times faster than a single-axis command.
  const linearMagnitude = Math.hypot(vx, vy);
  if (linearMagnitude > speed && linearMagnitude > 0) {
    const scale = speed / linearMagnitude;
    vx *= scale;
    vy *= scale;
  }

  // 鼠标负责转向
  wz =
    controlState.mouseCaptured && controlState.mouseControlEnabled
      ? controlState.mouseTurnWz
      : 0;

  if (vx === 0 && vy === 0 && wz === 0) {
    sendControlMoveVector(0, 0, 0);
    controlState.currentDirection = null;
  } else {
    sendControlMoveVector(vx, vy, wz);
    updateDirectionDisplayFromVector(vx, vy, wz);
  }
  updateDirectionDisplay();
}

// 发送控制向量
async function sendControlMoveVector(vx, vy, wz) {
  try {
    const response = await fetch("/api/control/move_vector", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ vx, vy, wz }),
    });

    const data = await response.json();

    if (data.success) {
      controlState.currentVector = { vx, vy, wz };
    } else {
      setControlStatusMessage("状态: " + (data.error || "移动失败"), true);
    }
  } catch (e) {
    console.error("Failed to send move vector:", e);
    setControlStatusMessage("状态: 移动请求失败", true);
  }
}

// 从向量更新方向显示
function updateDirectionDisplayFromVector(vx, vy, wz) {
  // 构建方向描述
  const parts = [];

  // X方向
  if (vx > 0) parts.push("前进");
  else if (vx < 0) parts.push("后退");

  // Y方向
  if (vy > 0) parts.push("左移");
  else if (vy < 0) parts.push("右移");

  // 转向
  if (wz > 0) parts.push("左转");
  else if (wz < 0) parts.push("右转");

  controlState.currentDirection = parts.length > 0 ? parts.join("+") : null;
}

// 高亮按键
function highlightKey(key, active) {
  const keyMap = {
    w: "w",
    s: "s",
    a: "a",
    d: "d",
    " ": "space",
    r: "r",
  };

  const dataKey = keyMap[key];
  if (!dataKey) return;

  const keyEl = document.querySelector(`.key[data-key="${dataKey}"]`);
  if (keyEl) {
    if (active) {
      keyEl.classList.add("active");
    } else {
      keyEl.classList.remove("active");
    }
  }
}

// 发送移动命令
async function sendControlMove(direction) {
  try {
    const response = await fetch("/api/control/move", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ direction }),
    });

    const data = await response.json();

    if (data.success) {
      controlState.currentDirection = direction;
      updateDirectionDisplay();
    } else {
      setControlStatusMessage("状态: " + (data.error || "移动失败"), true);
    }
  } catch (e) {
    console.error("Failed to send move command:", e);
    setControlStatusMessage("状态: 移动请求失败", true);
  }
}

// 发送弹射命令
async function sendControlKick() {
  try {
    const response = await fetch("/api/control/kick", {
      method: "POST",
    });

    const data = await response.json();

    if (data.success) {
      // 短暂高亮提示
      const statusEl = document.getElementById("control-status");
      if (statusEl) {
        statusEl.textContent = "状态: 弹射!";
        statusEl.style.color = "#22c55e";
        setTimeout(() => {
          statusEl.textContent = controlState.enabled
            ? "状态: 已启用"
            : "状态: 未启用";
          statusEl.style.color = "";
        }, 500);
      }
    } else {
      setControlStatusMessage("状态: " + (data.error || "弹射失败"), true);
    }
  } catch (e) {
    console.error("Failed to send kick command:", e);
    setControlStatusMessage("状态: 弹射请求失败", true);
  }
}

// 切换盘球状态
async function toggleDribble() {
  try {
    const response = await fetch("/api/control/dribble/toggle", {
      method: "POST",
    });

    const data = await response.json();

    if (data.success) {
      controlState.dribbleActive = data.dribble_active;
      updateDribbleDisplay();
    } else {
      setControlStatusMessage("状态: " + (data.error || "盘球失败"), true);
    }
  } catch (e) {
    console.error("Failed to toggle dribble:", e);
    setControlStatusMessage("状态: 盘球请求失败", true);
  }
}

// 更新控制 UI
function updateControlUI() {
  const btn = document.getElementById("control-enable-btn");
  const statusEl = document.getElementById("control-status");
  const enabledStatusEl = document.getElementById("control-enabled-status");

  if (btn) {
    btn.textContent = controlState.enabled
      ? "🛑 禁用键盘控制"
      : "🎮 启用键盘控制";
    btn.classList.toggle("btn-danger", controlState.enabled);
    btn.classList.toggle("btn-primary", !controlState.enabled);
  }

  if (statusEl) {
    statusEl.textContent = controlState.enabled
      ? "状态: 已启用"
      : "状态: 未启用";
    statusEl.style.color = controlState.enabled ? "#22c55e" : "";
  }

  if (enabledStatusEl) {
    enabledStatusEl.textContent = controlState.enabled ? "已启用" : "未启用";
    enabledStatusEl.classList.toggle("active", controlState.enabled);
    enabledStatusEl.classList.toggle("inactive", !controlState.enabled);
  }

  const profileSelect = document.getElementById("control-profile");
  if (profileSelect) profileSelect.disabled = controlState.enabled;

  updateMouseCaptureUI();
}

// 更新方向显示
function updateDirectionDisplay() {
  const el = document.getElementById("control-direction");
  if (!el) return;

  const directionNames = {
    forward: "前进",
    backward: "后退",
    left: "左移",
    right: "右移",
    turn_left: "左转",
    turn_right: "右转",
  };

  el.textContent = controlState.currentDirection
    ? directionNames[controlState.currentDirection] ||
      controlState.currentDirection
    : "--";
}

// 更新盘球显示
function updateDribbleDisplay() {
  const el = document.getElementById("control-dribble-status");
  if (!el) return;

  el.textContent = controlState.dribbleActive ? "开启" : "关闭";
  el.classList.toggle("active", controlState.dribbleActive);
  el.classList.toggle("inactive", !controlState.dribbleActive);

  // 更新 R 键显示
  const rKey = document.querySelector('.key[data-key="r"]');
  if (rKey) {
    rKey.classList.toggle("active", controlState.dribbleActive);
  }
}

function updateControlDiagnostics() {
  const runtimeEl = document.getElementById("control-runtime-status");
  const chassisEl = document.getElementById("control-chassis-status");
  const peripheralEl = document.getElementById("control-peripheral-status");

  if (runtimeEl) {
    runtimeEl.textContent = controlState.runtimeInitialized
      ? "已初始化"
      : "未初始化";
    runtimeEl.classList.toggle("active", controlState.runtimeInitialized);
    runtimeEl.classList.toggle("inactive", !controlState.runtimeInitialized);
  }
  if (chassisEl) {
    chassisEl.textContent = controlState.chassisAvailable ? "可用" : "不可用";
    chassisEl.classList.toggle("active", controlState.chassisAvailable);
    chassisEl.classList.toggle("inactive", !controlState.chassisAvailable);
  }
  if (peripheralEl) {
    peripheralEl.textContent = controlState.peripheralAvailable
      ? "可用"
      : "不可用";
    peripheralEl.classList.toggle("active", controlState.peripheralAvailable);
    peripheralEl.classList.toggle(
      "inactive",
      !controlState.peripheralAvailable,
    );
  }
}

// 更新控制状态（从服务器获取）
async function updateControlStatus() {
  if (state.currentPage !== "control") return;

  try {
    const response = await fetch("/api/control/status");
    const data = await response.json();

    const wasEnabled = controlState.enabled;
    controlState.enabled = data.enabled;
    controlState.speed = data.speed;
    controlState.turnSpeed = data.turn_speed;
    controlState.dribbleActive = data.dribble_active;
    controlState.runtimeInitialized = !!data.runtime_initialized;
    controlState.chassisAvailable = !!data.chassis_available;
    controlState.peripheralAvailable = !!data.peripheral_available;
    controlState.profile = data.profile || controlState.profile;

    // 更新滑块
    const speedSlider = document.getElementById("control-speed");
    const speedValue = document.getElementById("control-speed-value");
    if (speedSlider && speedValue) {
      speedSlider.value = controlState.speed;
      speedValue.textContent = Number(controlState.speed).toFixed(2);
    }

    const turnSpeedSlider = document.getElementById("control-turn-speed");
    const turnSpeedValue = document.getElementById("control-turn-speed-value");
    if (turnSpeedSlider && turnSpeedValue) {
      turnSpeedSlider.value = controlState.turnSpeed;
      turnSpeedValue.textContent = controlState.turnSpeed;
    }

    const profileSelect = document.getElementById("control-profile");
    if (profileSelect) profileSelect.value = controlState.profile;

    updateControlUI();
    updateDribbleDisplay();
    updateControlDiagnostics();
    if (wasEnabled && !data.enabled && data.disable_reason === "lease_expired") {
      releaseControlKeys();
      setControlStatusMessage("状态: 控制心跳超时，已安全停机", true);
    }
    if (data.runtime_error) {
      setControlStatusMessage("状态: " + data.runtime_error, true);
    }
  } catch (e) {
    console.error("Failed to update control status:", e);
  }
}

async function saveConfig() {
  // Use template-driven collection if available
  if (state.configTemplate && state.configTemplate.sections) {
    return saveConfigFromTemplate();
  }

  // Fallback to legacy save
  return saveConfigLegacy();
}

// Save config from dynamic template form
async function saveConfigFromTemplate() {
  const template = state.configTemplate;
  const config = {};

  for (const [sectionKey, sectionInfo] of Object.entries(template.sections)) {
    config[sectionKey] = {};

    for (const [fieldKey, fieldInfo] of Object.entries(
      sectionInfo.fields || {},
    )) {
      const value = collectFieldValue(sectionKey, fieldKey, fieldInfo);
      config[sectionKey][fieldKey] = value;
    }
  }

  console.log("[Config] Saving config from template:", config);

  try {
    const response = await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(config),
    });

    const result = await response.json();

    if (result.success) {
      state.config = config;
      updateSystemInfoBar(config);
      alert("配置已保存");
    } else {
      alert("保存失败: " + result.error);
    }
  } catch (e) {
    console.error("Failed to save config:", e);
    alert("保存失败: " + e.message);
  }
}

// Collect field value from dynamic form
function collectFieldValue(sectionKey, fieldKey, fieldInfo) {
  const type = fieldInfo.type || "string";
  const id = `cfg-${sectionKey}-${fieldKey}`;

  switch (type) {
    case "bool": {
      const el = document.getElementById(id);
      return el ? el.checked : fieldInfo.default || false;
    }

    case "int": {
      const el = document.getElementById(id);
      return el ? parseInt(el.value) || 0 : fieldInfo.default || 0;
    }

    case "float": {
      const el = document.getElementById(id);
      return el ? parseFloat(el.value) || 0 : fieldInfo.default || 0;
    }

    case "select":
    case "string": {
      const el = document.getElementById(id);
      return el ? el.value : fieldInfo.default || "";
    }

    case "list[int]":
    case "list[float]": {
      const isFloat = type === "list[float]";
      const length = fieldInfo.length || fieldInfo.default?.length || 0;
      const result = [];
      for (let i = 0; i < length; i++) {
        const el = document.getElementById(`${id}-${i}`);
        if (el) {
          result.push(
            isFloat ? parseFloat(el.value) || 0 : parseInt(el.value) || 0,
          );
        } else if (fieldInfo.default && fieldInfo.default[i] !== undefined) {
          result.push(fieldInfo.default[i]);
        }
      }
      return result;
    }

    case "object": {
      const result = {};
      for (const [nestedKey, nestedInfo] of Object.entries(
        fieldInfo.fields || {},
      )) {
        result[nestedKey] = collectFieldValue(
          sectionKey,
          `${fieldKey}.${nestedKey}`,
          nestedInfo,
        );
      }
      return result;
    }

    default: {
      const el = document.getElementById(id);
      return el ? el.value : "";
    }
  }
}

// Legacy save config (fallback for old HTML forms)
async function saveConfigLegacy() {
  const config = state.config || {};

  // Collect form values
  config.RoboInfo = {
    number: getFieldValue("robot-number"),
    type: getFieldValue("robot-type"),
    Bit: getFieldValue("robot-bit"),
    Rec: getFieldValue("robot-rec"),
    LidarBackend: config.RoboInfo?.LidarBackend || "ros2",
    LidarType: config.RoboInfo?.LidarType || "s3",
    LidarInverted: config.RoboInfo?.LidarInverted ?? false,
  };

  config.Ports = {
    LidarID: getFieldValue("lidar-id"),
    LidarPort: config.Ports?.LidarPort || "/dev/sllidar",
    IceLoongPort: getFieldValue("iceloong-port"),
    LowTigger: config.Ports?.LowTigger || 0,
    ReferPort: config.Ports?.ReferPort || 1,
    ElecMagnet: getFieldValue("motor-magnet"),
    Dribble: getFieldValue("motor-dribble"),
    LeftFront: getFieldValue("motor-lf"),
    LeftBack: getFieldValue("motor-lb"),
    RightBack: getFieldValue("motor-rb"),
    RightFront: getFieldValue("motor-rf"),
  };

  config.Sim = {
    UnityIP: getFieldValue("sim-unity-ip"),
    CmdPort: getFieldValue("sim-cmd-port"),
    StatusPort: getFieldValue("sim-status-port"),
    RobotId: getFieldValue("sim-robot-id"),
    TwistFromWheels: true,
    WheelToV: getFieldValue("sim-wheel-v"),
    WheelToWz: getFieldValue("sim-wheel-wz"),
    YawOffsetDeg: getFieldValue("sim-yaw-offset"),
    YawZeroAtStart: getFieldValue("sim-yaw-zero-start", "checkbox"),
    KickStrength: getFieldValue("sim-kick"),
    DribbleStrengthScale: getFieldValue("sim-dribble"),
    PlanarMoveSpeedScale: getFieldValue("sim-move-scale"),
    UnityToCm: getFieldValue("sim-unity-cm"),
    OriginXCM: config.Sim?.OriginXCM || 0.0,
    OriginYCM: config.Sim?.OriginYCM || 0.0,
    PoseScale: config.Sim?.PoseScale || 1.0,
    PoseOffsetXCM: config.Sim?.PoseOffsetXCM || 0.0,
    PoseOffsetYCM: config.Sim?.PoseOffsetYCM || 0.0,
    Debug: config.Sim?.Debug || false,
    CmdPortBase: getFieldValue("sim-port-base"),
    StatusPortBase: config.Sim?.StatusPortBase || 20002,
    PortStride: getFieldValue("sim-port-stride"),
    MultiRobotMode: getFieldValue("sim-multi-mode"),
  };

  config.VisionVals = config.VisionVals || {};
  config.VisionVals.Front = getFieldValue("cam-front");
  config.VisionVals.Right = getFieldValue("cam-right");
  config.VisionVals.Back = getFieldValue("cam-back");
  config.VisionVals.Left = getFieldValue("cam-left");
  config.VisionVals.ExposeVal = getFieldValue("cam-expose");
  config.VisionVals.AutoExpose = getFieldValue("cam-auto-expose", "checkbox")
    ? 1
    : 0;
  config.VisionVals.HailoModelName = getFieldValue("vision-hailo-model");
  config.VisionVals.TunaModelName = getFieldValue("vision-tuna-model");
  config.VisionVals.TunaEnginePath = getFieldValue("vision-tuna-engine");
  config.VisionVals.BallDiameterCM = getFieldValue("obj-ball-dia");
  config.VisionVals.ChassisWidthCM = getFieldValue("obj-chassis-w");
  config.VisionVals.ChassisHeightCM = getFieldValue("obj-chassis-h");
  config.VisionVals.BlueGoalWidthCM = getFieldValue("obj-blue-goal-w");
  config.VisionVals.BlueGoalHeightCM = getFieldValue("obj-blue-goal-h");
  config.VisionVals.YellowGoalWidthCM = getFieldValue("obj-yellow-goal-w");
  config.VisionVals.YellowGoalHeightCM = getFieldValue("obj-yellow-goal-h");
  config.VisionVals.EnableBBoxDistanceEstimate = getFieldValue(
    "vision-bbox-dist",
    "checkbox",
  );
  config.VisionVals.Record = getFieldValue("rec-record", "checkbox");
  config.VisionVals.RecordDir = getFieldValue("rec-dir");
  config.VisionVals.RecordWidth = getFieldValue("rec-width");
  config.VisionVals.RecordHeight = getFieldValue("rec-height");
  config.VisionVals.RecordFPS = getFieldValue("rec-fps");
  config.VisionVals.RecordSegmentSeconds = getFieldValue("rec-segment");
  config.VisionVals.RecordCodec = getFieldValue("rec-codec");
  config.VisionVals.RecordExt = getFieldValue("rec-ext");

  config.Transimission = {
    Type: getFieldValue("trans-type"),
    Method: getFieldValue("trans-method"),
  };

  config.WIFI = {
    SelfIP: getFieldValue("wifi-self-ip"),
    RemoteIP: getFieldValue("wifi-remote-ip"),
    Port: getFieldValue("wifi-port"),
    SSID: getFieldValue("wifi-ssid"),
    PWD: getFieldValue("wifi-pwd"),
  };

  config.Bounds = {
    ClsPos: config.Bounds?.ClsPos || [85, 35, 0],
    FarPos: config.Bounds?.FarPos || [95, 75, 0],
    Short: getFieldValue("field-short"),
    Long: getFieldValue("field-long"),
    Distance2Bound: config.Bounds?.Distance2Bound || 240,
    restricted_area_x: getFieldValue("field-restrict-x"),
    restricted_area_y: getFieldValue("field-restrict-y"),
  };

  config.CheckPoints = {
    Home: [
      getFieldValue("home-x"),
      getFieldValue("home-y"),
      getFieldValue("home-theta"),
    ],
  };

  config.Debug = {
    FullLog: getFieldValue("debug-full-log", "checkbox"),
    Beta: getFieldValue("debug-beta", "checkbox"),
    RecordMotion: getFieldValue("debug-record-motion", "checkbox"),
    RecordInterval: getFieldValue("debug-record-interval"),
    RecordSessionId: getFieldValue("debug-record-session") || null,
  };

  try {
    const response = await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(config),
    });

    const result = await response.json();

    if (result.success) {
      state.config = config;
      updateSystemInfoBar(config);
      alert("配置已保存");
    } else {
      alert("保存失败: " + result.error);
    }
  } catch (e) {
    console.error("Failed to save config:", e);
    alert("保存失败: " + e.message);
  }
}
