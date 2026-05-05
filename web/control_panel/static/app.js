/**
 * Xelerator Control Panel - Main Application
 */

// Configuration
const CONFIG = {
    apiBase: '/api',
    updateInterval: 100,  // ms
    visionApiBase: '',  // Will be set from vision dashboard
};

// State
const state = {
    currentPage: 'overview',
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
    fieldWidth: 260,  // cm
    fieldHeight: 200, // cm
    config: null,     // Configuration data
    localIP: '--',
};

// DOM Elements
const elements = {};

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', init);

function init() {
    cacheElements();
    setupNavigation();
    setupRecordingControls();
    setupReplayControls();
    setupVisionControls();
    setupSettingsTabs();
    setupConfigPanel();
    loadConfig();
    startPolling();
    loadSessions();
    initFieldCanvas();
}

function cacheElements() {
    // Navigation
    elements.navItems = document.querySelectorAll('.nav-item');
    elements.pages = document.querySelectorAll('.page');
    
    // Overview
    elements.uptime = document.getElementById('uptime');
    elements.robotX = document.getElementById('robot-x');
    elements.robotY = document.getElementById('robot-y');
    elements.robotYaw = document.getElementById('robot-yaw');
    elements.hasBall = document.getElementById('has-ball');
    elements.ballX = document.getElementById('ball-x');
    elements.ballY = document.getElementById('ball-y');
    elements.recIndicator = document.getElementById('rec-indicator');
    elements.recStatus = document.getElementById('rec-status');
    elements.recFrames = document.getElementById('rec-frames');
    elements.quickRecordBtn = document.getElementById('quick-record-btn');
    elements.fps = document.getElementById('fps');
    elements.latency = document.getElementById('latency');
    
    // Recording
    elements.sessionId = document.getElementById('session-id');
    elements.recordFreq = document.getElementById('record-freq');
    elements.sessionDesc = document.getElementById('session-desc');
    elements.startRecordBtn = document.getElementById('start-record-btn');
    elements.stopRecordBtn = document.getElementById('stop-record-btn');
    elements.sessionsTbody = document.getElementById('sessions-tbody');
    
    // Replay
    elements.replaySessionSelect = document.getElementById('replay-session-select');
    elements.replayPlay = document.getElementById('replay-play');
    elements.replayPrev = document.getElementById('replay-prev');
    elements.replayNext = document.getElementById('replay-next');
    elements.replayLoop = document.getElementById('replay-loop');
    elements.replaySpeed = document.getElementById('replay-speed');
    elements.speedValue = document.getElementById('speed-value');
    elements.replayProgress = document.getElementById('replay-progress');
    elements.currentTime = document.getElementById('current-time');
    elements.totalTime = document.getElementById('total-time');
    elements.fieldCanvas = document.getElementById('field-canvas');
    elements.frameNum = document.getElementById('frame-num');
    elements.frameTs = document.getElementById('frame-ts');
    elements.framePos = document.getElementById('frame-pos');
    elements.frameBall = document.getElementById('frame-ball');
    
    // Vision
    elements.cameraFeed = document.getElementById('camera-feed');
    elements.bboxOverlay = document.getElementById('bbox-overlay');
    elements.detectionList = document.getElementById('detection-list');
    elements.camBtns = document.querySelectorAll('.cam-btn');
}

// Navigation
function setupNavigation() {
    elements.navItems.forEach(item => {
        item.addEventListener('click', () => {
            const page = item.dataset.page;
            navigateTo(page);
        });
    });
}

function navigateTo(page) {
    state.currentPage = page;
    
    elements.navItems.forEach(item => {
        item.classList.toggle('active', item.dataset.page === page);
    });
    
    elements.pages.forEach(p => {
        p.classList.toggle('active', p.id === `page-${page}`);
    });
    
    // Page-specific initialization
    if (page === 'replay') {
        loadReplaySessions();
    }
}

// Vision Controls
function setupVisionControls() {
    elements.camBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            const cam = parseInt(btn.dataset.cam);
            selectCamera(cam);
        });
    });
}

function selectCamera(camIndex) {
    state.currentCam = camIndex;
    elements.camBtns.forEach(btn => {
        btn.classList.toggle('active', parseInt(btn.dataset.cam) === camIndex);
    });
    updateCameraFeed();
}

function updateCameraFeed() {
    elements.cameraFeed.src = `/api/cam/${state.currentCam}.jpg?t=${Date.now()}`;
}

// Recording Controls
function setupRecordingControls() {
    elements.startRecordBtn.addEventListener('click', startRecording);
    elements.stopRecordBtn.addEventListener('click', stopRecording);
    elements.quickRecordBtn.addEventListener('click', quickToggleRecording);
}

async function startRecording() {
    const sessionId = elements.sessionId.value || null;
    const freq = parseInt(elements.recordFreq.value) || 5;
    const desc = elements.sessionDesc.value || '';
    
    try {
        const response = await fetch('/api/recording/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sessionId, freq, description: desc })
        });
        
        const data = await response.json();
        
        if (data.success) {
            state.isRecording = true;
            state.recordingSessionId = data.sessionId;
            state.recordingFrameCount = 0;
            updateRecordingUI(true);
        } else {
            alert('启动记录失败: ' + data.error);
        }
    } catch (e) {
        console.error('Failed to start recording:', e);
        alert('启动记录失败');
    }
}

async function stopRecording() {
    try {
        const response = await fetch('/api/recording/stop', {
            method: 'POST'
        });
        
        const data = await response.json();
        
        if (data.success) {
            state.isRecording = false;
            updateRecordingUI(false);
            loadSessions();
        }
    } catch (e) {
        console.error('Failed to stop recording:', e);
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
    elements.quickRecordBtn.textContent = isRecording ? '⏹ 停止记录' : '▶ 开始记录';
    elements.quickRecordBtn.classList.toggle('btn-danger', isRecording);
    elements.quickRecordBtn.classList.toggle('btn-primary', !isRecording);
    
    elements.recIndicator.classList.toggle('active', isRecording);
    elements.recStatus.textContent = isRecording ? `记录中: ${state.recordingSessionId}` : '未记录';
}

// Sessions
async function loadSessions() {
    try {
        const response = await fetch('/api/sessions');
        const sessions = await response.json();
        renderSessionsTable(sessions);
        updateReplaySessionSelect(sessions);
    } catch (e) {
        console.error('Failed to load sessions:', e);
    }
}

function renderSessionsTable(sessions) {
    if (!sessions || sessions.length === 0) {
        elements.sessionsTbody.innerHTML = '<tr><td colspan="6" style="text-align:center;color:var(--text-muted)">暂无记录</td></tr>';
        return;
    }
    
    elements.sessionsTbody.innerHTML = sessions.map(s => `
        <tr>
            <td><code>${s.session_id}</code></td>
            <td>${formatDateTime(s.start_time)}</td>
            <td>${formatDuration(s.duration)}</td>
            <td>${s.frame_count}</td>
            <td>${s.description || '-'}</td>
            <td>
                <button class="btn btn-secondary" onclick="replaySession('${s.session_id}')">▶ 回放</button>
                <button class="btn btn-secondary" onclick="exportSession('${s.session_id}')">导出</button>
                <button class="btn btn-danger" onclick="deleteSession('${s.session_id}')">删除</button>
            </td>
        </tr>
    `).join('');
}

function updateReplaySessionSelect(sessions) {
    const select = elements.replaySessionSelect;
    select.innerHTML = '<option value="">-- 选择会话 --</option>' +
        sessions.map(s => `<option value="${s.session_id}">${s.session_id} (${s.frame_count} 帧)</option>`).join('');
}

// Replay Controls
function setupReplayControls() {
    elements.replaySessionSelect.addEventListener('change', (e) => {
        loadReplayData(e.target.value);
    });
    
    elements.replayPlay.addEventListener('click', toggleReplayPlay);
    elements.replayPrev.addEventListener('click', () => stepReplay(-1));
    elements.replayNext.addEventListener('click', () => stepReplay(1));
    elements.replayLoop.addEventListener('click', toggleReplayLoop);
    
    elements.replaySpeed.addEventListener('input', (e) => {
        state.replaySpeed = parseFloat(e.target.value);
        elements.speedValue.textContent = state.replaySpeed.toFixed(1) + 'x';
    });
    
    elements.replayProgress.addEventListener('input', (e) => {
        const frameIdx = Math.floor((e.target.value / 100) * (state.replayFrames.length - 1));
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
            const duration = state.replayFrames[state.replayFrames.length - 1].tick - 
                           state.replayFrames[0].tick;
            elements.totalTime.textContent = formatTime(duration);
        }
        
        renderReplayFrame();
    } catch (e) {
        console.error('Failed to load replay data:', e);
    }
}

function toggleReplayPlay() {
    if (state.replayIsPlaying) {
        state.replayIsPlaying = false;
        elements.replayPlay.textContent = '▶';
    } else {
        state.replayIsPlaying = true;
        elements.replayPlay.textContent = '⏸';
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
            elements.replayPlay.textContent = '▶';
            return;
        }
    }
    
    // Calculate delay based on frame timestamps and speed
    const currentFrame = state.replayFrames[state.replayCurrentFrame];
    const nextFrame = state.replayFrames[state.replayCurrentFrame + 1];
    let delay = 50;  // Default 50ms
    
    if (nextFrame && currentFrame) {
        delay = (nextFrame.tick - currentFrame.tick) * 1000 / state.replaySpeed;
        delay = Math.max(10, Math.min(delay, 500));  // Clamp between 10-500ms
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
    elements.replayLoop.classList.toggle('btn-primary', state.replayLoop);
}

function renderReplayFrame() {
    const frame = state.replayFrames[state.replayCurrentFrame];
    
    if (!frame) {
        clearFieldCanvas();
        elements.frameNum.textContent = '--';
        elements.frameTs.textContent = '--';
        elements.framePos.textContent = '--';
        elements.frameBall.textContent = '--';
        elements.currentTime.textContent = '00:00';
        elements.replayProgress.value = 0;
        return;
    }
    
    // Update frame info
    elements.frameNum.textContent = `${state.replayCurrentFrame + 1} / ${state.replayFrames.length}`;
    elements.frameTs.textContent = new Date(frame.tick * 1000).toLocaleTimeString();
    elements.framePos.textContent = `(${frame.x_cm.toFixed(1)}, ${frame.y_cm.toFixed(1)}) @ ${frame.yaw_deg.toFixed(1)}°`;
    elements.frameBall.textContent = `(${frame.ball_x_cm.toFixed(1)}, ${frame.ball_y_cm.toFixed(1)})`;
    
    // Update progress
    const progress = (state.replayCurrentFrame / Math.max(1, state.replayFrames.length - 1)) * 100;
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
    const ctx = canvas.getContext('2d');
    
    // Set canvas size
    canvas.width = 520;
    canvas.height = 400;
    
    clearFieldCanvas();
}

function clearFieldCanvas() {
    const canvas = elements.fieldCanvas;
    const ctx = canvas.getContext('2d');
    
    // Green field
    ctx.fillStyle = '#1a472a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Field lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
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
    const ctx = canvas.getContext('2d');
    
    // Clear
    clearFieldCanvas();
    
    // Scale factors (field is 260x200 cm, canvas is 520x400)
    const scaleX = (canvas.width - 40) / state.fieldWidth;
    const scaleY = (canvas.height - 40) / state.fieldHeight;
    
    const toCanvasX = (x) => canvas.width / 2 + x * scaleX;
    const toCanvasY = (y) => canvas.height / 2 - y * scaleY;  // Y is inverted
    
    // Draw ball
    if (frame.ball_x_cm !== 0 || frame.ball_y_cm !== 0) {
        const ballX = toCanvasX(frame.x_cm + frame.ball_x_cm);
        const ballY = toCanvasY(frame.y_cm + frame.ball_y_cm);
        
        ctx.fillStyle = '#ff6b6b';
        ctx.beginPath();
        ctx.arc(ballX, ballY, 8, 0, Math.PI * 2);
        ctx.fill();
    }
    
    // Draw robot
    const robotX = toCanvasX(frame.x_cm);
    const robotY = toCanvasY(frame.y_cm);
    
    // Robot body
    ctx.fillStyle = frame.has_ball ? '#22c55e' : '#3b82f6';
    ctx.beginPath();
    ctx.arc(robotX, robotY, 15, 0, Math.PI * 2);
    ctx.fill();
    
    // Direction indicator
    const yawRad = frame.yaw_deg * Math.PI / 180;
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(robotX + Math.cos(yawRad) * 20, robotY - Math.sin(yawRad) * 20);
    ctx.stroke();
    
    // Robot ID
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(frame.robot_id, robotX, robotY);
}

// Polling
function startPolling() {
    setInterval(updateOverview, CONFIG.updateInterval);
}

async function updateOverview() {
    try {
        // Fetch vision state
        const visionResponse = await fetch('/api/state');
        const visionData = await visionResponse.json();
        
        // Update robot status
        if (visionData.chassis_list_cm && visionData.chassis_list_cm.length >= 2) {
            elements.robotX.textContent = visionData.chassis_list_cm[0].toFixed(1) + ' cm';
            elements.robotY.textContent = visionData.chassis_list_cm[1].toFixed(1) + ' cm';
        }
        
        if (visionData.ball_pos_cm) {
            elements.ballX.textContent = visionData.ball_pos_cm[0] ? visionData.ball_pos_cm[0].toFixed(1) + ' cm' : '-- cm';
            elements.ballY.textContent = visionData.ball_pos_cm[1] ? visionData.ball_pos_cm[1].toFixed(1) + ' cm' : '-- cm';
        }
        
        // Update uptime
        if (visionData.uptime_s) {
            elements.uptime.textContent = formatDuration(visionData.uptime_s);
        }
        
        // Fetch recording state
        try {
            const recResponse = await fetch('/api/recording/status');
            const recData = await recResponse.json();
            
            state.isRecording = recData.is_recording;
            state.recordingSessionId = recData.session_id;
            state.recordingFrameCount = recData.frame_count || 0;
            
            updateRecordingUI(state.isRecording);
            elements.recFrames.textContent = state.recordingFrameCount + ' 帧';
            elements.recStatus.textContent = state.isRecording ? `记录中: ${state.recordingSessionId}` : '未记录';
        } catch (e) {
            // Recording API not available
        }
        
    } catch (e) {
        console.error('Polling error:', e);
    }
}

// Helper functions
function formatDateTime(timestamp) {
    if (!timestamp) return '--';
    return new Date(timestamp * 1000).toLocaleString('zh-CN');
}

function formatDuration(seconds) {
    if (!seconds) return '0:00';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
}

function formatTime(seconds) {
    if (!seconds || seconds < 0) return '00:00';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

// Global functions for inline handlers
window.replaySession = function(sessionId) {
    navigateTo('replay');
    elements.replaySessionSelect.value = sessionId;
    loadReplayData(sessionId);
};

window.exportSession = async function(sessionId) {
    try {
        const response = await fetch(`/api/sessions/${sessionId}/export`);
        const blob = await response.blob();
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `replay_${sessionId}.json`;
        a.click();
        URL.revokeObjectURL(url);
    } catch (e) {
        alert('导出失败: ' + e.message);
    }
};

window.deleteSession = async function(sessionId) {
    if (!confirm(`确定要删除会话 ${sessionId} 吗？`)) return;
    
    try {
        const response = await fetch(`/api/sessions/${sessionId}`, {
            method: 'DELETE'
        });
        const data = await response.json();
        
        if (data.success) {
            loadSessions();
        } else {
            alert('删除失败: ' + data.error);
        }
    } catch (e) {
        alert('删除失败: ' + e.message);
    }
};

// Settings Tabs
function setupSettingsTabs() {
    const tabBtns = document.querySelectorAll('.tab-btn');
    tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            const tabId = btn.dataset.tab;
            switchSettingsTab(tabId);
        });
    });
}

function switchSettingsTab(tabId) {
    // Update button states
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.tab === tabId);
    });
    
    // Update content visibility
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.toggle('active', content.id === `tab-${tabId}`);
    });
}

// Configuration Panel
function setupConfigPanel() {
    const saveBtn = document.getElementById('save-config-btn');
    if (saveBtn) {
        saveBtn.addEventListener('click', saveConfig);
    }
}

async function loadConfig() {
    try {
        const response = await fetch('/api/config');
        const config = await response.json();
        state.config = config;
        populateConfigForm(config);
        updateSystemInfoBar(config);
    } catch (e) {
        console.error('Failed to load config:', e);
    }
}

function populateConfigForm(config) {
    if (!config) return;
    
    // Robot info
    setFieldValue('robot-number', config.RoboInfo?.number);
    setFieldValue('robot-type', config.RoboInfo?.type);
    setFieldValue('robot-bit', config.RoboInfo?.Bit);
    setFieldValue('robot-rec', config.RoboInfo?.Rec);
    
    // Ports
    setFieldValue('lidar-id', config.Ports?.LidarID);
    setFieldValue('iceloong-port', config.Ports?.IceLoongPort);
    setFieldValue('motor-lf', config.Ports?.LeftFront);
    setFieldValue('motor-lb', config.Ports?.LeftBack);
    setFieldValue('motor-rb', config.Ports?.RightBack);
    setFieldValue('motor-rf', config.Ports?.RightFront);
    setFieldValue('motor-dribble', config.Ports?.Dribble);
    setFieldValue('motor-magnet', config.Ports?.ElecMagnet);
    
    // Simulator
    setFieldValue('sim-unity-ip', config.Sim?.UnityIP);
    setFieldValue('sim-robot-id', config.Sim?.RobotId);
    setFieldValue('sim-cmd-port', config.Sim?.CmdPort);
    setFieldValue('sim-status-port', config.Sim?.StatusPort);
    setFieldValue('sim-wheel-v', config.Sim?.WheelToV);
    setFieldValue('sim-wheel-wz', config.Sim?.WheelToWz);
    setFieldValue('sim-kick', config.Sim?.KickStrength);
    setFieldValue('sim-dribble', config.Sim?.DribbleStrengthScale);
    setFieldValue('sim-move-scale', config.Sim?.PlanarMoveSpeedScale);
    setFieldValue('sim-unity-cm', config.Sim?.UnityToCm);
    setFieldValue('sim-yaw-offset', config.Sim?.YawOffsetDeg);
    setFieldValue('sim-yaw-zero-start', config.Sim?.YawZeroAtStart, 'checkbox');
    setFieldValue('sim-port-base', config.Sim?.CmdPortBase);
    setFieldValue('sim-port-stride', config.Sim?.PortStride);
    setFieldValue('sim-multi-mode', config.Sim?.MultiRobotMode);
    
    // Vision
    setFieldValue('cam-front', config.VisionVals?.Front);
    setFieldValue('cam-right', config.VisionVals?.Right);
    setFieldValue('cam-back', config.VisionVals?.Back);
    setFieldValue('cam-left', config.VisionVals?.Left);
    setFieldValue('cam-expose', config.VisionVals?.ExposeVal);
    setFieldValue('cam-auto-expose', config.VisionVals?.AutoExpose === 1, 'checkbox');
    setFieldValue('vision-hailo-model', config.VisionVals?.HailoModelName);
    setFieldValue('vision-tuna-model', config.VisionVals?.TunaModelName);
    setFieldValue('vision-tuna-engine', config.VisionVals?.TunaEnginePath);
    setFieldValue('obj-ball-dia', config.VisionVals?.BallDiameterCM);
    setFieldValue('obj-chassis-w', config.VisionVals?.ChassisWidthCM);
    setFieldValue('obj-chassis-h', config.VisionVals?.ChassisHeightCM);
    setFieldValue('obj-blue-goal-w', config.VisionVals?.BlueGoalWidthCM);
    setFieldValue('obj-blue-goal-h', config.VisionVals?.BlueGoalHeightCM);
    setFieldValue('obj-yellow-goal-w', config.VisionVals?.YellowGoalWidthCM);
    setFieldValue('obj-yellow-goal-h', config.VisionVals?.YellowGoalHeightCM);
    setFieldValue('vision-bbox-dist', config.VisionVals?.EnableBBoxDistanceEstimate, 'checkbox');
    
    // Network
    setFieldValue('trans-type', config.Transimission?.Type);
    setFieldValue('trans-method', config.Transimission?.Method);
    setFieldValue('wifi-self-ip', config.WIFI?.SelfIP);
    setFieldValue('wifi-remote-ip', config.WIFI?.RemoteIP);
    setFieldValue('wifi-port', config.WIFI?.Port);
    setFieldValue('wifi-ssid', config.WIFI?.SSID);
    setFieldValue('wifi-pwd', config.WIFI?.PWD);
    
    // Data/Recording
    setFieldValue('rec-record', config.VisionVals?.Record, 'checkbox');
    setFieldValue('rec-dir', config.VisionVals?.RecordDir);
    setFieldValue('rec-width', config.VisionVals?.RecordWidth);
    setFieldValue('rec-height', config.VisionVals?.RecordHeight);
    setFieldValue('rec-fps', config.VisionVals?.RecordFPS);
    setFieldValue('rec-segment', config.VisionVals?.RecordSegmentSeconds);
    setFieldValue('rec-codec', config.VisionVals?.RecordCodec);
    setFieldValue('rec-ext', config.VisionVals?.RecordExt);
    
    // Field
    setFieldValue('field-short', config.Bounds?.Short);
    setFieldValue('field-long', config.Bounds?.Long);
    setFieldValue('field-restrict-x', config.Bounds?.restricted_area_x);
    setFieldValue('field-restrict-y', config.Bounds?.restricted_area_y);
    
    // Home
    const home = config.CheckPoints?.Home || [0, 0, 0];
    setFieldValue('home-x', home[0]);
    setFieldValue('home-y', home[1]);
    setFieldValue('home-theta', home[2]);
    
    // Debug
    setFieldValue('debug-full-log', config.Debug?.FullLog, 'checkbox');
    setFieldValue('debug-beta', config.Debug?.Beta, 'checkbox');
    
    // Motion recording (for replay)
    setFieldValue('debug-record-motion', config.Debug?.RecordMotion, 'checkbox');
    setFieldValue('debug-record-interval', config.Debug?.RecordInterval);
    setFieldValue('debug-record-session', config.Debug?.RecordSessionId);
}

function setFieldValue(id, value, type = 'input') {
    const el = document.getElementById(id);
    if (!el) return;
    
    if (type === 'checkbox') {
        el.checked = Boolean(value);
    } else {
        el.value = value ?? '';
    }
}

function getFieldValue(id, type = 'input') {
    const el = document.getElementById(id);
    if (!el) return null;
    
    if (type === 'checkbox') {
        return el.checked;
    } else if (el.type === 'number') {
        return parseFloat(el.value) || 0;
    } else {
        return el.value;
    }
}

function updateSystemInfoBar(config) {
    // Local IP
    const localIPEl = document.getElementById('local-ip');
    if (localIPEl) {
        localIPEl.textContent = config.WIFI?.SelfIP || '--';
    }
    
    // Unity IP
    const unityIPEl = document.getElementById('unity-ip-display');
    if (unityIPEl) {
        unityIPEl.textContent = config.Sim?.UnityIP || '--';
    }
    
    // Robot info
    const robotInfoEl = document.getElementById('robot-info');
    if (robotInfoEl && config.RoboInfo) {
        robotInfoEl.textContent = `#${config.RoboInfo.number} ${config.RoboInfo.type}`;
    }
    
    // Connection mode
    const connModeEl = document.getElementById('connection-mode');
    if (connModeEl && config.Transimission) {
        connModeEl.textContent = `${config.Transimission.Type} / ${config.Transimission.Method}`;
    }
}

async function saveConfig() {
    const config = state.config || {};
    
    // Collect form values
    config.RoboInfo = {
        number: getFieldValue('robot-number'),
        type: getFieldValue('robot-type'),
        Bit: getFieldValue('robot-bit'),
        Rec: getFieldValue('robot-rec'),
        LidarType: config.RoboInfo?.LidarType || 's1'
    };
    
    config.Ports = {
        LidarID: getFieldValue('lidar-id'),
        IceLoongPort: getFieldValue('iceloong-port'),
        LowTigger: config.Ports?.LowTigger || 0,
        ReferPort: config.Ports?.ReferPort || 1,
        ElecMagnet: getFieldValue('motor-magnet'),
        Dribble: getFieldValue('motor-dribble'),
        LeftFront: getFieldValue('motor-lf'),
        LeftBack: getFieldValue('motor-lb'),
        RightBack: getFieldValue('motor-rb'),
        RightFront: getFieldValue('motor-rf')
    };
    
    config.Sim = {
        UnityIP: getFieldValue('sim-unity-ip'),
        CmdPort: getFieldValue('sim-cmd-port'),
        StatusPort: getFieldValue('sim-status-port'),
        RobotId: getFieldValue('sim-robot-id'),
        TwistFromWheels: true,
        WheelToV: getFieldValue('sim-wheel-v'),
        WheelToWz: getFieldValue('sim-wheel-wz'),
        YawOffsetDeg: getFieldValue('sim-yaw-offset'),
        YawZeroAtStart: getFieldValue('sim-yaw-zero-start', 'checkbox'),
        KickStrength: getFieldValue('sim-kick'),
        DribbleStrengthScale: getFieldValue('sim-dribble'),
        PlanarMoveSpeedScale: getFieldValue('sim-move-scale'),
        UnityToCm: getFieldValue('sim-unity-cm'),
        OriginXCM: config.Sim?.OriginXCM || 0.0,
        OriginYCM: config.Sim?.OriginYCM || 0.0,
        PoseScale: config.Sim?.PoseScale || 1.0,
        PoseOffsetXCM: config.Sim?.PoseOffsetXCM || 0.0,
        PoseOffsetYCM: config.Sim?.PoseOffsetYCM || 0.0,
        Debug: config.Sim?.Debug || false,
        CmdPortBase: getFieldValue('sim-port-base'),
        StatusPortBase: config.Sim?.StatusPortBase || 20002,
        PortStride: getFieldValue('sim-port-stride'),
        MultiRobotMode: getFieldValue('sim-multi-mode')
    };
    
    config.VisionVals = config.VisionVals || {};
    config.VisionVals.Front = getFieldValue('cam-front');
    config.VisionVals.Right = getFieldValue('cam-right');
    config.VisionVals.Back = getFieldValue('cam-back');
    config.VisionVals.Left = getFieldValue('cam-left');
    config.VisionVals.ExposeVal = getFieldValue('cam-expose');
    config.VisionVals.AutoExpose = getFieldValue('cam-auto-expose', 'checkbox') ? 1 : 0;
    config.VisionVals.HailoModelName = getFieldValue('vision-hailo-model');
    config.VisionVals.TunaModelName = getFieldValue('vision-tuna-model');
    config.VisionVals.TunaEnginePath = getFieldValue('vision-tuna-engine');
    config.VisionVals.BallDiameterCM = getFieldValue('obj-ball-dia');
    config.VisionVals.ChassisWidthCM = getFieldValue('obj-chassis-w');
    config.VisionVals.ChassisHeightCM = getFieldValue('obj-chassis-h');
    config.VisionVals.BlueGoalWidthCM = getFieldValue('obj-blue-goal-w');
    config.VisionVals.BlueGoalHeightCM = getFieldValue('obj-blue-goal-h');
    config.VisionVals.YellowGoalWidthCM = getFieldValue('obj-yellow-goal-w');
    config.VisionVals.YellowGoalHeightCM = getFieldValue('obj-yellow-goal-h');
    config.VisionVals.EnableBBoxDistanceEstimate = getFieldValue('vision-bbox-dist', 'checkbox');
    config.VisionVals.Record = getFieldValue('rec-record', 'checkbox');
    config.VisionVals.RecordDir = getFieldValue('rec-dir');
    config.VisionVals.RecordWidth = getFieldValue('rec-width');
    config.VisionVals.RecordHeight = getFieldValue('rec-height');
    config.VisionVals.RecordFPS = getFieldValue('rec-fps');
    config.VisionVals.RecordSegmentSeconds = getFieldValue('rec-segment');
    config.VisionVals.RecordCodec = getFieldValue('rec-codec');
    config.VisionVals.RecordExt = getFieldValue('rec-ext');
    
    config.Transimission = {
        Type: getFieldValue('trans-type'),
        Method: getFieldValue('trans-method')
    };
    
    config.WIFI = {
        SelfIP: getFieldValue('wifi-self-ip'),
        RemoteIP: getFieldValue('wifi-remote-ip'),
        Port: getFieldValue('wifi-port'),
        SSID: getFieldValue('wifi-ssid'),
        PWD: getFieldValue('wifi-pwd')
    };
    
    config.Bounds = {
        ClsPos: config.Bounds?.ClsPos || [85, 35, 0],
        FarPos: config.Bounds?.FarPos || [95, 75, 0],
        Short: getFieldValue('field-short'),
        Long: getFieldValue('field-long'),
        Distance2Bound: config.Bounds?.Distance2Bound || 240,
        restricted_area_x: getFieldValue('field-restrict-x'),
        restricted_area_y: getFieldValue('field-restrict-y')
    };
    
    config.CheckPoints = {
        Home: [
            getFieldValue('home-x'),
            getFieldValue('home-y'),
            getFieldValue('home-theta')
        ]
    };
    
    config.Debug = {
        FullLog: getFieldValue('debug-full-log', 'checkbox'),
        Beta: getFieldValue('debug-beta', 'checkbox'),
        RecordMotion: getFieldValue('debug-record-motion', 'checkbox'),
        RecordInterval: getFieldValue('debug-record-interval'),
        RecordSessionId: getFieldValue('debug-record-session') || null
    };
    
    try {
        const response = await fetch('/api/config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });
        
        const result = await response.json();
        
        if (result.success) {
            state.config = config;
            updateSystemInfoBar(config);
            alert('配置已保存');
        } else {
            alert('保存失败: ' + result.error);
        }
    } catch (e) {
        console.error('Failed to save config:', e);
        alert('保存失败: ' + e.message);
    }
}
