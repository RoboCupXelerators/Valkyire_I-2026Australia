const statusEl = document.getElementById('status');
const radar = document.getElementById('radar');
const rctx = radar.getContext('2d');

const ballPosEl = document.getElementById('ballPos');
const ballBBoxEl = document.getElementById('ballBBox');
const blueGoalPosEl = document.getElementById('blueGoalPos');
const yellowGoalPosEl = document.getElementById('yellowGoalPos');
const goalBBoxEl = document.getElementById('goalBBox');
const chassisBody = document.querySelector('#chassisTable tbody');
const countersEl = document.getElementById('counters');

const camImgs = [0,1,2,3].map(i => document.getElementById('cam'+i));
const overlays = [0,1,2,3].map(i => document.getElementById('ov'+i));

// Radar config
// Dynamic range: automatically zoom to fit recent detections.
// RANGE_MIN: minimal half-range (cm)
// RANGE_MAX: maximal half-range (cm)
const RANGE_MIN = 120;
const RANGE_MAX = 1200;
const RANGE_MARGIN = 1.25; // add some padding so targets don't stick to border

let rangeCm = 300; // current half-range in cm

function fmt(v, digits=2){
  if (v === null || v === undefined) return '-';
  if (typeof v === 'number') return v.toFixed(digits);
  return String(v);
}

function goalXY(goal){
  if (!goal || !Array.isArray(goal) || goal.length < 2) return null;
  return {x: goal[0], y: goal[1], conf: goal.length >= 3 ? goal[2] : null};
}

function _computeDesiredRange(state){
  // Fit all targets (ball + chassis) into view.
  let maxAbs = 0;
  const bp = state.ball_pos_cm;
  if (bp && bp.length >= 2 && !(bp[0]===0 && bp[1]===0)){
    maxAbs = Math.max(maxAbs, Math.abs(bp[0]), Math.abs(bp[1]));
  }
  const cl = state.chassis_list_cm || [];
  for (let i=0; i<cl.length; i++){
    const c = cl[i];
    if (!c || c.length < 2) continue;
    maxAbs = Math.max(maxAbs, Math.abs(c[0]), Math.abs(c[1]));
  }
  const gl = [goalXY(state.blue_goal_cm), goalXY(state.yellow_goal_cm)];
  for (let i=0; i<gl.length; i++){
    const g = gl[i];
    if (!g) continue;
    maxAbs = Math.max(maxAbs, Math.abs(g.x), Math.abs(g.y));
  }
  if (maxAbs <= 0) return rangeCm; // keep current when no targets
  let desired = Math.ceil(maxAbs * RANGE_MARGIN / 10) * 10; // round to 10cm
  desired = Math.max(RANGE_MIN, Math.min(RANGE_MAX, desired));
  return desired;
}

function drawRadar(state){
  const w = radar.width, h = radar.height;
  rctx.clearRect(0,0,w,h);

  // background
  rctx.fillStyle = '#0c1430';
  rctx.fillRect(0,0,w,h);

  const cx = w/2, cy = h/2;
  // smooth zoom (avoid jitter)
  const desired = _computeDesiredRange(state);
  rangeCm = rangeCm * 0.92 + desired * 0.08;
  const scale = (Math.min(w,h) * 0.45) / rangeCm;

  // grid circles
  rctx.strokeStyle = 'rgba(76,201,240,0.18)';
  rctx.lineWidth = 1;
  // choose grid step based on range
  let step = 50;
  if (rangeCm > 600) step = 100;
  if (rangeCm > 900) step = 200;
  const maxR = Math.max(step, Math.floor(rangeCm / step) * step);
  for (let r=step; r<=maxR; r+=step){
    rctx.beginPath();
    rctx.arc(cx, cy, r*scale, 0, Math.PI*2);
    rctx.stroke();
  }
  // axes
  rctx.strokeStyle = 'rgba(233,238,252,0.15)';
  rctx.beginPath(); rctx.moveTo(0, cy); rctx.lineTo(w, cy); rctx.stroke();
  rctx.beginPath(); rctx.moveTo(cx, 0); rctx.lineTo(cx, h); rctx.stroke();

  // ego vehicle at center
  rctx.fillStyle = 'rgba(58,238,159,0.85)';
  rctx.beginPath();
  rctx.arc(cx, cy, 6, 0, Math.PI*2);
  rctx.fill();
  rctx.fillStyle = 'rgba(58,238,159,0.7)';
  rctx.fillText('EGO', cx+10, cy-10);

  // range label
  rctx.fillStyle = 'rgba(159,176,214,0.85)';
  rctx.fillText(`±${Math.round(rangeCm)}cm`, 10, 18);

  // helper: cm(x forward, y left) -> canvas(x right, y down)
  function toCanvas(x_cm, y_cm){
    // IMPORTANT: our convention is Y_left is positive.
    // On canvas, +x is to the right. Therefore, Y_left (+) should move to the LEFT.
    const x = cx - (y_cm * scale);
    const y = cy - (x_cm * scale);
    return [x,y];
  }

  // ball
  const bp = state.ball_pos_cm;
  if (bp && bp.length >= 2 && !(bp[0]===0 && bp[1]===0)){
    const [x,y] = toCanvas(bp[0], bp[1]);
    rctx.fillStyle = 'rgba(247,37,133,0.95)';
    rctx.beginPath();
    rctx.arc(x, y, 5, 0, Math.PI*2);
    rctx.fill();
    rctx.fillStyle = 'rgba(247,37,133,0.9)';
    rctx.fillText('BALL', x+8, y-8);
  }

  // chassis
  const cl = state.chassis_list_cm || [];
  rctx.fillStyle = 'rgba(76,201,240,0.95)';
  for (let i=0; i<cl.length; i++){
    const c = cl[i];
    if (!c || c.length < 5) continue;
    const x_cm = c[0], y_cm = c[1];
    const [x,y] = toCanvas(x_cm, y_cm);
    rctx.beginPath();
    rctx.rect(x-4, y-4, 8, 8);
    rctx.fill();
  }

  // goals (if provided)
  const blueGoal = goalXY(state.blue_goal_cm);
  const yellowGoal = goalXY(state.yellow_goal_cm);
  const goals = [
    ['blue', blueGoal],
    ['yellow', yellowGoal],
  ];
  for (const [kind, g] of goals){
    if (!g) continue;
    const [x,y] = toCanvas(g.x, g.y);
    if (kind === 'blue'){
      rctx.strokeStyle = 'rgba(59,130,246,0.95)';
      rctx.fillStyle = 'rgba(59,130,246,0.65)';
    }else{
      rctx.strokeStyle = 'rgba(250,204,21,0.95)';
      rctx.fillStyle = 'rgba(250,204,21,0.6)';
    }
    rctx.lineWidth = 2;
    rctx.beginPath();
    rctx.arc(x, y, 7, 0, Math.PI*2);
    rctx.fill();
    rctx.stroke();
    rctx.fillStyle = 'rgba(233,238,252,0.85)';
    rctx.fillText(`${kind.toUpperCase()} ${Number(g.conf||0).toFixed(2)}`, x+8, y+12);
  }
}

function updateTable(state){
  // ball
  const bp = state.ball_pos_cm;
  if (bp && bp.length >= 2){
    ballPosEl.textContent = `x=${fmt(bp[0])}, y=${fmt(bp[1])}`;
  } else {
    ballPosEl.textContent = '-';
  }
  const bb = state.ball_bbox_640;
  ballBBoxEl.textContent = bb ? `cam=${bb[0]} [${bb[1]},${bb[2]},${bb[3]},${bb[4]}] conf=${fmt(bb[5],3)}` : '-';

  // goals
  const blueGoal = goalXY(state.blue_goal_cm);
  const yellowGoal = goalXY(state.yellow_goal_cm);
  blueGoalPosEl.textContent = blueGoal
    ? `x=${fmt(blueGoal.x)}, y=${fmt(blueGoal.y)}, conf=${fmt(blueGoal.conf, 2)}`
    : '-';
  yellowGoalPosEl.textContent = yellowGoal
    ? `x=${fmt(yellowGoal.x)}, y=${fmt(yellowGoal.y)}, conf=${fmt(yellowGoal.conf, 2)}`
    : '-';

  const gb = state.goal_bbox_640;
  goalBBoxEl.textContent = gb
    ? `cam=${gb[0]} cls=${gb[1]} [${gb[2]},${gb[3]},${gb[4]},${gb[5]}] conf=${fmt(gb[6],3)}`
    : '-';

  // chassis
  chassisBody.innerHTML = '';
  const cl = state.chassis_list_cm || [];
  for (let i=0; i<cl.length; i++){
    const c = cl[i];
    const tr = document.createElement('tr');
    const cells = [
      String(i),
      fmt(c[0]),
      fmt(c[1]),
      fmt(c[2],0),
      fmt(c[3],0),
      fmt(c[4],3),
    ];
    for (let j=0; j<cells.length; j++){
      const td = document.createElement('td');
      td.textContent = cells[j];
      tr.appendChild(td);
    }
    chassisBody.appendChild(tr);
  }

  // counters
  const cts = state.counters || {};
  const parts = [];
  for (const k of Object.keys(cts)) parts.push(`${k}=${cts[k]}`);
  countersEl.textContent = parts.join('   ');
}

function drawOverlays(state){
  const ballPerCam = state.ball_bboxes_640 || [null,null,null,null];
  const chassisPerCam = state.chassis_bboxes_640 || [null,null,null,null];
    const goalPerCam = state.goal_bboxes_640 || [null,null,null,null];
  for (let i=0; i<4; i++){
    const img = camImgs[i];
    const ov = overlays[i];
    const ctx = ov.getContext('2d');
    // match canvas to current displayed size
    const w = img.clientWidth || 0;
    const h = img.clientHeight || 0;
    if (w === 0 || h === 0){
      continue;
    }
    if (ov.width !== Math.round(w) || ov.height !== Math.round(h)){
      ov.width = Math.round(w);
      ov.height = Math.round(h);
    }
    ctx.clearRect(0,0,ov.width, ov.height);

    const sx = ov.width / 640.0;
    const sy = ov.height / 640.0;

    // ball bbox
    const bb = ballPerCam[i];
    if (bb){
      // bb: (cam,x1,y1,x2,y2,conf) in 640x640
      const x1 = bb[1], y1 = bb[2], x2 = bb[3], y2 = bb[4];
      const conf = bb[5];
      ctx.strokeStyle = 'rgba(247,37,133,0.95)';
      ctx.lineWidth = 3;
      ctx.strokeRect(x1*sx, y1*sy, (x2-x1)*sx, (y2-y1)*sy);
      ctx.fillStyle = 'rgba(247,37,133,0.95)';
      ctx.font = '14px ui-monospace, monospace';
      ctx.fillText(`ball ${conf.toFixed(2)}`, x1*sx+6, Math.max(16, y1*sy-6));
    }

    // chassis bbox
    const cb = chassisPerCam[i];
    if (cb){
      const x1 = cb[1], y1 = cb[2], x2 = cb[3], y2 = cb[4];
      const conf = cb[5];
      ctx.strokeStyle = 'rgba(76,201,240,0.95)';
      ctx.lineWidth = 2;
      ctx.strokeRect(x1*sx, y1*sy, (x2-x1)*sx, (y2-y1)*sy);
      ctx.fillStyle = 'rgba(76,201,240,0.9)';
      ctx.font = '14px ui-monospace, monospace';
      ctx.fillText(`chassis ${conf.toFixed(2)}`, x1*sx+6, Math.min(ov.height-4, y2*sy+16));
    }

    // goal bbox
    const gb = goalPerCam[i];
    if (gb){
      const clsId = gb[1];
      const x1 = gb[2], y1 = gb[3], x2 = gb[4], y2 = gb[5];
      const conf = gb[6];
      const isBlue = Number(clsId) === 1;
      if (isBlue){
        ctx.strokeStyle = 'rgba(59,130,246,0.98)';
        ctx.fillStyle = 'rgba(59,130,246,0.18)';
      }else if (Number(clsId) === 2){
        ctx.strokeStyle = 'rgba(250,204,21,0.98)';
        ctx.fillStyle = 'rgba(250,204,21,0.18)';
      }else{
        ctx.strokeStyle = 'rgba(167,139,250,0.98)';
        ctx.fillStyle = 'rgba(167,139,250,0.18)';
      }
      ctx.lineWidth = 3;
      ctx.strokeRect(x1*sx, y1*sy, (x2-x1)*sx, (y2-y1)*sy);
      ctx.fillRect(x1*sx, y1*sy, (x2-x1)*sx, (y2-y1)*sy);
      ctx.fillStyle = 'rgba(233,238,252,0.95)';
      ctx.font = '14px ui-monospace, monospace';
      const label = isBlue ? 'blue_goal' : (Number(clsId) === 2 ? 'yellow_goal' : 'goal');
      ctx.fillText(`${label} ${conf.toFixed(2)}`, x1*sx+6, Math.max(16, y1*sy-6));
    }
  }
}

let lastOkTs = 0;
async function tick(){
  try{
    const res = await fetch('/api/state', {cache:'no-store'});
    if (!res.ok) throw new Error('http '+res.status);
    const state = await res.json();
    lastOkTs = Date.now();
    statusEl.textContent = `ok  uptime=${fmt(state.uptime_s,1)}s  ts=${new Date(state.ts*1000).toLocaleTimeString()}`;
    statusEl.style.color = '#9fb0d6';
    drawRadar(state);
    updateTable(state);
    drawOverlays(state);
  }catch(e){
    const dt = (Date.now()-lastOkTs)/1000.0;
    statusEl.textContent = `offline (${fmt(dt,1)}s)  ${e}`;
    statusEl.style.color = '#fca5a5';
  }
}

function startCamStreams(){
  // force refresh with cache-buster; keep it cheap
  function refresh(){
    const t = Date.now();
    for (let i=0; i<4; i++){
      camImgs[i].src = `/api/cam/${i}.jpg?size=720&t=${t}`;
    }
  }
  refresh();
  setInterval(refresh, 120); // ~8 FPS
}

window.addEventListener('load', ()=>{
  startCamStreams();
  tick();
  setInterval(tick, 80); // ~12.5Hz state
});
