"use strict";

const PI = Math.PI;
const CELL = 180;
const HALF_CELL = 90;
const DT = 0.001;
const G = 9.80665;
const MEASURED_GRIP = {muStatic:0.235, muKinetic:0.205};

const MACHINE = {
  massKg: 0.682,
  pulsePerRev: 400.0,
  wheelDiameterMm: 52.8,
  treadMm: 81.9,
  tyreWidthMm: 6.0,
  minSpeedMmS: 30.0
};
MACHINE.mmPerPulse = MACHINE.wheelDiameterMm * PI / MACHINE.pulsePerRev;

const DEFAULT_PROFILES = [
  {
    speed_mm_s:200, entry_mm:10,
    left_entry_adjust_mm:0, right_entry_adjust_mm:0,
    left_exit_adjust_mm:0, right_exit_adjust_mm:0,
    left_right_entry_extra_mm:0, right_left_entry_extra_mm:0,
    left_right_lateral_mm:0,
    heading_gain:12, correction_max_deg_s:15,
    wall_kp:.20, wall_kd:.070, wall_filter_ms:4, wall_d_filter_ms:30,
    wall_slew_deg_s2:300, sense_advance_mm:8, sense_skip_mm:2,
    front_stable_ms:3
  },
  {
    speed_mm_s:300, entry_mm:10,
    left_entry_adjust_mm:0, right_entry_adjust_mm:0,
    left_exit_adjust_mm:0, right_exit_adjust_mm:0,
    left_right_entry_extra_mm:0, right_left_entry_extra_mm:0,
    left_right_lateral_mm:0,
    heading_gain:12, correction_max_deg_s:15,
    wall_kp:.20, wall_kd:.070, wall_filter_ms:4, wall_d_filter_ms:30,
    wall_slew_deg_s2:300, sense_advance_mm:8, sense_skip_mm:2,
    front_stable_ms:3
  },
  {
    speed_mm_s:400, entry_mm:10,
    left_entry_adjust_mm:-2, right_entry_adjust_mm:1,
    left_exit_adjust_mm:0, right_exit_adjust_mm:0,
    left_right_entry_extra_mm:2, right_left_entry_extra_mm:0,
    left_right_lateral_mm:0,
    heading_gain:12, correction_max_deg_s:15,
    wall_kp:.20, wall_kd:.070, wall_filter_ms:4, wall_d_filter_ms:30,
    wall_slew_deg_s2:300, sense_advance_mm:6, sense_skip_mm:2,
    front_stable_ms:3
  },
  {
    speed_mm_s:500, entry_mm:10,
    left_entry_adjust_mm:-2, right_entry_adjust_mm:3,
    left_exit_adjust_mm:5, right_exit_adjust_mm:-1,
    left_right_entry_extra_mm:5, right_left_entry_extra_mm:0,
    left_right_lateral_mm:10,
    heading_gain:12, correction_max_deg_s:15,
    wall_kp:.20, wall_kd:.070, wall_filter_ms:4, wall_d_filter_ms:30,
    wall_slew_deg_s2:300, sense_advance_mm:6, sense_skip_mm:2,
    front_stable_ms:3
  }
];

let PROFILES = DEFAULT_PROFILES.map(function(p){ return Object.assign({}, p); });

const els = {
  canvas: document.querySelector("#simCanvas"),
  profile: document.querySelector("#profile"),
  pattern: document.querySelector("#pattern"),
  playBtn: document.querySelector("#playBtn"),
  resetBtn: document.querySelector("#resetBtn"),
  muPreset: document.querySelector("#muPreset"),
  muStatic: document.querySelector("#muStatic"),
  muKinetic: document.querySelector("#muKinetic"),
  wheelValues: document.querySelector("#wheelValues"),
  codeParams: document.querySelector("#codeParams"),
  applyCodeBtn: document.querySelector("#applyCodeBtn"),
  resultValues: document.querySelector("#resultValues"),
  turnValues: document.querySelector("#turnValues"),
  physicsValues: document.querySelector("#physicsValues"),
  gripStatus: document.querySelector("#gripStatus"),
  configWarning: document.querySelector("#configWarning")
};

const ctx = els.canvas.getContext("2d");
let sim = null;
let raf = null;
let playing = false;
let playStart = 0;

const editableKeys = [
  ["speed_mm_s","speed_mm_s",10],
  ["entry_mm","entry_mm",0.5],
  ["left_entry_adjust_mm","left entry adjust",0.5],
  ["right_entry_adjust_mm","right entry adjust",0.5],
  ["left_exit_adjust_mm","left exit adjust",0.5],
  ["right_exit_adjust_mm","right exit adjust",0.5],
  ["left_right_entry_extra_mm","L→R entry extra",0.5],
  ["right_left_entry_extra_mm","R→L entry extra",0.5],
  ["left_right_lateral_mm","L→R lateral",0.5],
  ["sense_advance_mm","sense advance",0.5],
  ["sense_skip_mm","sense skip",0.5],
  ["front_stable_ms","front stable ms",1],
  ["heading_gain","heading gain",1],
  ["correction_max_deg_s","correction max °/s",1]
];

function deg2rad(d){ return d * PI / 180; }
function rad2deg(r){ return r * 180 / PI; }
function sinD(d){ return Math.sin(deg2rad(d)); }
function cosD(d){ return Math.cos(deg2rad(d)); }

function numberValue(el, fallback, min, max){
  let v = el.value === "" ? fallback : Number(el.value);
  if(!Number.isFinite(v)) v = fallback;
  if(Number.isFinite(min)) v = Math.max(min, v);
  if(Number.isFinite(max)) v = Math.min(max, v);
  el.value = String(v);
  return v;
}

function renderCodeInputs(){
  const p = PROFILES[Number(els.profile.value)];
  els.codeParams.innerHTML = "";
  editableKeys.forEach(function(row){
    const key = row[0], label = row[1], step = row[2];
    const lab = document.createElement("label");
    lab.textContent = label;
    const inp = document.createElement("input");
    inp.type = "number";
    inp.step = String(step);
    inp.value = String(p[key]);
    inp.dataset.key = key;
    lab.appendChild(inp);
    els.codeParams.appendChild(lab);
  });
}

function applyCodeInputs(){
  const p = PROFILES[Number(els.profile.value)];
  els.codeParams.querySelectorAll("input[data-key]").forEach(function(inp){
    const v = Number(inp.value);
    if(Number.isFinite(v)) p[inp.dataset.key] = v;
  });
  regenerate();
}

function curveProjection(bias){
  let forward = 0, side = 0;
  for(let i=0;i<128;i++){
    const p = PI * (i + 0.5) / 128;
    const sine = Math.sin(p);
    const theta = (PI / 4) * (1 - Math.cos(p)) + bias * sine * sine;
    forward += Math.cos(theta) / 128;
    side += Math.sin(theta) / 128;
  }
  return {forward:forward, side:side};
}

function normalArcMm(profile){
  let projection = 0;
  for(let i=0;i<128;i++){
    const u = (i + 0.5) / 128;
    const theta = (PI / 4) * (1 - Math.cos(PI * u));
    projection += Math.cos(theta) / 128;
  }
  return (HALF_CELL - profile.entry_mm) / projection;
}

function leftRightCurve(profile, baseArc){
  const shift = profile.left_right_lateral_mm;
  if(!(shift > 0)) return {arcMm:baseArc, biasRad:0, special:false};

  const sideMm = HALF_CELL - profile.entry_mm;
  if(!Number.isFinite(shift) || shift < 0 || shift > 15 || sideMm <= shift){
    throw new Error("left_right_lateral_mm が実コードの有効範囲外です。");
  }

  const targetRatio = (sideMm - shift) / sideMm;
  let low = 0, high = PI / 8;
  const limit = curveProjection(high);
  if(targetRatio < limit.forward / limit.side){
    throw new Error("left_right_lateral_mm が大きすぎて曲線を構成できません。");
  }

  for(let i=0;i<16;i++){
    const middle = (low + high) / 2;
    const value = curveProjection(middle);
    if(value.forward / value.side > targetRatio) low = middle;
    else high = middle;
  }
  const biasRad = (low + high) / 2;
  const value = curveProjection(biasRad);
  const arcMm = sideMm / value.side;
  if(!Number.isFinite(arcMm) || arcMm <= 0) throw new Error("左→右専用曲線の計算に失敗しました。");
  return {arcMm:arcMm, biasRad:biasRad, special:true};
}

function validateProfile(profile, pattern){
  const errors = [];
  if(!(profile.entry_mm >= 0 && profile.entry_mm < HALF_CELL)) errors.push("entry_mm は0以上90未満");
  if(!(profile.speed_mm_s >= MACHINE.minSpeedMmS)) errors.push("speed_mm_s は30以上");
  if(!(profile.sense_skip_mm >= 0 && profile.sense_advance_mm > profile.sense_skip_mm && profile.sense_advance_mm < profile.entry_mm)){
    errors.push("sense_skip < sense_advance < entry_mm が必要");
  }
  if(profile.left_right_entry_extra_mm < 0 || profile.left_right_entry_extra_mm > 5) errors.push("L→R entry extra は0～5");
  if(profile.right_left_entry_extra_mm < 0 || profile.right_left_entry_extra_mm > 5) errors.push("R→L entry extra は0～5");
  if(profile.left_right_lateral_mm < 0 || profile.left_right_lateral_mm > 15) errors.push("L→R lateral は0～15");

  const leftEntry = profile.entry_mm + profile.left_entry_adjust_mm;
  const rightEntry = profile.entry_mm + profile.right_entry_adjust_mm;
  const leftExit = profile.entry_mm + profile.left_exit_adjust_mm;
  const rightExit = profile.entry_mm + profile.right_exit_adjust_mm;

  if(!(leftEntry > profile.sense_advance_mm && leftEntry < HALF_CELL)) errors.push("左入口がsense_advanceより長く90未満である必要");
  if(!(rightEntry > profile.sense_advance_mm && rightEntry < HALF_CELL)) errors.push("右入口がsense_advanceより長く90未満である必要");
  if(!(leftExit >= 0 && leftExit < HALF_CELL)) errors.push("左出口は0以上90未満");
  if(!(rightExit >= 0 && rightExit < HALF_CELL)) errors.push("右出口は0以上90未満");

  if(pattern === "left_right" && !(rightEntry + profile.left_right_entry_extra_mm < HALF_CELL)){
    errors.push("左→右の右入口が90mm未満である必要");
  }
  if(pattern === "right_left" && !(leftEntry + profile.right_left_entry_extra_mm < HALF_CELL)){
    errors.push("右→左の左入口が90mm未満である必要");
  }
  return errors;
}

function record(state, phase, speedMmS, omegaRadS, out){
  out.push({
    x:state.x,
    y:state.y,
    headingDeg:state.headingDeg,
    t:state.t,
    phase:phase,
    speedMmS:speedMmS,
    omegaRadS:omegaRadS
  });
}

function moveStraight(state, distanceMm, phase, profile, rampMode, out){
  if(distanceMm <= 0) return;
  let moved = 0;
  const baseStep = Math.max(0.1, Math.min(0.5, profile.speed_mm_s * DT));
  while(moved < distanceMm - 1e-9){
    const ds = Math.min(baseStep, distanceMm - moved);
    const fraction = distanceMm > 0 ? (moved + ds * 0.5) / distanceMm : 1;
    let speed = profile.speed_mm_s;
    if(rampMode === 1){
      const low2 = MACHINE.minSpeedMmS * MACHINE.minSpeedMmS;
      const high2 = profile.speed_mm_s * profile.speed_mm_s;
      speed = Math.sqrt(low2 + (high2 - low2) * fraction);
    }else if(rampMode === -1){
      const low2 = MACHINE.minSpeedMmS * MACHINE.minSpeedMmS;
      const high2 = profile.speed_mm_s * profile.speed_mm_s;
      speed = Math.sqrt(low2 + (high2 - low2) * (1 - fraction));
    }
    state.x += ds * sinD(state.headingDeg);
    state.y += ds * cosD(state.headingDeg);
    state.t += ds / Math.max(speed, 1e-6);
    moved += ds;
    record(state, phase, speed, 0, out);
  }
}

function curveTheta(u, biasRad){
  const sine = Math.sin(PI * u);
  return (PI / 4) * (1 - Math.cos(PI * u)) + biasRad * sine * sine;
}

function curveOmega(u, speedMmS, arcMm, biasRad){
  const sine = Math.sin(PI * u);
  if(biasRad === 0){
    return speedMmS * PI * PI / (4 * arcMm) * sine;
  }
  const cosine = Math.cos(PI * u);
  return speedMmS / arcMm * sine * (PI * PI / 4 + 2 * biasRad * PI * cosine);
}

function moveCurve(state, direction, profile, config, out){
  const startHeading = state.headingDeg;
  const arcMm = config.arcMm;
  let moved = 0;
  const baseStep = Math.max(0.1, profile.speed_mm_s * DT);

  while(moved < arcMm - 1e-9){
    const ds = Math.min(baseStep, arcMm - moved);
    const uMid = (moved + ds * 0.5) / arcMm;
    const uEnd = (moved + ds) / arcMm;
    const thetaMid = curveTheta(uMid, config.biasRad);
    const thetaEnd = curveTheta(uEnd, config.biasRad);
    const headingMid = startHeading + direction * rad2deg(thetaMid);
    const omegaMag = curveOmega(uMid, profile.speed_mm_s, arcMm, config.biasRad);

    state.x += ds * sinD(headingMid);
    state.y += ds * cosD(headingMid);
    state.headingDeg = startHeading + direction * rad2deg(thetaEnd);
    state.t += ds / profile.speed_mm_s;
    moved += ds;
    record(state, config.special ? "curveSpecial" : "curve", profile.speed_mm_s, direction * omegaMag, out);
  }

  state.headingDeg = startHeading + direction * 90;
}

function turnParameters(profile, direction, previousTurn){
  let entry = profile.entry_mm + (direction > 0 ? profile.right_entry_adjust_mm : profile.left_entry_adjust_mm);
  const exit = profile.entry_mm + (direction > 0 ? profile.right_exit_adjust_mm : profile.left_exit_adjust_mm);

  if(direction < 0 && previousTurn > 0) entry += profile.right_left_entry_extra_mm;
  if(direction > 0 && previousTurn < 0) entry += profile.left_right_entry_extra_mm;

  const specialLR = direction > 0 && previousTurn < 0 && profile.left_right_lateral_mm > 0;
  return {entryMm:entry, exitMm:exit, specialLR:specialLR};
}

function runTurn(state, direction, previousTurn, profile, out){
  const tp = turnParameters(profile, direction, previousTurn);
  const wallRead = profile.sense_advance_mm;

  moveStraight(state, wallRead, "wallread", profile, 0, out);
  moveStraight(state, tp.entryMm - wallRead, "entryexit", profile, 0, out);

  const baseArc = normalArcMm(profile);
  const config = tp.specialLR ? leftRightCurve(profile, baseArc) : {arcMm:baseArc, biasRad:0, special:false};
  moveCurve(state, direction, profile, config, out);
  moveStraight(state, tp.exitMm, "entryexit", profile, 0, out);

  return {
    direction:direction,
    entryMm:tp.entryMm,
    wallReadMm:wallRead,
    remainingEntryMm:tp.entryMm - wallRead,
    exitMm:tp.exitMm,
    arcMm:config.arcMm,
    biasDeg:rad2deg(config.biasRad),
    specialLR:config.special
  };
}

function idealCellsForPattern(pattern){
  if(pattern === "left_right") return [[1,0],[1,1],[0,1],[0,2]];
  if(pattern === "right_left") return [[1,0],[1,1],[2,1],[2,2]];
  if(pattern === "left") return [[1,0],[1,1],[0,1]];
  return [[1,0],[1,1],[2,1]];
}

function simulateRoute(){
  const profile = PROFILES[Number(els.profile.value)];
  const pattern = els.pattern.value;
  const errors = validateProfile(profile, pattern);
  if(errors.length) throw new Error(errors.join(" / "));

  const state = {
    x:CELL * 1.5,
    y:CELL * 0.5,
    headingDeg:0,
    t:0
  };
  const samples = [];
  record(state, "startstop", MACHINE.minSpeedMmS, 0, samples);

  moveStraight(state, HALF_CELL, "startstop", profile, 1, samples);

  let directions;
  if(pattern === "left_right") directions = [-1, 1];
  else if(pattern === "right_left") directions = [1, -1];
  else if(pattern === "left") directions = [-1];
  else directions = [1];

  const turns = [];
  let previousTurn = 0;
  directions.forEach(function(direction){
    const detail = runTurn(state, direction, previousTurn, profile, samples);
    turns.push(detail);
    previousTurn = direction;
  });

  moveStraight(state, HALF_CELL, "startstop", profile, -1, samples);

  const cells = idealCellsForPattern(pattern);
  const finalCell = cells[cells.length - 1];
  const idealEnd = {x:(finalCell[0] + 0.5) * CELL, y:(finalCell[1] + 0.5) * CELL};
  const errorX = state.x - idealEnd.x;
  const errorY = state.y - idealEnd.y;

  let maxOmega = 0;
  let maxAy = 0;
  let minInner = Infinity;
  let maxOuter = 0;

  samples.forEach(function(s){
    const omega = Math.abs(s.omegaRadS);
    if(omega <= 0) return;
    maxOmega = Math.max(maxOmega, omega);
    const vMmS = s.speedMmS;
    const diff = omega * MACHINE.treadMm / 2;
    minInner = Math.min(minInner, vMmS - diff);
    maxOuter = Math.max(maxOuter, vMmS + diff);
    maxAy = Math.max(maxAy, (vMmS / 1000) * omega);
  });

  if(!Number.isFinite(minInner)) minInner = profile.speed_mm_s;

  return {
    profile:profile,
    pattern:pattern,
    samples:samples,
    turns:turns,
    state:state,
    idealEnd:idealEnd,
    errorX:errorX,
    errorY:errorY,
    errorMm:Math.hypot(errorX,errorY),
    maxOmega:maxOmega,
    maxAy:maxAy,
    minInner:minInner,
    maxOuter:maxOuter,
    idealCells:cells
  };
}

function simulateFriction(commanded, {muStatic, muKinetic}){
  const staticLimit = muStatic * G;
  const kineticLimit = muKinetic * G;
  let actual = {...commanded[0], vx:0, vy:commanded[0].speedMmS / 1000, sliding:false};
  const samples = [actual];
  let lateralSpeed = 0;
  let firstSlide = null;
  let slideMm = 0;
  let slideTime = 0;
  let maxSlipAngleDeg = 0;
  let maxLateralErrorMm = 0;
  let maxErrorMm = 0;

  for(let i=1;i<commanded.length;i++){
    const prev = commanded[i - 1], next = commanded[i];
    const dt = next.t - prev.t;
    const forwardSpeed = next.speedMmS / 1000;
    const halfTurn = (next.headingDeg - prev.headingDeg) / 2;
    const headingMid = prev.headingDeg + halfTurn;
    const requiredAy = forwardSpeed * Math.abs(next.omegaRadS);

    // Wheel commands prescribe forward speed and yaw. Rotating the body axes
    // produces lateral velocity; floor friction acts against that velocity.
    const freeLateral = lateralSpeed * cosD(2 * halfTurn) - forwardSpeed * sinD(2 * halfTurn);
    const previousLateral = lateralSpeed;
    const slideStep = requiredAy > staticLimit || previousLateral !== 0;
    if(slideStep){
      const correction = Math.min(Math.abs(freeLateral), kineticLimit * dt);
      lateralSpeed = freeLateral - Math.sign(freeLateral) * correction;
    }else{
      lateralSpeed = 0;
    }

    // A sliding contact can stick again once kinetic friction arrests lateral motion.
    const lateralDistance = (previousLateral + lateralSpeed) / 2 * dt * 1000;
    const dx = next.x - prev.x + lateralDistance * cosD(headingMid);
    const dy = next.y - prev.y - lateralDistance * sinD(headingMid);
    if(slideStep){
      firstSlide ??= {index:i, x:actual.x, y:actual.y, t:prev.t};
      slideMm += Math.hypot(dx, dy);
      slideTime += dt;
    }

    actual = {
      x:next.x + (actual.x - prev.x) + lateralDistance * cosD(headingMid),
      y:next.y + (actual.y - prev.y) - lateralDistance * sinD(headingMid),
      headingDeg:next.headingDeg,
      vx:forwardSpeed * sinD(next.headingDeg) + lateralSpeed * cosD(next.headingDeg),
      vy:forwardSpeed * cosD(next.headingDeg) - lateralSpeed * sinD(next.headingDeg),
      sliding:slideStep
    };
    samples.push(actual);
    const errorX = actual.x - next.x, errorY = actual.y - next.y;
    maxErrorMm = Math.max(maxErrorMm, Math.hypot(errorX, errorY));
    maxLateralErrorMm = Math.max(maxLateralErrorMm,
      Math.abs(errorX * cosD(next.headingDeg) - errorY * sinD(next.headingDeg)));
    maxSlipAngleDeg = Math.max(maxSlipAngleDeg, rad2deg(Math.atan2(Math.abs(lateralSpeed), forwardSpeed)));
  }

  const end = commanded[commanded.length - 1];
  return {
    samples, firstSlide, slideMm, slideTime, maxErrorMm, maxLateralErrorMm, maxSlipAngleDeg,
    endErrorMm:Math.hypot(actual.x - end.x, actual.y - end.y),
    headingErrorDeg:actual.headingDeg - end.headingDeg
  };
}

function routeView(commanded, actual){
  let minX = 0, minY = 0, maxX = CELL * 3, maxY = CELL * 3;
  for(const samples of [commanded, actual]){
    for(const {x, y} of samples){
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
  }
  const span = Math.max(maxX - minX, maxY - minY);
  return {
    centerX:(minX + maxX) / 2,
    centerY:(minY + maxY) / 2,
    scale:(els.canvas.width - 110) / span
  };
}

function worldToCanvas(x,y){
  const {centerX, centerY, scale} = sim?.view ?? routeView([], []);
  return [els.canvas.width / 2 + (x - centerX) * scale, els.canvas.height / 2 - (y - centerY) * scale];
}

function drawMaze(){
  ctx.clearRect(0,0,els.canvas.width,els.canvas.height);
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(0,0,els.canvas.width,els.canvas.height);

  ctx.strokeStyle = "#c5ced9";
  ctx.lineWidth = 2;
  for(let i=0;i<=3;i++){
    let a = worldToCanvas(i * CELL, 0);
    let b = worldToCanvas(i * CELL, 3 * CELL);
    ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();

    a = worldToCanvas(0, i * CELL);
    b = worldToCanvas(3 * CELL, i * CELL);
    ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();
  }

  ctx.fillStyle = "#44536a";
  ctx.font = "14px system-ui";
  for(let x=0;x<3;x++){
    for(let y=0;y<3;y++){
      const p = worldToCanvas((x + 0.5) * CELL, (y + 0.5) * CELL);
      ctx.fillText("(" + x + "," + y + ")", p[0] - 20, p[1] + 5);
    }
  }
}

function drawIdealRoute(){
  if(!sim) return;
  ctx.save();
  ctx.strokeStyle = "#d7aa26";
  ctx.lineWidth = 2;
  ctx.setLineDash([9,7]);
  ctx.beginPath();
  sim.idealCells.forEach(function(cell,i){
    const p = worldToCanvas((cell[0] + 0.5) * CELL, (cell[1] + 0.5) * CELL);
    if(i === 0) ctx.moveTo(p[0],p[1]);
    else ctx.lineTo(p[0],p[1]);
  });
  ctx.stroke();
  ctx.restore();
}

function phaseColor(phase){
  if(phase === "startstop") return "#ef4444";
  if(phase === "wallread") return "#a855f7";
  if(phase === "entryexit") return "#f59e0b";
  return "#2563eb";
}

function drawRobot(s){
  const p = worldToCanvas(s.x,s.y);
  const scale = sim.view.scale;
  ctx.save();
  ctx.translate(p[0],p[1]);
  ctx.rotate(deg2rad(s.headingDeg));

  const tread = MACHINE.treadMm * scale;
  const tyre = MACHINE.tyreWidthMm * scale;
  const w = (MACHINE.treadMm + MACHINE.tyreWidthMm) * scale;
  const h = 72 * scale;

  ctx.fillStyle = "rgba(126,87,194,.88)";
  ctx.strokeStyle = "#1f2937";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-w/2,-h/2,w,h,7);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#111827";
  ctx.fillRect(-tread/2 - tyre/2,-h*.28,tyre,h*.56);
  ctx.fillRect(tread/2 - tyre/2,-h*.28,tyre,h*.56);

  ctx.fillStyle = "#f8fafc";
  ctx.beginPath();
  ctx.moveTo(0,-h*.43);
  ctx.lineTo(-6,-h*.25);
  ctx.lineTo(6,-h*.25);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawTraceToIndex(index){
  drawMaze();
  drawIdealRoute();
  if(!sim || !sim.samples.length) return;

  const end = Math.max(0, Math.min(index, sim.samples.length - 1));
  for(let i=1;i<=end;i++){
    const a = sim.samples[i-1], b = sim.samples[i];
    const pa = worldToCanvas(a.x,a.y), pb = worldToCanvas(b.x,b.y);
    ctx.strokeStyle = phaseColor(b.phase);
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(pa[0],pa[1]);
    ctx.lineTo(pb[0],pb[1]);
    ctx.stroke();
  }
  const actual = sim.friction.samples;
  // Keep the commanded phase colors visible through the gaps in the actual trace.
  ctx.save();
  ctx.strokeStyle = "#991b1b";
  ctx.lineWidth = 2;
  ctx.setLineDash([6,5]);
  ctx.beginPath();
  ctx.moveTo(...worldToCanvas(actual[0].x, actual[0].y));
  for(let i=1;i<=end;i++) ctx.lineTo(...worldToCanvas(actual[i].x, actual[i].y));
  ctx.stroke();

  ctx.setLineDash([]);
  ctx.lineWidth = 6;
  ctx.lineCap = "round";
  ctx.beginPath();
  for(let i=1;i<=end;i++){
    if(!actual[i].sliding) continue;
    if(!actual[i - 1].sliding) ctx.moveTo(...worldToCanvas(actual[i - 1].x, actual[i - 1].y));
    ctx.lineTo(...worldToCanvas(actual[i].x, actual[i].y));
  }
  ctx.stroke();
  ctx.restore();

  const first = sim.friction.firstSlide;
  if(first && first.index <= end){
    ctx.beginPath();
    ctx.arc(...worldToCanvas(first.x, first.y), 6, 0, 2 * PI);
    ctx.fillStyle = "#fff";
    ctx.strokeStyle = "#991b1b";
    ctx.lineWidth = 3;
    ctx.fill();
    ctx.stroke();
  }

  const pose = actual[end];
  drawRobot(pose);
  if(pose.sliding){
    const [x, y] = worldToCanvas(pose.x, pose.y);
    const angle = Math.atan2(pose.vx, pose.vy);
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + 44 * Math.sin(angle), y - 44 * Math.cos(angle));
    ctx.strokeStyle = "#991b1b";
    ctx.lineWidth = 3;
    ctx.stroke();
  }
  const {speedMmS, omegaRadS} = sim.samples[end];
  const halfDifference = omegaRadS * MACHINE.treadMm / 2;
  els.wheelValues.textContent = "指令輪速（左 / 右） "
    + (speedMmS + halfDifference).toFixed(1) + " / "
    + (speedMmS - halfDifference).toFixed(1) + " mm/s";
}

function indexAtTime(t){
  let lo = 0, hi = sim.samples.length - 1;
  while(lo < hi){
    const mid = Math.floor((lo + hi + 1) / 2);
    if(sim.samples[mid].t <= t) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

function setRows(el, rows){
  el.innerHTML = rows.map(function(row){
    return "<dt>" + row[0] + "</dt><dd>" + row[1] + "</dd>";
  }).join("");
}

function updateInfo(){
  const p = sim.profile;
  const {grip, friction} = sim;
  const muReq = sim.maxAy / G;
  const force = MACHINE.massKg * sim.maxAy;

  setRows(els.resultValues,[
    ["速度", p.speed_mm_s.toFixed(0) + " mm/s"],
    ["指令終了位置", sim.state.x.toFixed(2) + ", " + sim.state.y.toFixed(2) + " mm"],
    ["理想中心との差", sim.errorMm.toFixed(2) + " mm"],
    ["X / Y誤差", sim.errorX.toFixed(2) + " / " + sim.errorY.toFixed(2) + " mm"],
    ["指令終了姿勢", sim.state.headingDeg.toFixed(2) + "°"],
    ["総走行時間", sim.state.t.toFixed(3) + " s"],
    ["最大角速度", rad2deg(sim.maxOmega).toFixed(1) + " °/s"]
  ]);

  const turnRows = [];
  sim.turns.forEach(function(t,i){
    const name = t.direction > 0 ? "右" : "左";
    turnRows.push(["Turn " + (i+1), name + (t.specialLR ? "（L→R専用曲線）" : "")]);
    turnRows.push(["入口 / 壁読取", t.entryMm.toFixed(1) + " / " + t.wallReadMm.toFixed(1) + " mm"]);
    turnRows.push(["読取後の残り入口", t.remainingEntryMm.toFixed(1) + " mm"]);
    turnRows.push(["曲線長 / 出口", t.arcMm.toFixed(2) + " / " + t.exitMm.toFixed(1) + " mm"]);
    if(t.specialLR) turnRows.push(["bias", t.biasDeg.toFixed(3) + "°"]);
  });
  setRows(els.turnValues,turnRows);

  setRows(els.physicsValues,[
    ["最大横加速度", sim.maxAy.toFixed(3) + " m/s²"],
    ["最大横G", (sim.maxAy / G).toFixed(3) + " G"],
    ["必要横力", force.toFixed(3) + " N"],
    ["必要μ（最大）", muReq.toFixed(3)],
    ["静止 μs / 動 μk", grip.muStatic.toFixed(3) + " / " + grip.muKinetic.toFixed(3)],
    ["静止 / 滑走中の横加速度上限", (grip.muStatic * G).toFixed(3) + " / " + (grip.muKinetic * G).toFixed(3) + " m/s²"],
    ["最初の滑り開始 (X, Y)", friction.firstSlide
      ? friction.firstSlide.x.toFixed(2) + ", " + friction.firstSlide.y.toFixed(2) + " mm"
      : "—"],
    ["滑走区間の合計", friction.slideMm.toFixed(2) + " mm / " + (friction.slideTime * 1000).toFixed(1) + " ms"],
    ["最大横ずれ（対指令進行方向）", friction.maxLateralErrorMm.toFixed(2) + " mm"],
    ["最大スリップ角", friction.maxSlipAngleDeg.toFixed(2) + "°"],
    ["最大位置ずれ（対指令）", friction.maxErrorMm.toFixed(2) + " mm"],
    ["終了時の位置ずれ（対指令）", friction.endErrorMm.toFixed(2) + " mm"],
    ["終了時の姿勢ずれ（対指令）", friction.headingErrorDeg.toFixed(2) + "°"],
    ["最小内輪速度", sim.minInner.toFixed(1) + " mm/s"],
    ["最大外輪速度", sim.maxOuter.toFixed(1) + " mm/s"],
    ["1 pulse", MACHINE.mmPerPulse.toFixed(4) + " mm"]
  ]);

  const slides = friction.slideTime > 0;
  els.gripStatus.className = "status " + (slides ? "bad" : "ok");
  els.gripStatus.textContent = slides ? "摩擦限界を超える区間があります。" : "全区間で摩擦限界内です。";
}

function showWarning(message){
  if(message){
    els.configWarning.textContent = message;
    els.configWarning.classList.remove("hidden");
  }else{
    els.configWarning.textContent = "";
    els.configWarning.classList.add("hidden");
  }
}

function regenerate(){
  cancelAnimationFrame(raf);
  playing = false;
  els.playBtn.textContent = "▶ 再生";
  try{
    sim = simulateRoute();
    sim.grip = MEASURED_GRIP;
    if(els.muPreset.value === "custom"){
      const muStatic = numberValue(els.muStatic, MEASURED_GRIP.muStatic, 0, 5);
      const muKinetic = numberValue(els.muKinetic, MEASURED_GRIP.muKinetic, 0, muStatic);
      sim.grip = {muStatic, muKinetic};
    }
    sim.friction = simulateFriction(sim.samples, sim.grip);
    sim.view = routeView(sim.samples, sim.friction.samples);
    showWarning("");
    updateInfo();
    drawTraceToIndex(sim.samples.length - 1);
  }catch(err){
    sim = null;
    showWarning("設定エラー: " + err.message);
    drawMaze();
    els.resultValues.innerHTML = "";
    els.turnValues.innerHTML = "";
    els.physicsValues.innerHTML = "";
    els.gripStatus.textContent = "";
    els.wheelValues.textContent = "";
  }
}

function play(){
  if(!sim) regenerate();
  if(!sim) return;

  if(playing){
    playing = false;
    cancelAnimationFrame(raf);
    els.playBtn.textContent = "▶ 再生";
    return;
  }

  playing = true;
  playStart = performance.now();
  els.playBtn.textContent = "Ⅱ 一時停止";
  const totalMs = Math.max(600, sim.state.t * 1000);

  function frame(now){
    if(!playing) return;
    const elapsedMs = Math.min(totalMs, now - playStart);
    const simTime = sim.state.t * elapsedMs / totalMs;
    drawTraceToIndex(indexAtTime(simTime));
    if(elapsedMs < totalMs){
      raf = requestAnimationFrame(frame);
    }else{
      playing = false;
      els.playBtn.textContent = "▶ 再生";
    }
  }
  raf = requestAnimationFrame(frame);
}

els.profile.addEventListener("change",function(){
  renderCodeInputs();
  regenerate();
});
els.pattern.addEventListener("change",regenerate);
function updateFrictionInputs(){
  const custom = els.muPreset.value === "custom";
  for(const input of [els.muStatic, els.muKinetic]){
    input.parentElement.hidden = !custom;
    input.disabled = !custom;
  }
}
els.muPreset.addEventListener("change",function(){
  updateFrictionInputs();
  regenerate();
});
els.muStatic.addEventListener("change",regenerate);
els.muKinetic.addEventListener("change",regenerate);
els.applyCodeBtn.addEventListener("click",applyCodeInputs);
els.playBtn.addEventListener("click",play);
els.resetBtn.addEventListener("click",function(){
  PROFILES = DEFAULT_PROFILES.map(function(p){ return Object.assign({}, p); });
  els.profile.value = "2";
  els.pattern.value = "left_right";
  els.muPreset.value = "measured";
  els.muStatic.value = String(MEASURED_GRIP.muStatic);
  els.muKinetic.value = String(MEASURED_GRIP.muKinetic);
  updateFrictionInputs();
  renderCodeInputs();
  regenerate();
});

els.profile.value = "2";
els.pattern.value = "left_right";
updateFrictionInputs();
renderCodeInputs();
regenerate();
