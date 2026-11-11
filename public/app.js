"use strict";

const PROFILES = [
  {
    speed_mm_s:200, known_max_mm_s:500, left_curve_shift_mm:4.5, entry_mm:10,
    left_pre_offset_mm:0, right_pre_offset_mm:0,
    left_post_offset_mm:0, right_post_offset_mm:0,
    left_right_entry_extra_mm:0, right_left_entry_extra_mm:0,
    left_right_lateral_mm:0, sense_advance_mm:8, sense_skip_mm:2
  },
  {
    speed_mm_s:300, known_max_mm_s:600, left_curve_shift_mm:4.75, entry_mm:10,
    left_pre_offset_mm:0, right_pre_offset_mm:1.5,
    left_post_offset_mm:0, right_post_offset_mm:0,
    left_right_entry_extra_mm:0, right_left_entry_extra_mm:0,
    left_right_lateral_mm:0, sense_advance_mm:7, sense_skip_mm:2
  },
  {
    speed_mm_s:400, known_max_mm_s:700, left_curve_shift_mm:5.0, entry_mm:10,
    left_pre_offset_mm:0, right_pre_offset_mm:3.0,
    left_post_offset_mm:0, right_post_offset_mm:0,
    left_right_entry_extra_mm:2.0, right_left_entry_extra_mm:0,
    left_right_lateral_mm:0, sense_advance_mm:6, sense_skip_mm:2
  },
  {
    speed_mm_s:500, known_max_mm_s:800, left_curve_shift_mm:5.25, entry_mm:10,
    left_pre_offset_mm:0, right_pre_offset_mm:4.5,
    left_post_offset_mm:0, right_post_offset_mm:1.0,
    left_right_entry_extra_mm:3.0, right_left_entry_extra_mm:0,
    left_right_lateral_mm:10.0, sense_advance_mm:5, sense_skip_mm:2
  }
];

const VEHICLE = { massKg:0.682, treadMm:81, tyreWidthMm:6, outerWidthMm:87 };
const G = 9.80665;
const DT = 0.001;
const CELL = 180;

const els = {
  canvas:document.querySelector("#simCanvas"),
  profile:document.querySelector("#profile"),
  pattern:document.querySelector("#pattern"),
  playBtn:document.querySelector("#playBtn"),
  resetBtn:document.querySelector("#resetBtn"),
  wmax:document.querySelector("#wmax"),
  alpha:document.querySelector("#alpha"),
  slipK:document.querySelector("#slipK"),
  mu:document.querySelector("#mu"),
  profileValues:document.querySelector("#profileValues"),
  physicsValues:document.querySelector("#physicsValues"),
  gripStatus:document.querySelector("#gripStatus")
};
const ctx = els.canvas.getContext("2d");

let sim = null;
let animationFrame = null;
let animationStart = 0;
let playing = false;

function num(el, fallback, min=-Infinity, max=Infinity){
  let v = Number(el.value);
  if(!Number.isFinite(v)) v=fallback;
  v=Math.max(min,Math.min(max,v));
  el.value=String(v);
  return v;
}

function turnOffsets(profile, direction, sequenceType=null){
  let pre = profile.entry_mm + (direction>0 ? profile.right_pre_offset_mm : profile.left_pre_offset_mm);
  let post = profile.entry_mm + (direction>0 ? profile.right_post_offset_mm : profile.left_post_offset_mm);
  if(sequenceType==="left_right" && direction>0) pre += profile.left_right_entry_extra_mm;
  if(sequenceType==="right_left" && direction<0) pre += profile.right_left_entry_extra_mm;
  return {pre, post};
}

function simulateTurn(state, direction, profile, sequenceType=null){
  const speed = profile.speed_mm_s;
  const wmax = num(els.wmax,200,1,5000);
  const alpha = num(els.alpha,1000,1,100000);
  const K = num(els.slipK,0.002,0,1);
  const offsets = turnOffsets(profile,direction,sequenceType);
  const vms = speed/1000;
  const stepDist = speed*DT;

  let {x,y,angleDeg,t} = state;
  const startAngle = angleDeg;
  const targetAngle = startAngle + direction*90;
  let w = 0;

  const samples=[];
  const add=(phase, headingDeg, wDegS)=>{
    samples.push({x,y,angleDeg:headingDeg,bodyAngleDeg:angleDeg,wDegS,t,phase});
  };

  add("pre",angleDeg,0);
  let r = angleDeg*Math.PI/180;
  x += offsets.pre*Math.sin(r);
  y += offsets.pre*Math.cos(r);
  t += Math.abs(offsets.pre)/speed;
  add("pre",angleDeg,0);

  let guard=0;
  while(w < wmax && guard++<100000){
    w += alpha*DT;
    if(w>wmax)w=wmax;
    angleDeg += direction*w*DT;

    const omegaRad=w*Math.PI/180;
    const slipDeg=(K*vms*omegaRad)*180/Math.PI;
    const travelAngle=angleDeg-direction*slipDeg;
    const ar=travelAngle*Math.PI/180;

    x += stepDist*Math.sin(ar);
    y += stepDist*Math.cos(ar);
    t += DT;
    add("acc",travelAngle,w);
  }

  const brakeAngle=wmax*wmax/(2*alpha);
  guard=0;
  while(direction*(targetAngle-angleDeg) > brakeAngle && guard++<100000){
    angleDeg += direction*w*DT;

    const omegaRad=w*Math.PI/180;
    const slipDeg=(K*vms*omegaRad)*180/Math.PI;
    const travelAngle=angleDeg-direction*slipDeg;
    const ar=travelAngle*Math.PI/180;

    x += stepDist*Math.sin(ar);
    y += stepDist*Math.cos(ar);
    t += DT;
    add("const",travelAngle,w);
  }

  guard=0;
  while(w>0 && guard++<100000){
    w -= alpha*DT;
    if(w<0)w=0;
    angleDeg += direction*w*DT;

    const omegaRad=w*Math.PI/180;
    const slipDeg=(K*vms*omegaRad)*180/Math.PI;
    const travelAngle=angleDeg-direction*slipDeg;
    const ar=travelAngle*Math.PI/180;

    x += stepDist*Math.sin(ar);
    y += stepDist*Math.cos(ar);
    t += DT;
    add("dec",travelAngle,w);
  }

  const tr=targetAngle*Math.PI/180;
  x += offsets.post*Math.sin(tr);
  y += offsets.post*Math.cos(tr);
  t += Math.abs(offsets.post)/speed;
  angleDeg=targetAngle;
  add("post",angleDeg,0);

  return {
    samples,
    state:{x,y,angleDeg,t},
    offsets
  };
}

function buildSimulation(){
  const profile=PROFILES[Number(els.profile.value)];
  const pattern=els.pattern.value;

  let state={x:CELL*1.5,y:CELL*0.5,angleDeg:0,t:0};
  let all=[];
  let turns=[];

  const append=(direction, seq)=>{
    const turn=simulateTurn(state,direction,profile,seq);
    if(all.length && turn.samples.length) turn.samples.shift();
    all.push(...turn.samples);
    turns.push({direction, ...turn});
    state=turn.state;
  };

  if(pattern==="right") append(+1,null);
  if(pattern==="left") append(-1,null);
  if(pattern==="left_right"){
    append(-1,null);
    append(+1,"left_right");
  }
  if(pattern==="right_left"){
    append(+1,null);
    append(-1,"right_left");
  }

  let maxAy=0, maxW=0, maxSlipDeg=0;
  const speedM=profile.speed_mm_s/1000;
  const K=num(els.slipK,0.002,0,1);
  for(const s of all){
    const wr=Math.abs(s.wDegS)*Math.PI/180;
    maxAy=Math.max(maxAy,speedM*wr);
    maxW=Math.max(maxW,Math.abs(s.wDegS));
    maxSlipDeg=Math.max(maxSlipDeg,K*speedM*wr*180/Math.PI);
  }

  return {profile,pattern,samples:all,turns,state,maxAy,maxW,maxSlipDeg};
}

function worldToCanvas(x,y){
  const margin=55;
  const scale=(els.canvas.width-2*margin)/(CELL*3);
  return [margin+x*scale, els.canvas.height-margin-y*scale];
}

function drawMaze(){
  ctx.clearRect(0,0,els.canvas.width,els.canvas.height);
  ctx.fillStyle="#f8fafc";
  ctx.fillRect(0,0,els.canvas.width,els.canvas.height);

  ctx.strokeStyle="#c5ced9";
  ctx.lineWidth=2;
  for(let i=0;i<=3;i++){
    let a=worldToCanvas(i*CELL,0), b=worldToCanvas(i*CELL,CELL*3);
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    a=worldToCanvas(0,i*CELL); b=worldToCanvas(CELL*3,i*CELL);
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
  }

  ctx.strokeStyle="#d7aa26";
  ctx.lineWidth=1.5;
  ctx.setLineDash([8,7]);
  for(let i=0;i<3;i++){
    let a=worldToCanvas((i+.5)*CELL,0),b=worldToCanvas((i+.5)*CELL,CELL*3);
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    a=worldToCanvas(0,(i+.5)*CELL);b=worldToCanvas(CELL*3,(i+.5)*CELL);
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
  }
  ctx.setLineDash([]);

  ctx.fillStyle="#44536a";
  ctx.font="14px system-ui";
  for(let ix=0;ix<3;ix++){
    for(let iy=0;iy<3;iy++){
      const p=worldToCanvas((ix+.5)*CELL,(iy+.5)*CELL);
      ctx.fillText(`(${ix},${iy})`,p[0]-20,p[1]+5);
    }
  }
}

function phaseColor(phase){
  if(phase==="pre"||phase==="post")return "#ef4444";
  if(phase==="const")return "#22c55e";
  return "#2563eb";
}

function drawTrace(limit=1){
  drawMaze();
  if(!sim || !sim.samples.length)return;
  const n=Math.max(1,Math.floor(limit*(sim.samples.length-1)));

  for(let i=1;i<=n;i++){
    const a=sim.samples[i-1], b=sim.samples[i];
    const pa=worldToCanvas(a.x,a.y), pb=worldToCanvas(b.x,b.y);
    ctx.strokeStyle=phaseColor(b.phase);
    ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(...pa);ctx.lineTo(...pb);ctx.stroke();
  }

  const s=sim.samples[n];
  drawRobot(s);
}

function drawRobot(s){
  const p=worldToCanvas(s.x,s.y);
  const scale=(els.canvas.width-110)/(CELL*3);
  ctx.save();
  ctx.translate(p[0],p[1]);
  ctx.rotate(s.bodyAngleDeg*Math.PI/180);
  const w=87*scale, h=72*scale;
  ctx.fillStyle="rgba(126,87,194,.88)";
  ctx.strokeStyle="#1f2937";
  ctx.lineWidth=2;
  ctx.beginPath();
  ctx.roundRect(-w/2,-h/2,w,h,7);
  ctx.fill();ctx.stroke();

  const tread=81*scale;
  const tyre=6*scale;
  ctx.fillStyle="#111827";
  ctx.fillRect(-tread/2-tyre/2,-h*.28,tyre,h*.56);
  ctx.fillRect(tread/2-tyre/2,-h*.28,tyre,h*.56);

  ctx.fillStyle="#f8fafc";
  ctx.beginPath();
  ctx.moveTo(0,-h/2+4);
  ctx.lineTo(-7,-h/2+17);
  ctx.lineTo(7,-h/2+17);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function updateInfo(){
  if(!sim)return;
  const p=sim.profile;
  const turn2=sim.turns[1];
  const rows=[
    ["speed_mm_s",`${p.speed_mm_s.toFixed(0)} mm/s`],
    ["known_max_mm_s",`${p.known_max_mm_s.toFixed(0)} mm/s`],
    ["entry_mm",`${p.entry_mm.toFixed(1)} mm`],
    ["left_curve_shift_mm",`${p.left_curve_shift_mm.toFixed(2)} mm`],
    ["left_pre_offset",`${p.left_pre_offset_mm.toFixed(1)} mm`],
    ["right_pre_offset",`${p.right_pre_offset_mm.toFixed(1)} mm`],
    ["left_post_offset",`${p.left_post_offset_mm.toFixed(1)} mm`],
    ["right_post_offset",`${p.right_post_offset_mm.toFixed(1)} mm`],
    ["L→R entry extra",`${p.left_right_entry_extra_mm.toFixed(1)} mm`],
    ["R→L entry extra",`${p.right_left_entry_extra_mm.toFixed(1)} mm`],
    ["L→R lateral",`${p.left_right_lateral_mm.toFixed(1)} mm`]
  ];
  if(turn2) rows.push(["2個目実効入口",`${turn2.offsets.pre.toFixed(1)} mm`]);
  els.profileValues.innerHTML=rows.map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join("");

  const mu=num(els.mu,1,0.01,5);
  const muReq=sim.maxAy/G;
  const force=VEHICLE.massKg*sim.maxAy;
  els.physicsValues.innerHTML=[
    ["最大角速度",`${sim.maxW.toFixed(1)} °/s`],
    ["最大横加速度",`${sim.maxAy.toFixed(3)} m/s²`],
    ["最大横G",`${(sim.maxAy/G).toFixed(3)} G`],
    ["必要横力",`${force.toFixed(3)} N`],
    ["必要μ",muReq.toFixed(3)],
    ["最大スリップ角補正",`${sim.maxSlipDeg.toFixed(3)}°`]
  ].map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join("");

  if(mu>=muReq){
    els.gripStatus.textContent=`設定 μ=${mu.toFixed(2)} では理論上グリップ範囲。余裕 ${(mu-muReq).toFixed(3)}。`;
    els.gripStatus.style.border="1px solid #256d46";
  }else{
    els.gripStatus.textContent=`設定 μ=${mu.toFixed(2)} では必要μを ${(muReq-mu).toFixed(3)} 超過。横滑りの可能性あり。`;
    els.gripStatus.style.border="1px solid #8c3434";
  }
}

function regenerate(){
  cancelAnimationFrame(animationFrame);
  playing=false;
  els.playBtn.textContent="▶ 再生";
  sim=buildSimulation();
  updateInfo();
  drawTrace(1);
}

function play(){
  if(!sim)regenerate();
  if(playing){
    playing=false;
    cancelAnimationFrame(animationFrame);
    els.playBtn.textContent="▶ 再生";
    return;
  }
  playing=true;
  animationStart=performance.now();
  els.playBtn.textContent="Ⅱ 一時停止";
  const duration=Math.max(1200,sim.state.t*1000*1.4);
  const frame=(now)=>{
    if(!playing)return;
    const f=Math.min(1,(now-animationStart)/duration);
    drawTrace(f);
    if(f<1) animationFrame=requestAnimationFrame(frame);
    else{
      playing=false;
      els.playBtn.textContent="▶ 再生";
    }
  };
  animationFrame=requestAnimationFrame(frame);
}

els.playBtn.addEventListener("click",play);
els.resetBtn.addEventListener("click",()=>{
  els.profile.value="1";
  els.pattern.value="left_right";
  els.wmax.value="200";
  els.alpha.value="1000";
  els.slipK.value="0.002";
  els.mu.value="1.0";
  regenerate();
});
[els.profile,els.pattern,els.wmax,els.alpha,els.slipK,els.mu].forEach(el=>{
  el.addEventListener("change",regenerate);
});
window.addEventListener("resize",()=>drawTrace(playing?0:1));

els.pattern.value="left_right";
regenerate();