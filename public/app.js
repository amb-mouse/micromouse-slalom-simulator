"use strict";

const DEFAULT_PROFILES = [
  {speed_mm_s:200,known_max_mm_s:500,left_curve_shift_mm:4.5,entry_mm:10,left_pre_offset_mm:0,right_pre_offset_mm:0,left_post_offset_mm:0,right_post_offset_mm:0,left_right_entry_extra_mm:0,right_left_entry_extra_mm:0,left_right_lateral_mm:0,heading_gain:12,correction_max_deg_s:15,wall_kp:.20,wall_kd:.070,wall_filter_ms:4,wall_d_filter_ms:30,wall_slew_deg_s2:300,sense_advance_mm:8,sense_skip_mm:2,front_stable_ms:3},
  {speed_mm_s:300,known_max_mm_s:600,left_curve_shift_mm:4.75,entry_mm:10,left_pre_offset_mm:0,right_pre_offset_mm:1.5,left_post_offset_mm:0,right_post_offset_mm:0,left_right_entry_extra_mm:0,right_left_entry_extra_mm:0,left_right_lateral_mm:0,heading_gain:12,correction_max_deg_s:15,wall_kp:.20,wall_kd:.070,wall_filter_ms:4,wall_d_filter_ms:30,wall_slew_deg_s2:300,sense_advance_mm:7,sense_skip_mm:2,front_stable_ms:3},
  {speed_mm_s:400,known_max_mm_s:700,left_curve_shift_mm:5.0,entry_mm:10,left_pre_offset_mm:0,right_pre_offset_mm:3.0,left_post_offset_mm:0,right_post_offset_mm:0,left_right_entry_extra_mm:2.0,right_left_entry_extra_mm:0,left_right_lateral_mm:0,heading_gain:12,correction_max_deg_s:15,wall_kp:.20,wall_kd:.070,wall_filter_ms:4,wall_d_filter_ms:30,wall_slew_deg_s2:300,sense_advance_mm:6,sense_skip_mm:2,front_stable_ms:3},
  {speed_mm_s:500,known_max_mm_s:800,left_curve_shift_mm:5.25,entry_mm:10,left_pre_offset_mm:0,right_pre_offset_mm:4.5,left_post_offset_mm:0,right_post_offset_mm:1.0,left_right_entry_extra_mm:3.0,right_left_entry_extra_mm:0,left_right_lateral_mm:10.0,heading_gain:12,correction_max_deg_s:15,wall_kp:.20,wall_kd:.070,wall_filter_ms:4,wall_d_filter_ms:30,wall_slew_deg_s2:300,sense_advance_mm:5,sense_skip_mm:2,front_stable_ms:3}
];

let PROFILES = DEFAULT_PROFILES.map(p=>({...p}));

const CELL=180, DT=.001, G=9.80665;
const VEHICLE={massKg:.682,treadMm:81,tyreWidthMm:6,outerWidthMm:87};

const els={
  canvas:document.querySelector("#simCanvas"),
  profile:document.querySelector("#profile"),
  pattern:document.querySelector("#pattern"),
  playBtn:document.querySelector("#playBtn"),
  resetBtn:document.querySelector("#resetBtn"),
  wmax:document.querySelector("#wmax"),
  alpha:document.querySelector("#alpha"),
  basePre:document.querySelector("#basePre"),
  basePost:document.querySelector("#basePost"),
  slipK:document.querySelector("#slipK"),
  mu:document.querySelector("#mu"),
  codeParams:document.querySelector("#codeParams"),
  applyCodeBtn:document.querySelector("#applyCodeBtn"),
  resultValues:document.querySelector("#resultValues"),
  physicsValues:document.querySelector("#physicsValues"),
  gripStatus:document.querySelector("#gripStatus")
};
const ctx=els.canvas.getContext("2d");
let sim=null, raf=null, playing=false, playStart=0;

const editableKeys=[
  ["speed_mm_s","speed_mm_s",10],
  ["known_max_mm_s","known_max_mm_s",10],
  ["entry_mm","entry_mm",.5],
  ["left_curve_shift_mm","left_curve_shift_mm",.25],
  ["left_pre_offset_mm","left_pre_offset_mm",.5],
  ["right_pre_offset_mm","right_pre_offset_mm",.5],
  ["left_post_offset_mm","left_post_offset_mm",.5],
  ["right_post_offset_mm","right_post_offset_mm",.5],
  ["left_right_entry_extra_mm","L→R entry extra",.5],
  ["right_left_entry_extra_mm","R→L entry extra",.5],
  ["left_right_lateral_mm","L→R lateral",.5],
  ["heading_gain","heading_gain",1],
  ["correction_max_deg_s","correction_max_deg_s",1],
  ["wall_kp","wall_kp",.01],
  ["wall_kd","wall_kd",.005],
  ["wall_filter_ms","wall_filter_ms",1],
  ["wall_d_filter_ms","wall_d_filter_ms",1],
  ["wall_slew_deg_s2","wall_slew_deg_s2",10],
  ["sense_advance_mm","sense_advance_mm",.5],
  ["sense_skip_mm","sense_skip_mm",.5],
  ["front_stable_ms","front_stable_ms",1]
];

function n(el,fallback,min=-Infinity,max=Infinity){
  let v=Number(el.value); if(!Number.isFinite(v))v=fallback;
  v=Math.max(min,Math.min(max,v)); el.value=String(v); return v;
}
function deg2rad(d){return d*Math.PI/180}
function sinD(d){return Math.sin(deg2rad(d))}
function cosD(d){return Math.cos(deg2rad(d))}

function renderCodeInputs(){
  const p=PROFILES[Number(els.profile.value)];
  els.codeParams.innerHTML="";
  for(const [key,label,step] of editableKeys){
    const lab=document.createElement("label");
    lab.textContent=label;
    const inp=document.createElement("input");
    inp.type="number"; inp.step=String(step); inp.value=String(p[key]); inp.dataset.key=key;
    lab.appendChild(inp); els.codeParams.appendChild(lab);
  }
}

function applyCodeInputs(){
  const p=PROFILES[Number(els.profile.value)];
  els.codeParams.querySelectorAll("input[data-key]").forEach(inp=>{
    const v=Number(inp.value); if(Number.isFinite(v))p[inp.dataset.key]=v;
  });
  regenerate();
}

function straightToAxis(state, target, phase="straight"){
  const speed=state.speed;
  const samples=[];
  const add=()=>samples.push({x:state.x,y:state.y,bodyAngleDeg:state.angleDeg,travelAngleDeg:state.angleDeg,wDegS:0,t:state.t,phase});
  add();
  const heading=((state.angleDeg%360)+360)%360;
  let dist=0;
  if(Math.abs(heading-0)<.01) dist=target.y-state.y;
  else if(Math.abs(heading-90)<.01) dist=target.x-state.x;
  else if(Math.abs(heading-180)<.01) dist=state.y-target.y;
  else if(Math.abs(heading-270)<.01) dist=state.x-target.x;
  if(dist<0) dist=0;
  const steps=Math.max(1,Math.ceil(dist/(speed*DT)));
  const step=dist/steps;
  for(let i=0;i<steps;i++){
    state.x+=step*sinD(state.angleDeg);
    state.y+=step*cosD(state.angleDeg);
    state.t+=step/speed;
    add();
  }
  return samples;
}

function simulateCurve(state,direction){
  const wmax=n(els.wmax,200,1,5000);
  const alpha=n(els.alpha,1000,1,100000);
  const K=n(els.slipK,.002,0,1);
  const speed=state.speed, step=speed*DT, vms=speed/1000;
  let w=0;
  const startA=state.angleDeg, targetA=startA+direction*90;
  const samples=[];
  const add=(phase,travelA)=>samples.push({x:state.x,y:state.y,bodyAngleDeg:state.angleDeg,travelAngleDeg:travelA,wDegS:w,t:state.t,phase});
  add("acc",state.angleDeg);

  let guard=0;
  while(w<wmax && guard++<200000){
    w=Math.min(wmax,w+alpha*DT);
    state.angleDeg+=direction*w*DT;
    const wr=deg2rad(w);
    const slipDeg=(K*vms*wr)*180/Math.PI;
    const travel=state.angleDeg-direction*slipDeg;
    state.x+=step*sinD(travel); state.y+=step*cosD(travel); state.t+=DT;
    add("acc",travel);
  }

  const brakeAngle=wmax*wmax/(2*alpha);
  guard=0;
  while(direction*(targetA-state.angleDeg)>brakeAngle && guard++<200000){
    state.angleDeg+=direction*w*DT;
    const wr=deg2rad(w);
    const slipDeg=(K*vms*wr)*180/Math.PI;
    const travel=state.angleDeg-direction*slipDeg;
    state.x+=step*sinD(travel); state.y+=step*cosD(travel); state.t+=DT;
    add("const",travel);
  }

  guard=0;
  while(w>0 && guard++<200000){
    w=Math.max(0,w-alpha*DT);
    state.angleDeg+=direction*w*DT;
    const wr=deg2rad(w);
    const slipDeg=(K*vms*wr)*180/Math.PI;
    const travel=state.angleDeg-direction*slipDeg;
    state.x+=step*sinD(travel); state.y+=step*cosD(travel); state.t+=DT;
    add("dec",travel);
  }
  state.angleDeg=targetA;
  return samples;
}

function preDistance(profile,direction,seq){
  let d=n(els.basePre,90,0,400);
  d += direction>0 ? profile.right_pre_offset_mm : profile.left_pre_offset_mm;
  if(seq==="left_right" && direction>0)d+=profile.left_right_entry_extra_mm;
  if(seq==="right_left" && direction<0)d+=profile.right_left_entry_extra_mm;
  return d;
}
function postDistance(profile,direction){
  let d=n(els.basePost,55,0,400);
  d += direction>0 ? profile.right_post_offset_mm : profile.left_post_offset_mm;
  return d;
}

function approachTarget(corner,state,pre){
  const a=((state.angleDeg%360)+360)%360;
  if(Math.abs(a-0)<.01)return {x:state.x,y:corner.y-pre};
  if(Math.abs(a-90)<.01)return {x:corner.x-pre,y:state.y};
  if(Math.abs(a-180)<.01)return {x:state.x,y:corner.y+pre};
  return {x:corner.x+pre,y:state.y};
}
function exitTarget(state,post){
  return {x:state.x+post*sinD(state.angleDeg),y:state.y+post*cosD(state.angleDeg)};
}

function appendSamples(all,part){
  if(all.length&&part.length)part=part.slice(1);
  all.push(...part);
}

function simulateRoute(){
  const profile=PROFILES[Number(els.profile.value)];
  const pattern=els.pattern.value;
  const speed=profile.speed_mm_s;
  let state={x:CELL*1.5,y:CELL*.5,angleDeg:0,t:0,speed};
  const all=[{x:state.x,y:state.y,bodyAngleDeg:0,travelAngleDeg:0,wDegS:0,t:0,phase:"straight"}];
  const turns=[];

  let route;
  if(pattern==="left_right"){
    route={corners:[{x:CELL*1.5,y:CELL*1.5,dir:-1,seq:null},{x:CELL*.5,y:CELL*1.5,dir:+1,seq:"left_right"}],end:{x:CELL*.5,y:CELL*2.5}};
  } else if(pattern==="right_left"){
    route={corners:[{x:CELL*1.5,y:CELL*1.5,dir:+1,seq:null},{x:CELL*2.5,y:CELL*1.5,dir:-1,seq:"right_left"}],end:{x:CELL*2.5,y:CELL*2.5}};
  } else if(pattern==="left"){
    route={corners:[{x:CELL*1.5,y:CELL*1.5,dir:-1,seq:null}],end:{x:CELL*.5,y:CELL*1.5}};
  } else {
    route={corners:[{x:CELL*1.5,y:CELL*1.5,dir:+1,seq:null}],end:{x:CELL*2.5,y:CELL*1.5}};
  }

  for(const c of route.corners){
    const pre=preDistance(profile,c.dir,c.seq);
    const ap=approachTarget(c,state,pre);
    appendSamples(all,straightToAxis(state,ap,"straight"));

    const before={x:state.x,y:state.y};
    appendSamples(all,simulateCurve(state,c.dir));

    const post=postDistance(profile,c.dir);
    const ex=exitTarget(state,post);
    appendSamples(all,straightToAxis(state,ex,"straight"));

    turns.push({corner:c,before,after:{x:state.x,y:state.y},pre,post});
  }

  appendSamples(all,straightToAxis(state,route.end,"straight"));

  let maxAy=0,maxW=0,maxSlip=0;
  const vms=speed/1000, K=n(els.slipK,.002,0,1);
  for(const s of all){
    const wr=Math.abs(s.wDegS)*Math.PI/180;
    maxAy=Math.max(maxAy,vms*wr);
    maxW=Math.max(maxW,Math.abs(s.wDegS));
    maxSlip=Math.max(maxSlip,K*vms*wr*180/Math.PI);
  }
  return {profile,pattern,state,samples:all,turns,route,maxAy,maxW,maxSlip};
}

function worldToCanvas(x,y){
  const margin=55, scale=(els.canvas.width-2*margin)/(CELL*3);
  return [margin+x*scale,els.canvas.height-margin-y*scale];
}
function drawMaze(){
  ctx.clearRect(0,0,els.canvas.width,els.canvas.height);
  ctx.fillStyle="#f8fafc";ctx.fillRect(0,0,els.canvas.width,els.canvas.height);
  ctx.strokeStyle="#c5ced9";ctx.lineWidth=2;
  for(let i=0;i<=3;i++){
    let a=worldToCanvas(i*CELL,0),b=worldToCanvas(i*CELL,3*CELL);
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    a=worldToCanvas(0,i*CELL);b=worldToCanvas(3*CELL,i*CELL);
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
  }
  ctx.fillStyle="#44536a";ctx.font="14px system-ui";
  for(let x=0;x<3;x++)for(let y=0;y<3;y++){
    const p=worldToCanvas((x+.5)*CELL,(y+.5)*CELL);
    ctx.fillText(`(${x},${y})`,p[0]-20,p[1]+5);
  }
}
function drawTargetRoute(){
  if(!sim)return;
  let pts;
  if(sim.pattern==="left_right")pts=[[1,0],[1,1],[0,1],[0,2]];
  else if(sim.pattern==="right_left")pts=[[1,0],[1,1],[2,1],[2,2]];
  else if(sim.pattern==="left")pts=[[1,0],[1,1],[0,1]];
  else pts=[[1,0],[1,1],[2,1]];
  ctx.save();ctx.strokeStyle="#d7aa26";ctx.lineWidth=2;ctx.setLineDash([9,7]);
  ctx.beginPath();
  pts.forEach((q,i)=>{const p=worldToCanvas((q[0]+.5)*CELL,(q[1]+.5)*CELL);if(i)ctx.lineTo(...p);else ctx.moveTo(...p)});
  ctx.stroke();ctx.restore();
}
function phaseColor(p){
  if(p==="straight")return "#ef4444";
  if(p==="const")return "#22c55e";
  return "#2563eb";
}
function drawRobot(s){
  const p=worldToCanvas(s.x,s.y),scale=(els.canvas.width-110)/(CELL*3);
  ctx.save();ctx.translate(...p);ctx.rotate(deg2rad(s.bodyAngleDeg));
  const w=87*scale,h=72*scale;
  ctx.fillStyle="rgba(126,87,194,.88)";ctx.strokeStyle="#1f2937";ctx.lineWidth=2;
  ctx.beginPath();ctx.roundRect(-w/2,-h/2,w,h,7);ctx.fill();ctx.stroke();
  const tread=81*scale,tyre=6*scale;ctx.fillStyle="#111827";
  ctx.fillRect(-tread/2-tyre/2,-h*.28,tyre,h*.56);ctx.fillRect(tread/2-tyre/2,-h*.28,tyre,h*.56);
  ctx.restore();
}
function drawTrace(f=1){
  drawMaze();drawTargetRoute();
  if(!sim||!sim.samples.length)return;
  const nidx=Math.max(1,Math.floor((sim.samples.length-1)*f));
  for(let i=1;i<=nidx;i++){
    const a=sim.samples[i-1],b=sim.samples[i],pa=worldToCanvas(a.x,a.y),pb=worldToCanvas(b.x,b.y);
    ctx.strokeStyle=phaseColor(b.phase);ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(...pa);ctx.lineTo(...pb);ctx.stroke();
  }
  drawRobot(sim.samples[nidx]);
}

function updateInfo(){
  const p=sim.profile,mu=n(els.mu,1,.01,5),muReq=sim.maxAy/G;
  const end=sim.state;
  const rows=[
    ["終了座標",`${end.x.toFixed(1)}, ${end.y.toFixed(1)} mm`],
    ["終了姿勢",`${end.angleDeg.toFixed(1)}°`],
    ["総走行時間",`${end.t.toFixed(3)} s`],
    ["最大角速度",`${sim.maxW.toFixed(1)} °/s`],
    ["ターン1 pre/post",sim.turns[0]?`${sim.turns[0].pre.toFixed(1)} / ${sim.turns[0].post.toFixed(1)} mm`:"—"],
    ["ターン2 pre/post",sim.turns[1]?`${sim.turns[1].pre.toFixed(1)} / ${sim.turns[1].post.toFixed(1)} mm`:"—"]
  ];
  els.resultValues.innerHTML=rows.map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join("");

  const force=VEHICLE.massKg*sim.maxAy;
  els.physicsValues.innerHTML=[
    ["最大横加速度",`${sim.maxAy.toFixed(3)} m/s²`],
    ["最大横G",`${(sim.maxAy/G).toFixed(3)} G`],
    ["必要横力",`${force.toFixed(3)} N`],
    ["必要μ",muReq.toFixed(3)],
    ["最大スリップ角補正",`${sim.maxSlip.toFixed(3)}°`]
  ].map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join("");
  els.gripStatus.textContent=mu>=muReq?`μ=${mu.toFixed(2)} では理論上グリップ範囲。`:`μ=${mu.toFixed(2)} では必要μを超過。横滑りの可能性あり。`;
}

function regenerate(){
  cancelAnimationFrame(raf);playing=false;els.playBtn.textContent="▶ 再生";
  sim=simulateRoute();updateInfo();drawTrace(1);
}
function play(){
  if(!sim)regenerate();
  if(playing){playing=false;cancelAnimationFrame(raf);els.playBtn.textContent="▶ 再生";return}
  playing=true;playStart=performance.now();els.playBtn.textContent="Ⅱ 一時停止";
  const dur=Math.max(1400,sim.state.t*1000*1.1);
  const frame=now=>{if(!playing)return;const f=Math.min(1,(now-playStart)/dur);drawTrace(f);if(f<1)raf=requestAnimationFrame(frame);else{playing=false;els.playBtn.textContent="▶ 再生"}};
  raf=requestAnimationFrame(frame);
}

els.profile.addEventListener("change",()=>{renderCodeInputs();regenerate()});
els.pattern.addEventListener("change",regenerate);
[els.wmax,els.alpha,els.basePre,els.basePost,els.slipK,els.mu].forEach(e=>e.addEventListener("change",regenerate));
els.applyCodeBtn.addEventListener("click",applyCodeInputs);
els.playBtn.addEventListener("click",play);
els.resetBtn.addEventListener("click",()=>{
  PROFILES=DEFAULT_PROFILES.map(p=>({...p}));
  els.profile.value="1";els.pattern.value="left_right";els.wmax.value="200";els.alpha.value="1000";els.basePre.value="90";els.basePost.value="55";els.slipK.value=".002";els.mu.value="1";
  renderCodeInputs();regenerate();
});

els.profile.value="1";els.pattern.value="left_right";renderCodeInputs();regenerate();
