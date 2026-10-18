(function(){
"use strict";
const $=id=>document.getElementById(id);
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const lerp=(a,b,f)=>a+(b-a)*f;
const ease=k=>k<.5?4*k*k*k:1-Math.pow(-2*k+2,3)/2;
const win=(v,a,b)=>v>=a&&v<b;
const prog=(v,a,b)=>clamp((v-a)/(b-a),0,1);
/* ================= terrain model ================= */
const W=140,D=90;
const sm=(a,b,x)=>{const t=Math.max(0,Math.min(1,(x-a)/(b-a)));return t*t*(3-2*t)};
function hash(x,z){const s=Math.sin(x*127.1+z*311.7)*43758.5453;return s-Math.floor(s)}
function vn(x,z){const xi=Math.floor(x),zi=Math.floor(z),xf=x-xi,zf=z-zi,u=xf*xf*(3-2*xf),v=zf*zf*(3-2*zf);
const a=hash(xi,zi),b=hash(xi+1,zi),c=hash(xi,zi+1),d=hash(xi+1,zi+1);return a+(b-a)*u+(c-a)*v+(a-b-c+d)*u*v}
function fbm(x,z,o){let s=0,a=.5,f=1;for(let i=0;i<o;i++){s+=a*vn(x*f,z*f);f*=2.03;a*=.5}return s}
function rdg(x,z,o){let s=0,a=.5,f=1;for(let i=0;i<o;i++){s+=a*(1-Math.abs(2*vn(x*f,z*f)-1));f*=2.1;a*=.5}return s}
const RC=z=>24+4*Math.sin(z*0.05);
const RV=z=>43+4*Math.sin(z*0.06+0.8);
const ROAD=[[-70,43],[-56,34],[-43,26],[-31,19],[-20,12],[-10,4],[-1,-6],[5,-17],[9,-26],[11,-38]];
const G1P=[-4,-32];
function roadDist(x,z){let m=1e9;for(let i=0;i<ROAD.length-1;i++){const[a,b]=ROAD[i],[c,d]=ROAD[i+1];const dx=c-a,dz=d-b;let t=((x-a)*dx+(z-b)*dz)/(dx*dx+dz*dz);t=Math.max(0,Math.min(1,t));const px=a+t*dx-x,pz=b+t*dz-z;m=Math.min(m,px*px+pz*pz)}return Math.sqrt(m)}
function Hraw(x,z){
  let y=fbm(x*0.035+3,z*0.035+7,4)*4-1.2;
  const envZ=1-0.9*sm(-2,12,z);
  const dx=x-RC(z);
  y+=(6+5*rdg(x*0.07,z*0.07,4))*Math.exp(-dx*dx/72)*envZ;
  const mts=[[-52,-36,13,11],[60,-36,15,12],[-62,0,7,9],[62,10,9,8],[58,40,5,9]];
  for(const m of mts){const d2=(x-m[0])**2+(z-m[1])**2;y+=m[2]*(0.55+0.9*rdg(x*0.09+5,z*0.09,4))*Math.exp(-d2/(2*m[3]*m[3]))}
  const rd=x-RV(z);const dep=0.6+2.8*sm(30,12,z);y-=dep*Math.exp(-rd*rd/13);y-=Math.exp(-rd*rd/60);
  const hd=(x-G1P[0])**2+(z-G1P[1])**2;y=y*(1-0.7*Math.exp(-hd/60))+5.5*Math.exp(-hd/110);
  const r=roadDist(x,z);const k=sm(1.5,5,r);y=y*k+(1-k)*(fbm(x*0.035+3,z*0.035+7,4)*4-1.2)*0.6;
  y+=0.25*(fbm(x*0.6,z*0.6,3)-0.5)*k;
  return y+1.5;
}
const GX=281,GZ=181,HG=new Float32Array(GX*GZ);
for(let j=0;j<GZ;j++)for(let i=0;i<GX;i++){HG[j*GX+i]=Hraw(-W/2+i*W/(GX-1),-D/2+j*D/(GZ-1))}
function H(x,z){
  const fx=(x+W/2)/W*(GX-1),fz=(z+D/2)/D*(GZ-1);
  const i=Math.max(0,Math.min(GX-2,Math.floor(fx))),j=Math.max(0,Math.min(GZ-2,Math.floor(fz)));
  const u=Math.max(0,Math.min(1,fx-i)),v=Math.max(0,Math.min(1,fz-j));
  const a=HG[j*GX+i],b=HG[j*GX+i+1],c=HG[(j+1)*GX+i],d=HG[(j+1)*GX+i+1];
  return a*(1-u)*(1-v)+b*u*(1-v)+c*(1-u)*v+d*u*v;
}
const RCUM=[0];for(let i=1;i<ROAD.length;i++)RCUM.push(RCUM[i-1]+Math.hypot(ROAD[i][0]-ROAD[i-1][0],ROAD[i][1]-ROAD[i-1][1]));
function roadAt(s){
  let i=0;if(s>=0){while(i<ROAD.length-2&&RCUM[i+1]<s)i++}
  const[a,b]=ROAD[i],[c,d]=ROAD[i+1],L=RCUM[i+1]-RCUM[i],f=(s-RCUM[i])/L;
  return{x:a+(c-a)*f,z:b+(d-b)*f,dx:(c-a)/L,dz:(d-b)/L};
}


/* ================= scenario (T in minutes) ================= */
const vehS=(i,T)=>34+1.6*T-9*i;
const vehPos=(i,T)=>roadAt(vehS(i,T));
const PADL=[G1P[0]+3.2,G1P[1]+3.4];
const ST1=[14,-6],ST2=[30,18],STM=[36,10],M0=[-34,-8];
function path(keys,T){
  const g=k=>k[3]==null?H(k[1],k[2])+0.5:k[3];
  if(T<=keys[0][0])return{x:keys[0][1],z:keys[0][2],y:g(keys[0])};
  for(let i=0;i<keys.length-1;i++){const a=keys[i],b=keys[i+1];if(T<=b[0]){const f=ease((T-a[0])/(b[0]-a[0]));return{x:lerp(a[1],b[1],f),z:lerp(a[2],b[2],f),y:lerp(g(a),g(b),f)}}}
  const l=keys[keys.length-1];return{x:l[1],z:l[2],y:g(l)};
}
const L1K=[[0,ST1[0],ST1[1],24],[17.6,ST1[0],ST1[1],24],[19.6,ST2[0],ST2[1],27],[26.2,ST2[0],ST2[1],27]];
const M1K=[[0,M0[0],M0[1],30],[99,M0[0],M0[1],30]];
const L1DEAD=999;
function L1at(T){const p=path(L1K,T);
  if(T<17.6||T>19.6){const c=T>19.6?0.25:0.35;p.x+=2.2*Math.cos(T*c*6);p.z+=2.2*Math.sin(T*c*6)}
  else if(T>28.2&&T<L1DEAD){p.x+=1.4*Math.cos(T*5);p.z+=1.4*Math.sin(T*5)}
  return p}
function M1at(T){const p=path(M1K,T);{p.x+=2.4*Math.cos(T*1.1);p.z+=2.4*Math.sin(T*1.1)}return p}
function HDat(T){ if(T<8)return null;
  if(T<=11.5){const f=(T-8)/3.5;const x=lerp(66,24,f),z=lerp(-26,-2,f);return{x,z,y:H(x,z)+12}}
  const f=clamp((T-11.5)/23.5,0,1);const x=lerp(24,4,f)+1.5*Math.sin(T*2),z=lerp(-2,6,f)+1.5*Math.cos(T*2);return{x,z,y:H(x,z)+12}}
const enLeadZ=T=>44-(T-13)*2.3;
const LOSTZ=34;
const tankXZ=(i,T)=>{const z=enLeadZ(T)+i*4.2;return[RV(z),z]};
function TRat(T){ if(T<5.6)return null; // civilian truck coming down the road, then pulls over
  const s=T<7.4?lerp(122,106,(T-5.6)/1.8):106;const p=roadAt(s);const side=clamp((T-7.2)/0.5,0,1)*2.2;
  return{x:p.x-p.dz*side,z:p.z+p.dx*side,dx:-p.dx,dz:-p.dz}}

/* ================= script and timeline =================
   Each line: [u start, text, minimum seconds]. u is the story clock that drives the visuals.
   The player clock v is built from line lengths so voice and picture stay together. */
const SEG=[
 {u:[300,320],T:[0,0],t:'What ONEVIEW is',L:[
  [300,'A military unit in the field has sensors on every platform: tanks, vehicles, ground posts and drones. Today each one shows its own screen, and people join the dots in their heads.'],
  [307.5,'ONEVIEW collects data from every military platform, puts it in context, understands it, and shares one common operating picture. It knows what is happening, where it is happening, and which platform observed it.'],
  [315,'It suggests. People decide.',4.5]]},
 {u:[0,10],T:[0,0],t:'The valley',L:[
  [0,'Four of our vehicles are moving up this valley, with our drones overhead. Here’s how ONEVIEW helps them.']]},
 {u:[34.5,40],T:[0,0],t:'One network',L:[
  [34.5,'Every military platform here, vehicle, drone or ground post, carries a sensor node sized to what it can lift. All of them feed ONEVIEW.',5],
  [37.5,'ONEVIEW can move its own sensors to check things. Any decision to fire stays with a person.']]},
 {u:[230,250],T:[6,7.5],t:'A truck on the road',L:[
  [230,'The ground radar picks up a vehicle coming down the road towards the column.'],
  [234.5,'ONEVIEW turns our drone’s camera to look. It’s a small truck. The thermal camera shows only the engine, and there’s no military radio around it.'],
  [243,'ONEVIEW marks it civilian and low risk. Nobody needs to be alerted.']]},
 {u:[72,100],T:[7.5,11.5],t:'A contact in the air',L:[
  [72,'Then our drone hears a drone control signal, and the ground radar sees something small a kilometre out. Drone or bird?'],
  [82,'One source on its own means little. ONEVIEW raises its confidence only as more sources agree.'],
  [85,'So it turns our drone’s camera to check. The camera sees a quadcopter, and the thermal camera sees its warm motors.'],
  [91.5,'Four sources agree: likely enemy drone. The column gets its first warning about three minutes out, instead of one.']]},
 {u:[100,140],T:[13,22],t:'Tanks behind the ridge',L:[
  [100,'The radar sees six vehicles to the south, then loses them behind the ridge. ONEVIEW keeps the track instead of dropping it.'],
  [118,'So it sends our drone over the ridge to look.'],
  [128,'The light drone’s cameras find six tanks with their engines running.'],
  [134,'Every platform reports its own position to ONEVIEW, all the time. The two drones each send which way they are looking. ONEVIEW crosses the two sightings and places the tanks within metres.']]},
 {u:[178,198],T:[31,32],t:'The commander decides',L:[
  [178,'The commander gets each threat, how sure ONEVIEW is, and what it suggests. The decision is theirs. ONEVIEW doesn’t fire anything.']]},
 {u:[212,226],T:[35,35],t:'Summary',L:[
  [212,'No single platform needs to sense everything. ONEVIEW combines what each platform knows into one shared battlefield picture, and leaves the decisions to people.']]}
];
const LN=[];let vAcc=0;
SEG.forEach((s,si)=>{s.L.forEach((l,k)=>{const words=l[1].split(/\s+/).length;const d=clamp(1.2+words*0.1,2.5,4.5);
  const uEnd=k<s.L.length-1?s.L[k+1][0]:s.u[1];LN.push({si,u:l[0],uEnd,text:l[1],v:vAcc,vEnd:vAcc+d});vAcc+=d});
  s.v=LN.find(x=>x.si===si).v;vAcc+=0.5;LN[LN.length-1].vEnd+=0.5;});
const TOTAL=vAcc;
function lineAt(v){let i=0;for(let k=0;k<LN.length;k++)if(v>=LN[k].v)i=k;return i}
function toU(v){const L=LN[lineAt(v)];return lerp(L.u,L.uEnd,prog(v,L.v,L.vEnd))}
function segOfU(u){for(let i=0;i<SEG.length;i++){const s=SEG[i];if(u>=s.u[0]&&u<=s.u[1])return i}return SEG.length-1}
function simT(u){const s=SEG[segOfU(u)];return lerp(s.T[0],s.T[1],prog(u,s.u[0],s.u[1]))}

/* camera keyframes [u,pos,target] */
const CK=[
 [0,[-130,95,130],[0,0,0]],[9,[-92,80,104],[0,0,2]],
 [11.5,[-64,26,50],[-40,2,22]],[16,[-60,22,46],[-40,2,22]],
 [17.5,[-30,20,-4],[-4,6,-32]],[22,[-28,18,-8],[-4,6,-32]],
 [23.5,[-10,32,18],[14,21,-6]],[28,[-8,31,16],[14,21,-6]],
 [29.5,[-60,38,16],[-34,27,-8]],[34,[-58,37,14],[-34,27,-8]],
 [34.5,[-100,74,96],[-8,0,-2]],[40,[-92,68,88],[-8,0,-2]],
 [42,[-72,60,62],[-6,4,-18]],[52,[-62,50,56],[-10,4,-14]],
 [55,[-64,28,50],[-32,4,10]],[62,[-54,34,44],[-26,4,0]],[71,[-46,40,34],[-20,4,-6]],
 [74,[-34,52,40],[14,8,-12]],[86,[-22,46,44],[22,10,-10]],[99,[-12,44,50],[20,10,-4]],
 [102,[-8,54,80],[40,0,18]],[117,[-4,50,78],[42,0,18]],
 [121,[-12,46,60],[24,8,8]],[130,[6,40,66],[54,4,22]],[133.6,[8,38,65],[55,4,21]],[136,[-26,96,96],[12,0,10]],[139.5,[-24,94,94],[12,0,10]],
 [142,[-34,74,62],[14,0,2]],[154,[-30,76,64],[14,0,2]],
 [158,[86,50,80],[24,0,10]],[170,[82,46,70],[40,4,14]],[177,[76,44,64],[42,4,14]],
 [181,[-86,98,102],[0,0,2]],[197,[-86,98,102],[0,0,2]],
 [201,[-18,236,60],[-16,0,2]],[211,[-18,236,60],[-16,0,2]],
 [214,[-120,90,120],[0,0,0]],[226,[-80,96,138],[0,0,0]],
 [230,[-26,48,30],[6,4,-14]],[250,[-20,44,24],[8,4,-16]],
 [300,[-130,95,130],[0,0,0]],[320,[-130,95,130],[0,0,0]]
];
function camAt(u){
  let a=CK[0],b=CK[CK.length-1];
  if(u<=CK[0][0])return[CK[0][1],CK[0][2]];
  for(let i=0;i<CK.length-1;i++){if(u>=CK[i][0]&&u<=CK[i+1][0]){a=CK[i];b=CK[i+1];const f=ease((u-a[0])/(b[0]-a[0]||1));return[a[1].map((x,j)=>lerp(x,b[1][j],f)),a[2].map((x,j)=>lerp(x,b[2][j],f))]}}
  return[b[1],b[2]];
}

/* ================= track picture (confidence and risk) ================= */
const SRCN=[['radio','Radio'],['radar','Radar'],['day','Day camera'],['th','Thermal']];
const TRK=[
 [230,234.5,{id:'01',name:'Ground vehicle',aff:'unk',src:{radar:['ok','Moving down the road']},risk:'Unknown'}],
 [234.5,239.5,{id:'01',name:'Ground vehicle',aff:'unk',src:{radar:['ok','Moving down the road'],day:['ok','Small truck, no weapons seen']},risk:'Assessing'}],
 [239.5,243,{id:'01',name:'Ground vehicle',aff:'unk',src:{radar:['ok','Moving down the road'],day:['ok','Small truck, no weapons seen'],th:['ok','One heat source: engine'],radio:['ok','No military signals']},risk:'Assessing'}],
 [243,250,{id:'01',name:'Civilian truck',aff:'neu',src:{radar:['ok','Moving down the road'],day:['ok','Small truck, no weapons seen'],th:['ok','One heat source: engine'],radio:['ok','No military signals']},risk:'Low',note:'Keep watching. No alert sent.'}],
 [73,79,{id:'02',name:'Air contact',aff:'unk',src:{radio:['ok','Drone control signal, direction only']},risk:'Unknown'}],
 [79,85.5,{id:'02',name:'Air contact',aff:'unk',src:{radio:['ok','Drone control signal, direction only'],radar:['ok','Small, slow, 1 km']},risk:'Unknown'}],
 [85.5,89.5,{id:'02',name:'Air contact',aff:'unk',src:{radio:['ok','Drone control signal'],radar:['ok','Small, slow, 1 km'],day:['ok','Quadcopter']},risk:'Assessing'}],
 [89.5,100,{id:'02',name:'Likely enemy drone',aff:'hos',src:{radio:['ok','Drone control signal'],radar:['ok','Small, slow, 1 km'],day:['ok','Quadcopter'],th:['ok','Warm motors']},risk:'High',note:'Column warned. About 3 min.'}],
 [101,109,{id:'03',name:'Ground group',aff:'unk',src:{radar:['ok','6 slow movers']},risk:'Unknown'}],
 [109,124,{id:'03',name:'Ground group',aff:'unk',src:{radar:['deg','Blocked by ridge']},risk:'Unknown',note:'Track kept. Position estimated.'}],
 [124,128,{id:'03',name:'Ground group',aff:'unk',src:{radar:['deg','Blocked by ridge']},risk:'Unknown',note:'Light drone sent to look.'}],
 [128,131,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['ok','Six tanks']},risk:'High'}],
 [131,134.5,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['ok','Six tanks'],th:['ok','Engines running']},risk:'High',note:'Position not fixed yet'}],
 [134.5,137.5,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['ok','Six tanks'],th:['ok','Engines running']},risk:'High',note:'Crossing two drone sightings'}],
 [137.5,146,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['ok','Six tanks'],th:['ok','Engines running']},risk:'High',note:'Located within metres'}],
 [146,155,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['ok','Six tanks'],th:['ok','Engines running'],radio:['ok','Bearings cross here']},risk:'High',note:'Located within metres'}],
 [155,164,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['deg','Dust, poor view'],th:['ok','Engines running'],radio:['ok','Bearings cross here']},risk:'High',note:'Day camera trusted less in dust.'}],
 [164,174,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['deg','Blocked by ridge'],day:['deg','Dust, poor view'],th:['ok','Light drone following'],radio:['ok','Bearings cross here']},risk:'High',note:'Light drone kept on task. Battery low.'}],
 [174,178,{id:'03',name:'6 tanks',aff:'hos',src:{radar:['ok','Medium drone radar'],day:['deg','Dust, poor view'],th:['ok','Last report in'],radio:['ok','Bearings cross here']},risk:'High',note:'Light drone lost. Heading north, confirmed.'}]
];
function trkAt(u){for(const t of TRK)if(u>=t[0]&&u<t[1])return t;return null}
function trkHTML(s){
  let n=0;const rows=SRCN.map(([k,lab])=>{const x=s.src[k];let m='–',c='na',t='Not yet';if(x){t=x[1];if(x[0]==='ok'){m='✓';c='ok';n++}else{m='!';c='deg'}}
    return `<span class="n">${lab}</span><span class="m ${c}">${m}</span><span class="t ${x?'':'na'}">${t}</span>`}).join('');
  const lv=n>=3?'High':(n===2?'Medium':'Low');
  const rk={Low:1,Assessing:0,Unknown:0,High:3}[s.risk];
  const aff={unk:'Unknown',neu:'Neutral',hos:'Hostile'}[s.aff];
  const segs=(k,on)=>Array.from({length:k},(_,i)=>`<i class="${i<on?'on':''}"></i>`).join('');
  return `<div class="hd"><span class="cap">Track ${s.id}</span><span class="cap" style="color:var(--${s.aff==='hos'?'ho':s.aff==='neu'?'ne':'un'})">${aff}</span></div>
  <h3>${s.name}</h3><div class="src">${rows}</div>
  <div class="meter"><span class="cap">Confidence</span><div class="seg">${segs(4,n)}</div><span class="lvl">${lv}</span></div>
  <div class="meter risk"><span class="cap">Risk</span><div class="seg" style="color:var(--${s.risk==='High'?'ho':s.risk==='Low'?'ne':'un'})">${segs(3,rk)}</div><span class="lvl ${s.risk}">${s.risk}</span></div>
  ${s.note?`<div class="note">${s.note}</div>`:''}`;
}
/* camera window schedule [u0,u1,mode,target,label,range] */
const PIPS=[
 [235,239.5,'day','TR','Truck','0.8 km'],[239.5,247,'th','TR','Truck','0.8 km'],
 [85.5,89.5,'day','HD','Air contact','0.6 km'],[89.5,97,'th','HD','Air contact','0.6 km'],
 [128,131,'day','EN','Tanks','1.5 km'],[131,139,'th','EN','Tanks','1.5 km'],
 [155.5,158.5,'dust','EN','Tanks','1.3 km'],[158.5,173.6,'th','EN','Tanks','1.0 km'],[173.6,178,'lost','EN','Tanks','']
];
function pipAt(u){for(const p of PIPS)if(u>=p[0]&&u<p[1])return p;return null}

/* ================= three.js setup ================= */
const app=$('app'),cvs=$('gl');
let R;
try{R=new THREE.WebGLRenderer({canvas:cvs,antialias:true,powerPreference:'high-performance'})}catch(e){$('loading').textContent='WebGL is not available on this device.';return}
R.setPixelRatio(Math.min(devicePixelRatio,1.5));
R.outputEncoding=THREE.sRGBEncoding;R.toneMapping=THREE.ACESFilmicToneMapping;R.toneMappingExposure=1.0;
R.setClearColor(0x0b0d10,1);R.shadowMap.enabled=true;R.shadowMap.type=THREE.PCFShadowMap;
const S=new THREE.Scene();S.fog=new THREE.FogExp2(0x0b0d10,0.0022);
const cam=new THREE.PerspectiveCamera(38,1,0.5,1200);cam.position.set(-130,95,130);
const ctl=new THREE.OrbitControls(cam,cvs);ctl.enableDamping=true;ctl.dampingFactor=0.08;ctl.maxPolarAngle=1.45;ctl.minDistance=12;ctl.maxDistance=300;
S.add(new THREE.HemisphereLight(0xc9d3dc,0x1d1a14,0.6));
const sun=new THREE.DirectionalLight(0xfff3e3,1.3);sun.position.set(-70,90,-40);sun.castShadow=true;
sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-85,right:85,top:70,bottom:-70,near:10,far:300});sun.shadow.bias=-0.0006;S.add(sun);
const fillL=new THREE.DirectionalLight(0xbfd2e6,0.3);fillL.position.set(80,30,90);S.add(fillL);

const geo=new THREE.PlaneGeometry(W,D,GX-1,GZ-1);geo.rotateX(-Math.PI/2);
const pa=geo.attributes.position;
for(let k=0;k<pa.count;k++){pa.setY(k,H(pa.getX(k),pa.getZ(k)))}
geo.computeVertexNormals();
const nr=geo.attributes.normal,col=new Float32Array(pa.count*3);
const C=h=>new THREE.Color(h).convertSRGBToLinear();
const cValley=C(0x6b6650),cPlain=C(0x7a6f58),cRock=C(0x4a4844),cHigh=C(0xdcdfe2),cSand=C(0x9a8a6c),cRoad=C(0x857d70),cTmp=new THREE.Color();
for(let k=0;k<pa.count;k++){
  const x=pa.getX(k),y=pa.getY(k),z=pa.getZ(k),ny=nr.getY(k);
  const n=fbm(x*0.15,z*0.15,3);
  cTmp.copy(cPlain).lerp(cValley,sm(0.35,0.7,n)*0.7);
  cTmp.lerp(cRock,sm(0.93,0.75,ny));
  cTmp.lerp(cHigh,sm(8,15,y)*sm(0.55,0.9,ny)*0.85);
  const rd=Math.abs(x-RV(z));cTmp.lerp(cSand,sm(4,1.2,rd)*0.85);
  const r=roadDist(x,z);cTmp.lerp(cRoad,sm(1.6,0.6,r)*0.9);
  const sh=0.84+0.28*fbm(x*0.8,z*0.8,2);
  col[k*3]=cTmp.r*sh;col[k*3+1]=cTmp.g*sh;col[k*3+2]=cTmp.b*sh;
}
geo.setAttribute('color',new THREE.BufferAttribute(col,3));
const terrain=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({vertexColors:true,roughness:0.95,metalness:0.0}));
terrain.receiveShadow=true;terrain.castShadow=true;S.add(terrain);
(function skirt(){
  const g=new THREE.BufferGeometry(),pts=[],idx=[];const edge=[];const N=200;
  for(let i=0;i<=N;i++)edge.push([-W/2+W*i/N,-D/2]);
  for(let i=1;i<=N;i++)edge.push([W/2,-D/2+D*i/N]);
  for(let i=1;i<=N;i++)edge.push([W/2-W*i/N,D/2]);
  for(let i=1;i<N;i++)edge.push([-W/2,D/2-D*i/N]);
  edge.forEach(([x,z])=>{pts.push(x,H(x,z),z,x,-3,z)});
  const L=edge.length;for(let i=0;i<L;i++){const a=i*2,b=((i+1)%L)*2;idx.push(a,a+1,b,b,a+1,b+1)}
  g.setAttribute('position',new THREE.Float32BufferAttribute(pts,3));g.setIndex(idx);g.computeVertexNormals();
  S.add(new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:0x34312b,roughness:1,side:THREE.DoubleSide})));
})();
const tableMat=new THREE.MeshStandardMaterial({color:0x17191b,roughness:0.7,metalness:0.3});
const bezel=new THREE.MeshStandardMaterial({color:0x202326,roughness:0.6,metalness:0.4});
function bx(w,h,d,x,y,z,m){const b=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),m);b.position.set(x,y,z);b.receiveShadow=true;S.add(b);return b}
bx(W+18,6,D+18,0,-6,0,tableMat);
bx(W+18,3.4,4,0,-1.4,D/2+7,bezel);bx(W+18,3.4,4,0,-1.4,-D/2-7,bezel);bx(4,3.4,D+10,W/2+7,-1.4,0,bezel);bx(4,3.4,D+10,-W/2-7,-1.4,0,bezel);
bx(W+10,1,D+10,0,-3,0,tableMat);
(function(){
  const c=document.createElement('canvas');c.width=2048;c.height=96;const g=c.getContext('2d');
  g.fillStyle='#16181a';g.fillRect(0,0,2048,96);
  g.fillStyle='#bfc2c4';g.font='600 36px "Barlow Condensed","Arial Narrow",sans-serif';g.fillText('ONEVIEW',40,60);
  g.fillStyle='#7b7f83';g.font='600 28px "Barlow Condensed","Arial Narrow",sans-serif';g.fillText('SCENARIO 01   VALLEY COLUMN   FEDS',260,59);
  const t=new THREE.CanvasTexture(c);t.encoding=THREE.sRGBEncoding;
  const m=new THREE.Mesh(new THREE.PlaneGeometry(W+10,4.6),new THREE.MeshBasicMaterial({map:t}));
  m.position.set(0,-4,D/2+9.02);S.add(m);
})();
const floor=new THREE.Mesh(new THREE.PlaneGeometry(900,900),new THREE.MeshStandardMaterial({color:0x0b0d10,roughness:1}));floor.rotation.x=-Math.PI/2;floor.position.y=-9;floor.receiveShadow=true;S.add(floor);

/* ================= map overlay drawn on the terrain ================= */
const OCW=2048,OCH=1317;
const oc=document.createElement('canvas');oc.width=OCW;oc.height=OCH;const og=oc.getContext('2d');
const P=(x,z)=>[(x+W/2)/W*OCW,(z+D/2)/D*OCH];const U=OCW/W;
const contour=document.createElement('canvas');contour.width=OCW;contour.height=OCH;
(function(){
  const g=contour.getContext('2d');const step=1.6,levels=[];for(let l=0;l<26;l+=step)levels.push(l);
  const nx=200,nz=129,dx=W/nx,dz=D/nz;
  for(const lv of levels){
    const major=Math.abs((lv/step)%5)<0.01;
    g.strokeStyle=major?'rgba(255,255,255,0.20)':'rgba(255,255,255,0.08)';g.lineWidth=major?2:1.1;g.beginPath();
    for(let j=0;j<nz;j++)for(let i=0;i<nx;i++){
      const x0=-W/2+i*dx,z0=-D/2+j*dz;
      const a=H(x0,z0)-lv,b=H(x0+dx,z0)-lv,c=H(x0+dx,z0+dz)-lv,d=H(x0,z0+dz)-lv;
      const e=[];const it=(p,q,x1,z1,x2,z2)=>{if((p>0)!==(q>0)){const f=p/(p-q);e.push([x1+(x2-x1)*f,z1+(z2-z1)*f])}};
      it(a,b,x0,z0,x0+dx,z0);it(b,c,x0+dx,z0,x0+dx,z0+dz);it(d,c,x0,z0+dz,x0+dx,z0+dz);it(a,d,x0,z0,x0,z0+dz);
      if(e.length>=2){const[p1,p2]=[P(...e[0]),P(...e[1])];g.moveTo(p1[0],p1[1]);g.lineTo(p2[0],p2[1]);if(e.length===4){const[p3,p4]=[P(...e[2]),P(...e[3])];g.moveTo(p3[0],p3[1]);g.lineTo(p4[0],p4[1])}}
    }
    g.stroke();
  }
})();
const G1Y=H(G1P[0],G1P[1])+7;
const VS=document.createElement('canvas');VS.width=OCW;VS.height=OCH;
(function(){
  const g=VS.getContext('2d');const R2=95*95;
  const img=g.createImageData(W,D);
  for(let j=0;j<img.height;j++)for(let i=0;i<img.width;i++){
    const x=-W/2+(i+.5),z=-D/2+(j+.5);const dx=x-G1P[0],dz=z-G1P[1];
    if(dx*dx+dz*dz>R2)continue;
    const ty=H(x,z)+1.2;let hid=false;
    for(let s=1;s<70;s++){const f=s/70;if(H(G1P[0]+dx*f,G1P[1]+dz*f)>G1Y+(ty-G1Y)*f){hid=true;break}}
    if(hid){const o=(j*img.width+i)*4;img.data[o]=8;img.data[o+1]=9;img.data[o+2]=10;img.data[o+3]=130}
  }
  const tmp=document.createElement('canvas');tmp.width=img.width;tmp.height=img.height;tmp.getContext('2d').putImageData(img,0,0);
  g.imageSmoothingEnabled=true;g.drawImage(tmp,0,0,OCW,OCH);
  g.globalCompositeOperation='source-atop';g.strokeStyle='rgba(255,255,255,0.22)';g.lineWidth=2;
  for(let k=-OCH;k<OCW;k+=20){g.beginPath();g.moveTo(k,0);g.lineTo(k+OCH,OCH);g.stroke()}
})();
const ovTex=new THREE.CanvasTexture(oc);ovTex.encoding=THREE.sRGBEncoding;ovTex.anisotropy=R.capabilities.getMaxAnisotropy();
const ovMesh=new THREE.Mesh(geo,new THREE.MeshBasicMaterial({map:ovTex,transparent:true,depthWrite:false,polygonOffset:true,polygonOffsetFactor:-2,polygonOffsetUnits:-2,toneMapped:false}));
ovMesh.renderOrder=2;S.add(ovMesh);

const CF='77,163,255',CH='255,74,61',CN='63,210,127',CU='255,216,74',CO='255,154,31',CW='255,255,255';
function lineP(pts){og.beginPath();pts.forEach((p,i)=>{const[a,b]=P(p[0],p[1]);i?og.lineTo(a,b):og.moveTo(a,b)})}
function circ(x,z,r,stroke,fill,dash,w){const[a,b]=P(x,z);og.beginPath();og.arc(a,b,r*U,0,Math.PI*2);if(fill){og.fillStyle=fill;og.fill()}og.setLineDash(dash||[]);og.strokeStyle=stroke;og.lineWidth=w||3;og.stroke();og.setLineDash([])}
function txt(x,z,s,color){const[a,b]=P(x,z);og.font='600 28px "Barlow Condensed","Arial Narrow",sans-serif';og.fillStyle=color;og.textAlign='center';og.fillText(s,a,b)}
function seg(x1,z1,x2,z2,stroke,dash,w){lineP([[x1,z1],[x2,z2]]);og.setLineDash(dash||[]);og.strokeStyle=stroke;og.lineWidth=w||3;og.stroke();og.setLineDash([])}
const rgba=(c,a)=>`rgba(${c},${a})`;

function drawOverlay(u,T){
  og.clearRect(0,0,OCW,OCH);og.drawImage(contour,0,0);
  const rp=[];for(let z=-45;z<=45;z+=1.5)rp.push([RV(z),z]);
  lineP(rp);og.strokeStyle='rgba(255,255,255,0.18)';og.lineWidth=0.6*U;og.lineCap='round';og.lineJoin='round';og.stroke();
  lineP(ROAD);og.strokeStyle='rgba(255,255,255,0.30)';og.lineWidth=0.3*U;og.setLineDash([18,12]);og.stroke();og.setLineDash([]);
  if(win(u,10,40)){
    circ(G1P[0],G1P[1],20,rgba(CF,.6),rgba(CF,.04),[14,10],2.5);
    for(let i=0;i<4;i++){const p=vehPos(i,T);circ(p.x,p.z,6,rgba(CF,.5),null,null,2)}
  }
  if(win(u,230,250)){const tr=TRat(T);if(tr&&u<236)seg(G1P[0],G1P[1],tr.x,tr.z,rgba(CF,.7),[10,8],2.5);
    if(tr){const c=u<243?CU:CN;circ(tr.x,tr.z,3.2,rgba(c,.9),rgba(c,.08),[6,6],2.5)}}
  if(win(u,72,100)){
    const l=L1at(T);const hd=HDat(T);
    circ(l.x,l.z,40,rgba(CF,.45),rgba(CF,.025),[6,10],2.5);
    if(u>=78)circ(G1P[0],G1P[1],20,rgba(CF,.65),rgba(CF,.04),[12,8],2.5);
    if(hd&&u<85){const dx=hd.x-l.x,dz=hd.z-l.z;seg(l.x,l.z,l.x+dx*1.25,l.z+dz*1.25,rgba(CU,.85),[8,10],2.5)}
  }
  if(T>=8&&u<226){const tr=[];for(let t=8;t<=T;t+=0.25){const p=HDat(t);tr.push([p.x,p.z])}
    if(tr.length>1){lineP(tr);og.setLineDash([4,9]);og.strokeStyle=rgba(u>=89.5?CH:CU,.85);og.lineWidth=3;og.stroke();og.setLineDash([])}}
  if(u>=100&&u<212){
    og.globalAlpha=win(u,100,140)?0.95:0.4;og.drawImage(VS,0,0);og.globalAlpha=1;
    const z0=enLeadZ(T);const hc=u>=128?CH:CU;
    const seen=[];for(let z=Math.max(LOSTZ,z0);z<=Math.min(45,z0+22);z+=1)seen.push([RV(z),z]);
    if(seen.length>1){lineP(seen);og.strokeStyle=rgba(hc,.16);og.lineWidth=4.4*U;og.lineCap='round';og.stroke()}
    if(z0<LOSTZ&&u<128){const hp=[];for(let z=LOSTZ;z>=z0-6;z-=1)hp.push([RV(z),z]);lineP(hp);og.setLineDash([3,8]);og.strokeStyle=rgba(CU,.9);og.lineWidth=3;og.stroke();og.setLineDash([]);
      circ(RV(LOSTZ),LOSTZ,2.4,rgba(CU,.9),null,[5,5],2.5)}
  }
  if(false){const a=0;const L=[],Rr=[];for(let z=10;z<=42;z+=1.5){L.push([RV(z)-4.2,z]);Rr.unshift([RV(z)+4.2,z])}
    lineP(L.concat(Rr));og.closePath();og.fillStyle=rgba(CO,.10*a);og.fill();og.setLineDash([14,10]);og.strokeStyle=rgba(CO,.9*a);og.lineWidth=3;og.stroke();og.setLineDash([]);
    og.save();og.clip();og.strokeStyle=rgba(CW,.22*a);og.lineWidth=2;for(let k=-OCH;k<OCW;k+=20){og.beginPath();og.moveTo(k,0);og.lineTo(k+OCH,OCH);og.stroke()}og.restore();}
  if(win(u,118,140)){
    if(u<128){lineP([[ST1[0],ST1[1]],[(ST1[0]+ST2[0])/2-2,(ST1[1]+ST2[1])/2],[ST2[0],ST2[1]]]);og.setLineDash([12,10]);og.strokeStyle=rgba(CO,.95);og.lineWidth=4;og.stroke();og.setLineDash([]);circ(ST2[0],ST2[1],1.6,rgba(CO,.95),null,null,3)}
    if(u>=128){const f=prog(u,136.6,138.6);const r=lerp(4.2,1.6,f);for(let i=0;i<6;i++){const[x,z]=tankXZ(i,T);circ(x,z,r,rgba(CH,.9),rgba(CH,.05),[6,6],2.5)}
      if(u>=134.6){const[tx,tz]=tankXZ(2,T);const g=prog(u,134.8,136.6);const ls=L1at(T),ms=M1at(T);
        [[ls.x,ls.z],[ms.x,ms.z]].forEach(([sx,sz])=>{const dx=tx-sx,dz=tz-sz,k=1.12*Math.max(g,0.02);seg(sx,sz,sx+dx*k,sz+dz*k,rgba(CO,1),[],5);circ(sx,sz,2.2,rgba(CO,.95),rgba(CO,.12),null,2.5)})}
      if(f>=1){const[x,z]=tankXZ(5,T);txt(x+9,z+3,'LOCATED','rgba(255,200,195,.95)')}}
  }
  if(win(u,140,155)){
    const[tx,tz]=tankXZ(2,T);const v1=vehPos(0,T);const f=prog(u,144,148);
    [[G1P[0],G1P[1]],[v1.x,v1.z]].forEach(([sx,sz])=>{const dx=tx-sx,dz=tz-sz;seg(sx,sz,sx+dx*1.15*Math.max(f,0.02),sz+dz*1.15*Math.max(f,0.02),rgba(CO,.9),[10,9],3)});
    if(u>=148){const[a,b]=P(tx,tz);og.beginPath();og.ellipse(a,b,3.8*U,2.4*U,-0.4,0,Math.PI*2);og.fillStyle=rgba(CO,.15);og.fill();og.strokeStyle=rgba(CO,1);og.lineWidth=3;og.stroke()}
  }
  if(false){
    const fx=lerp(60,10,prog(u,155,161));
    og.fillStyle='rgba(190,150,90,0.12)';og.beginPath();const[sx]=P(fx,0);og.rect(sx,0,OCW-sx,OCH);og.fill();
    const fp=[];for(let z=-45;z<=45;z+=3)fp.push([fx+2.5*Math.sin(z*0.2),z]);lineP(fp);og.setLineDash([20,10]);og.strokeStyle=rgba(CU,.85);og.lineWidth=4;og.stroke();og.setLineDash([]);
    if(win(u,164,178)){const tr=[];for(let t=26.2;t<=Math.min(T,29.45);t+=0.1){const p=L1at(t);tr.push([p.x,p.z])}if(tr.length>1){lineP(tr);og.setLineDash([10,8]);og.strokeStyle=rgba(CO,.9);og.lineWidth=3;og.stroke();og.setLineDash([])}}
  }
}

/* ================= models ================= */
const mOwn=new THREE.MeshStandardMaterial({color:0x56657a,roughness:0.65,metalness:0.3});
const mOwnD=new THREE.MeshStandardMaterial({color:0x2f3742,roughness:0.75,metalness:0.25});
const mHos=new THREE.MeshStandardMaterial({color:0x5a4f3a,roughness:0.8,metalness:0.15});
const mHosD=new THREE.MeshStandardMaterial({color:0x332c22,roughness:0.85});
const mNode=new THREE.MeshStandardMaterial({color:0xb9bdc1,roughness:0.45,metalness:0.5});
const mPod=new THREE.MeshStandardMaterial({color:0x2a2d30,roughness:0.5,metalness:0.4});
const mCiv=new THREE.MeshStandardMaterial({color:0xd8d6cf,roughness:0.6,metalness:0.2});
const mDead=new THREE.MeshStandardMaterial({color:0x3a3b3c,roughness:0.9});
function shadowAll(o){o.traverse(c=>{if(c.isMesh){c.castShadow=true;c.receiveShadow=true}});return o}
function hot(o){o.traverse(c=>{if(c.isMesh)c.userData.hot=true});return o}
function apc(){const g=new THREE.Group();
  const hull=new THREE.Mesh(new THREE.BoxGeometry(3.2,0.8,1.7),mOwn);hull.position.y=0.75;g.add(hull);
  const top=new THREE.Mesh(new THREE.BoxGeometry(2.0,0.5,1.4),mOwn);top.position.set(-0.3,1.35,0);g.add(top);
  const nose=new THREE.Mesh(new THREE.BoxGeometry(0.7,0.5,1.6),mOwn);nose.position.set(1.65,0.6,0);nose.rotation.z=0.5;g.add(nose);
  [-0.8,0.8].forEach(zz=>{const tr=new THREE.Mesh(new THREE.BoxGeometry(3.3,0.55,0.35),mOwnD);tr.position.set(0,0.35,zz);g.add(tr)});
  const mast=new THREE.Mesh(new THREE.CylinderGeometry(0.05,0.05,1.1,6),mNode);mast.position.set(-0.8,2.1,0.3);g.add(mast);
  const ws=new THREE.Mesh(new THREE.CylinderGeometry(0.18,0.18,0.12,10),mNode);ws.position.set(-0.8,2.7,0.3);g.add(ws);
  return hot(shadowAll(g))}
function tank(){const g=new THREE.Group();
  const hull=new THREE.Mesh(new THREE.BoxGeometry(3.4,0.7,1.9),mHos);hull.position.y=0.7;g.add(hull);
  [-0.85,0.85].forEach(zz=>{const tr=new THREE.Mesh(new THREE.BoxGeometry(3.5,0.6,0.4),mHosD);tr.position.set(0,0.35,zz);g.add(tr)});
  const tur=new THREE.Mesh(new THREE.CylinderGeometry(0.75,0.85,0.5,8),mHos);tur.position.set(-0.2,1.3,0);g.add(tur);
  const gun=new THREE.Mesh(new THREE.CylinderGeometry(0.08,0.09,2.6,8),mHosD);gun.rotation.z=Math.PI/2;gun.position.set(1.5,1.35,0);g.add(gun);
  return hot(shadowAll(g))}
function truck(){const g=new THREE.Group();
  const cab=new THREE.Mesh(new THREE.BoxGeometry(0.9,0.9,1.3),mCiv);cab.position.set(1.0,0.75,0);g.add(cab);
  const box=new THREE.Mesh(new THREE.BoxGeometry(1.9,1.2,1.4),new THREE.MeshStandardMaterial({color:0x8f8a7c,roughness:0.8}));box.position.set(-0.45,0.9,0);g.add(box);
  [[1,0.6],[1,-0.6],[-0.9,0.6],[-0.9,-0.6]].forEach(w=>{const wh=new THREE.Mesh(new THREE.CylinderGeometry(0.25,0.25,0.2,10),mOwnD);wh.rotation.x=Math.PI/2;wh.position.set(w[0],0.25,w[1]);g.add(wh)});
  shadowAll(g);cab.userData.hot=true;return g}
function quad(mat,s,podW,radar){const g=new THREE.Group();
  const a1=new THREE.Mesh(new THREE.BoxGeometry(3.2,0.16,0.26),mat);a1.rotation.y=Math.PI/4;const a2=a1.clone();a2.rotation.y=-Math.PI/4;
  const body=new THREE.Mesh(new THREE.BoxGeometry(0.9,0.42,0.9),mat);g.add(a1,a2,body);
  const rm=new THREE.MeshBasicMaterial({color:0xd0d4d8,transparent:true,opacity:0.25,depthWrite:false});
  [[1.1,1.1],[-1.1,1.1],[1.1,-1.1],[-1.1,-1.1]].forEach(r=>{const ro=new THREE.Mesh(new THREE.CylinderGeometry(0.62,0.62,0.04,20),rm);ro.position.set(r[0],0.2,r[1]);ro.userData.rotor=true;g.add(ro)});
  if(podW){const pod=new THREE.Mesh(new THREE.BoxGeometry(podW,podW*0.55,podW*0.6),mPod);pod.position.y=-0.45;g.add(pod);
    if(radar){const rd=new THREE.Mesh(new THREE.BoxGeometry(0.12,podW*0.5,podW*0.55),new THREE.MeshStandardMaterial({color:0xe8eaec,roughness:0.6}));rd.position.set(podW/2+0.06,-0.45,0);g.add(rd)}}
  g.scale.setScalar(s||1);return hot(shadowAll(g))}
function node(){const g=new THREE.Group();
  const base=new THREE.Mesh(new THREE.BoxGeometry(2.4,0.8,2.4),mNode);base.position.y=0.4;g.add(base);
  const mast=new THREE.Mesh(new THREE.CylinderGeometry(0.14,0.18,6.2,8),mNode);mast.position.y=3.8;g.add(mast);
  const dome=new THREE.Mesh(new THREE.SphereGeometry(0.75,16,12),mNode);dome.position.y=7.2;g.add(dome);
  const panel=new THREE.Mesh(new THREE.BoxGeometry(1.6,1.0,0.12),mNode);panel.position.set(0,5.9,0.5);g.add(panel);
  const vane=new THREE.Mesh(new THREE.BoxGeometry(1.4,0.06,0.06),mNode);vane.position.y=8.1;g.add(vane);
  return shadowAll(g)}

const HELP=[];
function ringTex(){const c=document.createElement('canvas');c.width=c.height=256;const g=c.getContext('2d');
  g.strokeStyle='#fff';g.lineWidth=10;g.beginPath();g.arc(128,128,116,0,Math.PI*2);g.stroke();
  g.lineWidth=4;g.setLineDash([14,12]);g.beginPath();g.arc(128,128,96,0,Math.PI*2);g.stroke();return new THREE.CanvasTexture(c)}
const RT=ringTex();
function ring(color,r){const m=new THREE.Mesh(new THREE.PlaneGeometry(1,1),new THREE.MeshBasicMaterial({map:RT,color,transparent:true,opacity:.85,depthWrite:false,toneMapped:false}));
  m.rotation.x=-Math.PI/2;m.scale.set(r*2,r*2,1);m.renderOrder=3;S.add(m);HELP.push(m);return m}
const pinMat=new THREE.LineBasicMaterial({color:0xd8dcdf,transparent:true,opacity:0.6});
function pin(){const g=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(0,1,0)]);const l=new THREE.Line(g,pinMat);S.add(l);HELP.push(l);return l}
const pktTex=(function(){const c=document.createElement('canvas');c.width=c.height=32;const g=c.getContext('2d');g.fillStyle='#fff';g.fillRect(10,10,12,12);return new THREE.CanvasTexture(c)})();
const links=[];
function link(color){const curve=new THREE.QuadraticBezierCurve3(new THREE.Vector3(),new THREE.Vector3(),new THREE.Vector3());
  const g=new THREE.BufferGeometry().setFromPoints(curve.getPoints(40));
  const line=new THREE.Line(g,new THREE.LineBasicMaterial({color,transparent:true,opacity:0.6,toneMapped:false}));S.add(line);HELP.push(line);
  const pk=[0,1].map(()=>{const s=new THREE.Sprite(new THREE.SpriteMaterial({map:pktTex,color,transparent:true,depthWrite:false,toneMapped:false}));s.scale.setScalar(0.8);S.add(s);HELP.push(s);return s});
  const L={curve,line,pk,on:false,ph:links.length*0.37%1};links.push(L);return L}
function setLink(L,a,b,lift){const m=a.clone().lerp(b,0.5);m.y=Math.max(a.y,b.y)+(lift==null?a.distanceTo(b)*0.22:lift);L.curve.v0.copy(a);L.curve.v1.copy(m);L.curve.v2.copy(b);
  const p=L.curve.getPoints(40);const pos=L.line.geometry.attributes.position;for(let i=0;i<p.length;i++)pos.setXYZ(i,p[i].x,p[i].y,p[i].z);pos.needsUpdate=true;L.line.geometry.computeBoundingSphere();L.on=true}
function dline(color){const g=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]);const l=new THREE.Line(g,new THREE.LineDashedMaterial({color,dashSize:0.8,gapSize:0.6,transparent:true,opacity:0.7,toneMapped:false}));S.add(l);HELP.push(l);return l}
function setD(l,a,b){const p=l.geometry.attributes.position;p.setXYZ(0,a.x,a.y,a.z);p.setXYZ(1,b.x,b.y,b.z);p.needsUpdate=true;l.computeLineDistances();l.geometry.computeBoundingSphere();l.visible=true}
function cone(color,op){const g=new THREE.ConeGeometry(1,1,32,1,true);g.translate(0,-0.5,0);const m=new THREE.Mesh(g,new THREE.MeshBasicMaterial({color,transparent:true,opacity:op,depthWrite:false,side:THREE.DoubleSide,toneMapped:false}));S.add(m);HELP.push(m);return m}
function aimCone(c,from,to,w){const dir=to.clone().sub(from);const len=dir.length();c.position.copy(from);c.scale.set(w,len,w);c.quaternion.setFromUnitVectors(new THREE.Vector3(0,-1,0),dir.normalize());c.visible=true}

const FR=0x4da3ff,HO=0xff4a3d,NE=0x3fd27f,UN=0xffd84a,OV=0xff9a1f;
const V=[0,1,2,3].map(()=>{const m=apc();S.add(m);return m});
const Vring=V.map(()=>ring(FR,2.6));
const G1=node();G1.position.set(G1P[0],H(G1P[0],G1P[1]),G1P[1]);S.add(G1);
const G1top=G1.position.clone().add(new THREE.Vector3(0,7.2,0));
const mL1=new THREE.MeshStandardMaterial({color:0x8a97a6,roughness:0.5,metalness:0.4});
const L1=quad(mL1,0.85,0.7,false);S.add(L1);
const M1=quad(new THREE.MeshStandardMaterial({color:0x7a8796,roughness:0.5,metalness:0.4}),1.35,0.95,true);S.add(M1);
const L1drop=dline(FR),M1drop=dline(FR),HDdrop=dline(HO);
const L1ring=ring(OV,3),M1ring=ring(OV,3.4);
const HD=quad(new THREE.MeshStandardMaterial({color:0x8a4a40,roughness:0.5,metalness:0.3}),0.9);S.add(HD);
const HDring=ring(UN,3.4);
const EN=[0,1,2,3,4,5].map(()=>{const m=tank();S.add(m);return m});
const ENring=EN.map(()=>ring(UN,2.6));
const TR=truck();S.add(TR);const TRring=ring(UN,2.8);
const eoCone=cone(0xffffff,0.07),radCone=cone(FR,0.08);
const rangeL=dline(HO),altL=dline(OV);
function arrow(color){const a=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),6,color,1.6,1);a.line.material.toneMapped=false;a.cone.material.toneMapped=false;S.add(a);HELP.push(a);return a}
const windV=arrow(UN),windG=arrow(FR);
const dustTex=(function(){const c=document.createElement('canvas');c.width=c.height=128;const g=c.getContext('2d');const gr=g.createRadialGradient(64,64,0,64,64,64);gr.addColorStop(0,'rgba(255,255,255,.5)');gr.addColorStop(1,'rgba(255,255,255,0)');g.fillStyle=gr;g.fillRect(0,0,128,128);return new THREE.CanvasTexture(c)})();
const dust=[];for(let i=0;i<46;i++){const s=new THREE.Sprite(new THREE.SpriteMaterial({map:dustTex,color:0xb08a5a,transparent:true,opacity:0,depthWrite:false}));const x=14+hash(i,1)*56,z=-42+hash(i,2)*84;s.position.set(x,H(x,z)+2+hash(i,3)*4,z);s.scale.setScalar(14+hash(i,4)*12);s.userData={x,z};S.add(s);dust.push(s)}
function recv(){const g=new THREE.Group();const b=new THREE.Mesh(new THREE.CylinderGeometry(1.6,1.6,0.4,6),new THREE.MeshStandardMaterial({color:0x2a2e33,metalness:0.5,roughness:0.5}));g.add(b);const t=new THREE.Mesh(new THREE.CylinderGeometry(1.2,1.2,0.12,6),new THREE.MeshBasicMaterial({color:FR,toneMapped:false}));t.position.y=0.26;g.add(t);S.add(g);return g}
const RX={CP:recv(),ADJ:recv(),TRL:recv()};
RX.CP.position.set(-82,1,-30);RX.ADJ.position.set(30,1,-57);RX.TRL.position.set(-82,1,44);
const LK={v:[0,1,2,3].map(()=>link(FR)),l1:link(FR),m1:link(FR),cp:link(FR),adj:link(FR),trl:link(FR)};
const LB={V1:{},V2:{},V3:{},V4:{},G1:{},L1:{},M1:{},HD:{},EN:{},TR:{}};
let playing=false,lastKey='',lastDraw=0;
const tmp=new THREE.Vector3();
function place(o,x,z,y){o.position.set(x,y==null?H(x,z):y,z)}
function orient(o,dx,dz){o.rotation.y=Math.atan2(-dz,dx)}
function pulse(r,base,now,sp){const k=(now*(sp||0.0008))%1;r.scale.set(base*2*(0.9+0.25*k),base*2*(0.9+0.25*k),1);r.material.opacity=0.9-k*0.5}
const inTable=(x,z)=>Math.abs(x)<W/2-1&&Math.abs(z)<D/2-1;
function project(p,w,h){tmp.copy(p).project(cam);return{x:(tmp.x+1)/2*w,y:(1-tmp.y)/2*h,ok:tmp.z<1&&tmp.z>-1&&Math.abs(tmp.x)<1.05&&Math.abs(tmp.y)<1.05}}

function update(u,now){
  const T=simT(u);const MAIN=u<226;
  V.forEach((m,i)=>{const p=vehPos(i,T);const ok=inTable(p.x,p.z);m.visible=ok;place(m,p.x,p.z);orient(m,p.dx,p.dz);
    const r=Vring[i];r.visible=ok&&(win(u,10,17)||win(u,198,212));r.position.set(p.x,H(p.x,p.z)+0.25,p.z);pulse(r,2.6,now+i*300);
    LB['V'+(i+1)]._p=ok&&(u<52||win(u,198,212))?new THREE.Vector3(p.x,H(p.x,p.z)+3.2,p.z):null});
  LB.G1._p=G1.position.clone().add(new THREE.Vector3(0,9.4,0));
  // light drone
  const l=L1at(T);const alive=T<L1DEAD,down=T>=29.45;
  place(L1,l.x,l.z,l.y+(alive?Math.sin(now/700)*0.2:0));
  L1.rotation.set(down?0.5:0,alive?T*0.6:L1.rotation.y,down?0.35:0);
  L1.traverse(c=>{if(c.isMesh&&!c.userData.rotor&&c.material!==mPod)c.material=down?mDead:mL1;if(c.userData.rotor)c.visible=!down});
  if(!down)setD(L1drop,new THREE.Vector3(l.x,H(l.x,l.z),l.z),L1.position);else L1drop.visible=false;
  L1ring.visible=win(u,22.5,28.5)||win(u,85,89.5)||win(u,118,128)||win(u,234.5,239.5);L1ring.position.set(l.x,H(l.x,l.z)+0.3,l.z);pulse(L1ring,3,now);
  LB.L1._p=L1.position.clone().add(new THREE.Vector3(0,1.6,0));LB.L1.className='lbl'+(down?' x':'');
  // medium drone
  const m=M1at(T);place(M1,m.x,m.z,m.y+Math.sin(now/800)*0.2);M1.rotation.y=T*0.4;
  setD(M1drop,new THREE.Vector3(m.x,H(m.x,m.z),m.z),M1.position);
  M1ring.visible=win(u,28.5,34.5)||win(u,169,178);M1ring.position.set(m.x,H(m.x,m.z)+0.3,m.z);pulse(M1ring,3.4,now);
  LB.M1._p=M1.position.clone().add(new THREE.Vector3(0,2.2,0));
  // truck
  const tr=TRat(T);TR.visible=!!tr;
  if(tr){place(TR,tr.x,tr.z);orient(TR,tr.dx,tr.dz);TRring.visible=win(u,230,250);TRring.material.color.set(u>=243?NE:UN);TRring.position.set(tr.x,H(tr.x,tr.z)+0.3,tr.z);pulse(TRring,2.8,now);
    LB.TR._p=win(u,230,250)?TR.position.clone().add(new THREE.Vector3(0,2.6,0)):null;LB.TR.textContent=u>=243?'Track 01 · Civilian':'Track 01';LB.TR.className='lbl'+(u>=243?' n':'')}
  else{TRring.visible=false;LB.TR._p=null}
  // enemy drone
  const hd=HDat(T);HD.visible=!!hd;
  if(hd){HD.position.set(hd.x,hd.y+Math.sin(now/500)*0.2,hd.z);HD.rotation.y=now*0.001;setD(HDdrop,new THREE.Vector3(hd.x,H(hd.x,hd.z),hd.z),HD.position);
    HDring.visible=u>=72&&MAIN;HDring.material.color.set(u<89.5?UN:HO);HDring.position.set(hd.x,H(hd.x,hd.z)+0.3,hd.z);pulse(HDring,3.4,now);
    LB.HD._p=u>=73&&u<212?HD.position.clone().add(new THREE.Vector3(0,1.6,0)):null;LB.HD.className='lbl'+(u>=89.5?' h':'')}
  else{HDdrop.visible=false;HDring.visible=false;LB.HD._p=null}
  // tanks
  EN.forEach((t,i)=>{const[x,z]=tankXZ(i,T);const ok=T>=13&&inTable(x,z);t.visible=ok;if(ok){place(t,x,z);orient(t,RV(z-1)-x,-1)}
    const r=ENring[i];r.visible=ok&&u>=100&&u<212&&(z>=LOSTZ||u>=128);r.material.color.set(u>=128?HO:UN);r.position.set(x,H(x,z)+0.3,z);pulse(r,2.6,now+i*180)});
  LB.EN._p=u>=101&&u<212&&EN[2].visible?EN[2].position.clone().add(new THREE.Vector3(0,3,0)):null;LB.EN.className='lbl'+(u>=128?' h':'');
  // cones
  eoCone.visible=false;radCone.visible=false;rangeL.visible=false;altL.visible=false;
  const pp=pipAt(u);
  if(pp&&pp[2]!=='lost'){const tgt=pp[3]==='TR'?TR.position:pp[3]==='HD'?HD.position:EN[2].position.clone().add(new THREE.Vector3(0,0.8,0));aimCone(eoCone,L1.position,tgt,pp[3]==='EN'?4:1.4);if(win(u,129,140))setD(rangeL,L1.position,tgt)}
  if(false)aimCone(radCone,M1.position,EN[2].position,7);
  // wind arrows
  windV.visible=windG.visible=false;
  if(win(u,53,72)){
    const vp=vehPos(0,T);const base=V[0].position.clone().add(new THREE.Vector3(0,3.2,0));
    const tw=new THREE.Vector3(-1,0,0.25).normalize();const vel=new THREE.Vector3(vp.dx,0,vp.dz).normalize();
    const app=tw.clone().multiplyScalar(15).sub(vel.clone().multiplyScalar(20));
    const f=prog(u,62,64.5);const cur=app.clone().lerp(tw.clone().multiplyScalar(15),f);const len=cur.length();
    windV.visible=true;windV.position.copy(base).sub(cur.clone().normalize().multiplyScalar(len*0.15));windV.setDirection(cur.clone().normalize());windV.setLength(len*0.3,1.6,1.1);
    windV.setColor(new THREE.Color(f<1?UN:FR));
    if(u>=63){windG.visible=true;windG.position.copy(G1top).add(new THREE.Vector3(2.5,2,0));windG.setDirection(tw);windG.setLength(4.5,1.6,1.1)}
  }
  // dust
  const dA=false?(u<178?0.28:0.14)*prog(u,155,161):0;
  dust.forEach(s=>{const o=s.userData;const x=o.x-((now/1000)%40)*0.4;s.position.x=x<12?x+56:x;s.material.opacity=dA;s.visible=dA>0.005});
  Object.values(RX).forEach(r=>{r.visible=win(u,198,212);r.rotation.y=now*0.0006});
  links.forEach(L=>L.on=false);
  if(win(u,34.5,40)){V.forEach((mm,i)=>{if(mm.visible)setLink(LK.v[i],mm.position.clone().add(new THREE.Vector3(0,1.8,0)),G1top)})}
  if(alive&&pp&&pp[2]!=='lost')setLink(LK.l1,L1.position,G1top);
  if(win(u,173,178))setLink(LK.m1,M1.position,G1top);
  if(win(u,198,212)){setLink(LK.cp,G1top,RX.CP.position);setLink(LK.adj,G1top,RX.ADJ.position);setLink(LK.trl,G1top,RX.TRL.position)}
  links.forEach(L=>{L.line.visible=L.on;L.pk.forEach((p,j)=>{p.visible=L.on;if(L.on){const f=((now*0.0004)+L.ph+j/2)%1;p.position.copy(L.curve.getPoint(f))}})});
  const key=segOfU(u)+'|'+Math.round(T*6)+'|'+Math.floor(u*2);
  if(key!==lastKey&&(!playing||now-lastDraw>160)){lastKey=key;lastDraw=now;drawOverlay(u,T);ovTex.needsUpdate=true}
  return T;
}
const U0=133;                                  /* "Six tanks, engines running" */
const HOME_P=new THREE.Vector3(-70,82,124), HOME_T=new THREE.Vector3(20,0,14);
cam.position.copy(HOME_P);ctl.target.copy(HOME_T);
ctl.enableDamping=true;ctl.dampingFactor=0.06;
ctl.rotateSpeed=0.55;ctl.panSpeed=0.7;ctl.zoomSpeed=0.7;
ctl.minDistance=45;ctl.maxDistance=230;ctl.minPolarAngle=0.18;ctl.maxPolarAngle=1.22;
ctl.screenSpacePanning=false;ctl.enableZoom=false;
const REDUCED=matchMedia('(prefers-reduced-motion: reduce)').matches;
const SPIN=0.32;                               /* full turn in about three minutes */
ctl.autoRotate=!REDUCED;ctl.autoRotateSpeed=0;
let busy=false,lastTouch=-1e9,tween=null,seen=true;

const hint=$('hint'),toast=$('toast');
let hintGone=false;function dropHint(){if(!hintGone){hintGone=true;hint.classList.add('gone')}}
ctl.addEventListener('start',()=>{busy=true;tween=null;dropHint()});
ctl.addEventListener('end',()=>{busy=false;lastTouch=performance.now()});

let toastT=0;
app.addEventListener('wheel',e=>{
  const z=e.ctrlKey||e.metaKey;ctl.enableZoom=z;
  if(z){dropHint();lastTouch=performance.now()}
  else if(e.target===cvs){toast.classList.add('on');clearTimeout(toastT);toastT=setTimeout(()=>toast.classList.remove('on'),1400)}
},{capture:true,passive:true});

function go(p,t,dur){tween={fp:cam.position.clone(),ft:ctl.target.clone(),tp:p,tt:t,t0:performance.now(),dur:dur||1200};lastTouch=performance.now()+dur}
function zoomBy(f){const off=cam.position.clone().sub(ctl.target);const d=clamp(off.length()*f,ctl.minDistance,ctl.maxDistance);go(ctl.target.clone().add(off.setLength(d)),ctl.target.clone(),600);dropHint()}
$('zin').onclick=()=>zoomBy(0.75);
$('zout').onclick=()=>zoomBy(1.33);
$('reset').onclick=()=>{go(HOME_P.clone(),HOME_T.clone(),1400);dropHint()};

function frameView(){const w=app.clientWidth,h=cvs.clientHeight||app.clientHeight;R.setSize(w,h,false);cam.aspect=w/h;
  if(w>900)cam.setViewOffset(w,h,-w*0.2,h*0.02,w,h);else cam.clearViewOffset();cam.updateProjectionMatrix()}
addEventListener('resize',frameView);frameView();

if('IntersectionObserver' in window)new IntersectionObserver(es=>{seen=es[0].isIntersecting},{threshold:0}).observe(app);

const lo=new THREE.Vector3(-56,0,-38),hi=new THREE.Vector3(56,8,38);
let lastNow=performance.now();
function frame(now){
  requestAnimationFrame(frame);
  if(!seen||document.hidden){lastNow=now;return}
  if(tween){const k=ease(clamp((now-tween.t0)/tween.dur,0,1));cam.position.lerpVectors(tween.fp,tween.tp,k);ctl.target.lerpVectors(tween.ft,tween.tt,k);if(k>=1)tween=null}
  if(!REDUCED){const idle=now-lastTouch;const k=busy||tween?0:ease(clamp((idle-4000)/3000,0,1));ctl.autoRotateSpeed=SPIN*k}
  ctl.update();
  const before=ctl.target.clone();ctl.target.clamp(lo,hi);const d=ctl.target.clone().sub(before);if(d.lengthSq()>0)cam.position.add(d);
  update(U0,now);
  R.render(S,cam);
  lastNow=now;
}
requestAnimationFrame(frame);
setTimeout(()=>{$('loading').style.opacity=0;setTimeout(()=>$('loading').remove(),700)},300);
})();
