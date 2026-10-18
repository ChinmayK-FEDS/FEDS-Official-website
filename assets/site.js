(function(){
  var t=document.querySelector('.menu-toggle'),n=document.querySelector('.nav');
  if(t&&n){t.addEventListener('click',function(){var o=n.classList.toggle('open');t.setAttribute('aria-expanded',o?'true':'false');t.textContent=o?'Close':'Menu'})}
  document.querySelectorAll('.dd > button').forEach(function(b){
    b.addEventListener('click',function(){var d=b.parentElement,o=d.classList.toggle('open');b.setAttribute('aria-expanded',o?'true':'false')});
  });
  document.addEventListener('click',function(e){document.querySelectorAll('.dd.open').forEach(function(d){if(!d.contains(e.target)){d.classList.remove('open');d.querySelector('button').setAttribute('aria-expanded','false')}})});
})();
(function(){
  var RM=window.matchMedia&&matchMedia('(prefers-reduced-motion: reduce)').matches;
  /* reveal on scroll */
  var els=[].slice.call(document.querySelectorAll('[data-reveal]'));
  if(!RM&&'IntersectionObserver' in window){
    var io=new IntersectionObserver(function(es){es.forEach(function(e){if(e.isIntersecting){var d=e.target.getAttribute('data-delay')||0;setTimeout(function(){e.target.classList.add('in')},+d);io.unobserve(e.target)}})},{rootMargin:'0px 0px -8% 0px',threshold:.08});
    els.forEach(function(el){io.observe(el)});
    setTimeout(function(){els.forEach(function(el){el.classList.add('in')})},6000);
  }else els.forEach(function(el){el.classList.add('in')});
  /* cursor glow on cards */
  document.querySelectorAll('.card').forEach(function(c){c.addEventListener('pointermove',function(e){var r=c.getBoundingClientRect();c.style.setProperty('--mx',(e.clientX-r.left)+'px');c.style.setProperty('--my',(e.clientY-r.top)+'px')})});
  /* decode effect on eyebrows */
  var G='ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/<>#';
  function decode(el){var t=el.textContent,n=0,len=t.length,f=0;var tick=function(){var out='';for(var i=0;i<len;i++){out+=(i<n||t[i]===' ')?t[i]:G[Math.floor(Math.random()*G.length)]}el.textContent=out;f++;if(f%2===0)n++;if(n<=len)requestAnimationFrame(tick);else el.textContent=t};tick()}
  if(!RM&&'IntersectionObserver' in window){
    var eo=new IntersectionObserver(function(es){es.forEach(function(e){if(e.isIntersecting){var el=e.target;var tn=[].slice.call(el.childNodes).filter(function(n){return n.nodeType===3&&n.textContent.trim()});tn.forEach(function(n){var s=document.createElement('span');s.textContent=n.textContent;n.replaceWith(s);decode(s)});eo.unobserve(el)}})},{threshold:.6});
    document.querySelectorAll('.eyebrow').forEach(function(el){eo.observe(el)});
  }
  /* hero telemetry */
  var clk=document.getElementById('t-clock'),hdg=document.getElementById('t-hdg'),wnd=document.getElementById('t-wind');
  if(clk){var h0=328.1;setInterval(function(){var d=new Date();clk.textContent=d.toISOString().substr(11,8)+'Z';if(!RM){h0+=(Math.random()-.5)*.2;hdg.textContent=h0.toFixed(1)+'°';wnd.textContent=(14+Math.random()*2).toFixed(1)+' km/h'}},1000)}
})();

/* radar */
(function(){
  var cv=document.getElementById('radar');if(!cv)return;
  var RM=window.matchMedia&&matchMedia('(prefers-reduced-motion: reduce)').matches;
  var ctx=cv.getContext('2d'),tip=document.getElementById('radar-tip'),rows=document.querySelectorAll('.tracks > div');
  var C={hos:'#ff5a4d',unk:'#ffc34a',neu:'#3fd27f',fr:'#6aa5e0'};
  /* a: bearing deg (0 = north, clockwise), r: 0..1 range, v: deg/s drift, dr: range change/s */
  var T=[
   {id:'FR',name:'Our column',kind:'fr',a:200,r:.18,info:'4 vehicles · moving north',sym:'sq'},
   {id:'01',name:'Civilian truck',kind:'neu',a:20,r:.42,dr:-.004,info:'Radar + sight + thermal + radio · Low risk',sym:'ci'},
   {id:'02',name:'Quadcopter',kind:'hos',a:95,r:.66,dr:-.012,v:-1.2,info:'4 sources agree · Column warned',sym:'tri'},
   {id:'03',name:'6 tanks',kind:'hos',a:150,r:.78,dr:-.002,info:'Behind ridge · Located by 2 drones',sym:'grp'},
   {id:'04',name:'Fast mover',kind:'unk',a:300,r:.95,dr:-.05,v:4,info:'Air · Fast · Checking sources',sym:'tri',loop:true},
   {id:'L1',name:'Light drone',kind:'fr',a:130,r:.55,v:6,info:'Tasked by OneView · Looking at Track 03',sym:'ci'}
  ];
  T.forEach(function(t){t.r0=t.r});
  var W=0,dpr=Math.min(devicePixelRatio||1,2),sweep=0,last=performance.now(),hover=null;
  function size(){var b=cv.getBoundingClientRect();W=b.width;cv.width=W*dpr;cv.height=W*dpr;ctx.setTransform(dpr,0,0,dpr,0,0)}
  addEventListener('resize',size);size();
  function pos(t){var R=W/2*.9,ang=(t.a-90)*Math.PI/180;return{x:W/2+Math.cos(ang)*t.r*R,y:W/2+Math.sin(ang)*t.r*R}}
  function sym(t,p,al){ctx.save();ctx.globalAlpha=al;ctx.strokeStyle=C[t.kind];ctx.fillStyle=C[t.kind];ctx.lineWidth=1.6;var s=7;
    if(t.sym==='sq'){ctx.strokeRect(p.x-s,p.y-s,2*s,2*s)}
    else if(t.sym==='tri'){ctx.beginPath();ctx.moveTo(p.x,p.y-s-1);ctx.lineTo(p.x+s,p.y+s-1);ctx.lineTo(p.x-s,p.y+s-1);ctx.closePath();ctx.stroke()}
    else if(t.sym==='grp'){for(var k=0;k<6;k++){ctx.beginPath();ctx.arc(p.x+(k%3-1)*7,p.y+(k<3?-4:4),2.4,0,7);ctx.fill()}ctx.beginPath();ctx.arc(p.x,p.y,15,0,7);ctx.setLineDash([3,3]);ctx.stroke()}
    else{ctx.beginPath();ctx.arc(p.x,p.y,s-1,0,7);ctx.stroke()}
    ctx.setLineDash([]);ctx.font='500 11px "JetBrains Mono",monospace';ctx.fillText(t.id==='FR'?'OWN':(/^\d/.test(t.id)?'T'+t.id:t.id),p.x+12,p.y-10);ctx.restore()}
  function draw(now){
    var dt=Math.min(.1,(now-last)/1000);last=now;
    if(!RM){sweep=(sweep+dt*90)%360;T.forEach(function(t){if(t.v)t.a=(t.a+t.v*dt+360)%360;if(t.dr){t.r+=t.dr*dt;if(t.r<.15){t.r=t.r0}}})}
    ctx.clearRect(0,0,W,W);var c=W/2,R=W/2*.9;
    ctx.strokeStyle='rgba(106,165,224,.18)';ctx.lineWidth=1;
    for(var i=1;i<=4;i++){ctx.beginPath();ctx.arc(c,c,R*i/4,0,7);ctx.stroke()}
    for(var d=0;d<360;d+=30){var a=(d-90)*Math.PI/180;ctx.beginPath();ctx.moveTo(c+Math.cos(a)*R*.04,c+Math.sin(a)*R*.04);ctx.lineTo(c+Math.cos(a)*R,c+Math.sin(a)*R);ctx.stroke()}
    ctx.fillStyle='rgba(159,182,204,.6)';ctx.font='500 10px "JetBrains Mono",monospace';ctx.textAlign='center';
    ['N','E','S','W'].forEach(function(l,k){var a=(k*90-90)*Math.PI/180;ctx.fillText(l,c+Math.cos(a)*(R+14),c+Math.sin(a)*(R+14)+4)});
    ctx.textAlign='left';
    /* sweep */
    var sa=(sweep-90)*Math.PI/180;
    if(!RM){var g=ctx.createConicGradient?ctx.createConicGradient(sa-1.2,c,c):null;
      if(g){g.addColorStop(0,'rgba(106,165,224,0)');g.addColorStop(.19,'rgba(106,165,224,.22)');g.addColorStop(.191,'rgba(106,165,224,0)');ctx.fillStyle=g;ctx.beginPath();ctx.moveTo(c,c);ctx.arc(c,c,R,sa-1.2,sa);ctx.closePath();ctx.fill()}
      ctx.strokeStyle='rgba(156,197,238,.8)';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(c,c);ctx.lineTo(c+Math.cos(sa)*R,c+Math.sin(sa)*R);ctx.stroke()}
    T.forEach(function(t){var p=pos(t);var diff=RM?0:((sweep-t.a)%360+360)%360;var al=t.kind==='fr'?1:Math.max(.25,1-diff/300);if(hover===t)al=1;
      if(!RM&&diff<8&&t.kind!=='fr'){ctx.save();ctx.strokeStyle=C[t.kind];ctx.globalAlpha=1-diff/8;ctx.beginPath();ctx.arc(p.x,p.y,10+diff*2,0,7);ctx.stroke();ctx.restore()}
      sym(t,p,al)});
    /* link lines from OneView centre to tasked drone */
    if(!RM)requestAnimationFrame(draw)}
  requestAnimationFrame(draw);
  cv.addEventListener('pointermove',function(e){var b=cv.getBoundingClientRect(),x=e.clientX-b.left,y=e.clientY-b.top,best=null,bd=22;
    T.forEach(function(t){var p=pos(t),d=Math.hypot(p.x-x,p.y-y);if(d<bd){bd=d;best=t}});
    hover=best;rows.forEach(function(r){r.classList.toggle('hl',!!best&&r.getAttribute('data-id')===best.id)});
    if(best){var p=pos(best);tip.innerHTML='<b style="color:'+C[best.kind]+'">'+(best.id==='FR'?'OWN FORCES':(/^\d/.test(best.id)?'TRACK '+best.id:'OWN · '+best.id))+'</b><br>'+best.name+'<br><span style="color:#808c98">'+best.info+'</span>';tip.style.left=p.x+'px';tip.style.top=p.y+'px';tip.classList.add('on')}else tip.classList.remove('on');
    if(RM)draw(performance.now())});
  cv.addEventListener('pointerleave',function(){hover=null;tip.classList.remove('on');rows.forEach(function(r){r.classList.remove('hl')})});
  rows.forEach(function(r){r.addEventListener('pointerenter',function(){var id=r.getAttribute('data-id');hover=T.filter(function(t){return t.id===id})[0]||null;r.classList.add('hl')});r.addEventListener('pointerleave',function(){hover=null;r.classList.remove('hl')})});
})();
/* sand table hotspots */
(function(){
  var hots=document.querySelectorAll('.hot'),rows=document.querySelectorAll('.tracks > div[data-id]');if(!hots.length)return;
  function set(id){hots.forEach(function(h){h.classList.toggle('on',h.getAttribute('data-id')===id)});rows.forEach(function(r){r.classList.toggle('hl',r.getAttribute('data-id')===id)})}
  hots.forEach(function(h){h.addEventListener('pointerenter',function(){set(h.getAttribute('data-id'))});h.addEventListener('focus',function(){set(h.getAttribute('data-id'))});h.addEventListener('click',function(){set(h.getAttribute('data-id'))})});
  rows.forEach(function(r){r.addEventListener('pointerenter',function(){set(r.getAttribute('data-id'))})});
  var box=document.querySelector('.table-box');if(box)box.addEventListener('pointerleave',function(){set(null)});
  var tr=document.querySelector('.tracks');if(tr)tr.addEventListener('pointerleave',function(){set(null)});
})();

/* terrain moves with scroll */
(function(){
  var el=document.querySelector('.terrain');if(!el)return;
  if(window.matchMedia&&matchMedia('(prefers-reduced-motion: reduce)').matches)return;
  var ticking=false;
  function upd(){ticking=false;var max=Math.max(1,document.documentElement.scrollHeight-innerHeight);var p=Math.min(1,scrollY/max);
    el.style.transform='translate3d(0,'+(-p*14).toFixed(2)+'%,0) rotate('+(p*10).toFixed(2)+'deg) scale('+(1+p*.12).toFixed(3)+')'}
  addEventListener('scroll',function(){if(!ticking){ticking=true;requestAnimationFrame(upd)}},{passive:true});
  addEventListener('resize',upd);upd();
})();
