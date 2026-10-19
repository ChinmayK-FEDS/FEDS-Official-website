/* FEDS site effects
   1. Crosshair: click or tap empty space and the crosshair recoils and leaves
      a bullet mark. Links, buttons, form
      fields and other interactive elements are left alone.
   2. Background drones: a few small quadcopters fly slow routes behind the
      content and sometimes pause to scan. */
(function () {
  "use strict";
  var MARK_LIFE = 2000;   // ms a mark stays: 1.5 s visible, then a 0.5 s fade
  var FADE = 500;
  var MAX_MARKS = 12;
  var RECOIL = 380;       // ms for the crosshair kick and settle

  var SKIP = 'a,button,input,textarea,select,option,label,summary,details,video,audio,iframe,' +
             '[role="button"],[role="link"],[role="tab"],[role="menuitem"],[contenteditable],' +
             '[tabindex]:not([tabindex="-1"]),.hot,.dd,.dd-menu,.menu-toggle,#gl,[data-no-fx]';

  var mq = function (q) { return window.matchMedia && matchMedia(q).matches; };
  var reduce = mq("(prefers-reduced-motion: reduce)");
  var fine = mq("(hover: hover) and (pointer: fine)");

  // Same drawing as the site's own crosshair cursor (site.css), so the recoil matches it.
  var CROSS = '<svg viewBox="0 0 32 32" width="32" height="32"><g fill="none" stroke="#6aa5e0" stroke-width="1.5">' +
    '<circle cx="16" cy="16" r="7"/><path d="M16 2v7M16 23v7M2 16h7M23 16h7"/></g>' +
    '<circle cx="16" cy="16" r="1.5" fill="#e9eef3"/></svg>';

  /* ---------- marks layer ---------- */
  var layer = document.createElement("div");
  layer.className = "fx-layer";
  layer.setAttribute("aria-hidden", "true");
  document.body.appendChild(layer);
  var marks = [];

  var defs = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  defs.setAttribute("width", "0"); defs.setAttribute("height", "0");
  defs.style.position = "absolute";
  defs.innerHTML = '<defs><radialGradient id="fxg"><stop offset="0" stop-color="#000" stop-opacity=".9"/><stop offset="1" stop-color="#000" stop-opacity="0"/></radialGradient></defs>';
  layer.appendChild(defs);

  function holeSVG() {
    var cracks = "", n = 4 + Math.floor(Math.random() * 3);
    for (var i = 0; i < n; i++) {
      var a = (i / n) * Math.PI * 2 + (Math.random() - 0.5) * 0.9;
      var r0 = 5.5, r1 = 9 + Math.random() * 7;
      var bend = a + (Math.random() - 0.5) * 0.5, rm = (r0 + r1) / 2;
      cracks += '<path d="M' + (18 + Math.cos(a) * r0).toFixed(1) + " " + (18 + Math.sin(a) * r0).toFixed(1) +
        " L" + (18 + Math.cos(bend) * rm).toFixed(1) + " " + (18 + Math.sin(bend) * rm).toFixed(1) +
        " L" + (18 + Math.cos(a) * r1).toFixed(1) + " " + (18 + Math.sin(a) * r1).toFixed(1) + '"/>';
    }
    return '<svg viewBox="0 0 36 36" width="36" height="36">' +
      '<circle cx="18" cy="18" r="8" fill="url(#fxg)" opacity=".5"/>' +
      '<g stroke="rgba(233,238,243,.45)" stroke-width=".9" fill="none" stroke-linecap="round">' + cracks + "</g>" +
      '<circle cx="18" cy="18" r="4.6" fill="none" stroke="rgba(233,238,243,.55)" stroke-width="1"/>' +
      '<circle cx="18" cy="18" r="3.2" fill="#040506"/>' +
      '<circle cx="17.2" cy="17.2" r="1" fill="rgba(255,255,255,.18)"/></svg>';
  }

  function mark(x, y) {
    var m = document.createElement("span");
    m.className = "fx-hole";
    m.style.left = x + "px"; m.style.top = y + "px";
    m.style.setProperty("--rot", Math.floor(Math.random() * 360) + "deg");
    m.innerHTML = holeSVG();
    layer.appendChild(m);
    marks.push(m);
    if (marks.length > MAX_MARKS) { marks.shift().remove(); }
    setTimeout(function () { m.classList.add("out"); }, MARK_LIFE - FADE);
    setTimeout(function () { m.remove(); var i = marks.indexOf(m); if (i > -1) marks.splice(i, 1); }, MARK_LIFE);
  }

  /* ---------- recoil: a copy of the cursor kicks up and settles ---------- */
  var firing = 0;
  function recoil(x, y) {
    if (reduce) return;
    var c = document.createElement("span");
    c.className = "fx-cross" + (fine ? "" : " tap");
    c.style.left = x + "px"; c.style.top = y + "px";
    c.innerHTML = CROSS;
    layer.appendChild(c);
    if (fine) { firing++; document.documentElement.classList.add("fx-firing"); }
    setTimeout(function () {
      c.remove();
      if (fine && --firing <= 0) { firing = 0; document.documentElement.classList.remove("fx-firing"); }
    }, RECOIL);
  }

  document.addEventListener("click", function (e) {
    if (e.button !== 0 || e.defaultPrevented) return;
    var t = e.target;
    if (!(t instanceof Element) || t.closest(SKIP)) return;
    if (window.getSelection && String(window.getSelection()).length) return;  // user was selecting text
    if (!fine || (e.pointerType && e.pointerType !== "mouse")) return;          // no shots on touch screens
    layer.style.height = document.documentElement.scrollHeight + "px";
    mark(e.pageX, e.pageY);
    recoil(e.pageX, e.pageY);
  });

  /* ---------- background drones ---------- */
  var cv = document.createElement("canvas");
  cv.className = "fx-drones";
  cv.setAttribute("aria-hidden", "true");
  var terrain = document.querySelector(".terrain");
  if (terrain && terrain.parentNode) terrain.parentNode.insertBefore(cv, terrain.nextSibling);
  else document.body.insertBefore(cv, document.body.firstChild);
  var ctx = cv.getContext("2d");
  var W = 0, H = 0, dpr = 1, drones = [], bits = [];
  var INK = "190,210,230";
  var SPEEDS = [5, 8.5, 12, 6.5];           // px per second, one per drone, so no two move alike

  function rand(a, b) { return a + Math.random() * (b - a); }
  function far(d, x, y) {              // keep each drone's route away from the others
    var minGap = Math.min(W, H) * 0.45;
    return drones.every(function (o) {
      if (o === d || !o.alive || o.x === undefined) return true;
      return Math.hypot(o.x - x, o.y - y) > minGap && (o.tx === undefined || Math.hypot(o.tx - x, o.ty - y) > minGap);
    });
  }
  function pickTarget(d) {
    var m = 70, x, y;
    for (var i = 0; i < 20; i++) {
      x = rand(m, Math.max(m + 1, W - m)); y = rand(m, Math.max(m + 1, H - m));
      if (far(d, x, y)) break;
    }
    d.tx = x; d.ty = y;
  }
  function spawn(d, fromEdge) {
    d.alive = true; d.fall = null; d.hold = 0; d.scan = -1; d.nextScan = rand(10, 20);
    if (fromEdge) {
      var side = Math.floor(Math.random() * 4);
      d.x = side === 0 ? -20 : side === 1 ? W + 20 : rand(0, W);
      d.y = side === 2 ? -20 : side === 3 ? H + 20 : rand(0, H);
    } else {
      for (var i = 0; i < 20; i++) { d.x = rand(60, W - 60); d.y = rand(60, H - 60); if (far(d, d.x, d.y)) break; }
    }
    d.a = Math.atan2(H / 2 - d.y, W / 2 - d.x);
    pickTarget(d);
  }
  function makeDrones() {
    var n = W < 760 ? 2 : W > 1800 ? 4 : 3;
    drones = [];
    for (var i = 0; i < n; i++) {
      var d = { v: SPEEDS[i % SPEEDS.length] * rand(0.9, 1.1), s: rand(0.85, 1.1), bob: rand(0, 6.28) };
      drones.push(d); spawn(d, false);
    }
  }
  function resize() {
    dpr = Math.min(1.5, window.devicePixelRatio || 1);
    W = window.innerWidth; H = window.innerHeight;
    cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
    if (!drones.length) makeDrones();
  }

  function body(x, y, ang, k, alpha) {
    ctx.save();
    ctx.translate(x, y); ctx.rotate(ang + Math.PI / 4);
    ctx.strokeStyle = "rgba(" + INK + "," + (0.22 * alpha).toFixed(3) + ")"; ctx.lineWidth = 1;
    var r = 7 * k;
    ctx.beginPath(); ctx.moveTo(-r, -r); ctx.lineTo(r, r); ctx.moveTo(r, -r); ctx.lineTo(-r, r); ctx.stroke();
    for (var i = 0; i < 4; i++) {
      ctx.beginPath(); ctx.arc(i & 1 ? r : -r, i & 2 ? r : -r, 3 * k, 0, 6.283); ctx.stroke();
    }
    ctx.fillStyle = "rgba(" + INK + "," + (0.3 * alpha).toFixed(3) + ")";
    ctx.fillRect(-2 * k, -2 * k, 4 * k, 4 * k);
    ctx.restore();
  }

  function drawDrone(d, t) {
    if (!d.alive) return;
    if (d.fall) {                               // going down
      var f = d.fall, p = f.t / f.dur;
      body(d.x, d.y, d.a + f.spin * f.t, d.s * (1 - 0.35 * p), 1 - p);
      return;
    }
    body(d.x, d.y + Math.sin(t * 1.4 + d.bob) * 1, d.a, d.s, 1);
    if (d.scan >= 0) {
      ctx.strokeStyle = "rgba(106,165,224," + (0.16 * (1 - d.scan)).toFixed(3) + ")";
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(d.x, d.y, 8 + d.scan * 60, 0, 6.283); ctx.stroke();
    }
  }

  function drawBits() {
    bits.forEach(function (b) {
      var p = b.t / b.life;
      if (b.kind === "flash") {
        ctx.strokeStyle = "rgba(255,190,120," + (0.35 * (1 - p)).toFixed(3) + ")"; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(b.x, b.y, 4 + p * 22, 0, 6.283); ctx.stroke();
      } else if (b.kind === "smoke") {
        ctx.fillStyle = "rgba(" + INK + "," + (0.1 * (1 - p)).toFixed(3) + ")";
        ctx.beginPath(); ctx.arc(b.x, b.y, 2 + p * 5, 0, 6.283); ctx.fill();
      } else {
        ctx.save(); ctx.translate(b.x, b.y); ctx.rotate(b.r);
        ctx.strokeStyle = "rgba(" + INK + "," + (0.3 * (1 - p)).toFixed(3) + ")"; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(-b.len / 2, 0); ctx.lineTo(b.len / 2, 0); ctx.stroke(); ctx.restore();
      }
    });
  }

  function step(dt) {
    drones.forEach(function (d) {
      if (!d.alive) { d.respawn -= dt; if (d.respawn <= 0) spawn(d, true); return; }
      if (d.fall) {
        var f = d.fall; f.t += dt; f.vy += 60 * dt;
        d.x += f.vx * dt; d.y += f.vy * dt;
        f.puff -= dt;
        if (f.puff <= 0) { f.puff = 0.12; bits.push({ kind: "smoke", x: d.x, y: d.y, t: 0, life: 1.1 }); }
        if (f.t >= f.dur) { d.alive = false; d.respawn = rand(6, 10); }
        return;
      }
      if (d.hold > 0) {
        d.hold -= dt;
        if (d.scan >= 0) { d.scan += dt / 2; if (d.scan >= 1) d.scan = d.hold > 1 ? 0 : -1; }
        if (d.hold <= 0) { d.scan = -1; pickTarget(d); }
        return;
      }
      var dx = d.tx - d.x, dy = d.ty - d.y, dist = Math.hypot(dx, dy);
      var want = Math.atan2(dy, dx), diff = Math.atan2(Math.sin(want - d.a), Math.cos(want - d.a));
      d.a += Math.max(-0.5 * dt, Math.min(0.5 * dt, diff));    // slow, wide turns
      var sp = d.v * Math.min(1, dist / 60 + 0.3);
      d.x += Math.cos(d.a) * sp * dt; d.y += Math.sin(d.a) * sp * dt;
      d.nextScan -= dt;
      // if another drone gets close, head somewhere else
      for (var i = 0; i < drones.length; i++) {
        var o = drones[i];
        if (o !== d && o.alive && !o.fall && Math.hypot(o.x - d.x, o.y - d.y) < Math.min(W, H) * 0.3) { pickTarget(d); break; }
      }
      if (dist < 12) {
        if (d.nextScan <= 0) { d.hold = rand(3, 5); d.scan = 0; d.nextScan = rand(14, 26); }
        else pickTarget(d);
      }
    });
    bits = bits.filter(function (b) {
      b.t += dt;
      if (b.vx !== undefined) { b.x += b.vx * dt; b.y += b.vy * dt; b.vy += 40 * dt; b.r += b.vr * dt; }
      else if (b.kind === "smoke") { b.y -= 6 * dt; }
      return b.t < b.life;
    });
  }

  function render(t) {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    drawBits();
    drones.forEach(function (d) { drawDrone(d, t); });
  }

  // Tap a drone: it spins down. Only when the drone is
  // actually visible at that spot (not hidden behind the hero, a card or a photo).
  var COVER = ".hfeed,.card,.alt,img,video,.site-head,.site-foot,.table-box,.media";
  function hitDrone(e) {
    var t = e.target;
    if (!(t instanceof Element) || t.closest(SKIP) || t.closest(COVER)) return false;
    var x = e.clientX, y = e.clientY;
    for (var i = 0; i < drones.length; i++) {
      var d = drones[i];
      if (!d.alive || d.fall || Math.hypot(d.x - x, d.y - y) > 18) continue;
      if (reduce) { d.alive = false; d.respawn = 8; render(0); return true; }
      d.fall = { t: 0, dur: 1.8, vx: rand(-14, 14), vy: 8, spin: rand(4, 7) * (Math.random() < 0.5 ? -1 : 1), puff: 0 };
      return true;
    }
    return false;
  }
  document.addEventListener("click", function (e) { if (e.button === 0) hitDrone(e); }, true);

  resize();
  window.addEventListener("resize", function () {
    var oldW = W, oldH = H; resize();
    drones.forEach(function (d) { d.x *= W / oldW; d.y *= H / oldH; pickTarget(d); });
    if (reduce) render(0);
  });
  if (reduce) { render(0); }
  else {
    var last = 0, acc = 0;
    var loop = function (now) {
      requestAnimationFrame(loop);
      if (document.hidden) { last = now; return; }
      var dt = last ? Math.min(0.1, (now - last) / 1000) : 0; last = now; acc += dt;
      if (acc < 1 / 30) return;                 // 30 fps is plenty for slow drones
      step(acc); acc = 0;
      render(now / 1000);
    };
    requestAnimationFrame(loop);
  }
  window.__fxDrones = drones;
})();
