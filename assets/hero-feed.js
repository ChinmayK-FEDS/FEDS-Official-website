/* FEDS hero: simulated aerial sensor feed with a OneView detection sequence.
   Everything is drawn in code. Illustrative scenario, not live data. */
(function () {
  "use strict";
  var cv = document.getElementById("feed");
  if (!cv) return;
  var ctx = cv.getContext("2d");
  var logEl = document.getElementById("feed-log");
  var recEl = document.getElementById("feed-rec");
  var statEl = document.getElementById("feed-stat");
  var reduce = window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches;

  var C = { fr: "#6aa5e0", hos: "#ff5a4d", unk: "#ffc34a", neu: "#3fd27f", ink: "#e9eef3" };
  var MONO = '"JetBrains Mono", ui-monospace, Menlo, Consolas, monospace';
  var LOOP = 26;

  /* ---------- terrain ---------- */
  var TW = 1400, TH = 875;
  if (Math.max(screen.width, screen.height) * Math.min(2, window.devicePixelRatio || 1) >= 2600) { TW = 2000; TH = 1250; }

  function hash(x, y) {
    var h = Math.imul(x, 374761393) + Math.imul(y, 668265263);
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    return ((h ^ (h >>> 16)) >>> 0) / 4294967295;
  }
  function vnoise(x, y) {
    var xi = Math.floor(x), yi = Math.floor(y), xf = x - xi, yf = y - yi;
    var u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
    var a = hash(xi, yi), b = hash(xi + 1, yi), c = hash(xi, yi + 1), d = hash(xi + 1, yi + 1);
    return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
  }
  function fbm(x, y, o) {
    var s = 0, a = 0.5, f = 1;
    for (var i = 0; i < o; i++) { s += a * vnoise(x * f, y * f); f *= 2.03; a *= 0.5; }
    return s;
  }
  function rfbm(x, y, o) {
    var s = 0, a = 0.5, f = 1, w = 1;
    for (var i = 0; i < o; i++) { var n = 1 - Math.abs(2 * vnoise(x * f, y * f) - 1); n *= n * w; w = Math.min(1, n * 1.6); s += a * n; f *= 2.05; a *= 0.5; }
    return s;
  }
  function sstep(a, b, x) { var t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); }

  // Main valley road (normalised world coords). Our column drives up it.
  var ROAD = [[0.47, 1.06], [0.55, 0.88], [0.63, 0.76], [0.72, 0.68], [0.84, 0.63], [0.95, 0.55], [1.06, 0.48]];
  // Track across the far side. The civilian truck uses it.
  var TRACK = [[1.06, 0.20], [0.94, 0.235], [0.86, 0.215], [0.78, 0.17], [0.66, 0.15], [0.55, 0.12]];

  function densify(poly, n) {
    var seg = [], tot = 0, i;
    for (i = 1; i < poly.length; i++) {
      var dx = poly[i][0] - poly[i - 1][0], dy = (poly[i][1] - poly[i - 1][1]) * 0.625;
      var l = Math.hypot(dx, dy); seg.push(l); tot += l;
    }
    var out = [];
    for (var k = 0; k <= n; k++) {
      var d = (k / n) * tot, j = 0;
      while (j < seg.length - 1 && d > seg[j]) { d -= seg[j]; j++; }
      var f = Math.min(1, d / seg[j]);
      // Catmull-Rom for smooth curves
      var p0 = poly[Math.max(0, j - 1)], p1 = poly[j], p2 = poly[j + 1], p3 = poly[Math.min(poly.length - 1, j + 2)];
      var t = f, t2 = t * t, t3 = t2 * t;
      var cr = function (a, b, c, e) { return 0.5 * (2 * b + (-a + c) * t + (2 * a - 5 * b + 4 * c - e) * t2 + (-a + 3 * b - 3 * c + e) * t3); };
      out.push([cr(p0[0], p1[0], p2[0], p3[0]), cr(p0[1], p1[1], p2[1], p3[1])]);
    }
    return out;
  }
  var roadPts = densify(ROAD, 400), trackPts = densify(TRACK, 300);

  function at(pts, s) {
    s = Math.min(0.9999, Math.max(0, s));
    var f = s * (pts.length - 1), i = Math.floor(f), r = f - i;
    var a = pts[i], b = pts[i + 1];
    return { x: a[0] + (b[0] - a[0]) * r, y: a[1] + (b[1] - a[1]) * r, a: Math.atan2((b[1] - a[1]) * TH, (b[0] - a[0]) * TW) };
  }

  // Coarse distance field to the road and the track, for carving and drawing.
  function distField(pts, gw, gh) {
    var out = new Float32Array(gw * gh);
    for (var y = 0; y < gh; y++) for (var x = 0; x < gw; x++) {
      var nx = x / (gw - 1), ny = y / (gh - 1), best = 9;
      for (var i = 0; i < pts.length; i += 2) {
        var dx = (nx - pts[i][0]) * 1.6, dy = ny - pts[i][1], d = dx * dx + dy * dy;
        if (d < best) best = d;
      }
      out[y * gw + x] = Math.sqrt(best);
    }
    return out;
  }
  function sample(f, gw, gh, nx, ny) {
    var x = Math.min(gw - 1.001, Math.max(0, nx * (gw - 1))), y = Math.min(gh - 1.001, Math.max(0, ny * (gh - 1)));
    var xi = x | 0, yi = y | 0, xf = x - xi, yf = y - yi, i = yi * gw + xi;
    return (f[i] * (1 - xf) + f[i + 1] * xf) * (1 - yf) + (f[i + gw] * (1 - xf) + f[i + gw + 1] * xf) * yf;
  }

  var HOST = { x: 0.755, y: 0.315 };   // where the six vehicles sit, behind the ridge
  var POST = { x: 0.6, y: 0.43 };      // own ground post: radar and direction finder

  function ridgeY(nx) { return 0.53 - 0.17 * nx + 0.025 * Math.sin(nx * 11); }

  var terrain = document.createElement("canvas");
  function buildTerrain() {
    var GW = 220, GH = 138;
    var dRoad = distField(roadPts, GW, GH), dTrack = distField(trackPts, GW, GH);
    var H = new Float32Array(TW * TH), x, y, i;
    for (y = 0; y < TH; y++) {
      var ny = y / TH;
      for (x = 0; x < TW; x++) {
        var nx = x / TW, X = nx * 1.6, Y = ny;
        var h = 0.22 * fbm(X * 4.2, Y * 4.2, 5);
        var rd = (ny - ridgeY(nx)) / 0.1;
        var mtn = rfbm(X * 3.2 + 7.3, Y * 3.2 + 1.1, 6);
        h += Math.exp(-rd * rd) * (0.25 + 1.05 * mtn) * (0.7 + 0.5 * vnoise(nx * 5, 4.2));
        var far = ny / 0.16;
        h += 0.9 * Math.exp(-far * far) * rfbm(X * 3 + 2, Y * 3 + 5, 5);
        var sw = Math.hypot((nx - 0.25) * 1.6, ny - 0.66) / 0.32;
        h += 0.5 * Math.exp(-sw * sw) * rfbm(X * 4 + 9, Y * 4, 4);
        var dr = sample(dRoad, GW, GH, nx, ny), dt = sample(dTrack, GW, GH, nx, ny);
        h *= 0.3 + 0.7 * sstep(0.004, 0.09, dr);
        h *= 0.55 + 0.45 * sstep(0.003, 0.05, dt);
        var hp = Math.hypot((nx - HOST.x) * 1.6, ny - HOST.y) / 0.06;
        h *= 1 - 0.5 * Math.exp(-hp * hp);
        H[y * TW + x] = h;
      }
    }
    terrain.width = TW; terrain.height = TH;
    var tctx = terrain.getContext("2d"), img = tctx.createImageData(TW, TH), px = img.data;
    for (y = 0; y < TH; y++) {
      for (x = 0; x < TW; x++) {
        i = y * TW + x;
        var xl = x > 0 ? i - 1 : i, xr = x < TW - 1 ? i + 1 : i, yu = y > 0 ? i - TW : i, yd = y < TH - 1 ? i + TW : i;
        var gx = (H[xr] - H[xl]) * TW * 0.5, gy = (H[yd] - H[yu]) * TH * 0.5;
        var lit = 0.48 + (gx + gy) * 0.018;
        lit = Math.min(1, Math.max(0, lit));
        var hv = H[i];
        var v = 14 + 200 * lit * (0.45 + 0.55 * Math.min(1.2, hv));
        if (hv > 0.72) v += Math.min(1, (hv - 0.72) * 4) * 70 * lit;          // snow on the high ridge
        var c = (hv * 16) % 1;                                  // contour lines
        if (c < 0.05 || c > 0.97) v -= 9;
        var nxx = x / TW, nyy = y / TH;
        var dr2 = sample(dRoad, GW, GH, nxx, nyy), dt2 = sample(dTrack, GW, GH, nxx, nyy);
        if (dr2 < 0.0055) v = v * 0.55 + 70 + 22 * (1 - dr2 / 0.0055);
        if (dt2 < 0.0035) v = v * 0.6 + 52;
        v += (hash(x, y + 999) - 0.5) * 12;
        v = Math.max(0, Math.min(255, v));
        var j = i * 4;
        px[j] = v * 0.94; px[j + 1] = v * 0.98; px[j + 2] = v; px[j + 3] = 255;
      }
    }
    tctx.putImageData(img, 0, 0);
  }

  /* ---------- camera ---------- */
  var W = 0, Hc = 0, dpr = 1, copyTop = 0, cam = { s: 1, fx: 0.6, fy: 0.48, ax: 0.5, ay: 0.5 };
  var leadEl = document.querySelector(".hfeed .lead") || document.querySelector(".lead");
  function resize() {
    dpr = Math.min(2, window.devicePixelRatio || 1);
    var r = cv.getBoundingClientRect();
    W = Math.max(1, Math.round(r.width * dpr)); Hc = Math.max(1, Math.round(r.height * dpr));
    cv.width = W; cv.height = Hc;
    copyTop = leadEl ? (leadEl.getBoundingClientRect().top - r.top) * dpr : Hc * 0.55;
  }
  function setCam(t) {
    var p = t / LOOP, narrow = W / Hc < 1;
    var base = Math.max(W / TW, Hc / TH);
    if (narrow) {
      // Fit the action (hostiles at the top to our column on the road) into the space above the text.
      var A = Math.min(Hc, Math.max(Hc * 0.3, copyTop - 10 * dpr));
      cam.s = Math.max(W / TW, (A * 0.94) / (0.56 * TH)) * (1 + 0.03 * p);
      cam.fy = 0.2; cam.ay = (0.05 * A) / Hc; cam.ax = 0.5; cam.fx = 0.71;
    } else {
      cam.s = base * (1.2 + 0.07 * p);
      cam.fx = 0.585 + 0.015 * Math.sin(p * Math.PI * 2); cam.fy = 0.52 + 0.01 * Math.cos(p * Math.PI * 2);
      cam.ax = 0.5; cam.ay = 0.5;
    }
    // never let the terrain edge show at the sides
    var half = W / (2 * TW * cam.s);
    cam.fx = Math.min(1 - half, Math.max(half, cam.fx));
  }
  function P(nx, ny) {
    return { x: W * cam.ax + (nx - cam.fx) * TW * cam.s, y: Hc * cam.ay + (ny - cam.fy) * TH * cam.s };
  }

  /* ---------- scene ---------- */
  var HOSTILES = [[-0.022, -0.012, 0.3], [-0.008, -0.018, 0.35], [0.008, -0.01, 0.2], [-0.016, 0.008, 0.4], [0.002, 0.006, 0.3], [0.018, 0.012, 0.25]];
  var DRONE_PATH = [[0.69, 0.66], [0.7, 0.52], [0.72, 0.42], [0.745, 0.37]];

  function lerp(a, b, t) { return a + (b - a) * t; }
  function ease(t) { t = Math.min(1, Math.max(0, t)); return t * t * (3 - 2 * t); }
  function win(t, a, b) { return ease((t - a) / (b - a)); }

  function column(t) {
    var out = [], s0 = 0.27 + t * 0.0042;
    for (var k = 0; k < 4; k++) out.push(at(roadPts, s0 - k * 0.028));
    return out;
  }
  function truck(t) { return at(trackPts, 0.05 + t * 0.0105); }
  function drone(t) {
    if (t < 9) return { x: 0.685, y: 0.665 - 0.006 * Math.sin(t), a: -1.4 };
    if (t < 13.5) {
      var f = ease((t - 9) / 4.5) * (DRONE_PATH.length - 1), i = Math.min(DRONE_PATH.length - 2, Math.floor(f)), r = f - i;
      var a = DRONE_PATH[i], b = DRONE_PATH[i + 1];
      return { x: lerp(a[0], b[0], r), y: lerp(a[1], b[1], r), a: Math.atan2((b[1] - a[1]) * TH, (b[0] - a[0]) * TW) };
    }
    var w = (t - 13.5) * 0.35 - 1.2, R = 0.04;
    var ox = HOST.x + R * Math.cos(w) * 0.8, oy = HOST.y + 0.03 + R * Math.sin(w) * 0.9;
    var bx = DRONE_PATH[3][0], by = DRONE_PATH[3][1], m = Math.min(1, (t - 13.5) / 1.2);
    return { x: lerp(bx, ox, m), y: lerp(by, oy, m), a: w + Math.PI / 2 };
  }

  function veh(p, ang, k, alpha, hot) {
    ctx.save();
    ctx.translate(p.x, p.y); ctx.rotate(ang);
    ctx.globalAlpha = alpha;
    ctx.shadowColor = hot ? "rgba(255,240,220,0.9)" : "rgba(255,255,255,0.7)";
    ctx.shadowBlur = 8 * dpr;
    ctx.fillStyle = "#f4f6f8";
    var L = 10 * k, B = 5 * k;
    ctx.fillRect(-L / 2, -B / 2, L, B);
    ctx.shadowBlur = 0;
    ctx.fillStyle = "rgba(20,24,28,0.55)";
    ctx.fillRect(-L * 0.1, -B * 0.28, L * 0.3, B * 0.56);
    ctx.restore();
  }
  function droneShape(p, ang, k) {
    ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(ang);
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1.6 * dpr; ctx.shadowColor = "rgba(255,255,255,.8)"; ctx.shadowBlur = 6 * dpr;
    var r = 6 * k;
    ctx.beginPath(); ctx.moveTo(-r, -r); ctx.lineTo(r, r); ctx.moveTo(r, -r); ctx.lineTo(-r, r); ctx.stroke();
    ctx.fillStyle = C.ink;
    [[-r, -r], [r, r], [r, -r], [-r, r]].forEach(function (q) { ctx.beginPath(); ctx.arc(q[0], q[1], 2.2 * k, 0, 7); ctx.fill(); });
    ctx.restore();
  }
  function brackets(cx, cy, w, h, col, alpha, pulse) {
    var g = Math.min(w, h) * 0.32;
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = col; ctx.lineWidth = 1.6 * dpr;
    if (pulse) { ctx.shadowColor = col; ctx.shadowBlur = 10 * dpr; }
    var x0 = cx - w / 2, y0 = cy - h / 2, x1 = cx + w / 2, y1 = cy + h / 2;
    ctx.beginPath();
    ctx.moveTo(x0, y0 + g); ctx.lineTo(x0, y0); ctx.lineTo(x0 + g, y0);
    ctx.moveTo(x1 - g, y0); ctx.lineTo(x1, y0); ctx.lineTo(x1, y0 + g);
    ctx.moveTo(x1, y1 - g); ctx.lineTo(x1, y1); ctx.lineTo(x1 - g, y1);
    ctx.moveTo(x0 + g, y1); ctx.lineTo(x0, y1); ctx.lineTo(x0, y1 - g);
    ctx.stroke(); ctx.restore();
  }
  function tag(x, y, lines, col, alpha, align) {
    if (alpha <= 0.01) return;
    ctx.save(); ctx.globalAlpha = alpha;
    var fs = 11 * dpr, pad = 7 * dpr, lh = 15 * dpr;
    ctx.font = "600 " + fs + "px " + MONO;
    var w = 0;
    lines.forEach(function (l) { w = Math.max(w, ctx.measureText(l.t).width); });
    w += pad * 2 + 3 * dpr;
    var h = lines.length * lh + pad * 1.2;
    var x0 = align === "left" ? x - w : align === "center" ? x - w / 2 : x, y0 = align === "center" ? y : y - h / 2;
    ctx.fillStyle = "rgba(8,11,14,0.82)"; ctx.fillRect(x0, y0, w, h);
    ctx.fillStyle = col; ctx.fillRect(x0, y0, 3 * dpr, h);
    ctx.textBaseline = "middle";
    lines.forEach(function (l, i) {
      ctx.font = (i === 0 ? "700 " : "500 ") + fs + "px " + MONO;
      ctx.fillStyle = l.c || (i === 0 ? col : "rgba(233,238,243,0.86)");
      ctx.fillText(l.t, x0 + pad + 3 * dpr, y0 + pad * 0.6 + lh * (i + 0.5));
    });
    ctx.restore();
  }
  function link(a, b, col, alpha, t) {
    if (alpha <= 0.01) return;
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = col; ctx.lineWidth = 1.2 * dpr;
    ctx.setLineDash([5 * dpr, 6 * dpr]); ctx.lineDashOffset = -t * 30 * dpr;
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); ctx.restore();
  }
  function ring(p, r, col, alpha) {
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = col; ctx.lineWidth = 1.2 * dpr;
    ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.stroke(); ctx.restore();
  }
  function textAt(p, s, col, alpha, dx, dy) {
    ctx.save(); ctx.globalAlpha = alpha; ctx.font = "600 " + 10 * dpr + "px " + MONO; ctx.fillStyle = col;
    ctx.fillText(s, p.x + dx * dpr, p.y + dy * dpr); ctx.restore();
  }

  /* ---------- the sequence ---------- */
  function draw(t) {
    setCam(t);
    var k = dpr * Math.min(1.25, Math.max(0.75, cam.s / dpr * 1.1));
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = "#07090b"; ctx.fillRect(0, 0, W, Hc);
    var o = P(0, 0);
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(terrain, o.x, o.y, TW * cam.s, TH * cam.s);
    var tb = o.y + TH * cam.s;
    if (tb < Hc) {
      var fg = ctx.createLinearGradient(0, tb - 0.15 * TH * cam.s, 0, tb);
      fg.addColorStop(0, "rgba(7,9,11,0)"); fg.addColorStop(1, "#07090b");
      ctx.fillStyle = fg; ctx.fillRect(0, tb - 0.15 * TH * cam.s, W, 0.15 * TH * cam.s + 1);
      ctx.fillStyle = "#07090b"; ctx.fillRect(0, tb, W, Hc - tb);
    }

    var narrow = W / Hc < 1;
    var fade = Math.min(win(t, 0, 0.8), 1 - win(t, LOOP - 0.9, LOOP));

    // scan line across the frame at the start
    if (t < 2.4) {
      var sx = W * ease(t / 2.4);
      var g = ctx.createLinearGradient(sx - 160 * dpr, 0, sx, 0);
      g.addColorStop(0, "rgba(106,165,224,0)"); g.addColorStop(1, "rgba(106,165,224,0.16)");
      ctx.fillStyle = g; ctx.fillRect(sx - 160 * dpr, 0, 160 * dpr, Hc);
      ctx.fillStyle = "rgba(160,200,240,0.55)"; ctx.fillRect(sx, 0, 1.5 * dpr, Hc);
    }

    // own ground post, radar pulses
    var post = P(POST.x, POST.y);
    for (var q = 0; q < 2; q++) {
      var rp = ((t * 0.5 + q * 0.5) % 1);
      ring(post, rp * 120 * dpr * (cam.s / dpr), C.fr, (1 - rp) * 0.4 * fade);
    }
    ctx.save(); ctx.globalAlpha = fade; ctx.fillStyle = C.fr; ctx.shadowColor = C.fr; ctx.shadowBlur = 8 * dpr;
    ctx.beginPath(); ctx.moveTo(post.x, post.y - 6 * k); ctx.lineTo(post.x + 5 * k, post.y + 4 * k); ctx.lineTo(post.x - 5 * k, post.y + 4 * k); ctx.closePath(); ctx.fill(); ctx.restore();
    if (!narrow) textAt(post, "G1 · GROUND POST", C.fr, 0.85 * fade, 9, 4);

    // vehicles
    var col = column(t), tr = truck(t);
    col.forEach(function (v) { veh(P(v.x, v.y), v.a, k, fade, false); });
    veh(P(tr.x, tr.y), tr.a + Math.PI, k, fade, false);
    var reveal = win(t, 12.2, 14.2);
    HOSTILES.forEach(function (h, i) {
      var jig = 0.0008 * Math.sin(t * 1.3 + i);
      veh(P(HOST.x + h[0] + jig, HOST.y + h[1]), h[2], k * 0.95, fade * (0.22 + 0.78 * reveal), true);
    });

    // drone and its camera footprint
    var dr = drone(t), dp = P(dr.x, dr.y);
    if (t > 12.6) {
      var fr = win(t, 12.6, 13.8);
      ctx.save(); ctx.globalAlpha = 0.12 * fr * fade; ctx.fillStyle = C.fr;
      var hp = P(HOST.x, HOST.y);
      ctx.beginPath(); ctx.moveTo(dp.x, dp.y); ctx.arc(hp.x, hp.y, 70 * k, 0, Math.PI * 2); ctx.fill(); ctx.restore();
      link(dp, hp, C.fr, 0.6 * fr * fade, t);
    }
    droneShape(dp, dr.a, k);
    textAt(dp, t > 9 ? "L1 · TASKED" : "L1 · OWN DRONE", C.fr, 0.85 * fade, 11, -9);

    // ---- contact 1: our column ----
    var c1 = P(col[1].x * 0.5 + col[2].x * 0.5, col[1].y * 0.5 + col[2].y * 0.5);
    var a1 = win(t, 2.2, 2.7) * fade;
    var v1 = win(t, 4.6, 5.0);
    if (a1 > 0) {
      link(post, c1, C.fr, 0.5 * a1 * (1 - win(t, 6, 7)), t);
      var bw = 150 * k * 0.6, bh = 90 * k * 0.6;
      brackets(c1.x, c1.y, bw * (1.3 - 0.3 * a1), bh * (1.3 - 0.3 * a1), v1 > 0.5 ? C.fr : C.unk, a1, false);
      var l1 = v1 < 0.5
        ? [{ t: "TRACK 01 · UNKNOWN" }, { t: "4 ground vehicles" }, { t: t > 3.4 ? "RADAR ✓  IFF ··" : "RADAR ··" }]
        : [{ t: "TRACK 01 · OWN FORCES" }, { t: "4 vehicles · IFF verified" }, { t: "RADAR ✓  IFF ✓  RF ✓", c: C.neu }];
      if (!narrow) tag(c1.x + bw / 2 + 10 * dpr, c1.y, l1, v1 > 0.5 ? C.fr : C.unk, a1 * (1 - 0.85 * win(t, 17.2, 17.8)), "right");
    }

    // ---- contact 2: civilian truck ----
    var c2 = P(tr.x, tr.y), a2 = win(t, 6, 6.5) * fade, v2 = win(t, 7.9, 8.3);
    if (a2 > 0) {
      link(post, c2, C.fr, 0.45 * a2 * (1 - win(t, 9, 10)), t);
      brackets(c2.x, c2.y, 46 * k, 34 * k, v2 > 0.5 ? C.neu : C.unk, a2 * (1 - 0.5 * win(t, 11, 12)), false);
      var l2 = v2 < 0.5
        ? [{ t: "TRACK 02 · UNKNOWN" }, { t: "1 vehicle · far road" }]
        : [{ t: "TRACK 02 · CLEARED" }, { t: "Civilian truck · no alert" }];
      if (!narrow) tag(c2.x - 30 * k, c2.y + 34 * k, l2, v2 > 0.5 ? C.neu : C.unk, a2 * (1 - 0.6 * win(t, 11, 12)), "left");
    }

    // ---- contact 3: six vehicles behind the ridge ----
    var a3 = win(t, 12.8, 13.3) * fade, v3 = win(t, 15.4, 15.9);
    if (a3 > 0) {
      HOSTILES.forEach(function (h, i) {
        var p = P(HOST.x + h[0], HOST.y + h[1]);
        brackets(p.x, p.y, 20 * k, 16 * k, v3 > 0.5 ? C.hos : C.unk, a3 * win(t, 13 + i * 0.12, 13.4 + i * 0.12), v3 > 0.5);
      });
      var hc = P(HOST.x, HOST.y);
      var pulse = v3 > 0.5 ? 0.75 + 0.25 * Math.sin(t * 6) : 1;
      brackets(hc.x, hc.y, 150 * k, 110 * k, v3 > 0.5 ? C.hos : C.unk, a3 * pulse, v3 > 0.5);
      var l3 = v3 < 0.5
        ? [{ t: "TRACK 03 · UNKNOWN" }, { t: "6 contacts · behind ridge" }, { t: t > 14.3 ? "THERMAL: engines running" : "THERMAL ··" }, { t: t > 14.9 ? "IFF: no response" : "IFF ··" }]
        : [{ t: "TRACK 03 · HOSTILE" }, { t: "6 armoured vehicles" }, { t: "4 sources agree · 94%" }, { t: "3.2 km from column", c: C.unk }];
      if (narrow) tag(hc.x, hc.y + 62 * k, l3, v3 > 0.5 ? C.hos : C.unk, a3, "center");
      else tag(hc.x - 75 * k - 12 * dpr, hc.y, l3, v3 > 0.5 ? C.hos : C.unk, a3, "left");
      // threat line to our column
      var tl = win(t, 16.5, 17.3) * fade;
      if (tl > 0) {
        ctx.save(); ctx.globalAlpha = 0.85 * tl; ctx.strokeStyle = C.hos; ctx.lineWidth = 1.4 * dpr;
        ctx.setLineDash([3 * dpr, 5 * dpr]); ctx.lineDashOffset = t * 24 * dpr;
        var mid = P(col[0].x, col[0].y);
        ctx.beginPath(); ctx.moveTo(hc.x, hc.y + 55 * k); ctx.lineTo(lerp(hc.x, mid.x, tl), lerp(hc.y + 55 * k, mid.y, tl)); ctx.stroke(); ctx.restore();
      }
    }

    // frame vignette
    var vg = ctx.createRadialGradient(W * 0.6, Hc * 0.45, Math.min(W, Hc) * 0.3, W * 0.5, Hc * 0.5, Math.max(W, Hc) * 0.8);
    vg.addColorStop(0, "rgba(0,0,0,0)"); vg.addColorStop(1, "rgba(0,0,0,0.55)");
    ctx.fillStyle = vg; ctx.fillRect(0, 0, W, Hc);
    if (fade < 1) { ctx.fillStyle = "rgba(7,9,11," + (1 - fade) + ")"; ctx.fillRect(0, 0, W, Hc); }

    ui(t);
  }

  /* ---------- DOM: event log, status, recommendation ---------- */
  var EVENTS = [
    [2.3, "unk", "TRACK 01", "4 vehicles detected · radar"],
    [4.8, "fr", "TRACK 01", "IFF verified · own column"],
    [6.1, "unk", "TRACK 02", "vehicle on far road"],
    [8.0, "neu", "TRACK 02", "civilian truck · cleared"],
    [9.2, "fr", "L1", "drone tasked to ridge"],
    [13.0, "unk", "TRACK 03", "6 contacts behind ridge"],
    [15.5, "hos", "TRACK 03", "hostile · 6 armoured · 94%"],
    [17.4, "fr", "CMD", "column warned · awaiting decision"]
  ];
  var shown = -1, lastLoop = -1;
  function clock(offset) {
    var d = new Date(Date.now() - offset * 1000);
    return d.toISOString().slice(11, 19) + "Z";
  }
  function ui(t) {
    if (!logEl) return;
    var n = 0;
    while (n < EVENTS.length && EVENTS[n][0] <= t) n++;
    if (n - 1 !== shown) {
      if (n - 1 < shown) { logEl.innerHTML = ""; shown = -1; }
      for (var i = Math.max(0, shown + 1); i < n; i++) {
        var e = EVENTS[i], li = document.createElement("li");
        li.innerHTML = '<time>' + clock(reduce ? (n - 1 - i) * 2 : 0) + '</time><i class="tg-' + e[1] + '">' + e[2] + "</i><span>" + e[3] + "</span>";
        logEl.appendChild(li);
        while (logEl.children.length > 3) logEl.removeChild(logEl.firstChild);
      }
      shown = n - 1;
    }
    if (statEl) {
      var hos = t > 15.5 ? 1 : 0, tracks = t > 13 ? 3 : t > 6.1 ? 2 : t > 2.3 ? 1 : 0;
      statEl.innerHTML = "TRACKS <b>" + String(tracks).padStart(2, "0") + "</b>" + (hos ? ' · <b class="hos">' + hos + " HOSTILE</b>" : "");
    }
    if (recEl) recEl.classList.toggle("on", t > 17.6 && t < LOOP - 1);
  }

  /* ---------- run ---------- */
  var t0 = 0, running = true, fixedT = null;
  function frame(now) {
    if (!t0) t0 = now;
    var t = ((now - t0) / 1000) % LOOP;
    if (fixedT !== null) t = fixedT;
    if (running) draw(t);
    requestAnimationFrame(frame);
  }
  function start() {
    resize();
    buildTerrain();
    window.__feedTerrain = terrain;
    cv.classList.add("ready");
    if (reduce) { draw(19); return; }
    requestAnimationFrame(frame);
  }
  window.addEventListener("resize", function () { resize(); if (reduce) draw(19); });
  if ("IntersectionObserver" in window) {
    new IntersectionObserver(function (es) { running = es[0].isIntersecting; }).observe(cv);
  }
  // let the page paint first, then build the terrain
  if (document.fonts && document.fonts.ready) document.fonts.ready.then(function () { setTimeout(start, 30); });
  else setTimeout(start, 30);
  window.__feedDraw = function (t) { fixedT = t; draw(t); };
})();
