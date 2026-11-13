/* Interface de calibração de controladoras ArduCopter (APM 2.x com 3.2, Pixhawk com 4.x) — Zune Drones */
"use strict";

// ------------------------------------------------------------------ estado
const S = {
  snap: { connected: false },
  params: {},
  meta: null,
  page: (location.hash || "#diag").slice(1),
  paramsSeen: -1,
  manual: loadLocal("apm-manual", {}),
  mode: loadLocal("apm-mode", null), // escolha do usuário: "gps" | "no_gps"
};

function loadLocal(k, d) { try { return JSON.parse(localStorage.getItem(k)) ?? d; } catch { return d; } }
function saveLocal(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* sem storage */ } }
function setManual(k, v) { S.manual[k] = v; saveLocal("apm-manual", S.manual); refreshNav(); }

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const fx = (v, d = 1) => (v === undefined || v === null || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const P = (n) => S.params[n];
// família de firmware conectada: "ac32" (APM 2.x) ou "ac4" (Pixhawk)
const is4 = () => (S.snap.board || S.meta?.board) === "ac4";
// canal da chave de modos: fixo no 5 no 3.2, FLTMODE_CH no 4.x
const modeCh = () => (is4() && P("FLTMODE_CH") >= 5 ? P("FLTMODE_CH") : 5);
const B = (apm, px) => (is4() ? px : apm);
const OUT = () => B("saída", "MAIN OUT");
const BOARD = () => B("APM", "Pixhawk");

async function api(method, path, body) {
  const r = await fetch(path, { method, headers: body ? { "Content-Type": "application/json" } : {}, body: body ? JSON.stringify(body) : undefined });
  const ct = r.headers.get("content-type") || "";
  const data = ct.includes("json") ? await r.json() : await r.text();
  if (!r.ok) throw new Error((data && data.detail) || r.statusText);
  return data;
}

function toast(msg, kind = "") {
  const d = document.createElement("div");
  d.className = kind;
  d.textContent = msg;
  $("#toast").append(d);
  setTimeout(() => d.remove(), kind === "bad" ? 7000 : 4000);
}

async function run(btn, fn) {
  if (btn) btn.disabled = true;
  try { return await fn(); }
  catch (e) { toast(e.message, "bad"); }
  finally { if (btn) btn.disabled = false; }
}

async function setParams(obj, label) {
  const res = await api("POST", "/api/params/set", { params: obj });
  const bad = res.filter((r) => !r.ok);
  res.filter((r) => r.ok).forEach((r) => (S.params[r.name] = r.value));
  if (bad.length) toast(bad.map((b) => b.error).join(" · "), "bad");
  else toast(`${label || "Parâmetros"} gravado(s) e conferido(s).`, "ok");
  refreshNav();
  return res;
}

// ------------------------------------------------------- modo GPS / sem GPS
function cfgMode() {
  const en = is4() ? "COMPASS_ENABLE" : "MAG_ENABLE";
  if (P(en) === undefined) return S.mode;
  if (P(en) === 1 && P("COMPASS_USE") === 1) return "gps";
  if (P("COMPASS_USE") === 0) return "no_gps";
  return null;
}
const usesGps = () => cfgMode() === "gps";

// ---------------------------------------------------------------- checagens
const sensor = (name) => (S.snap.sensors || []).find((s) => s.name === name);
function accelCalibrated() {
  const o = ["X", "Y", "Z"].map((a) => P(`INS_ACCOFFS_${a}`));
  const s = ["X", "Y", "Z"].map((a) => P(`INS_ACCSCAL_${a}`));
  if (o[0] === undefined) return null;
  return o.some((v) => v !== 0) || s.some((v) => v !== 1);
}
function receiverPresent() {
  const rc = S.snap.rc;
  if (!rc || S.snap.rc_age > 2) return false;
  // Pixhawk sem sinal: nenhum canal. APM 2.x sem sinal: o encoder PPM põe o throttle em ~900 µs
  return is4() ? S.snap.rc_count > 0 && rc[2] > 900 : rc[2] > 950;
}
function radioCalibrated() {
  if (P("RC1_MIN") === undefined) return null;
  return [1, 2, 3, 4].every((c) => P(`RC${c}_MIN`) < 1250 && P(`RC${c}_MAX`) > 1750 && P(`RC${c}_MAX`) - P(`RC${c}_MIN`) > 500);
}

function checks() {
  const c = {};
  const s = S.snap;
  const conn = s.connected;
  const loaded = Object.keys(S.params).length > 0;
  const need = (ok, msgOk, msgBad, warn = false) => ({ s: ok ? "ok" : warn ? "warn" : "bad", msg: ok ? msgOk : msgBad });

  if (!conn) c.diag = { s: "bad", msg: "Conecte a placa." };
  else {
    const core = ["SENSOR_3D_GYRO", "SENSOR_3D_ACCEL", "SENSOR_ABSOLUTE_PRESSURE"].every((n) => sensor(n)?.healthy);
    const vccOk = s.vcc > B(4.6, 4.4) && s.vcc < B(5.4, 5.6);
    c.diag = need(core && vccOk && !s.i2c_err, `Sensores ok · Vcc ${fx(s.vcc, 2)} V`, !core ? "Giroscópio, acelerômetro ou barômetro com falha." : !vccOk ? `Vcc ${fx(s.vcc, 2)} V fora de 4,6–5,4 V.` : `Erros I2C: ${s.i2c_err}`);
  }
  const m = cfgMode();
  c.config = !loaded ? { s: "na", msg: "Aguardando parâmetros." } : !m ? { s: "bad", msg: "Escolha com ou sem GPS." }
    : P("ARMING_CHECK") === 0 ? { s: "warn", msg: "ARMING_CHECK=0: a placa arma sem checar nada. Aplique a escolha de novo." }
    : { s: "ok", msg: m === "gps" ? "Com GPS + bússola" : "Sem GPS (modos manuais)" };
  const ac = accelCalibrated();
  c.accel = ac === null ? { s: "na", msg: "Aguardando parâmetros." } : need(ac, "Calibrado.", "Nunca calibrado.");
  if (m !== "gps") { c.compass = { s: "na", msg: "Não usado sem GPS." }; c.gps = { s: "na", msg: "Não usado." }; }
  else {
    const mag = sensor("SENSOR_3D_MAG");
    const ofs = ["X", "Y", "Z"].map((a) => P(`COMPASS_OFS_${a}`) || 0);
    if (!mag) c.compass = { s: "bad", msg: `Bússola não detectada (reinicie após ${B("MAG_ENABLE", "COMPASS_ENABLE")}=1).` };
    else if (is4() && !mag.healthy) c.compass = { s: "bad", msg: "Bússola com falha (sem leitura). Use uma externa." };
    else if (ofs.every((v) => v === 0)) c.compass = { s: "bad", msg: "Detectada, mas não calibrada." };
    else if (Math.max(...ofs.map(Math.abs)) > 400) c.compass = { s: "warn", msg: "Offsets > 400: interferência." };
    else c.compass = { s: "ok", msg: `Offsets ${ofs.join(", ")}` };
    const g = s.gps;
    c.gps = !g ? { s: "bad", msg: "Sem dados de GPS." } : g.fix >= 3 ? { s: g.sats >= 6 && g.hdop < 2.3 ? "ok" : "warn", msg: `Fix 3D · ${g.sats} satélites · HDOP ${fx(g.hdop, 1)}` } : { s: g.fix === 0 ? "bad" : "warn", msg: g.fix === 0 ? "GPS não detectado." : "Sem fix: vá para céu aberto." };
  }
  const rcal = radioCalibrated();
  c.radio = !conn ? { s: "na", msg: "Conecte." } : !receiverPresent() ? { s: "bad", msg: B("Receptor sem sinal (CH3 ≈ 900).", "Receptor sem sinal no RC IN.") } : rcal ? { s: "ok", msg: "Receptor ok e calibrado." } : { s: "bad", msg: "Rádio não calibrado." };
  if (loaded) {
    const modes = [1, 2, 3, 4, 5, 6].map((i) => P(`FLTMODE${i}`));
    const gpsModes = new Set(S.meta?.gps_modes || []);
    const badGps = m !== "gps" && modes.some((x) => gpsModes.has(x));
    c.modes = need(!badGps, "Modos compatíveis.", "Há modo que precisa de GPS.");
    const fs = P("FS_THR_ENABLE");
    const fsOk = fs > 0 && P("FS_THR_VALUE") < P("RC3_MIN") - 10;
    c.failsafe = !fsOk ? { s: "bad", msg: fs > 0 ? "FS_THR_VALUE precisa ficar abaixo do RC3_MIN." : "Failsafe de rádio desligado." } : S.manual.failsafe_tested ? { s: "ok", msg: "Configurado e testado." } : { s: "warn", msg: "Configurado; falta testar." };
    const bm = P("BATT_MONITOR");
    c.battery = bm >= 3 && P("BATT_VOLT_PIN") >= 0 ? (P(S.meta?.battery?.fs_enable) > 0 ? { s: "ok", msg: "Monitor e failsafe ligados." } : { s: "warn", msg: "Monitor ligado, failsafe de bateria desligado." }) : { s: "warn", msg: "Sem monitor de bateria." };
  } else { c.modes = c.failsafe = c.battery = { s: "na", msg: "Aguardando parâmetros." }; }
  c.esc = S.manual.esc_done ? { s: "ok", msg: "Calibrados (confirmado)." } : { s: "warn", msg: "Ainda não confirmado." };
  const mt = ["A", "B", "C", "D"].every((k) => S.manual[`motor_${k}`]);
  c.motors = mt ? { s: "ok", msg: "Ordem e sentido conferidos." } : { s: "warn", msg: "Teste os 4 motores." };
  const pa = s.prearm || {};
  c.prearm = !conn ? { s: "na", msg: "Conecte." } : pa.messages?.length ? { s: "bad", msg: pa.messages.join(" · ") }
    : pa.ok === false ? { s: "bad", msg: P("ARMING_CHECK") === 0 ? "A placa não passa no PreArm, e com ARMING_CHECK=0 ela não diz o motivo." : "A placa não passa no PreArm (clique em Checar PreArm no Diagnóstico)." }
    : pa.ok || pa.conclusive ? { s: "ok", msg: "Nenhum bloqueio de PreArm." } : { s: "warn", msg: `Observando (${pa.watched_s}/40 s)…` };
  return c;
}

// ------------------------------------------------------------------ páginas
const PAGES = [
  { id: "diag", title: "Diagnóstico", group: "Placa" },
  { id: "config", title: "GPS ou sem GPS", group: "Placa" },
  { id: "accel", title: "Acelerômetro", group: "Sensores" },
  { id: "compass", title: "Bússola", group: "Sensores", gps: true },
  { id: "gps", title: "GPS", group: "Sensores", gps: true },
  { id: "radio", title: "Rádio", group: "Controle" },
  { id: "modes", title: "Modos de voo", group: "Controle" },
  { id: "failsafe", title: "Failsafe", group: "Controle" },
  { id: "battery", title: "Bateria", group: "Potência" },
  { id: "esc", title: "ESCs", group: "Potência" },
  { id: "motors", title: "Motores", group: "Potência" },
  { id: "preflight", title: "Checklist pré-voo", group: "Voo" },
  { id: "params", title: "Parâmetros", group: "Ferramentas" },
];

function buildNav() {
  let html = "", group = "";
  PAGES.forEach((p, i) => {
    if (p.group !== group) { group = p.group; html += `<div class="group">${group}</div>`; }
    html += `<a href="#${p.id}" data-page="${p.id}"><span class="n">${i + 1}</span>${p.title}<span class="st" data-st="${p.id}"></span></a>`;
  });
  $("#nav").innerHTML = html;
}
function refreshNav() {
  const c = checks();
  $$("#nav a").forEach((a) => a.classList.toggle("active", a.dataset.page === S.page));
  $$("[data-st]").forEach((el) => {
    const k = el.dataset.st;
    const st = k === "preflight" ? overall(c) : k === "params" ? { s: Object.keys(S.params).length ? "ok" : "na" } : c[k];
    el.className = `st ${st ? st.s : "na"}`;
    el.title = st?.msg || "";
  });
}
function overall(c) {
  const keys = ["diag", "config", "accel", "compass", "gps", "radio", "modes", "failsafe", "battery", "esc", "motors", "prearm"];
  const sts = keys.map((k) => c[k]?.s);
  if (sts.includes("bad")) return { s: "bad", msg: "Há itens bloqueantes." };
  if (sts.includes("warn")) return { s: "warn", msg: "Há itens a conferir." };
  return { s: "ok", msg: "Pronto para voar." };
}

function go(page) {
  if (!PAGES.find((p) => p.id === page)) page = "diag";
  S.page = page;
  const el = $("#main");
  el.innerHTML = `<section id="page-${page}">${RENDER[page]()}</section>`;
  BIND[page]?.(el);
  refreshNav();
  update();
  window.scrollTo(0, 0);
}
window.addEventListener("hashchange", () => go(location.hash.slice(1)));

function notConnected() {
  return `<div class="card overlay-disconnected"><img src="/static/img/zune-logo.png" alt="">
    <h3>Conecte a controladora</h3><p class="muted">Ligue a controladora na USB, escolha a porta no topo e clique em <b>Conectar</b>. A interface reconhece sozinha se é uma <b>APM 2.x</b> (ArduCopter 3.2) ou uma <b>Pixhawk</b> (ArduCopter 4.x).<br>
    Na APM, abrir a USB reinicia a placa: deixe-a parada por ~5 s enquanto o giroscópio calibra.<br>
    Feche o APM Planner/Mission Planner antes: só um programa pode usar a serial.</p></div>`;
}
const statusRow = (label, st) => `<div class="check"><span class="badge ${st.s}">${{ ok: "OK", warn: "ATENÇÃO", bad: "FALTA", na: "N/A" }[st.s]}</span><div class="txt"><b>${label}</b><span>${esc(st.msg)}</span></div></div>`;

// ================================================================ renders
const RENDER = {};
const BIND = {};
const UPDATE = {};

// ---------------------------------------------------------------- diagnóstico
RENDER.diag = () => `
  <h2>Diagnóstico da controladora</h2>
  <p class="lead">Confere se a controladora está viva: comunicação MAVLink, sensores internos, alimentação e bloqueios de PreArm. Incline a placa com a mão e veja o horizonte acompanhar.</p>
  <div id="diag-body"></div>`;
UPDATE.diag = () => {
  const s = S.snap, b = $("#diag-body");
  if (!s.connected) { b.innerHTML = notConnected(); return; }
  if (!$("#hz", b)) {
    b.innerHTML = `
    <div class="grid">
      <div class="card accent"><h3>Placa</h3><dl class="kv" id="d-board"></dl></div>
      <div class="card"><h3>Atitude</h3><canvas id="hz" width="300" height="180"></canvas><dl class="kv" id="d-att" style="margin-top:10px"></dl></div>
      <div class="card"><h3>IMU</h3><dl class="kv" id="d-imu"></dl><div id="d-imu-note"></div></div>
    </div>
    <div class="grid two">
      <div class="card"><h3>Sensores (SYS_STATUS)</h3><table><thead><tr><th>Sensor</th><th>Presente</th><th>Ativo</th><th>Saudável</th></tr></thead><tbody id="d-sens"></tbody></table></div>
      <div class="card"><h3>PreArm</h3><div id="d-prearm"></div><h3 style="margin-top:16px">Mensagens da placa</h3><div class="log" id="d-log"></div></div>
    </div>
    <div class="card"><h3>Ferramentas</h3><div class="row">
      <button class="secondary" data-cal="gyro">Recalibrar giroscópio</button>
      <button class="secondary" data-cal="baro">Recalibrar barômetro</button>
      <button class="secondary" id="btn-reboot">Reiniciar placa</button>
      ${is4() ? `<button class="secondary" id="btn-prearm">Checar PreArm agora</button>` : ""}
      <span class="muted">Mantenha a placa parada.</span></div></div>`;
    $$("[data-cal]", b).forEach((btn) => btn.onclick = () => run(btn, async () => {
      const r = await api("POST", `/api/calibrate/${btn.dataset.cal}`);
      toast(r.ok ? "Calibração aceita." : `Resultado ${r.result ?? "sem resposta"}`, r.ok ? "ok" : "bad");
    }));
    $("#btn-reboot", b).onclick = (e) => run(e.target, async () => { await api("POST", "/api/reboot"); toast("Placa reiniciada.", "ok"); });
    $("#btn-prearm", b)?.addEventListener("click", (e) => run(e.target, async () => { await api("POST", "/api/prearm"); toast("Pedido enviado: as mensagens aparecem no log."); }));
  }
  const mrev = P("SYSID_SW_MREV");
  $("#d-board").innerHTML = `
    <dt>Placa</dt><dd>${esc(s.board_label || "—")}${s.board_text ? ` (${esc(s.board_text)})` : ""}</dd>
    <dt>Firmware</dt><dd>${esc(s.firmware || (mrev ? `ArduCopter 3.2.x (MREV ${mrev})` : "—"))}</dd>
    <dt>Frame</dt><dd>${esc(s.frame_text || S.meta?.frames?.[P(S.meta?.frame_param)] || "—")}</dd>
    <dt>Vcc</dt><dd>${fx(s.vcc, 2)} V${is4() && s.vservo != null ? ` · servo ${fx(s.vservo, 2)} V` : ""}</dd>
    ${is4() ? `<dt>Chave de segurança</dt><dd>${s.safety_off ? "liberada (vermelho fixo)" : "ativa (piscando)"}</dd>` : `<dt>Erros I2C</dt><dd>${s.i2c_err ?? "—"}</dd>`}
    <dt>Carga CPU</dt><dd>${fx(s.load_pct, 1)} %</dd>
    <dt>RAM livre</dt><dd>${s.freemem ?? "—"} B</dd>
    <dt>Estado</dt><dd>${esc(s.system_status)} · ${esc(s.mode)}</dd>
    <dt>Parâmetros</dt><dd>${s.params.received}/${s.params.total ?? "?"}</dd>`;
  const a = s.attitude || {};
  drawHorizon($("#hz"), a.roll || 0, a.pitch || 0);
  $("#d-att").innerHTML = `<dt>Roll</dt><dd>${fx(a.roll)}°</dd><dt>Pitch</dt><dd>${fx(a.pitch)}°</dd><dt>Yaw</dt><dd>${fx(a.yaw)}°</dd>`;
  const i = s.imu;
  if (i) {
    $("#d-imu").innerHTML = `<dt>Acel. (mG)</dt><dd>${i.acc.join(" / ")}</dd><dt>|a|</dt><dd>${fx(i.acc_norm)} mG</dd>
      <dt>Giro (mrad/s)</dt><dd>${i.gyro.join(" / ")}</dd><dt>Mag (mG)</dt><dd>${i.mag.join(" / ")}</dd>
      <dt>Baro</dt><dd>${fx(s.baro?.press, 1)} hPa · ${fx(s.baro?.temp, 1)} °C</dd>`;
    const err = (i.acc_norm - 1000) / 10;
    $("#d-imu-note").innerHTML = Math.abs(err) < 2 ? `<div class="note ok">|a| a ${fx(err)} % de 1 g com a placa parada.</div>` : `<div class="note warn">|a| a ${fx(err)} % de 1 g. Se a placa está parada, calibre o acelerômetro.</div>`;
  }
  $("#d-sens").innerHTML = (s.sensors || []).map((x) => `<tr><td>${esc(x.name.replace("SENSOR_", ""))}</td><td>${yn(x.present)}</td><td>${yn(x.enabled)}</td><td>${yn(x.healthy)}</td></tr>`).join("");
  $("#d-prearm").innerHTML = statusRow("Bloqueios de armar", checks().prearm);
  renderLog($("#d-log"));
};
const yn = (v) => `<span class="badge ${v ? "ok" : "na"}">${v ? "sim" : "não"}</span>`;
function renderLog(el) {
  const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 5;
  el.innerHTML = (S.snap.texts || []).map((t) => `<div><span class="t">${new Date(t.t * 1000).toLocaleTimeString()}</span><span class="${t.text.startsWith("PreArm") ? "pre" : ""}">${esc(t.text)}</span></div>`).join("");
  if (atBottom) el.scrollTop = el.scrollHeight;
}

function drawHorizon(cv, roll, pitch) {
  const ctx = cv.getContext("2d"), w = cv.width, h = cv.height;
  ctx.save();
  ctx.clearRect(0, 0, w, h);
  ctx.translate(w / 2, h / 2);
  ctx.rotate((-roll * Math.PI) / 180);
  const off = pitch * 3;
  ctx.fillStyle = "#4a8fd8"; ctx.fillRect(-w, -h * 2 + off, w * 2, h * 2);
  ctx.fillStyle = "#8a5a2b"; ctx.fillRect(-w, off, w * 2, h * 2);
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-w, off); ctx.lineTo(w, off); ctx.stroke();
  ctx.lineWidth = 1; ctx.font = "10px system-ui"; ctx.fillStyle = "#fff";
  for (let p = -30; p <= 30; p += 10) {
    if (!p) continue;
    const y = off - p * 3;
    ctx.beginPath(); ctx.moveTo(-20, y); ctx.lineTo(20, y); ctx.stroke();
    ctx.fillText(String(p), 24, y + 3);
  }
  ctx.restore();
  ctx.strokeStyle = "#f6c945"; ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(w / 2 - 60, h / 2); ctx.lineTo(w / 2 - 15, h / 2); ctx.lineTo(w / 2, h / 2 + 10); ctx.lineTo(w / 2 + 15, h / 2); ctx.lineTo(w / 2 + 60, h / 2); ctx.stroke();
}

// ---------------------------------------------------------------- GPS ou não
RENDER.config = () => is4() ? CONFIG4() : `
  <h2>Vai voar com GPS ou sem GPS?</h2>
  <p class="lead">A APM 2.6 não tem bússola na placa: ela fica no módulo GPS externo. A escolha muda quais calibrações e modos de voo fazem sentido.</p>
  <div class="grid two">
    <div class="card choice" data-choice="gps"><h3>Com GPS + bússola</h3>
      <ul><li>Liga a bússola externa (<span class="mono">MAG_ENABLE=1</span>, <span class="mono">COMPASS_USE=1</span>)</li>
      <li>Todas as checagens de armar (<span class="mono">ARMING_CHECK=1</span>)</li>
      <li>Declinação automática pelo GPS</li>
      <li>Libera LOITER, POSHOLD, RTL, AUTO</li>
      <li>Precisa reiniciar a placa para detectar a bússola</li></ul>
      <div class="note warn"><b>Atenção ao módulo</b>O firmware 3.2 só reconhece a bússola <b>HMC5883L</b>. Módulos M8N genéricos com <b>QMC5883L</b> não funcionam.</div>
    </div>
    <div class="card choice" data-choice="no_gps"><h3>Sem GPS</h3>
      <ul><li>Desliga a bússola (<span class="mono">MAG_ENABLE=0</span>, <span class="mono">COMPASS_USE=0</span>)</li>
      <li>Todas as checagens menos bússola (<span class="mono">ARMING_CHECK=-5</span>)</li>
      <li>Só modos manuais: STABILIZE, ACRO, ALT_HOLD, LAND</li>
      <li>O yaw deriva devagar (rumo só pelo giroscópio): corrija no stick</li>
      <li>Failsafe pousa no lugar (não há RTL)</li></ul>
    </div>
  </div>
  <div class="card"><div class="row"><div id="cfg-current"></div><span class="spacer"></span><button id="cfg-apply">Aplicar escolha</button></div><div id="cfg-result"></div></div>`;
const CONFIG4 = () => `
  <h2>Vai voar com GPS ou sem GPS?</h2>
  <p class="lead">A Pixhawk tem bússola interna, mas ela sofre interferência dos fios de força: em voo com GPS use a bússola do módulo GPS externo. A escolha muda as fontes do EKF, as calibrações e os modos de voo.</p>
  <div class="grid two">
    <div class="card choice" data-choice="gps"><h3>Com GPS + bússola</h3>
      <ul><li>Liga as bússolas (<span class="mono">COMPASS_ENABLE=1</span>, <span class="mono">COMPASS_USE=1</span>)</li>
      <li>EKF usa GPS para posição e bússola para rumo (<span class="mono">EK3_SRC1_*</span>)</li>
      <li>Todas as checagens de armar (<span class="mono">ARMING_CHECK=1</span>)</li>
      <li>Libera LOITER, POSHOLD, RTL, SMART_RTL, AUTO</li>
      <li>Precisa reiniciar a placa</li></ul>
      <div class="note ok"><b>Módulos M8N funcionam</b>O ArduCopter 4.x reconhece QMC5883L, HMC5883L, IST8310 e outras bússolas. Ligue o conector de 4 fios na porta GPS e o de 2 fios (SDA/SCL) na porta I2C.</div>
    </div>
    <div class="card choice" data-choice="no_gps"><h3>Sem GPS</h3>
      <ul><li>Não usa bússola (<span class="mono">COMPASS_USE=0</span>)</li>
      <li>EKF só com IMU e barômetro (<span class="mono">EK3_SRC1_POSXY/VELXY/YAW=0</span>)</li>
      <li>Todas as checagens de armar (<span class="mono">ARMING_CHECK=1</span>)</li>
      <li>Só modos manuais: STABILIZE, ACRO, ALT_HOLD, LAND</li>
      <li>Failsafe pousa no lugar (não há RTL)</li></ul>
    </div>
  </div>
  <div class="card"><div class="row"><div id="cfg-current"></div><span class="spacer"></span><button id="cfg-apply">Aplicar escolha</button></div><div id="cfg-result"></div></div>`;
BIND.config = (el) => {
  let choice = cfgMode() || S.mode;
  const paint = () => $$(".choice", el).forEach((c) => c.classList.toggle("selected", c.dataset.choice === choice));
  $$(".choice", el).forEach((c) => (c.onclick = () => { choice = c.dataset.choice; paint(); }));
  paint();
  $("#cfg-apply", el).onclick = (e) => run(e.target, async () => {
    if (!choice) throw new Error("Escolha uma opção.");
    const r = await api("POST", `/api/preset/${choice}`);
    S.mode = choice; saveLocal("apm-mode", choice);
    r.results.filter((x) => x.ok).forEach((x) => (S.params[x.name] = x.value));
    const bad = r.results.filter((x) => !x.ok);
    $("#cfg-result").innerHTML = bad.length ? `<div class="note bad">${bad.map((b) => esc(b.error)).join("<br>")}</div>`
      : `<div class="note ok"><b>Gravado e conferido.</b>${r.needs_reboot ? `Algumas mudanças só valem depois de reiniciar. <button id="cfg-reboot" class="secondary">Reiniciar placa agora</button>` : ""}</div>`;
    $("#cfg-reboot")?.addEventListener("click", (ev) => run(ev.target, async () => { await api("POST", "/api/reboot"); toast("Placa reiniciada.", "ok"); }));
    refreshNav();
  });
};
UPDATE.config = () => {
  const m = cfgMode();
  $("#cfg-current").innerHTML = `Configuração atual na placa: <b>${m === "gps" ? "com GPS + bússola" : m === "no_gps" ? "sem GPS" : "indefinida"}</b>
    <span class="muted mono">(${is4() ? `COMPASS_ENABLE=${P("COMPASS_ENABLE") ?? "?"}, COMPASS_USE=${P("COMPASS_USE") ?? "?"}, EK3_SRC1_YAW=${P("EK3_SRC1_YAW") ?? "?"}` : `MAG_ENABLE=${P("MAG_ENABLE") ?? "?"}, COMPASS_USE=${P("COMPASS_USE") ?? "?"}`}, ARMING_CHECK=${P("ARMING_CHECK") ?? "?"})</span>
    ${P("ARMING_CHECK") === 0 ? `<div class="note bad"><b>ARMING_CHECK=0.</b>A placa arma sem conferir sensores, rádio nem bateria. Aplique uma das escolhas para voltar a 1.</div>` : ""}`;
  $("#cfg-apply").disabled = !S.snap.connected;
};

// ---------------------------------------------------------------- acelerômetro
const POS = {
  level: { t: "Nivelada", d: "Na posição normal de voo, sobre uma superfície plana.", view: "side", rot: 0 },
  left: { t: "Lado esquerdo", d: "Deitada sobre o lado ESQUERDO (o lado esquerdo encosta na mesa).", view: "rear", rot: -90 },
  right: { t: "Lado direito", d: "Deitada sobre o lado DIREITO (o lado direito encosta na mesa).", view: "rear", rot: 90 },
  nose_down: { t: "Nariz para baixo", d: "Em pé, com a seta/frente apontando para a mesa.", view: "side", rot: 90 },
  nose_up: { t: "Nariz para cima", d: "Em pé, com a seta/frente apontando para o teto.", view: "side", rot: -90 },
  back: { t: "De costas", d: "De cabeça para baixo (o topo encosta na mesa).", view: "side", rot: 180 },
};
const POS_ORDER = ["level", "left", "right", "nose_down", "nose_up", "back"];
function boardSvg(key, w = 170, h = 170) {
  const p = POS[key];
  const body = p.view === "side"
    ? `<rect x="-55" y="-10" width="110" height="20" rx="4" fill="var(--zune)"/><rect x="-55" y="-14" width="110" height="5" rx="2" fill="#f6c945"/>
       <path d="M58 0 l16 0 m-7 -7 l7 7 l-7 7" stroke="var(--ok)" stroke-width="4" fill="none" stroke-linecap="round"/>`
    : `<rect x="-55" y="-10" width="110" height="20" rx="4" fill="var(--zune)"/><rect x="-55" y="-14" width="110" height="5" rx="2" fill="#f6c945"/>
       <circle cx="0" cy="0" r="6" fill="var(--ok)"/>`;
  return `<svg viewBox="${-w / 2} ${-h / 2} ${w} ${h}"><line x1="${-w / 2}" y1="${h / 2 - 4}" x2="${w / 2}" y2="${h / 2 - 4}" stroke="var(--muted)" stroke-width="2"/>
    <g transform="rotate(${p.rot})">${body}</g></svg>`;
}
RENDER.accel = () => `
  <h2>Calibração do acelerômetro</h2>
  <p class="lead">Seis posições. A placa pede uma de cada vez, mede a gravidade e calcula offsets e escalas. Em cada posição: posicione, <b>solte</b>, espere 1–2 s e clique em <b>Medir</b>. Não precisa de esquadro, mas tem que estar parada.${is4() ? " Na Pixhawk a calibração acontece dentro da placa, que continua respondendo durante o processo." : ""}</p>
  <div class="card"><div class="positions">${POS_ORDER.map((k, i) => `<div class="pos" data-pos="${k}">${boardSvg(k)}<div><b>${i + 1}. ${POS[k].t}</b></div></div>`).join("")}</div>
    <p class="muted" style="margin:10px 0 0;font-size:13px">Faixa amarela = topo da placa · seta/ponto verde = frente (nariz). Lado esquerdo/direito: visto por trás.</p></div>
  <div class="card accent" id="ac-main"></div>
  <div class="grid">
    <div class="card"><h3>Gravidade medida agora</h3><div class="big" id="ac-norm">—</div><div class="muted">alvo: 1000 mG com a placa parada</div></div>
    <div class="card"><h3>Valores na placa</h3><dl class="kv" id="ac-params"></dl></div>
    <div class="card"><h3>Nivelamento (opcional)</h3><p class="muted" style="margin-top:0">Se o drone deriva em STABILIZE sem vento, coloque-o nivelado e grave o trim (AHRS_TRIM).</p><button class="secondary" id="ac-level">Gravar nível</button></div>
  </div>`;
BIND.accel = (el) => {
  $("#ac-level", el).onclick = (e) => run(e.target, async () => {
    const r = await api("POST", "/api/calibrate/level");
    toast(r.ok ? "Nível gravado." : `Placa respondeu ${r.result ?? "sem resposta"} (pode não suportar).`, r.ok ? "ok" : "bad");
    api("POST", "/api/params/refresh");
  });
  $("#ac-main", el).addEventListener("click", (e) => {
    const b = e.target.closest("button[data-act]");
    if (!b) return;
    run(b, async () => {
      await api("POST", `/api/accel/${b.dataset.act}`);
      if (b.dataset.act === "start") toast("Calibração iniciada.");
    });
  });
};
UPDATE.accel = () => {
  const s = S.snap, a = s.accel || {};
  const doneIdx = a.state === "success" ? 6 : Math.max(0, a.step - (a.state === "waiting" ? 1 : 0));
  $$(".pos").forEach((d) => {
    const i = POS_ORDER.indexOf(d.dataset.pos);
    d.classList.toggle("current", ["waiting", "measuring"].includes(a.state) && d.dataset.pos === a.position);
    d.classList.toggle("done", i < doneIdx && !(["waiting", "measuring"].includes(a.state) && d.dataset.pos === a.position));
  });
  const main = $("#ac-main");
  let html;
  if (!s.connected) html = notConnected();
  else if (a.state === "idle") html = `<div class="row"><div><h3>Pronto para começar</h3><span class="muted">Desarmada, hélices fora, cabo USB com folga para girar a placa. ${accelCalibrated() ? "Já existe uma calibração; refazer substitui." : "Esta placa ainda não foi calibrada."}</span></div><span class="spacer"></span><button data-act="start">Iniciar calibração</button></div>`;
  else if (a.state === "starting") html = `<h3>Iniciando…</h3><span class="muted">Aguardando a placa pedir a primeira posição.</span>`;
  else if (a.state === "waiting" || a.state === "measuring") {
    const p = POS[a.position] || { t: a.prompt, d: "" };
    html = `<div class="hero-step">${a.position in POS ? boardSvg(a.position, 190, 190) : ""}<div>
      <div class="muted">Posição ${a.step} de 6</div><h3 style="font-size:22px;margin:4px 0">${esc(p.t)}</h3><p>${esc(p.d)}</p>
      <p class="muted mono" style="font-size:12px">firmware: ${esc(a.prompt)}</p>
      ${a.state === "waiting" ? `<button data-act="next">Posicionei, medir</button>` : `<button disabled>Medindo, não mexa…</button>`}</div></div>`;
  } else if (a.state === "success") {
    const r = a.result || {};
    html = `<div class="note ok"><b>Calibração concluída.</b>Offsets ${["X", "Y", "Z"].map((x) => fx(r[`INS_ACCOFFS_${x}`], 3)).join(" / ")} m/s² · escalas ${["X", "Y", "Z"].map((x) => fx(r[`INS_ACCSCAL_${x}`], 4)).join(" / ")}.
      Coloque a placa nivelada e confira se a gravidade medida fica perto de 1000 mG.${a.inferred ? " <span class=\"muted\">(A mensagem final da placa se perdeu; o resultado foi confirmado relendo os parâmetros.)</span>" : ""}</div><button class="secondary" data-act="reset">Fechar</button>`;
  } else {
    html = `<div class="note bad"><b>A calibração falhou.</b>Nada foi gravado. Causas comuns: placa se mexeu durante a medida, posição repetida, ou cabo puxando. Repita segurando cada posição bem parada.${a.inferred ? " (Confirmado relendo os parâmetros: nada mudou.)" : ""}</div><button data-act="start">Tentar de novo</button>`;
  }
  if (main.dataset.html !== html) { main.innerHTML = html; main.dataset.html = html; }
  const n = s.imu?.acc_norm;
  $("#ac-norm").textContent = n ? `${fx(n)} mG (${fx((n - 1000) / 10)} %)` : "—";
  $("#ac-params").innerHTML = ["X", "Y", "Z"].map((x) => `<dt>OFFS_${x} / SCAL_${x}</dt><dd>${fx(P(`INS_ACCOFFS_${x}`), 3)} / ${fx(P(`INS_ACCSCAL_${x}`), 4)}</dd>`).join("")
    + `<dt>AHRS_TRIM X/Y</dt><dd>${fx(P("AHRS_TRIM_X"), 4)} / ${fx(P("AHRS_TRIM_Y"), 4)} rad</dd>`;
};

// ---------------------------------------------------------------- bússola
const ORIENTS = ["None", "Yaw45", "Yaw90", "Yaw135", "Yaw180", "Yaw225", "Yaw270", "Yaw315", "Roll180", "Roll180Yaw45", "Roll180Yaw90", "Roll180Yaw135", "Pitch180", "Roll180Yaw225", "Roll180Yaw270", "Roll180Yaw315"];
function gpsOnly() {
  return cfgMode() === "gps" ? "" : `<div class="note warn"><b>Configuração atual: sem GPS.</b>Esta etapa só é necessária com GPS + bússola. <a href="#config">Mudar a escolha</a>.</div>`;
}
RENDER.compass = () => is4() ? COMPASS4() : `
  <h2>Bússola (externa)</h2>
  <p class="lead">No ArduCopter 3.2 a calibração da bússola é feita pela estação de solo: você gira o drone em todas as direções, a interface coleta o campo magnético e ajusta uma esfera para achar os offsets.</p>
  ${gpsOnly()}
  <div class="card"><h3>1. Hardware e detecção</h3><div id="cp-detect"></div>
    <ol class="steps-list" style="margin-top:12px">
      <li><b>Ligue o módulo GPS+bússola</b>Cabo de 5/6 vias na porta GPS e cabo de 4 vias na porta I2C da APM. Seta do módulo para a frente do drone, longe da bateria e dos fios de força.</li>
      <li><b>MAG_ENABLE=1 e reiniciar</b>A bússola só é procurada no boot. <button class="secondary" id="cp-enable">Ligar bússola e reiniciar</button></li>
    </ol></div>
  <div class="card"><h3>2. Orientação</h3>
    <div class="row"><label class="field">COMPASS_EXTERNAL<select id="cp-ext"><option value="1">1: externa</option><option value="0">0: interna</option></select></label>
      <label class="field">COMPASS_ORIENT<select id="cp-orient">${ORIENTS.map((o, i) => `<option value="${i}">${i}: ${o}</option>`).join("")}</select></label>
      <button class="secondary" id="cp-orient-save">Gravar</button></div>
    <p class="muted">Módulo 3DR antigo (bússola embaixo da placa): <b>Roll180 (8)</b>. A maioria dos módulos com a seta para a frente: <b>None (0)</b>. Confira abaixo: aponte o nariz para o norte (use a bússola do celular) e gire 90° de cada vez.</p>
    <div class="row"><div><div class="muted">Rumo atual</div><div class="big" id="cp-heading">—</div></div><div class="muted" style="max-width:420px">Norte ≈ 0°, Leste ≈ 90°, Sul ≈ 180°, Oeste ≈ 270°. Se andar ao contrário ou pular 180°, a orientação está errada. A declinação em Natal é de cerca de −21°.</div></div></div>
  <div class="card accent"><h3>3. Calibração (gire o drone)</h3><div id="cp-cal"></div>
    <div class="grid" style="margin-top:12px"><canvas id="cp-xy" width="220" height="220"></canvas><canvas id="cp-xz" width="220" height="220"></canvas><canvas id="cp-yz" width="220" height="220"></canvas></div></div>
  <div class="card"><h3>4. Declinação</h3><div class="row"><label class="row"><input type="checkbox" id="cp-autodec"> COMPASS_AUTODEC: declinação automática pelo GPS (recomendado)</label></div></div>`;
BIND.compass = (el) => {
  if (is4()) return BIND_COMPASS4(el);
  $("#cp-enable", el).onclick = (e) => run(e.target, async () => {
    await setParams({ MAG_ENABLE: 1, COMPASS_USE: 1 }, "Bússola");
    await api("POST", "/api/reboot");
    toast("Placa reiniciada.", "ok");
  });
  $("#cp-ext", el).value = String(P("COMPASS_EXTERNAL") ?? 1);
  $("#cp-orient", el).value = String(P("COMPASS_ORIENT") ?? 0);
  $("#cp-orient-save", el).onclick = (e) => run(e.target, () => setParams({ COMPASS_EXTERNAL: +$("#cp-ext").value, COMPASS_ORIENT: +$("#cp-orient").value }, "Orientação"));
  $("#cp-autodec", el).checked = P("COMPASS_AUTODEC") === 1;
  $("#cp-autodec", el).onchange = (e) => run(null, () => setParams({ COMPASS_AUTODEC: e.target.checked ? 1 : 0 }, "COMPASS_AUTODEC"));
  $("#cp-cal", el).addEventListener("click", (e) => {
    const b = e.target.closest("button[data-act]");
    if (b) run(b, async () => {
      const r = await api("POST", `/api/compass/${b.dataset.act}`);
      if (b.dataset.act === "apply") { r.filter((x) => x.ok).forEach((x) => (S.params[x.name] = x.value)); toast("Offsets gravados.", "ok"); await api("POST", "/api/compass/reset"); }
    });
  });
};
UPDATE.compass = () => {
  if (is4()) return UPDATE_COMPASS4();
  const s = S.snap, c = s.compass || {};
  const mag = sensor("SENSOR_3D_MAG");
  $("#cp-detect").innerHTML = !s.connected ? notConnected() : mag
    ? `<div class="note ${mag.healthy ? "ok" : "warn"}"><b>Bússola detectada${mag.healthy ? "" : ", mas não saudável"}.</b>Campo: ${s.imu?.mag.join(" / ")} mG</div>`
    : `<div class="note bad"><b>Bússola não detectada.</b>MAG_ENABLE=${P("MAG_ENABLE") ?? "?"}. Conecte o módulo, ligue a bússola e reinicie.</div>`;
  $("#cp-heading").textContent = s.hud ? `${s.hud.heading}°` : "—";
  const el = $("#cp-cal");
  let html;
  if (c.state === "idle") html = `<div class="row"><span class="muted">Longe de metal, celular e computador. Ao começar, gire o drone lentamente por ~60 s: cada face para cima, nariz para cima/baixo, e dê voltas completas em cada posição.</span><span class="spacer"></span><button data-act="start" ${mag ? "" : "disabled"}>Começar coleta</button></div>`;
  else if (c.state === "collecting") {
    const lv = c.live;
    html = `<div class="row"><div><div class="muted">${c.n} amostras · ${c.elapsed} s</div><div class="big">${lv ? Math.round(lv.coverage * 100) : 0}% coberto</div></div><span class="spacer"></span>
      <div style="width:260px"><div class="cells">${(lv?.cells || Array(32).fill(0)).map((x) => `<i class="${x ? "on" : ""}"></i>`).join("")}</div><div class="muted" style="font-size:12px;margin-top:4px">32 regiões da esfera: preencha todas</div></div>
      <button data-act="stop">Terminar e calcular</button></div>`;
  } else if (c.state === "done") {
    const r = c.result;
    const k = { boa: "ok", "aceitável": "warn", ruim: "bad" }[r.quality];
    html = `<div class="note ${k}"><b>Qualidade ${r.quality}.</b>${esc(r.message)}</div>
      <dl class="kv"><dt>Novos offsets</dt><dd>${r.offsets.join(" / ")}</dd><dt>Raio (campo)</dt><dd>${r.radius} mG</dd><dt>Resíduo</dt><dd>${r.rms} mG (${r.rms_pct} %)</dd><dt>Cobertura</dt><dd>${Math.round(r.coverage * 100)} % · ${r.n} amostras</dd></dl>
      <div class="row" style="margin-top:12px"><button data-act="apply">Gravar offsets</button><button class="secondary" data-act="reset">Descartar</button></div>`;
  }
  if (el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; }
  const pts = c.points || [];
  const r = c.live?.radius || c.result?.radius || 300;
  drawProj($("#cp-xy"), pts, 0, 1, r, "X × Y"); drawProj($("#cp-xz"), pts, 0, 2, r, "X × Z"); drawProj($("#cp-yz"), pts, 1, 2, r, "Y × Z");
};
// bússola no ArduCopter 4.x: calibração feita pela placa (MAG_CAL_PROGRESS / MAG_CAL_REPORT)
const COMPASS4 = () => `
  <h2>Bússola</h2>
  <p class="lead">No ArduCopter 4.x a própria placa calibra a bússola: você gira o drone e ela ajusta offsets, escala e até descobre a orientação de cada bússola. A interface mostra o progresso e grava o resultado.</p>
  ${gpsOnly()}
  <div class="card"><h3>1. Hardware e detecção</h3><div id="cp-detect"></div><div id="cp-prio"></div>
    <ol class="steps-list" style="margin-top:12px">
      <li><b>Ligue o módulo GPS+bússola</b>Conector de 4 fios (VCC, GND, TX, RX) na porta <b>GPS</b>, conector de 2 fios (SDA, SCL) na porta <b>I2C</b>. Seta do módulo para a frente, em um mastro longe da bateria e dos ESCs.</li>
      <li><b>COMPASS_ENABLE=1 e reiniciar</b>As bússolas são procuradas no boot. <button class="secondary" id="cp-enable">Ligar bússola e reiniciar</button></li>
      <li><b>Bússola interna</b>Ela fica perto dos fios de força. Com uma externa funcionando, desligue a interna se a calibração reclamar de interferência: <button class="secondary" id="cp-noint">Usar só a externa</button></li>
    </ol></div>
  <div class="card accent"><h3>2. Calibração (gire o drone)</h3><div id="cp-cal"></div></div>
  <div class="card"><h3>3. Conferir o rumo</h3>
    <div class="row"><div><div class="muted">Rumo atual</div><div class="big" id="cp-heading">—</div></div><div class="muted" style="max-width:460px">Aponte o nariz para o norte (bússola do celular) e gire 90° de cada vez: Norte ≈ 0°, Leste ≈ 90°, Sul ≈ 180°, Oeste ≈ 270°. Com <span class="mono">COMPASS_AUTODEC</span> ligado, a declinação de Natal (≈ −21°) é corrigida pelo GPS.</div></div>
    <label class="row" style="margin-top:10px"><input type="checkbox" id="cp-autodec"> COMPASS_AUTODEC: declinação automática pelo GPS (recomendado)</label></div>`;
function BIND_COMPASS4(el) {
  $("#cp-enable", el).onclick = (e) => run(e.target, async () => {
    await setParams({ COMPASS_ENABLE: 1, COMPASS_USE: 1 }, "Bússola");
    await api("POST", "/api/reboot");
    toast("Placa reiniciada.", "ok");
  });
  $("#cp-noint", el).onclick = (e) => run(e.target, () => setParams(internalOff(), "Bússolas"));
  $("#cp-autodec", el).checked = P("COMPASS_AUTODEC") === 1;
  $("#cp-autodec", el).onchange = (e) => run(null, () => setParams({ COMPASS_AUTODEC: e.target.checked ? 1 : 0 }, "COMPASS_AUTODEC"));
  $("#cp-cal", el).addEventListener("click", (e) => {
    const b = e.target.closest("button[data-act]");
    if (b) run(b, async () => {
      if (b.dataset.act === "reboot") { await api("POST", "/api/reboot"); toast("Placa reiniciada.", "ok"); return; }
      await api("POST", `/api/compass/${b.dataset.act}`);
      if (b.dataset.act === "accept") toast("Calibração gravada. Reinicie a placa para aplicar.", "ok");
    });
  });
}
// COMPASS_EXTERNAL=0 marca a bússola interna; desliga o "USE" das que são internas
function internalOff() {
  const out = {};
  [["", "COMPASS_EXTERNAL", "COMPASS_USE"], ["2", "COMPASS_EXTERN2", "COMPASS_USE2"], ["3", "COMPASS_EXTERN3", "COMPASS_USE3"]].forEach(([, ext, use]) => {
    if (P(ext) === 0 && P(use) !== undefined) out[use] = 0;
  });
  if (!Object.keys(out).length) throw new Error("Nenhuma bússola externa identificada ainda (COMPASS_EXTERNAL). Calibre primeiro.");
  return out;
}
// COMPASS_DEV_ID / PRIOn_ID: bits 0–2 tipo de barramento, 3–7 barramento, 8–15 endereço, 16–23 modelo
const MAG_TYPES = { 1: "HMC5883 (antigo)", 2: "LSM303D", 4: "AK8963", 5: "BMM150", 6: "LSM9DS1", 7: "HMC5883", 8: "LIS3MDL", 9: "AK09916", 10: "IST8310", 11: "ICM20948", 12: "MMC3416", 13: "QMC5883L", 14: "MAG3110", 16: "IST8308", 17: "RM3100", 19: "MMC5883", 20: "AK09918", 21: "AK09915", 22: "QMC5883P" };
const BUS_TYPES = { 1: "I2C", 2: "SPI", 3: "DroneCAN", 5: "MSP", 6: "serial" };
function decodeMagId(id) {
  if (!id) return "—";
  const bt = id & 7, bus = (id >> 3) & 31, addr = (id >> 8) & 255, dt = (id >> 16) & 255;
  return `${MAG_TYPES[dt] || `modelo ${dt}`} · ${BUS_TYPES[bt] || bt} ${bus} · 0x${addr.toString(16).padStart(2, "0")}`;
}
// bússolas presentes neste boot (o 4.x lista em COMPASS_DEV_ID..ID8)
const magDetected = () => [...new Set(["", 2, 3, 4, 5, 6, 7, 8].map((i) => P(`COMPASS_DEV_ID${i}`)).filter((x) => x))];
const MAGST = { 0: "não começou", 1: "aguardando", 2: "girando: passo 1", 3: "girando: passo 2", 4: "ok", 5: "falhou", 6: "orientação ruim", 7: "raio ruim (interferência)" };
function UPDATE_COMPASS4() {
  // página desenhada para a APM antes de a placa ser identificada
  if (!$("#cp-prio")) return go("compass");
  const s = S.snap, c = s.compass || {};
  const mag = sensor("SENSOR_3D_MAG");
  const found = magDetected();
  const prio = [1, 2, 3].map((n) => ({ n, id: P(`COMPASS_PRIO${n}_ID`), use: P(n === 1 ? "COMPASS_USE" : `COMPASS_USE${n}`) })).filter((x) => x.id);
  const missing = prio.some((x) => !found.includes(x.id)) || found.some((id) => !prio.some((x) => x.id === id));
  $("#cp-detect").innerHTML = !s.connected ? notConnected() : (mag
    ? `<div class="note ${mag.healthy ? "ok" : "bad"}"><b>${mag.healthy ? "Bússola funcionando." : "Bússola detectada, mas sem leitura (com falha)."}</b>Campo: ${s.imu?.mag.join(" / ")} mG${mag.healthy ? "" : ". Se for a interna da Pixhawk, ligue uma externa."}</div>`
    : `<div class="note bad"><b>Nenhuma bússola.</b>COMPASS_ENABLE=${P("COMPASS_ENABLE") ?? "?"}. Conecte o módulo, ligue a bússola e reinicie.</div>`);
  const prioEl = $("#cp-prio");
  const prioHtml = !s.connected ? "" : (prio.length || found.length ? `<table style="margin-top:10px"><thead><tr><th>Prioridade</th><th>Bússola</th><th>Presente agora</th><th>Usar</th></tr></thead><tbody>
      ${prio.map((x) => `<tr><td>${x.n}</td><td>${decodeMagId(x.id)} <span class="muted mono">(${x.id})</span></td><td>${yn(found.includes(x.id))}</td><td>${yn(x.use === 1)}</td></tr>`).join("")}
      ${found.filter((id) => !prio.some((x) => x.id === id)).map((id) => `<tr><td>—</td><td>${decodeMagId(id)} <span class="muted mono">(${id})</span></td><td>${yn(true)}</td><td>fora da lista</td></tr>`).join("")}
      </tbody></table>` : "")
    + (missing && found.length ? `<div class="note warn" style="margin-top:10px"><b>A lista de prioridade não bate com as bússolas ligadas.</b>A placa guarda as bússolas de configurações antigas; uma que não está presente na prioridade 1 deixa a bússola "com falha". <button class="secondary" id="cp-reorder">Usar as bússolas presentes e reiniciar</button></div>` : "");
  if (prioEl.dataset.html === prioHtml) return finishCompass4(s, c, mag);
  prioEl.innerHTML = prioEl.dataset.html = prioHtml;
  $("#cp-reorder")?.addEventListener("click", (e) => run(e.target, async () => {
    const f = magDetected();
    await setParams({ COMPASS_PRIO1_ID: f[0] || 0, COMPASS_PRIO2_ID: f[1] || 0, COMPASS_PRIO3_ID: f[2] || 0, COMPASS_USE: 1 }, "Prioridade das bússolas");
    await api("POST", "/api/reboot");
    toast("Placa reiniciada. Agora calibre a bússola.", "ok");
  }));
  finishCompass4(s, c, mag);
}
function finishCompass4(s, c, mag) {
  $("#cp-heading").textContent = s.hud ? `${s.hud.heading}°` : "—";
  const el = $("#cp-cal");
  let html;
  if (c.state === "running") {
    html = `<div class="muted">${c.elapsed ?? 0} s · gire devagar: cada face para cima e voltas completas em cada posição</div>` + (c.progress || []).map((p) =>
      `<div style="margin-top:12px"><div class="row"><b>Bússola ${p.id + 1}</b><span class="muted">${MAGST[p.status] || p.status}${p.attempt > 1 ? ` · tentativa ${p.attempt}` : ""}</span><span class="spacer"></span><b>${p.pct}%</b></div>
      <div class="bar"><div class="fill" style="width:${p.pct}%"></div></div>
      <div class="cells" style="grid-template-columns:repeat(20,1fr);margin-top:6px">${p.cells.map((x) => `<i class="${x ? "on" : ""}"></i>`).join("")}</div></div>`).join("")
      + (c.progress?.length ? "" : `<p class="muted">Aguardando a placa começar…</p>`)
      + `<div class="row" style="margin-top:12px"><button class="secondary" data-act="reset">Cancelar</button></div>`;
  } else if (c.state === "done" || c.state === "failed" || c.state === "accepted") {
    const ok = c.state !== "failed";
    html = `<div class="note ${c.state === "accepted" ? "ok" : ok ? "ok" : "bad"}"><b>${c.state === "accepted" ? "Gravada. Reinicie a placa para aplicar." : ok ? "Calibração concluída." : "A calibração falhou."}</b>${ok ? "" : "Gire mais devagar e mais longe de metal. Raio ruim = interferência (fios, bateria, mesa de metal)."}</div>
      <table><thead><tr><th>Bússola</th><th>Resultado</th><th class="num">Fitness</th><th>Offsets</th><th>Orientação</th></tr></thead><tbody>
      ${(c.reports || []).map((r) => `<tr><td>${r.id + 1}</td><td><span class="badge ${r.ok ? "ok" : "bad"}">${MAGST[r.status] || r.message}</span></td><td class="num">${r.fitness}</td><td class="mono">${r.offsets.join(" / ")}</td><td>${r.orientation ?? "—"}</td></tr>`).join("")}
      </tbody></table><p class="muted" style="font-size:13px">Fitness é o erro do ajuste em mG: abaixo de 8 é ótimo, até 16 aceitável. Offsets acima de 600 indicam interferência.</p>
      <div class="row">${c.state === "done" ? `<button data-act="accept">Gravar calibração</button>` : ""}${c.state === "accepted" ? `<button data-act="reboot">Reiniciar placa</button>` : ""}<button class="secondary" data-act="${ok ? "reset" : "start"}">${ok ? "Fechar" : "Tentar de novo"}</button></div>`;
  } else {
    html = `<div class="row"><span class="muted">Longe de metal, celular e computador. Ao começar, gire o drone por 1–2 min em todas as direções até as barras chegarem a 100%.</span><span class="spacer"></span><button data-act="start" ${mag ? "" : "disabled"}>Começar calibração</button></div>`;
  }
  if (el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; }
}
function drawProj(cv, pts, a, b, r, label) {
  if (!cv) return;
  const ctx = cv.getContext("2d"), w = cv.width, h = cv.height, k = (w / 2 - 14) / (r * 1.25);
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = "#7766914d"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(w / 2, h / 2, r * k, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(w / 2, 8); ctx.lineTo(w / 2, h - 8); ctx.moveTo(8, h / 2); ctx.lineTo(w - 8, h / 2); ctx.stroke();
  ctx.fillStyle = "#3a205f";
  pts.forEach((p) => ctx.fillRect(w / 2 + p[a] * k - 1.5, h / 2 - p[b] * k - 1.5, 3, 3));
  ctx.fillStyle = "#6b6178"; ctx.font = "12px system-ui"; ctx.fillText(label, 8, 16);
}

// ---------------------------------------------------------------- GPS
const FIX = ["Sem GPS", "Sem fix", "Fix 2D", "Fix 3D", "DGPS", "RTK float", "RTK fixo"];
RENDER.gps = () => `
  <h2>GPS</h2>
  <p class="lead">O GPS precisa de céu aberto. Antes de voar em LOITER/RTL espere fix 3D, pelo menos 6 satélites (AHRS_GPS_MINSATS) e HDOP abaixo de 2,3 (GPS_HDOP_GOOD).</p>
  ${gpsOnly()}
  <div class="grid"><div class="card accent"><h3>Estado</h3><div class="big" id="g-fix">—</div><dl class="kv" id="g-kv" style="margin-top:10px"></dl></div>
  <div class="card"><h3>Se não aparecer nada</h3><ol class="steps-list">
    <li><b>Cabo na porta GPS</b>Não na porta Telem. TX do módulo no RX da placa e vice-versa. O LED do módulo pisca quando tem fix.</li>
    <li><b>GPS_TYPE=1 (auto)</b>${B("A APM detecta uBlox e MTK em 38400/57600 baud.", "A Pixhawk detecta e configura u-blox (6M, 7M, M8N, M9N…) sozinha. A porta GPS é a SERIAL3.")}</li>
    <li><b>Primeiro fix</b>Pode levar alguns minutos no "cold start". Longe de prédios.</li></ol></div></div>`;
UPDATE.gps = () => {
  const g = S.snap.gps;
  $("#g-fix").textContent = g ? FIX[g.fix] || g.fix : "—";
  $("#g-kv").innerHTML = g ? `<dt>Satélites</dt><dd>${g.sats}</dd><dt>HDOP</dt><dd>${fx(g.hdop, 2)}</dd><dt>Lat / Lon</dt><dd>${fx(g.lat, 6)} / ${fx(g.lon, 6)}</dd><dt>GPS_TYPE</dt><dd>${P("GPS_TYPE") ?? "—"}</dd>` : "";
};

// ---------------------------------------------------------------- rádio
const RC_NAMES = ["Roll (aileron)", "Pitch (profundor)", "Throttle", "Yaw (leme)", "Modo de voo", "Ch 6 (tuning)", "Ch 7 (aux)", "Ch 8 (aux)"];
// nome do canal pela função configurada (modos / RCn_OPTION no 4.x)
function rcName(i) {
  if (i < 4) return RC_NAMES[i];
  if (i === modeCh() - 1) return "Modo de voo";
  const opt = is4() ? P(`RC${i + 1}_OPTION`) : 0;
  if (opt > 0) return AUX_OPTIONS[opt] || `opção ${opt}`;
  return is4() ? `Ch ${i + 1} (livre)` : RC_NAMES[i];
}
const RC_HINT = ["Stick à direita → aumenta", "Stick para FRENTE (cima) → DIMINUI", "Stick para cima → aumenta", "Stick à direita → aumenta", "Chave de 3/6 posições", "", "", ""];
const RC_DIR = [1, -1, 1, 1]; // sinal de (PWM - trim) para direita / frente / cima / direita
RENDER.radio = () => `
  <h2>Rádio (receptor)</h2>
  <p class="lead">Ligue o transmissor e o receptor (${B("entradas 1–8 da APM; em PPM, tudo na entrada 1", "porta RC IN da Pixhawk: PPM ou SBUS")}). Mova cada stick e confira a direção. Depois calibre: a placa precisa saber o mínimo, o máximo e o centro de cada canal.</p>
  <div id="rc-alert"></div>
  <div class="card"><h3>Canais ao vivo</h3><div id="rc-bars"></div>
    <p class="muted" style="font-size:13px">Verde = trim gravado (RCn_TRIM), amarelo = mínimo e máximo gravados. Se uma direção estiver invertida, inverta o canal <b>no transmissor</b>.</p></div>
  <div class="card accent"><h3>Calibração</h3><div id="rc-cal"></div></div>`;
BIND.radio = (el) => {
  $("#rc-bars", el).innerHTML = RC_NAMES.map((n, i) => `<div class="rc-row"><div><b>CH${i + 1}</b> <span class="muted">${rcName(i)}</span></div>
    <div class="bar" id="rcb${i}"><div class="fill"></div><div class="mark" data-m="min"></div><div class="mark" data-m="max"></div><div class="mark trim" data-m="trim"></div><div class="lbl"><span></span><span></span></div></div>
    <div class="hint muted" id="rch${i}">${i === modeCh() - 1 ? RC_HINT[4] : i < 4 ? RC_HINT[i] : ""}</div></div>`).join("");
  $("#rc-cal", el).addEventListener("click", (e) => {
    const b = e.target.closest("button[data-act]");
    if (!b) return;
    run(b, async () => {
      if (b.dataset.act === "save") {
        const res = await api("POST", "/api/rc/save");
        res.filter((x) => x.ok).forEach((x) => (S.params[x.name] = x.value));
        const bad = res.filter((x) => !x.ok);
        toast(bad.length ? bad.map((x) => x.error).join(" · ") : "Calibração do rádio gravada.", bad.length ? "bad" : "ok");
      } else await api("POST", `/api/rc/${b.dataset.act}`);
    });
  });
};
const pct = (v) => Math.max(0, Math.min(100, ((v - 800) / 1400) * 100));
UPDATE.radio = () => {
  const s = S.snap, rc = s.rc || [], cal = s.rc_cal || {};
  $("#rc-alert").innerHTML = !s.connected ? notConnected() : receiverPresent() ? "" : is4()
    ? `<div class="note bad"><b>Receptor sem sinal.</b>A Pixhawk não está recebendo nenhum canal. Receptor no <b>RC IN</b> (ele alimenta o receptor), saída em <b>PPM</b> ou SBUS (no FS-i6: System → RX setup → Output mode → PPM), transmissor ligado.</div>`
    : `<div class="note bad"><b>Receptor sem sinal.</b>O throttle está em ${rc[2] ?? "—"} µs. Sem sinal, a APM 2.x reporta ~900 µs no CH3. Ligue o transmissor e confira o bind e o cabo do receptor.</div>`;
  rc.forEach((v, i) => {
    const b = $(`#rcb${i}`);
    if (!b) return;
    $(".fill", b).style.width = `${pct(v)}%`;
    const capturing = cal.state === "capturing" || cal.state === "captured";
    const mn = capturing && cal.maxs[i] ? cal.mins[i] : P(`RC${i + 1}_MIN`);
    const mx = capturing && cal.maxs[i] ? cal.maxs[i] : P(`RC${i + 1}_MAX`);
    const tr = capturing && cal.trims ? cal.trims[i] : P(`RC${i + 1}_TRIM`);
    $('[data-m="min"]', b).style.left = `${pct(mn)}%`;
    $('[data-m="max"]', b).style.left = `${pct(mx)}%`;
    $('[data-m="trim"]', b).style.left = `${pct(tr)}%`;
    const lbl = $$(".lbl span", b);
    lbl[0].textContent = `${v} µs`;
    lbl[1].textContent = mn && mx ? `${mn}–${mx}` : "";
    if (i < 4 && tr && receiverPresent() && Math.abs(v - tr) > 120) {
      // pela convenção do ArduCopter: pitch para frente = PWM menor; os outros = PWM maior para direita/cima
      const up = (v - tr) * RC_DIR[i] > 0;
      const name = i === 1 ? (up ? "frente" : "trás") : i === 2 ? (v > tr ? "cima" : "baixo") : up ? "direita" : "esquerda";
      $(`#rch${i}`).innerHTML = `${RC_HINT[i]}<br>a placa entende: <b>${name}</b>`;
    } else if (i < 4) $(`#rch${i}`).textContent = RC_HINT[i];
    if (i === modeCh() - 1) {
      const slot = fltSlot(v);
      $(`#rch${i}`).innerHTML = `Posição ${slot} → <b>${S.meta?.modes?.[P(`FLTMODE${slot}`)] ?? "?"}</b>`;
    }
  });
  const el = $("#rc-cal");
  let html;
  if (cal.state === "capturing") html = `<ol class="steps-list"><li><b>Mova todos os sticks até o fim do curso</b>Círculos completos nos dois sticks e todas as chaves (CH5–CH8) nas posições extremas. As marcas amarelas mostram o que já foi capturado.</li>
      <li><b>Volte os sticks ao centro e o throttle todo para baixo</b>Então clique em terminar.</li></ol><div class="row"><button data-act="finish">Terminar captura</button><button class="secondary" data-act="reset">Cancelar</button></div>`;
  else if (cal.state === "captured") html = `<table><thead><tr><th>Canal</th><th class="num">Mín</th><th class="num">Trim</th><th class="num">Máx</th><th class="num">Curso</th></tr></thead><tbody>
      ${cal.mins.map((mn, i) => { const sp = cal.maxs[i] - mn; return `<tr><td>CH${i + 1} ${rcName(i)}</td><td class="num">${mn}</td><td class="num">${i === 2 ? mn : cal.trims[i]}</td><td class="num">${cal.maxs[i]}</td><td class="num">${sp > 0 ? sp : "—"}${sp < 200 ? " (não alterado)" : ""}</td></tr>`; }).join("")}
      </tbody></table><div class="row" style="margin-top:12px"><button data-act="save">Gravar na placa</button><button class="secondary" data-act="start">Refazer</button><button class="secondary" data-act="reset">Cancelar</button></div>`;
  else html = `<div class="row"><span class="muted">${radioCalibrated() ? "Já existe uma calibração gravada. Refaça se trocou de rádio ou de receptor." : "O rádio não está calibrado."}</span><span class="spacer"></span><button data-act="start" ${receiverPresent() ? "" : "disabled"}>Iniciar calibração</button></div>`;
  if (el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; }
};
function fltSlot(pwm) {
  const r = S.meta?.fltmode_pwm || [];
  const i = r.findIndex(([a, b]) => pwm >= a && pwm <= b);
  return i < 0 ? 1 : i + 1;
}

// ---------------------------------------------------------------- modos
const MODE_HELP = {
  STABILIZE: "Manual, nivela sozinho. Primeiro voo.", ACRO: "Taxa pura, sem nivelamento. Avançado.", ALT_HOLD: "Segura altura pelo barômetro.",
  LAND: "Pousa no lugar.", LOITER: "Segura posição e altura (GPS).", POSHOLD: "Como LOITER, com resposta mais direta (GPS).",
  RTL: "Volta para casa e pousa (GPS).", AUTO: "Executa a missão (GPS).", DRIFT: "Curvas coordenadas (GPS).", SPORT: "Taxa com limite de ângulo.",
  CIRCLE: "Circula um ponto (GPS).", GUIDED: "Comandado pela estação (GPS).", AUTOTUNE: "Ajuste automático de PID.", FLIP: "Flip automático.", OF_LOITER: "Optical flow.",
  BRAKE: "Para no ar (GPS).", SMART_RTL: "Volta pelo caminho percorrido (GPS).", GUIDED_NOGPS: "Comandado sem GPS.", FLOWHOLD: "Segura posição por optical flow.",
  THROW: "Lança com a mão (GPS).", FOLLOW: "Segue outro veículo (GPS).", ZIGZAG: "Pulverização em zigue-zague (GPS).", AUTO_RTL: "Missão até o pouso (GPS).", TURTLE: "Desvira o drone.",
};
// chave de 3 posições cai nas posições 1, 4 e 6: STABILIZE / ALT_HOLD / LOITER (ou LAND sem GPS)
const MODE_PRESETS = { no_gps: [0, 0, 2, 2, 9, 9], gps: [0, 0, 2, 2, 5, 5] };
// RCn_OPTION mais usados (ArduCopter 4.x)
const AUX_OPTIONS = { 0: "nada", 4: "RTL (volta para casa)", 17: "AutoTune (ajuste de PID)", 18: "LAND", 42: "SmartRTL",
  3: "Modo simples", 9: "Disparo de câmera", 31: "Parada de emergência dos motores", 33: "Brake" };
const AUX_HELP = {
  17: "Em voo, em ALT_HOLD, LOITER ou POSHOLD: chave para cima inicia o ajuste automático de PID. Chave para baixo volta aos PIDs antigos. Para gravar os novos, pouse e desarme com a chave ainda para cima.",
  4: "Chave para cima: volta para o ponto de decolagem e pousa. Precisa de GPS.", 31: "Chave para cima: corta os motores na hora (mesmo em voo).",
};
RENDER.modes = () => `
  <h2>Modos de voo</h2>
  <p class="lead">O canal ${modeCh()} escolhe entre 6 modos. Uma chave de 3 posições usa só as posições 1, 4 e 6. Mude a chave do transmissor e veja qual posição acende. Sem GPS, use só modos que não dependem de posição.</p>
  <div class="card accent"><div class="modes" id="fm-slots"></div>
    <div class="row" style="margin-top:14px"><button id="fm-save">Gravar modos</button><button class="secondary" id="fm-preset">Usar sugestão (${cfgMode() === "gps" ? "com GPS" : "sem GPS"})</button><span class="spacer"></span><span class="muted" id="fm-now"></span></div></div>
  ${is4() ? `<div class="card"><h3>Chaves do rádio</h3><p class="muted">Escolha qual canal troca os modos e o que as outras chaves fazem.</p>
    <div class="row"><label class="field">Canal dos modos (FLTMODE_CH)<select id="fm-ch">${[5, 6, 7, 8].map((c) => `<option value="${c}">CH${c}</option>`).join("")}</select></label></div>
    <div id="fm-aux"></div><div class="row" style="margin-top:12px"><button id="fm-aux-save">Gravar chaves</button></div></div>` : ""}`;
BIND.modes = (el) => {
  const opts = Object.entries(S.meta?.modes || {}).map(([k, v]) => `<option value="${k}">${k}: ${v}</option>`).join("");
  $("#fm-slots", el).innerHTML = [1, 2, 3, 4, 5, 6].map((i) => { const [a, b] = S.meta?.fltmode_pwm?.[i - 1] || [0, 0]; return `<div class="slot" data-slot="${i}"><b>Posição ${i}</b><span class="muted mono">${a}–${b}</span><div class="row"><select data-fm="${i}">${opts}</select><span class="muted" data-fmh="${i}"></span></div></div>`; }).join("");
  const sync = () => $$("[data-fm]", el).forEach((s) => { s.value = String(P(`FLTMODE${s.dataset.fm}`) ?? 0); help(s); });
  const help = (s) => {
    const name = S.meta.modes[s.value];
    const gpsBad = cfgMode() !== "gps" && S.meta.gps_modes.includes(+s.value);
    $(`[data-fmh="${s.dataset.fm}"]`, el).innerHTML = gpsBad ? `<span class="badge bad">precisa de GPS</span>` : esc(MODE_HELP[name] || "");
  };
  sync();
  $$("[data-fm]", el).forEach((s) => (s.onchange = () => help(s)));
  $("#fm-preset", el).onclick = () => { const p = MODE_PRESETS[cfgMode() === "gps" ? "gps" : "no_gps"]; $$("[data-fm]", el).forEach((s, i) => { s.value = String(p[i]); help(s); }); toast("Sugestão carregada. Clique em Gravar."); };
  $("#fm-save", el).onclick = (e) => run(e.target, () => setParams(Object.fromEntries($$("[data-fm]", el).map((s) => [`FLTMODE${s.dataset.fm}`, +s.value])), "Modos de voo"));
  if (!is4()) return;
  const ch = $("#fm-ch", el);
  const auxOpts = Object.entries(AUX_OPTIONS).map(([k, v]) => `<option value="${k}">${k}: ${v}</option>`).join("");
  const drawAux = () => {
    const keep = Object.fromEntries($$("[data-aux]", el).map((s) => [s.dataset.aux, s.value]));
    $("#fm-aux", el).innerHTML = [5, 6, 7, 8].filter((c) => c !== +ch.value).map((c) => {
      const cur = String(keep[c] ?? P(`RC${c}_OPTION`) ?? 0);
      const extra = AUX_OPTIONS[cur] === undefined ? `<option value="${cur}">${cur}: outra</option>` : "";
      return `<div class="row"><label class="field">CH${c} (RC${c}_OPTION)<select data-aux="${c}">${auxOpts}${extra}</select></label><span class="muted" data-auxh="${c}"></span></div>`;
    }).join("");
    $$("[data-aux]", el).forEach((s) => {
      s.value = keep[s.dataset.aux] ?? String(P(`RC${s.dataset.aux}_OPTION`) ?? 0);
      const h = () => ($(`[data-auxh="${s.dataset.aux}"]`, el).textContent = AUX_HELP[s.value] || "");
      s.onchange = h; h();
    });
  };
  ch.value = String(modeCh());
  ch.onchange = drawAux;
  drawAux();
  $("#fm-aux-save", el).onclick = (e) => run(e.target, async () => {
    // o canal dos modos não pode ter outra função ao mesmo tempo
    const obj = { FLTMODE_CH: +ch.value, [`RC${ch.value}_OPTION`]: 0 };
    $$("[data-aux]", el).forEach((s) => (obj[`RC${s.dataset.aux}_OPTION`] = +s.value));
    await setParams(obj, "Chaves do rádio");
    go("modes");
  });
};
UPDATE.modes = () => {
  const c = modeCh();
  const pwm = S.snap.rc?.[c - 1];
  const slot = pwm ? fltSlot(pwm) : 0;
  $$(".slot").forEach((d) => d.classList.toggle("active", +d.dataset.slot === slot));
  $("#fm-now").textContent = S.snap.connected ? `CH${c} = ${pwm ?? "—"} µs · modo atual: ${S.snap.mode ?? "—"}` : "";
};

// ---------------------------------------------------------------- failsafe
RENDER.failsafe = () => `
  <h2>Failsafe do rádio</h2>
  <p class="lead">Se o transmissor desligar ou sair de alcance, a placa precisa perceber e pousar (ou voltar). ${B("A APM 2.x percebe pela queda do throttle abaixo de <span class=\"mono\">FS_THR_VALUE</span>.", "A Pixhawk percebe quando o receptor para de mandar sinal ou quando o throttle cai abaixo de <span class=\"mono\">FS_THR_VALUE</span>.")}</p>
  <div class="grid two">
    <div class="card accent"><h3>Configuração</h3>
      <div class="row"><label class="field">FS_THR_ENABLE<select id="fs-en">${Object.entries(S.meta?.fs_thr_options || {}).map(([k, v]) => `<option value="${k}">${k}: ${v}</option>`).join("")}</select></label>
      <label class="field">FS_THR_VALUE (µs)<input type="number" id="fs-val" min="910" max="1100" step="1"></label></div>
      <p class="muted" id="fs-sug"></p>
      <div class="row"><button id="fs-save">Gravar</button><button class="secondary" id="fs-rec">Usar recomendado</button></div></div>
    <div class="card"><h3>Ao vivo</h3><dl class="kv"><dt>Throttle (CH3)</dt><dd id="fs-ch3">—</dd><dt>RC3_MIN</dt><dd id="fs-min">—</dd><dt>Modo</dt><dd id="fs-mode">—</dd></dl><div id="fs-state"></div></div>
  </div>
  <div class="card"><h3>Configurar o receptor e testar (sem hélices!)</h3><ol class="steps-list">
    <li><b>Failsafe do receptor</b>No transmissor/receptor, configure o failsafe para "sem pulsos" ou para levar o throttle ABAIXO de FS_THR_VALUE. Receptores PWM que "seguram a última posição" anulam o failsafe.</li>
    <li><b>Teste</b>Hélices fora, bateria conectada, arme no transmissor (throttle embaixo + yaw à direita por 2 s), suba um pouco o throttle.</li>
    <li><b>Desligue o transmissor</b>O throttle aqui deve cair abaixo do limite e o modo mudar para LAND (ou RTL). Os motores devem desacelerar.</li>
    <li><b>Ligue de novo</b>Troque o modo na chave para retomar o controle. Desarme.</li></ol>
    <label class="row"><input type="checkbox" id="fs-tested"> Testei e o failsafe funcionou</label></div>`;
BIND.failsafe = (el) => {
  $("#fs-en", el).value = String(P("FS_THR_ENABLE") ?? 0);
  $("#fs-val", el).value = P("FS_THR_VALUE") ?? 975;
  $("#fs-tested", el).checked = !!S.manual.failsafe_tested;
  $("#fs-tested", el).onchange = (e) => setManual("failsafe_tested", e.target.checked);
  const rec = () => { $("#fs-en").value = cfgMode() === "gps" ? "1" : "3"; $("#fs-val").value = Math.min(975, (P("RC3_MIN") || 1000) - 40); };
  $("#fs-sug", el).textContent = `Recomendado ${cfgMode() === "gps" ? "com GPS: 1 (RTL; sem fix vira LAND)" : "sem GPS: 3 (LAND)"} e FS_THR_VALUE pelo menos 10 µs abaixo do RC3_MIN (${P("RC3_MIN") ?? "?"}).`;
  $("#fs-rec", el).onclick = rec;
  $("#fs-save", el).onclick = (e) => run(e.target, () => {
    const v = +$("#fs-val").value;
    if (v >= (P("RC3_MIN") || 0) - 10) throw new Error("FS_THR_VALUE precisa ficar pelo menos 10 µs abaixo do RC3_MIN.");
    return setParams({ FS_THR_ENABLE: +$("#fs-en").value, FS_THR_VALUE: v }, "Failsafe");
  });
};
UPDATE.failsafe = () => {
  const s = S.snap, ch3 = s.rc?.[2];
  $("#fs-ch3").textContent = ch3 ? `${ch3} µs` : "—";
  $("#fs-min").textContent = P("RC3_MIN") ?? "—";
  $("#fs-mode").textContent = s.mode ?? "—";
  const st = checks().failsafe;
  $("#fs-state").innerHTML = statusRow("Failsafe", st) + (ch3 && ch3 < (P("FS_THR_VALUE") || 975) ? `<div class="note warn"><b>Throttle abaixo do limite agora.</b>É o que acontece sem sinal: se estivesse armado e com FS_THR_ENABLE ligado, o failsafe dispararia.</div>` : "");
};

// ---------------------------------------------------------------- bateria
RENDER.battery = () => `
  <h2>Bateria</h2>
  <p class="lead">Com o Power Module ${B("da APM 2.5/2.6", "na porta POWER da Pixhawk")} a placa mede tensão e corrente da bateria, avisa e pousa quando ela fica fraca.</p>
  ${S.meta?.battery?.reboot_note ? `<div class="note warn">${esc(S.meta.battery.reboot_note)}</div>` : ""}
  <div class="grid two">
    <div class="card accent"><h3>Monitor</h3>
      <div class="row"><label class="field">BATT_MONITOR<select id="b-mon"><option value="0">0: desligado</option><option value="3">3: só tensão</option><option value="4">4: tensão e corrente</option></select></label>
      <label class="field">Capacidade (mAh)<input type="number" id="b-cap" step="100" min="0"></label></div>
      <div class="row" style="margin-top:10px"><button id="b-pm">Usar Power Module ${B("APM 2.5/2.6", "da Pixhawk")}</button><button class="secondary" id="b-save">Gravar</button></div>
      <p class="muted mono" style="font-size:12px" id="b-pins"></p></div>
    <div class="card"><h3>Ao vivo</h3><div class="big" id="b-volt">—</div><dl class="kv" id="b-kv"></dl>
      <h3 style="margin-top:14px">Calibrar tensão</h3><div class="row"><label class="field">Medido no multímetro (V)<input type="number" id="b-meas" step="0.01"></label><button class="secondary" id="b-cal">Corrigir multiplicador</button></div></div>
  </div>
  <div class="card"><h3>Failsafe de bateria</h3><div class="row">
    <label class="field">${S.meta?.battery?.fs_enable}<select id="b-fs">${Object.entries(S.meta?.battery?.fs_options || {}).map(([k, v]) => `<option value="${k}">${k}: ${v}</option>`).join("")}</select></label>
    <label class="field">Células (S)<input type="number" id="b-cells" min="2" max="6" value="3"></label>
    <label class="field">${S.meta?.battery?.fs_volt} (V)<input type="number" id="b-fsv" step="0.1"></label>
    <label class="field">${S.meta?.battery?.fs_mah}<input type="number" id="b-fsm" step="50" min="0"></label>
    <button id="b-fs-save">Gravar</button></div>
    <p class="muted">Sugestão: 3,5 V por célula (3S = 10,5 V) e o limite de mAh em ~20 % da capacidade.</p></div>`;
BIND.battery = (el) => {
  const bt = S.meta?.battery || {};
  const sync = () => {
    $("#b-mon").value = String(P("BATT_MONITOR") ?? 0); $("#b-cap").value = P("BATT_CAPACITY") ?? 0;
    $("#b-fs").value = String(P(bt.fs_enable) ?? 0); $("#b-fsv").value = P(bt.fs_volt) ?? 10.5; $("#b-fsm").value = P(bt.fs_mah) ?? 0;
    $("#b-pins").textContent = `VOLT_PIN=${P("BATT_VOLT_PIN") ?? "—"} CURR_PIN=${P("BATT_CURR_PIN") ?? "—"} VOLT_MULT=${fx(P("BATT_VOLT_MULT"), 3)} ${bt.amp}=${fx(P(bt.amp), 2)}`;
  };
  sync();
  $("#b-pm", el).onclick = (e) => run(e.target, async () => {
    // na Pixhawk os BATT_* de pino só existem depois de BATT_MONITOR + reboot
    const pm = Object.fromEntries(Object.entries(S.meta.power_module).filter(([k]) => P(k) !== undefined));
    await setParams({ BATT_MONITOR: 4, ...pm }, "Power Module");
    if (Object.keys(pm).length < Object.keys(S.meta.power_module).length) toast("BATT_MONITOR=4 gravado. Reinicie a placa (Diagnóstico) e clique de novo para gravar os pinos.", "bad");
    sync();
  });
  $("#b-save", el).onclick = (e) => run(e.target, async () => { await setParams({ BATT_MONITOR: +$("#b-mon").value, BATT_CAPACITY: +$("#b-cap").value }, "Monitor"); sync(); });
  $("#b-cells", el).oninput = () => { $("#b-fsv").value = (3.5 * +$("#b-cells").value).toFixed(1); };
  $("#b-fs-save", el).onclick = (e) => run(e.target, () => setParams({ [bt.fs_enable]: +$("#b-fs").value, [bt.fs_volt]: +$("#b-fsv").value, [bt.fs_mah]: +$("#b-fsm").value }, "Failsafe de bateria"));
  $("#b-cal", el).onclick = (e) => run(e.target, async () => {
    const meas = +$("#b-meas").value, rep = S.snap.battery?.voltage;
    if (!(meas > 1) || !(rep > 1)) throw new Error("Conecte a bateria e digite a tensão medida.");
    await setParams({ BATT_VOLT_MULT: +(P("BATT_VOLT_MULT") * (meas / rep)).toFixed(4) }, "BATT_VOLT_MULT");
    sync();
  });
};
UPDATE.battery = () => {
  const b = S.snap.battery || {};
  $("#b-volt").textContent = P("BATT_MONITOR") >= 3 ? `${fx(b.voltage, 2)} V` : "monitor desligado";
  $("#b-kv").innerHTML = `<dt>Corrente</dt><dd>${b.current != null ? fx(b.current, 1) + " A" : "—"}</dd><dt>Restante</dt><dd>${b.remaining >= 0 ? b.remaining + " %" : "—"}</dd>`;
};

// ---------------------------------------------------------------- ESCs
RENDER.esc = () => is4() ? ESC4() : `
  <h2>Calibração dos ESCs</h2>
  <p class="lead">Ensina aos ESCs a faixa de throttle do rádio para que os 4 motores comecem a girar juntos. Método "todos de uma vez" do ArduCopter 3.2. É feito pelo transmissor, com a placa alimentada só pela bateria.</p>
  <div class="note bad"><b>Tire as hélices.</b>Durante a calibração o throttle do rádio vai direto para os motores, sem nenhuma proteção.</div>
  <div class="card accent"><ol class="steps-list">
    <li><b>Desconecte o USB</b>A placa precisa reiniciar junto com a bateria. Esta página perde a conexão: é esperado.</li>
    <li><b>Transmissor ligado, throttle no MÁXIMO</b></li>
    <li><b>Conecte a bateria</b>Os LEDs vermelho, azul e amarelo piscam em sequência: a APM vai entrar em calibração no próximo boot.</li>
    <li><b>Desconecte a bateria</b>Mantenha o throttle no máximo.</li>
    <li><b>Conecte a bateria de novo</b>Os ESCs bipam (número de células e depois o bip de máximo).</li>
    <li><b>Throttle todo para baixo</b>Bip longo de confirmação: faixa gravada.</li>
    <li><b>Teste</b>Suba o throttle devagar: os motores devem começar a girar juntos e na mesma velocidade.</li>
    <li><b>Desconecte a bateria</b>Isso sai do modo de calibração. Na próxima vez a placa inicia normal.</li></ol>
    <label class="row"><input type="checkbox" id="esc-done"> Os ESCs foram calibrados e os motores começam a girar juntos</label></div>
  <div class="card"><h3>Parâmetros de motor</h3><dl class="kv" id="esc-kv"></dl>
    <p class="muted">MOT_SPIN_ARMED: rotação com o drone armado parado (0 = parado). THR_MIN: throttle mínimo em voo.</p></div>`;
const ESC4 = () => `
  <h2>Calibração dos ESCs</h2>
  <p class="lead">Ensina aos ESCs a faixa de PWM da Pixhawk (MOT_PWM_MIN–MOT_PWM_MAX) para que os 4 motores comecem a girar juntos. No 4.x a calibração é automática: a placa manda o máximo e depois o mínimo sozinha.</p>
  <div class="note bad"><b>Tire as hélices.</b>Durante a calibração os motores recebem throttle máximo.</div>
  <div id="esc-dshot"></div>
  <div class="card accent"><ol class="steps-list">
    <li><b>Prepare a calibração</b>Grava <span class="mono">ESC_CALIBRATION=3</span> (calibrar no próximo boot). <button id="esc-arm">Preparar calibração</button> <span class="muted" id="esc-state"></span></li>
    <li><b>Desconecte o USB e a bateria</b>A placa precisa iniciar só pela bateria. Esta página perde a conexão: é esperado.</li>
    <li><b>Conecte a bateria</b>A Pixhawk inicia e os LEDs piscam em sequência.</li>
    <li><b>Segure a chave de segurança</b>Até o LED dela ficar vermelho fixo. Os ESCs recebem o máximo, bipam, depois recebem o mínimo e dão o bip longo (número de células + confirmação).</li>
    <li><b>Desconecte a bateria</b>A placa volta <span class="mono">ESC_CALIBRATION</span> para 0 sozinha: no próximo boot inicia normal.</li>
    <li><b>Confira</b>Na página Motores, todos devem começar a girar juntos com o mesmo throttle.</li></ol>
    <label class="row"><input type="checkbox" id="esc-done"> Os ESCs foram calibrados e os motores começam a girar juntos</label></div>
  <div class="card"><h3>Parâmetros de motor</h3><dl class="kv" id="esc-kv"></dl>
    <p class="muted">MOT_PWM_TYPE: 0 = PWM normal (precisa calibrar), 4–7 = DShot (digital, não precisa). MOT_SPIN_ARM: giro com o drone armado parado; MOT_SPIN_MIN: mínimo em voo.</p></div>`;
BIND.esc = (el) => {
  $("#esc-done", el).checked = !!S.manual.esc_done;
  $("#esc-done", el).onchange = (e) => setManual("esc_done", e.target.checked);
  $("#esc-arm", el)?.addEventListener("click", (e) => run(e.target, () => setParams({ ESC_CALIBRATION: 3 }, "ESC_CALIBRATION=3")));
};
UPDATE.esc = () => {
  $("#esc-kv").innerHTML = (S.meta?.esc_params || []).map((k) => `<dt>${k}</dt><dd>${P(k) ?? "—"}</dd>`).join("");
  if (!is4()) return;
  $("#esc-state").textContent = P("ESC_CALIBRATION") === 3 ? "Pronto: siga os próximos passos." : "";
  $("#esc-dshot").innerHTML = P("MOT_PWM_TYPE") >= 4 ? `<div class="note ok"><b>DShot ligado (MOT_PWM_TYPE=${P("MOT_PWM_TYPE")}).</b>Sinal digital: os ESCs não precisam de calibração.</div>` : "";
};

// ---------------------------------------------------------------- motores
function motorSvg(frame, active) {
  const lay = S.meta?.motor_layout?.[frame];
  if (!lay) return `<p class="muted">Diagrama só para frames X e +.</p>`;
  const posXY = { "frente-direita": [70, -70], "trás-direita": [70, 70], "trás-esquerda": [-70, 70], "frente-esquerda": [-70, -70], frente: [0, -95], direita: [95, 0], "trás": [0, 95], esquerda: [-95, 0] };
  const arms = lay.map(([, p]) => `<line x1="0" y1="0" x2="${posXY[p][0]}" y2="${posXY[p][1]}" stroke="var(--muted)" stroke-width="8" stroke-linecap="round"/>`).join("");
  const motors = lay.map(([l, p, dir, out]) => {
    const [x, y] = posXY[p];
    const cw = dir === "horário";
    const on = active === l;
    return `<g transform="translate(${x},${y})"><circle r="30" fill="${on ? "var(--zune)" : "var(--card)"}" stroke="${cw ? "#c53030" : "#2b6cb0"}" stroke-width="4"/>
      <path d="${cw ? "M -20 -8 A 21 21 0 0 1 8 -20" : "M 20 -8 A 21 21 0 0 0 -8 -20"}" stroke="${cw ? "#c53030" : "#2b6cb0"}" stroke-width="3" fill="none" marker-end="url(#ah${cw ? "r" : "b"})"/>
      <text y="7" text-anchor="middle" font-size="20" font-weight="700" fill="${on ? "#fff" : "var(--ink)"}">${l}</text><text y="48" text-anchor="middle" font-size="11" fill="var(--muted)">${OUT()} ${out}</text></g>`;
  }).join("");
  return `<svg viewBox="-140 -150 280 300" style="width:100%;max-width:320px;height:auto"><defs>
    <marker id="ahr" markerWidth="6" markerHeight="6" refX="3" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 z" fill="#c53030"/></marker>
    <marker id="ahb" markerWidth="6" markerHeight="6" refX="3" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 z" fill="#2b6cb0"/></marker></defs>
    <path d="M0 -140 l10 18 h-20 z" fill="var(--ok)"/><text x="16" y="-124" font-size="12" fill="var(--muted)">frente</text>${arms}<rect x="-22" y="-22" width="44" height="44" rx="8" fill="var(--zune)"/>${motors}</svg>`;
}
RENDER.motors = () => `
  <h2>Teste de motores</h2>
  <p class="lead">Gira cada motor por alguns segundos, na ordem de teste do ArduCopter (A, B, C, D em sentido horário a partir da frente). Confira se gira o motor da posição certa e no sentido certo. Precisa da bateria conectada.</p>
  <div class="note bad"><b>Tire as hélices antes.</b>Prenda o drone. O teste não exige armar.</div>
  <div id="mt-safety"></div>
  <div class="grid two">
    <div class="card" id="mt-diagram"></div>
    <div class="card accent"><h3>Teste</h3>
      <label class="row"><input type="checkbox" id="mt-props"> As hélices foram removidas</label>
      <div class="row" style="margin:12px 0"><label class="field">Throttle: <b id="mt-thr-v">7 %</b><input type="range" id="mt-thr" min="3" max="20" value="7"></label>
      <label class="field">Duração (s)<input type="number" id="mt-dur" min="1" max="5" value="2"></label></div>
      <div id="mt-list"></div>
      <p class="muted" style="font-size:13px">Sentido errado: troque <b>dois</b> dos três fios entre o ESC e o motor. Motor errado: troque os cabos de sinal nas saídas ${B("da APM", "MAIN OUT da Pixhawk")}.</p></div>
  </div>`;
BIND.motors = (el) => {
  const frame = P(S.meta?.frame_param) ?? 1;
  const lay = S.meta?.motor_layout?.[frame] || [];
  const paint = (active) => ($("#mt-diagram", el).innerHTML = `<h3>Frame ${esc(S.meta?.frames?.[frame] || frame)} · visto de cima</h3>${motorSvg(frame, active)}<p class="muted" style="font-size:13px">Vermelho = horário (CW), azul = anti-horário (CCW).</p>`);
  paint(null);
  $("#mt-thr", el).oninput = (e) => ($("#mt-thr-v").textContent = `${e.target.value} %`);
  $("#mt-list", el).innerHTML = lay.map(([l, p, dir, out], i) => `<div class="check"><button data-motor="${i + 1}" data-l="${l}">Girar ${l}</button>
    <div class="txt"><b>${l}: ${p}, ${dir}</b><span>${OUT()} ${out} ${B("da APM", "da Pixhawk")}</span></div><label class="row"><input type="checkbox" data-ok="${l}"> ok</label></div>`).join("");
  $$("[data-ok]", el).forEach((c) => { c.checked = !!S.manual[`motor_${c.dataset.ok}`]; c.onchange = () => setManual(`motor_${c.dataset.ok}`, c.checked); });
  $$("[data-motor]", el).forEach((b) => (b.onclick = () => run(b, async () => {
    const dur = +$("#mt-dur").value;
    paint(b.dataset.l);
    const r = await api("POST", "/api/motor_test", { motor: +b.dataset.motor, throttle: +$("#mt-thr").value, duration: dur, props_removed: $("#mt-props").checked });
    if (!r.ok) toast(`A placa recusou (resultado ${r.result ?? "sem resposta"}). ${r.messages.join(" ")}`, "bad");
    setTimeout(() => paint(null), dur * 1000);
  })));
};
UPDATE.motors = () => {
  const el = $("#mt-safety");
  if (!el) return;
  const html = !is4() ? "" : S.snap.safety_off ? `<div class="note ok"><b>Chave de segurança liberada.</b>A placa pode mandar sinal aos motores.</div>`
    : `<div class="note warn"><b>Chave de segurança ativa.</b>Segure o botão da Pixhawk (ou do módulo GPS) até o LED ficar vermelho fixo. Sem isso a placa recusa o teste.</div>`;
  if (el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; }
};

// ---------------------------------------------------------------- checklist
RENDER.preflight = () => `
  <h2>Checklist pré-voo</h2>
  <p class="lead">Junta tudo que a interface consegue verificar com os itens que só você pode conferir. Só voe com tudo verde.</p>
  <div class="card accent" id="pf-overall"></div>
  <div class="grid two"><div class="card"><h3>Verificado pela placa</h3><div id="pf-auto"></div></div>
  <div class="card"><h3>Conferido por você</h3><div id="pf-manual"></div></div></div>
  <div class="card"><h3>Armar e primeiro voo</h3><ol class="steps-list">
    <li><b>Armar</b>${B("", "Libere a chave de segurança (vermelho fixo). ")}Modo STABILIZE, throttle todo para baixo e yaw à direita por 2 s. ${B("LED vermelho fica aceso.", "O LED principal fica verde fixo (com GPS) ou azul fixo (sem GPS).")} Desarmar: throttle embaixo e yaw à esquerda.</li>
    <li><b>Se não armar</b>O LED pisca e a placa manda "PreArm: …" (aparece no Diagnóstico com o USB ligado).</li>
    <li><b>Primeiro voo</b>Em STABILIZE, a ~1 m de altura, em área aberta e sem pessoas por perto. Sem GPS: corrija a deriva de yaw no stick.</li></ol></div>`;
const MANUAL = [
  ["props", "Hélices certas (CW/CCW) em cada motor e bem apertadas"],
  ["frame", "Braços, parafusos e controladora firmes; seta da controladora para a frente"],
  ["vibration", "Controladora sobre amortecedor de vibração"],
  ["battery", "Bateria carregada e presa"],
  ["area", "Área aberta, sem pessoas e sem vento forte"],
];
UPDATE.preflight = () => {
  const c = checks();
  const items = [["Placa e sensores", "diag"], ["GPS ou sem GPS", "config"], ["Acelerômetro", "accel"], ["Bússola", "compass"], ["GPS", "gps"], ["Rádio", "radio"], ["Modos de voo", "modes"], ["Failsafe do rádio", "failsafe"], ["Bateria", "battery"], ["ESCs", "esc"], ["Motores", "motors"], ["Bloqueios de PreArm", "prearm"]];
  $("#pf-auto").innerHTML = items.map(([l, k]) => statusRow(`<a href="#${k === "prearm" ? "diag" : k}">${l}</a>`, c[k])).join("");
  const man = $("#pf-manual");
  if (!man.childElementCount) {
    man.innerHTML = MANUAL.map(([k, t]) => `<label class="check"><input type="checkbox" data-man="${k}"><div class="txt"><b>${t}</b></div></label>`).join("");
    $$("[data-man]", man).forEach((x) => { x.checked = !!S.manual[`pf_${x.dataset.man}`]; x.onchange = () => { setManual(`pf_${x.dataset.man}`, x.checked); update(); }; });
  }
  const o = overall(c);
  const manOk = MANUAL.every(([k]) => S.manual[`pf_${k}`]);
  const final = o.s === "ok" && manOk ? { s: "ok", msg: "Tudo verificado: pronto para o primeiro voo." } : o.s === "bad" ? o : { s: "warn", msg: manOk ? o.msg : "Falta conferir itens manuais." };
  $("#pf-overall").innerHTML = `<div class="row"><span class="badge ${final.s}" style="font-size:16px;padding:6px 14px">${{ ok: "PRONTO", warn: "QUASE", bad: "NÃO VOE" }[final.s]}</span><b style="font-size:18px">${esc(final.msg)}</b></div>`;
};

// ---------------------------------------------------------------- parâmetros
RENDER.params = () => `
  <h2>Parâmetros</h2>
  <p class="lead">Backup e restauração no formato do Mission Planner/APM Planner (<span class="mono">NOME,VALOR</span>). Faça um backup antes de mudanças grandes.</p>
  <div class="grid two">
    <div class="card accent"><h3>Backup</h3><p class="muted" id="pm-count"></p><div class="row"><a class="btn" href="/api/params/export">Baixar .param</a><button class="secondary" id="pm-refresh">Reler da placa</button></div></div>
    <div class="card"><h3>Restaurar</h3><input type="file" id="pm-file" accept=".param,.parm,.txt"><div id="pm-diff"></div></div>
  </div>
  <div class="card"><div class="row"><h3 style="margin:0">Todos</h3><span class="spacer"></span><input type="search" id="pm-q" placeholder="Filtrar (ex.: RC3, FS_, COMPASS)"></div><div id="pm-table" style="margin-top:10px;max-height:520px;overflow:auto"></div></div>`;
BIND.params = (el) => {
  $("#pm-refresh", el).onclick = (e) => run(e.target, async () => { await api("POST", "/api/params/refresh"); toast("Relendo parâmetros…"); });
  const table = () => {
    const q = $("#pm-q").value.trim().toUpperCase();
    const rows = Object.keys(S.params).sort().filter((k) => !q || k.includes(q)).slice(0, 400);
    $("#pm-table").innerHTML = `<table><thead><tr><th>Nome</th><th class="num">Valor</th><th></th></tr></thead><tbody>${rows.map((k) => `<tr><td class="mono">${k}</td><td class="num"><input data-pn="${k}" value="${+S.params[k].toPrecision(7)}" style="width:120px;text-align:right"></td><td><button class="secondary" data-ps="${k}">Gravar</button></td></tr>`).join("")}</tbody></table>`;
  };
  table();
  $("#pm-q", el).oninput = table;
  $("#pm-table", el).addEventListener("click", (e) => {
    const b = e.target.closest("[data-ps]");
    if (b) run(b, () => setParams({ [b.dataset.ps]: +$(`[data-pn="${b.dataset.ps}"]`).value }, b.dataset.ps));
  });
  $("#pm-file", el).onchange = async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    const text = await f.text();
    run(null, async () => {
      const d = await api("POST", "/api/params/diff", { text });
      $("#pm-diff").innerHTML = `<p class="muted">${d.total} parâmetros no arquivo · ${d.changes.length} diferentes · ${d.unknown.length} desconhecidos · ${d.skipped.length} ignorados (calculados pela placa)</p>
        ${d.changes.length ? `<table><thead><tr><th><input type="checkbox" id="pm-all" checked></th><th>Nome</th><th class="num">Atual</th><th class="num">Arquivo</th></tr></thead><tbody>
        ${d.changes.map((c) => `<tr><td><input type="checkbox" data-dc="${c.name}" data-v="${c.new}" checked></td><td class="mono">${c.name}</td><td class="num">${+c.current.toPrecision(7)}</td><td class="num">${+c.new.toPrecision(7)}</td></tr>`).join("")}</tbody></table>
        <button id="pm-apply" style="margin-top:10px">Gravar selecionados</button>` : `<div class="note ok">A placa já está igual ao arquivo.</div>`}`;
      $("#pm-all")?.addEventListener("change", (ev) => $$("[data-dc]").forEach((x) => (x.checked = ev.target.checked)));
      $("#pm-apply")?.addEventListener("click", (ev) => run(ev.target, () => setParams(Object.fromEntries($$("[data-dc]:checked").map((x) => [x.dataset.dc, +x.dataset.v])), "Restauração")));
    });
  };
};
UPDATE.params = () => {
  const p = S.snap.params || {};
  $("#pm-count").textContent = S.snap.connected ? `${p.received}/${p.total ?? "?"} parâmetros carregados${p.loading ? " (carregando…)" : ""}.` : "Não conectado.";
};

// ================================================================ ciclo
function update() {
  const s = S.snap;
  const pill = $("#pill-conn");
  pill.className = `pill ${s.connected ? "ok" : "bad"}`;
  $("span:last-child", pill).textContent = s.connected ? `${s.port?.replace("/dev/", "")} · ${s.baud}` : "Desconectado";
  $("#btn-connect").textContent = s.connected ? "Desconectar" : "Conectar";
  $("#port-select").hidden = !!s.connected;
  $("#pill-armed").hidden = !s.armed;
  $("#pill-armed").className = "pill armed";
  const pm = $("#pill-mode");
  pm.hidden = !s.connected || !s.mode;
  pm.textContent = s.mode || "";
  try { UPDATE[S.page]?.(); } catch (e) { console.error(e); }
}

async function loadParams() {
  try { const r = await api("GET", "/api/params"); S.params = r.params; } catch { /* ignore */ }
}

function connectWs() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onmessage = (ev) => {
    S.snap = JSON.parse(ev.data);
    if (S.snap.board && S.meta && S.snap.board !== S.meta.board && !S.metaLoading) reloadMeta();
    const rec = S.snap.params?.received ?? 0;
    if (rec !== S.paramsSeen && (rec === S.snap.params?.total || Date.now() - (S.lastParamLoad || 0) > 1500)) {
      S.paramsSeen = rec; S.lastParamLoad = Date.now();
      const had = Object.keys(S.params).length > 0;
      // a página foi desenhada sem parâmetros (textos dependem deles): redesenha quando chegam
      loadParams().then(() => { refreshNav(); if (!had && Object.keys(S.params).length) go(S.page); });
    }
    if (!S.snap.connected && Object.keys(S.params).length && !S.snap.port) S.params = {};
    update();
    if (Date.now() - (S.lastNav || 0) > 1000) { S.lastNav = Date.now(); refreshNav(); }
  };
  ws.onclose = () => { S.snap = { connected: false }; update(); setTimeout(connectWs, 1500); };
}

// constantes da placa conectada (modos, parâmetros de bateria, ESC…); redesenha a página atual
async function reloadMeta() {
  S.metaLoading = true;
  try { S.meta = await api("GET", "/api/meta"); $("#board-label").textContent = `${S.meta.label} · Zune Drones · UFRN`; go(S.page); }
  catch { /* tenta de novo no próximo snapshot */ }
  finally { S.metaLoading = false; }
}

async function loadPorts() {
  const sel = $("#port-select");
  try {
    const ports = await api("GET", "/api/ports");
    const cur = sel.value;
    const KIND = { apm: "APM 2.x (Mega 2560)", pixhawk: "Pixhawk" };
    sel.innerHTML = ports.length ? ports.map((p) => `<option value="${p.device}">${p.device.replace("/dev/", "")}${p.kind ? ` · ${KIND[p.kind]}${p.kind === "pixhawk" && p.description ? ` (${esc(p.description)})` : ""}` : p.description ? " · " + esc(p.description) : ""}</option>`).join("") : `<option value="">nenhuma porta</option>`;
    if (cur && ports.some((p) => p.device === cur)) sel.value = cur;
  } catch { sel.innerHTML = `<option value="">servidor offline</option>`; }
}

$("#btn-connect").onclick = (e) => run(e.target, async () => {
  if (S.snap.connected) { await api("POST", "/api/disconnect"); S.params = {}; S.paramsSeen = -1; toast("Desconectado. A porta está livre para outro programa."); return; }
  const port = $("#port-select").value;
  if (!port) throw new Error("Nenhuma porta serial encontrada. A placa está na USB?");
  e.target.textContent = "Conectando…";
  await api("POST", "/api/connect", { port, baud: 115200 });
  toast("Conectado. Carregando parâmetros…", "ok");
});

(async function init() {
  buildNav();
  try { S.meta = await api("GET", "/api/meta"); if (S.snap.connected || S.meta.board === "ac4") $("#board-label").textContent = `${S.meta.label} · Zune Drones · UFRN`; } catch { /* servidor ainda subindo */ }
  await loadPorts();
  setInterval(() => { if (!S.snap.connected) loadPorts(); }, 4000);
  go(S.page);
  connectWs();
})();
