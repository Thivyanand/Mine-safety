// ---------------------------------------------------------------------------
// twin.js — Worker Intelligence panel: UWB+IMU localization demo and A*
// safe-route demo, plus automatic sensor-category reveal during emergencies.
//
// This file is purely ADDITIVE on top of app.js. It shares app.js's
// top-level `let`/`function` bindings as plain identifiers (both files are
// classic, non-module <script> tags parsed into the same global scope), so
// it reuses existing state (latestWorkers, latestSensors, latestUwb,
// latestEmergency, selectedWorkerId, scene, camera, workerMarkers,
// uwbMarkers, THREE, drawSelectedRoute, clearSelectedRoute, selectWorker,
// renderDetailPanel, showToast, hexToRgbString) instead of duplicating
// polling or re-implementing the 3D scene.
//
// Everything shown by this module for worker position / UWB ranging / IMU
// values is SIMULATED PROTOTYPE DATA — no real hardware is attached.
// ---------------------------------------------------------------------------

// Mirrors dashboard_base.py EMERGENCY_SENSOR_MAP — which sensor categories
// are relevant to which zone-based emergency type. Kept as a UI-only
// constant; it does not change backend hazard logic, only which sensor
// categories the map auto-reveals.
const WI_EMERGENCY_SENSOR_MAP = {
  gas_leak: ['gas'],
  co_leak: ['gas'],
  fire: ['temperature'],
  low_oxygen: ['oxygen'],
  tunnel_collapse: ['vibration', 'structural'],
  rock_fall: ['vibration'],
};

let wiTimers = [];              // pending setTimeout ids for the current animation
let wiRunning = false;          // true while a simulate-* animation is in flight
let wiLastSelected = undefined; // last selectedWorkerId this module rendered
let wiLocalizedFor = null;      // worker_id currently "localized" (READY -> DONE)
let wiRoutedFor = null;         // worker_id whose route was last simulated
let highlightedUwbAnchors = new Set();
let activeRangingLines = [];    // { line, worker } three.js objects, cleaned up on reset
let imuGroup = null;
let selectionRing = null;
let selectionLabelEl = null;
let imuLabelEl = null;
let movingDot = null;
let movingDotPath = null;
let movingDotStart = 0;
let lastEmergencyKeySeen = null;
let lastEmergencyActiveSeen = false;
let wiLocalizationSnapshot = {}; // worker_id -> last UWB+IMU localization response

function wiClearTimers() {
  wiTimers.forEach((id) => clearTimeout(id));
  wiTimers = [];
}

function wiRunSteps(steps, onDone) {
  wiClearTimers();
  wiRunning = true;
  let t = 0;
  steps.forEach((step) => {
    t += step.delay;
    const id = setTimeout(() => {
      try {
        step.fn();
      } catch (e) {
        console.error('WorkerIntel step failed', e);
      }
    }, t);
    wiTimers.push(id);
  });
  const doneId = setTimeout(() => {
    wiRunning = false;
    if (onDone) onDone();
  }, t + 50);
  wiTimers.push(doneId);
}

function wiSetStatus(text, cls) {
  const el = document.getElementById('wi-status-line');
  if (!el) return;
  el.textContent = text || '';
  el.className = 'wi-status-line' + (cls ? ' ' + cls : '');
}

function wiLog(text, cls) {
  const log = document.getElementById('wi-log');
  if (!log) return;
  const row = document.createElement('div');
  row.className = 'wi-log-row' + (cls ? ' ' + cls : '');
  row.textContent = text;
  log.appendChild(row);
  log.scrollTop = log.scrollHeight;
}

function wiClearLog() {
  const log = document.getElementById('wi-log');
  if (log) log.innerHTML = '';
}

function dist3d(a, b) {
  return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2);
}

function nearestAnchors(worker, n) {
  const list = Object.values(latestUwb || {}).filter((a) => a.connectivity !== 'OFFLINE');
  if (!list.length) return [];
  return list
    .map((a) => ({ a, d: dist3d(worker, a) }))
    .sort((x, y) => x.d - y.d)
    .slice(0, n);
}

// ---------------------------------------------------------------------------
// Panel rendering
// ---------------------------------------------------------------------------

function renderWorkerIntelPanel() {
  const empty = document.getElementById('wi-empty');
  const content = document.getElementById('wi-content');
  const w = selectedWorkerId ? latestWorkers[selectedWorkerId] : null;

  if (!w) {
    if (empty) empty.classList.remove('hidden');
    if (content) content.classList.add('hidden');
    return;
  }
  if (empty) empty.classList.add('hidden');
  if (content) content.classList.remove('hidden');

  document.getElementById('wi-worker-id').textContent = `${w.name} (${w.worker_id})`;
  document.getElementById('wi-x').textContent = w.x.toFixed(1);
  document.getElementById('wi-y').textContent = w.y.toFixed(1);
  document.getElementById('wi-z').textContent = w.z.toFixed(1);
  document.getElementById('wi-zone').textContent = w.zone || '—';

  document.getElementById('wi-localization').textContent =
    wiLocalizedFor === w.worker_id ? 'UWB + IMU (DONE)' : 'READY';
  document.getElementById('wi-route').textContent =
    wiRoutedFor === w.worker_id ? 'A* (DONE)' : 'READY';

  const cached = wiLocalizationSnapshot[w.worker_id] || {};
  const imu = cached.imu || latestImu[w.worker_id] || null;
  const setText = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; };
  if (imu) {
    setText('wi-imu-id', imu.imu_id || `IMU-${w.worker_id}`);
    setText('wi-imu-motion', imu.motion_state || '—');
    setText('wi-imu-accel', `${Number(imu.ax_g || 0).toFixed(2)}, ${Number(imu.ay_g || 0).toFixed(2)}, ${Number(imu.az_g || 0).toFixed(2)} g`);
    setText('wi-imu-gyro', `${Number(imu.gyro_x_dps || 0).toFixed(1)}, ${Number(imu.gyro_y_dps || 0).toFixed(1)}, ${Number(imu.gyro_z_dps || 0).toFixed(1)} °/s`);
    setText('wi-imu-heading', `${Number(imu.yaw_deg || 0).toFixed(1)}°`);
    setText('wi-imu-link', imu.connectivity || 'ONLINE');
  } else {
    ['wi-imu-id','wi-imu-motion','wi-imu-accel','wi-imu-gyro','wi-imu-heading','wi-imu-link'].forEach((id) => setText(id, '—'));
  }

  const locBtn = document.getElementById('wi-simulate-location-btn');
  const routeBtn = document.getElementById('wi-simulate-route-btn');
  if (locBtn) locBtn.disabled = wiRunning;
  if (routeBtn) routeBtn.disabled = wiRunning;
}

// ---------------------------------------------------------------------------
// Worker Location simulation (UWB trilateration + IMU)
// ---------------------------------------------------------------------------

function clearRangingLines() {
  activeRangingLines.forEach((entry) => {
    if (entry.line && entry.line.parent) entry.line.parent.remove(entry.line);
    if (entry.line && entry.line.geometry) entry.line.geometry.dispose();
    if (entry.line && entry.line.material) entry.line.material.dispose();
    if (entry.pulse && entry.pulse.parent) entry.pulse.parent.remove(entry.pulse);
    if (entry.pulse && entry.pulse.geometry) entry.pulse.geometry.dispose();
    if (entry.pulse && entry.pulse.material) entry.pulse.material.dispose();
  });
  activeRangingLines = [];
}

function drawRangingLine(worker, anchor) {
  if (!scene) return;
  const start = new THREE.Vector3(worker.x, worker.y + 4, worker.z);
  const end = new THREE.Vector3(anchor.x, anchor.y + 4, anchor.z);
  const geo = new THREE.BufferGeometry().setFromPoints([start, end]);
  const mat = new THREE.LineBasicMaterial({
    color: 0x00e5ff,
    transparent: true,
    opacity: 0.75,
  });
  const line = new THREE.Line(geo, mat);
  scene.add(line);

  // Moving dot makes UWB ranging visibly worker-specific: the pulse travels
  // between this worker and the exact anchor selected for this run.
  const pulse = new THREE.Mesh(
    new THREE.SphereGeometry(1.8, 10, 10),
    new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.95 })
  );
  pulse.position.copy(start);
  scene.add(pulse);
  activeRangingLines.push({ line, pulse, start, end, worker, anchorId: anchor.anchor_id });
}

function buildLocalizationFallback(worker) {
  const anchors = nearestAnchors(worker, 3).map((e) => ({
    ...e.a,
    measured_distance_m: Number(e.d.toFixed(2)),
    true_distance_m: Number(e.d.toFixed(2)),
  }));
  return {
    worker_id: worker.worker_id,
    zone: worker.zone,
    anchors,
    imu: latestImu[worker.worker_id] || {
      imu_id: `IMU-${worker.worker_id}`,
      worker_id: worker.worker_id,
      motion_state: 'UNKNOWN',
      ax_g: 0, ay_g: 1, az_g: 0,
      gyro_x_dps: 0, gyro_y_dps: 0, gyro_z_dps: 0,
      yaw_deg: 0,
      connectivity: 'ONLINE',
      simulated: true,
    },
    estimated_position: { x: worker.x, y: worker.y, z: worker.z },
    method: 'UWB trilateration + worker-mounted IMU sensor fusion',
    accuracy_label: 'SIMULATED PROTOTYPE DATA',
  };
}

function runWorkerLocationAnimation(worker, data, opts) {
  opts = opts || {};
  const short = !!opts.short;
  const onDone = opts.onDone || null;
  wiLocalizationSnapshot[worker.worker_id] = data;
  clearRangingLines();
  wiClearLog();

  const anchors = (data.anchors || []).slice(0, 3).map((a) => ({
    a,
    d: Number(a.measured_distance_m ?? dist3d(worker, a)),
  }));
  highlightedUwbAnchors = new Set(anchors.map((e) => e.a.anchor_id));
  const imu = data.imu || latestImu[worker.worker_id] || {};
  const estimate = data.estimated_position || { x: worker.x, y: worker.y, z: worker.z };

  const steps = [];
  steps.push({ delay: 50, fn: () => wiSetStatus('LOCATING WORKER...') });
  if (!short) {
    steps.push({ delay: 420, fn: () => wiLog(`Selected ${worker.worker_id} — requesting nearest fixed UWB anchors...`) });
  }

  steps.push({
    delay: short ? 220 : 520,
    fn: () => {
      wiSetStatus('UWB RANGING...');
      wiLog(`Worker-specific anchors: ${anchors.map((e) => e.a.anchor_id).join(', ') || 'none found'}`);
      anchors.forEach((e) => drawRangingLine(worker, e.a));
      updateUwbMarkers(Object.values(latestUwb));
    },
  });

  steps.push({
    delay: short ? 320 : 700,
    fn: () => {
      anchors.forEach((e) => {
        const q = e.a.signal_quality != null ? `  Signal ${Number(e.a.signal_quality).toFixed(0)}%` : '';
        wiLog(`${e.a.anchor_id} ↔ ${worker.worker_id}: ${e.d.toFixed(2)} m${q}`);
      });
    },
  });

  steps.push({
    delay: short ? 250 : 620,
    fn: () => {
      wiSetStatus('IMU SENSOR FUSION...');
      const ax = Number(imu.ax_g || 0).toFixed(2);
      const ay = Number(imu.ay_g || 0).toFixed(2);
      const az = Number(imu.az_g || 0).toFixed(2);
      const gx = Number(imu.gyro_x_dps || 0).toFixed(1);
      const gy = Number(imu.gyro_y_dps || 0).toFixed(1);
      const gz = Number(imu.gyro_z_dps || 0).toFixed(1);
      wiLog(`${imu.imu_id || `IMU-${worker.worker_id}`} (worker-mounted) → Localization Engine`, 'wi-good');
      wiLog(`Motion: ${imu.motion_state || 'UNKNOWN'} | Accel: [${ax}, ${ay}, ${az}] g`);
      if (!short) wiLog(`Gyro: [${gx}, ${gy}, ${gz}] °/s | Heading: ${Number(imu.yaw_deg || 0).toFixed(1)}°`);
    },
  });

  if (!short) {
    steps.push({
      delay: 560,
      fn: () => {
        wiSetStatus('TRILATERATING POSITION...');
        wiLog('3 UWB ranges + IMU motion/orientation → worker XYZ estimate');
      },
    });
  }

  steps.push({
    delay: short ? 260 : 620,
    fn: () => {
      wiSetStatus('POSITION FOUND', 'wi-status-done');
      wiLog(
        `WORKER ${worker.worker_id} LOCATED  X:${Number(estimate.x).toFixed(1)} Y:${Number(estimate.y).toFixed(1)} Z:${Number(estimate.z).toFixed(1)}  Zone: ${data.zone || worker.zone || '—'}  (${data.accuracy_label || 'SIMULATED'})`,
        'wi-good'
      );
      wiLocalizedFor = worker.worker_id;
      pulseSelectionRing();
      renderWorkerIntelPanel();
    },
  });

  if (!short) {
    steps.push({
      delay: 1300,
      fn: () => clearRangingLines(),
    });
  }

  wiRunSteps(steps, onDone);
}

function simulateWorkerLocation(worker, opts) {
  opts = opts || {};
  wiSetStatus('REQUESTING LOCAL UWB ANCHORS...');
  fetch(`/api/localization/${worker.worker_id}`)
    .then((r) => {
      if (!r.ok) throw new Error('localization request failed');
      return r.json();
    })
    .then((data) => runWorkerLocationAnimation(worker, data, opts))
    .catch((err) => {
      console.warn('Localization API fallback:', err);
      runWorkerLocationAnimation(worker, buildLocalizationFallback(worker), opts);
    });
}

// ---------------------------------------------------------------------------
// Safe Route simulation (A*, hazard-aware)
// ---------------------------------------------------------------------------

function spawnScanPulse(worker) {
  if (!scene) return;
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(2, 4, 32),
    new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false })
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.set(worker.x, worker.y + 2, worker.z);
  scene.add(ring);
  const start = performance.now();
  const duration = 900;
  function tick() {
    const p = (performance.now() - start) / duration;
    if (p >= 1) {
      scene.remove(ring);
      ring.geometry.dispose();
      ring.material.dispose();
      return;
    }
    const s = 1 + p * 14;
    ring.scale.set(s, s, 1);
    ring.material.opacity = 0.6 * (1 - p);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);
}

function startMovingDot(coords) {
  stopMovingDot();
  if (!scene || !coords || coords.length < 2) return;
  movingDot = new THREE.Mesh(
    new THREE.SphereGeometry(3, 12, 12),
    new THREE.MeshBasicMaterial({ color: 0xffffff })
  );
  scene.add(movingDot);
  movingDotPath = coords.map((c) => new THREE.Vector3(c[0], c[1] + 3.5, c[2]));
  movingDotStart = performance.now();
}

function stopMovingDot() {
  if (movingDot && movingDot.parent) movingDot.parent.remove(movingDot);
  if (movingDot) {
    movingDot.geometry.dispose();
    movingDot.material.dispose();
  }
  movingDot = null;
  movingDotPath = null;
}

function simulateSafeRoute(worker) {
  wiClearLog();

  function afterLocalized() {
    const routeSteps = [];

    routeSteps.push({
      delay: 50,
      fn: () => {
        wiSetStatus('IDENTIFYING START NODE...');
        wiLog('START NODE IDENTIFIED near worker position.');
      },
    });

    routeSteps.push({
      delay: 500,
      fn: () => {
        wiSetStatus('SCANNING TUNNEL GRAPH...');
        wiLog('Exit 1 / Exit 2 / Exit 3 — scanning reachability...');
        spawnScanPulse(worker);
      },
    });

    routeSteps.push({
      delay: 600,
      fn: () => {
        wiSetStatus('INITIALIZING A*...');
        wiLog('A* PATHFINDING');
        wiLog('g(n) = travelled cost   h(n) = distance to exit   f(n) = g(n) + h(n)');
        spawnScanPulse(worker);
      },
    });

    routeSteps.push({
      delay: 700,
      fn: () => {
        fetch(`/api/routes/${worker.worker_id}`)
          .then((r) => r.json())
          .then((route) => {
            const emergencyActive = !!(latestEmergency && latestEmergency.active);
            (route.exit_options || []).forEach((opt) => {
              if (opt.reachable) {
                wiLog(`${opt.label}  Route: ${opt.distance_m} m`);
              } else {
                wiLog(`${opt.label}  Route: Blocked`, 'wi-hazard');
              }
            });

            const anyBlocked = (route.exit_options || []).some((o) => !o.reachable);
            if (emergencyActive && anyBlocked) {
              const originId = latestEmergency.origin_sensor_id;
              const originSensor = originId ? latestSensors[originId] : null;
              wiSetStatus('HAZARD NODE REJECTED', 'wi-status-warn');
              if (originSensor) {
                wiLog(`${originSensor.sensor_id} = CRITICAL`, 'wi-hazard');
                wiLog(`${originSensor.zone} = UNSAFE`, 'wi-hazard');
                wiLog(`A* avoids ${originSensor.zone}`, 'wi-hazard');
              } else {
                wiLog('Hazard zone detected — blocked segment rejected.', 'wi-hazard');
              }
              wiLog('ALTERNATIVE ROUTE SELECTED', 'wi-good');
            }

            if (!route.reachable) {
              wiSetStatus('NO SAFE ROUTE FOUND', 'wi-status-warn');
              wiLog('No reachable exit from this position.', 'wi-hazard');
              wiRunning = false;
              renderWorkerIntelPanel();
              return;
            }

            wiSetStatus('SAFE ROUTE FOUND', 'wi-status-done');
            wiLog(`SHORTEST SAFE ROUTE \u2192 ${route.destination_label} (${route.distance_m} m, ETA ${Math.round(route.eta_seconds)}s)`, 'wi-good');

            drawSelectedRoute(route);
            startMovingDot(route.route_coordinates);
            wiRoutedFor = worker.worker_id;
            wiRunning = false;
            renderWorkerIntelPanel();
          })
          .catch(() => {
            wiSetStatus('ROUTE REQUEST FAILED', 'wi-status-warn');
            wiRunning = false;
            renderWorkerIntelPanel();
          });
      },
    });

    wiRunSteps(routeSteps, null);
  }

  if (wiLocalizedFor !== worker.worker_id) {
    // Shorter localization pass first, per spec section 7.
    simulateWorkerLocation(worker, { short: true, onDone: afterLocalized });
  } else {
    afterLocalized();
  }
}

// ---------------------------------------------------------------------------
// Selection highlight (bright cyan/blue, distinct from sensor status colors)
// ---------------------------------------------------------------------------

function ensureSelectionObjects() {
  if (!scene || selectionRing) return;

  const group = new THREE.Group();
  const ringGeo = new THREE.TorusGeometry(16, 1.4, 8, 40);
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.85 });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = Math.PI / 2;
  group.add(ring);

  const dotGeo = new THREE.SphereGeometry(2.2, 10, 10);
  const dotMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
  const dot = new THREE.Mesh(dotGeo, dotMat);
  dot.position.y = 20;
  group.add(dot);

  group.visible = false;
  scene.add(group);
  selectionRing = { group, ring };

  selectionLabelEl = document.createElement('div');
  selectionLabelEl.className = 'selected-worker-label';
  selectionLabelEl.style.display = 'none';
  document.getElementById('viewer-canvas').parentElement.appendChild(selectionLabelEl);

  // IMU badge — a small marker beside the worker wearable, only visible
  // while that worker is selected. Represents ONE wearable IMU, not a
  // fixed mine sensor.
  imuGroup = new THREE.Group();
  const imuGeo = new THREE.OctahedronGeometry(2.6, 0);
  const imuMat = new THREE.MeshBasicMaterial({ color: 0xb98cff });
  const imuMesh = new THREE.Mesh(imuGeo, imuMat);
  imuGroup.add(imuMesh);
  imuGroup.visible = false;
  scene.add(imuGroup);

  imuLabelEl = document.createElement('div');
  imuLabelEl.className = 'imu-label';
  imuLabelEl.textContent = 'IMU';
  imuLabelEl.style.display = 'none';
  document.getElementById('viewer-canvas').parentElement.appendChild(imuLabelEl);
}

let selectionPulseBoost = 0; // temporary strong-pulse window after "position found"
function pulseSelectionRing() {
  selectionPulseBoost = performance.now() + 1600;
}

function project(vec, w, h) {
  const p = vec.clone().project(camera);
  return { x: (p.x * 0.5 + 0.5) * w, y: (-p.y * 0.5 + 0.5) * h, behind: p.z > 1 };
}

function twinAnimate() {
  requestAnimationFrame(twinAnimate);
  if (!scene || !camera) return;
  ensureSelectionObjects();
  if (!selectionRing) return;

  const t = performance.now() / 1000;
  const wrap = document.getElementById('viewer-canvas').parentElement;
  const w = wrap.clientWidth, h = wrap.clientHeight;

  const marker = selectedWorkerId ? workerMarkers[selectedWorkerId] : null;
  if (marker) {
    const pos = marker.group.position;
    selectionRing.group.position.set(pos.x, pos.y, pos.z);
    selectionRing.group.visible = true;

    const strong = performance.now() < selectionPulseBoost;
    const amp = strong ? 0.35 : 0.12;
    const speed = strong ? 8 : 2.2;
    const scale = 1 + amp * (0.5 + 0.5 * Math.sin(t * speed));
    selectionRing.group.scale.setScalar(scale);
    selectionRing.ring.material.opacity = strong ? 0.95 : 0.7;

    const name = (latestWorkers[selectedWorkerId] || {}).name || selectedWorkerId;
    selectionLabelEl.textContent = `\u25CF LIVE LOCATION — ${name} (${selectedWorkerId})`;
    const proj = project(new THREE.Vector3(pos.x, pos.y + 26, pos.z), w, h);
    selectionLabelEl.style.left = proj.x + 'px';
    selectionLabelEl.style.top = proj.y + 'px';
    selectionLabelEl.style.display = proj.behind ? 'none' : 'block';

    imuGroup.position.set(pos.x + 10, pos.y + 6, pos.z);
    imuGroup.visible = true;
    imuGroup.rotation.y = t * 1.5;
    const imuProj = project(new THREE.Vector3(pos.x + 10, pos.y + 12, pos.z), w, h);
    const liveImu = latestImu[selectedWorkerId] || (wiLocalizationSnapshot[selectedWorkerId] || {}).imu || {};
    imuLabelEl.textContent = `${liveImu.imu_id || `IMU-${selectedWorkerId}`} • ${liveImu.motion_state || 'LIVE'}`;
    imuLabelEl.style.left = imuProj.x + 'px';
    imuLabelEl.style.top = imuProj.y + 'px';
    imuLabelEl.style.display = imuProj.behind ? 'none' : 'block';
  } else {
    selectionRing.group.visible = false;
    imuGroup.visible = false;
    selectionLabelEl.style.display = 'none';
    imuLabelEl.style.display = 'none';
  }

  // Force-show only the UWB anchors relevant to the current localization
  // demo, regardless of the manual "UWB" category toggle in the Sensor
  // Network panel (which app.js's own poll loop otherwise controls).
  highlightedUwbAnchors.forEach((id) => {
    const m = uwbMarkers[id];
    if (m) m.group.visible = true;
  });

  // Gentle pulse on the ranging lines so they read as "live" signal, not a
  // static overlay.
  activeRangingLines.forEach((entry, i) => {
    entry.line.material.opacity = 0.45 + 0.35 * Math.abs(Math.sin(t * 3 + i));
    if (entry.pulse && entry.start && entry.end) {
      const frac = 0.5 + 0.5 * Math.sin(t * 2.6 + i * 1.7);
      entry.pulse.position.copy(entry.start.clone().lerp(entry.end, frac));
      entry.pulse.material.opacity = 0.65 + 0.35 * Math.abs(Math.sin(t * 5 + i));
    }
  });

  // Animate the direction dot along the simulated safe route, looping.
  if (movingDot && movingDotPath && movingDotPath.length >= 2) {
    const speed = 0.15; // path-fraction per second
    const elapsed = (performance.now() - movingDotStart) / 1000;
    const frac = (elapsed * speed) % 1;
    const segF = frac * (movingDotPath.length - 1);
    const i0 = Math.floor(segF);
    const i1 = Math.min(i0 + 1, movingDotPath.length - 1);
    const localT = segF - i0;
    const p = movingDotPath[i0].clone().lerp(movingDotPath[i1], localT);
    movingDot.position.copy(p);
  }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

function resetWorkerIntel() {
  wiClearTimers();
  wiRunning = false;
  wiLocalizedFor = null;
  wiRoutedFor = null;
  wiLocalizationSnapshot = {};
  highlightedUwbAnchors = new Set();
  clearRangingLines();
  stopMovingDot();
  clearSelectedRoute();
  wiClearLog();
  wiSetStatus('');

  selectedWorkerId = null;
  document.querySelectorAll('.roster-row').forEach((el) => el.classList.remove('selected'));
  renderDetailPanel();
  renderWorkerIntelPanel();

  visibleSensorTypes = new Set();
  uwbVisible = false;
  updateSensorMarkers(Object.values(latestSensors));
  updateUwbMarkers(Object.values(latestUwb));
}

// ---------------------------------------------------------------------------
// Emergency-driven automatic sensor reveal + Worker SOS handling
// ---------------------------------------------------------------------------

function applyEmergencySensorReveal(data) {
  const mapped = WI_EMERGENCY_SENSOR_MAP[data.type];
  if (data.type === 'manual_sos') {
    // "Do not show all environmental sensors. Show only UWB anchors
    // related to that worker + the worker IMU indicator."
    visibleSensorTypes = new Set();
    if (data.primary_worker_id && latestWorkers[data.primary_worker_id]) {
      selectWorker(data.primary_worker_id);
      const w = latestWorkers[data.primary_worker_id];
      const anchors = nearestAnchors(w, 3);
      highlightedUwbAnchors = new Set(anchors.map((e) => e.a.anchor_id));
    }
  } else if (mapped) {
    visibleSensorTypes = new Set(mapped);
  } else {
    visibleSensorTypes = new Set();
  }
  updateSensorMarkers(Object.values(latestSensors));
  updateUwbMarkers(Object.values(latestUwb));
}

function wiPoll() {
  // 1) Selection changed (roster click, marker click, or emergency
  // auto-select) — refresh the panel and drop any route line drawn for a
  // previously-selected worker so it doesn't linger on the map.
  if (selectedWorkerId !== wiLastSelected) {
    if (!wiRunning) {
      clearRangingLines();
      clearSelectedRoute();
      stopMovingDot();
      highlightedUwbAnchors = new Set();
    }
    wiLastSelected = selectedWorkerId;
    renderWorkerIntelPanel();
  } else if (selectedWorkerId && !wiRunning) {
    // Keep live X/Y/Z/Zone fields fresh as the worker moves.
    renderWorkerIntelPanel();
  }

  // 2) Emergency-driven automatic sensor reveal (reads the shared
  // `latestEmergency` that app.js's own pollEmergency() keeps updated).
  const data = latestEmergency || { active: false };
  const key = data.active ? `${data.type}:${data.started_at}` : null;
  const isNewTrigger = !!data.active && key !== lastEmergencyKeySeen;
  lastEmergencyKeySeen = key;

  if (isNewTrigger) {
    applyEmergencySensorReveal(data);
  }

  if (lastEmergencyActiveSeen && !data.active) {
    // Emergency was just cleared — return sensors to the clean default.
    visibleSensorTypes = new Set();
    uwbVisible = false;
    highlightedUwbAnchors = new Set();
    updateSensorMarkers(Object.values(latestSensors));
    updateUwbMarkers(Object.values(latestUwb));
  }
  lastEmergencyActiveSeen = !!data.active;
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

window.addEventListener('DOMContentLoaded', () => {
  const locBtn = document.getElementById('wi-simulate-location-btn');
  const routeBtn = document.getElementById('wi-simulate-route-btn');
  const resetBtn = document.getElementById('reset-emergency-btn');

  if (locBtn) {
    locBtn.addEventListener('click', () => {
      if (wiRunning) return;
      const w = selectedWorkerId ? latestWorkers[selectedWorkerId] : null;
      if (!w) return;
      renderWorkerIntelPanel();
      simulateWorkerLocation(w, { onDone: renderWorkerIntelPanel });
      renderWorkerIntelPanel();
    });
  }

  if (routeBtn) {
    routeBtn.addEventListener('click', () => {
      if (wiRunning) return;
      const w = selectedWorkerId ? latestWorkers[selectedWorkerId] : null;
      if (!w) return;
      wiRunning = true;
      renderWorkerIntelPanel();
      simulateSafeRoute(w);
    });
  }

  if (resetBtn) {
    // Additional listener alongside app.js's own — both fire independently.
    resetBtn.addEventListener('click', resetWorkerIntel);
  }

  setInterval(wiPoll, 400);
  requestAnimationFrame(twinAnimate);
  renderWorkerIntelPanel();
});
