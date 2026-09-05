// ---------------------------------------------------------------------------
// Mine Safety Control Room — frontend
// ---------------------------------------------------------------------------

const STATUS_COLORS = {
  normal: 0x4caf7d,
  warning: 0xffd23f,
  critical: 0xe5533d,
  stale: 0x8b98a3,
};

const EXIT_COLOR = 0xffd23f; // theme yellow, per spec
const PRIMARY_ROUTE_COLOR = 0x00e5ff; // neon cyan — shortest/rescue path
const ALT_ROUTE_COLOR = 0xe88a2c;
const BUDDY_LINE_COLOR = 0x39ff14; // neon green — max contrast against dark tunnel geometry

let scene, camera, renderer, controls, clock;
let workerMarkers = {};      // worker_id -> { group, sphere, ring, labelEl }
let exitGroup = null;
let exitPulseRings = [];
let hazardSphere = null;
let hazardShell = null;      // outer rotating wireframe shell around the hazard sphere
let hazardBeacon = null;     // vertical warning-beam + flare, strobes while an emergency is active
let shockwaves = [];         // one-shot expanding rings spawned the instant an emergency is declared
let lastEmergencyKey = null; // (type + started_at) — used to detect a *new* trigger vs. an unchanged active emergency
let routeLinesGroup = null;
let selectedRouteGroup = null;
let buddyLinesGroup = null;
let raycaster, mouse;
let selectedWorkerId = null;
let latestWorkers = {};
let latestEmergency = { active: false };


function hexToRgbString(hex) {
  const clean = String(hex || '#e5533d').replace('#', '');
  const num = parseInt(clean, 16) || 0xe5533d;
  return `${(num >> 16) & 255}, ${(num >> 8) & 255}, ${num & 255}`;
}

let _flareTexCache = null;
function makeFlareTexture() {
  if (_flareTexCache) return _flareTexCache;
  const size = 128;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');
  const grad = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.35, 'rgba(255,255,255,0.55)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, size, size);
  _flareTexCache = new THREE.CanvasTexture(canvas);
  return _flareTexCache;
}

function disposeMesh(obj) {
  if (!obj || !obj.geometry) return;
  obj.geometry.dispose();
  if (obj.material) obj.material.dispose();
}

// ---------------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------------

function initScene() {
  const canvas = document.getElementById('viewer-canvas');
  const wrap = canvas.parentElement;

  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x0d141b, 0.0011);

  camera = new THREE.PerspectiveCamera(
    55,
    wrap.clientWidth / wrap.clientHeight,
    0.1,
    10000
  );
  camera.position.set(400, 350, 600);

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(wrap.clientWidth, wrap.clientHeight);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.85;
  renderer.outputEncoding = THREE.sRGBEncoding;

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 40;
  controls.maxDistance = 2500;

  // Balanced lighting: keep the tunnel dark enough for the UI, while
  // allowing the GLB's vertex colours to remain vivid and readable.
  const hemi = new THREE.HemisphereLight(0x9fc8e8, 0x101820, 0.75);
  scene.add(hemi);
  const ambient = new THREE.AmbientLight(0x647788, 0.55);
  scene.add(ambient);
  const dir1 = new THREE.DirectionalLight(0xffffff, 0.85);
  dir1.position.set(300, 500, 200);
  scene.add(dir1);
  const dir2 = new THREE.DirectionalLight(0x7fb8e8, 0.35);
  dir2.position.set(-400, 200, -300);
  scene.add(dir2);
  const dir3 = new THREE.DirectionalLight(0xffd59a, 0.25);
  dir3.position.set(0, -300, 400);
  scene.add(dir3);

  // Soft camera fill so the tunnel remains visible from any orbit angle
  // without washing out its vertex colours.
  const camLight = new THREE.PointLight(0xffffff, 0.25, 0, 2);
  camera.add(camLight);
  scene.add(camera);

  routeLinesGroup = new THREE.Group();
  scene.add(routeLinesGroup);

  // Separate group for the currently-selected worker's route, so it can be
  // shown any time a worker is clicked — not just during an active
  // emergency (that's routeLinesGroup, driven by pollEmergency).
  selectedRouteGroup = new THREE.Group();
  scene.add(selectedRouteGroup);

  // Dashed links drawn between a worker in distress and every nearby
  // "buddy" the system has notified — driven by pollBuddyAlerts().
  buddyLinesGroup = new THREE.Group();
  scene.add(buddyLinesGroup);

  clock = new THREE.Clock();
  raycaster = new THREE.Raycaster();
  mouse = new THREE.Vector2();

  window.addEventListener('resize', onResize);
  renderer.domElement.addEventListener('click', onCanvasClick);

  loadTunnelMesh();
  animate();
}

function onResize() {
  const wrap = document.getElementById('viewer-canvas').parentElement;
  camera.aspect = wrap.clientWidth / wrap.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(wrap.clientWidth, wrap.clientHeight);
}

function loadTunnelMesh() {
  const loader = new THREE.GLTFLoader();
  loader.load(
    '/static/mine_network.glb',
    (gltf) => {
      const model = gltf.scene;
      model.traverse((child) => {
        if (child.isMesh) {
          if (child.material) {
            const materials = Array.isArray(child.material) ? child.material : [child.material];
            materials.forEach((mat) => {
              mat.vertexColors = true;
              // Preserve the GLB's vertex colours.  The previous version
              // multiplied the base colour and added a strong cyan emissive
              // wash, which made the whole map look pale/grey.
              mat.vertexColors = true;
              if (mat.color) mat.color.set(0xffffff);
              if ('emissive' in mat) {
                mat.emissive = new THREE.Color(0x000000);
                mat.emissiveIntensity = 0.0;
              }
              if ('roughness' in mat) mat.roughness = 0.82;
              if ('metalness' in mat) mat.metalness = 0.05;
              mat.needsUpdate = true;
            });
          }
        }
      });
      scene.add(model);

      const box = new THREE.Box3().setFromObject(model);
      const size = box.getSize(new THREE.Vector3());
      const center = box.getCenter(new THREE.Vector3());
      const maxDim = Math.max(size.x, size.y, size.z);
      const dist = maxDim * 1.1;
      camera.position.set(center.x + dist * 0.6, center.y + dist * 0.5, center.z + dist * 0.6);
      controls.target.copy(center);
      controls.maxDistance = maxDim * 4;
      controls.update();
    },
    undefined,
    (err) => {
      console.error('GLB load failed', err);
      showToast('Failed to load tunnel mesh — check /static/mine_network.glb');
    }
  );
}

function showToast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.classList.remove('hidden');
  setTimeout(() => el.classList.add('hidden'), 5000);
}

// ---------------------------------------------------------------------------
// Worker markers
// ---------------------------------------------------------------------------

function markerFor(id) {
  if (workerMarkers[id]) return workerMarkers[id];

  const group = new THREE.Group();

  const sphereGeo = new THREE.SphereGeometry(6, 16, 16);
  const sphereMat = new THREE.MeshStandardMaterial({
    color: STATUS_COLORS.normal,
    emissive: STATUS_COLORS.normal,
    emissiveIntensity: 0.6,
  });
  const sphere = new THREE.Mesh(sphereGeo, sphereMat);
  group.add(sphere);

  const ringGeo = new THREE.TorusGeometry(11, 1, 8, 32);
  const ringMat = new THREE.MeshBasicMaterial({
    color: STATUS_COLORS.normal,
    transparent: true,
    opacity: 0.6,
  });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = Math.PI / 2;
  group.add(ring);

  scene.add(group);

  const labelEl = document.createElement('div');
  labelEl.className = 'worker-label';
  document.getElementById('viewer-canvas').parentElement.appendChild(labelEl);

  const entry = { group, sphere, ring, labelEl, userId: id };
  workerMarkers[id] = entry;
  return entry;
}

function removeMarker(id) {
  const m = workerMarkers[id];
  if (!m) return;
  scene.remove(m.group);
  m.labelEl.remove();
  delete workerMarkers[id];
}

function updateWorkerMarkers(workers) {
  if (!scene) return; // 3D viewer never initialized — roster/detail panel still work without it
  const seen = new Set();
  workers.forEach((w) => {
    seen.add(w.worker_id);
    const m = markerFor(w.worker_id);
    m.group.position.set(w.x, w.y, w.z);

    const color = STATUS_COLORS[w.status] || STATUS_COLORS.normal;
    m.sphere.material.color.setHex(color);
    m.sphere.material.emissive.setHex(color);
    m.ring.material.color.setHex(color);

    const pulse = w.status === 'critical';
    const scale = pulse ? 1 + 0.15 * Math.sin(performance.now() / 150) : 1;
    m.sphere.scale.setScalar(scale);

    m.labelEl.textContent = `${w.name}  ${Math.round(w.hr)}bpm  ${w.temp.toFixed(1)}°C  ${w.status.toUpperCase()}`;
    m.labelEl.style.borderColor = `#${color.toString(16).padStart(6, '0')}55`;
  });

  Object.keys(workerMarkers).forEach((id) => {
    if (!seen.has(id)) removeMarker(id);
  });
}

function projectLabels() {
  const wrap = document.getElementById('viewer-canvas').parentElement;
  const w = wrap.clientWidth, h = wrap.clientHeight;

  Object.values(workerMarkers).forEach((m) => {
    const pos = m.group.position.clone();
    pos.project(camera);
    const behind = pos.z > 1;
    const x = (pos.x * 0.5 + 0.5) * w;
    const y = (-pos.y * 0.5 + 0.5) * h;
    m.labelEl.style.left = `${x}px`;
    m.labelEl.style.top = `${y}px`;
    m.labelEl.style.display = behind ? 'none' : 'block';
  });

  exitPulseRings.forEach((e) => {
    const pos = e.worldPos.clone();
    pos.project(camera);
    const behind = pos.z > 1;
    const x = (pos.x * 0.5 + 0.5) * w;
    const y = (-pos.y * 0.5 + 0.5) * h;
    e.labelEl.style.left = `${x}px`;
    e.labelEl.style.top = `${y}px`;
    e.labelEl.style.display = behind ? 'none' : 'block';
  });
}

// ---------------------------------------------------------------------------
// Fixed exit markers — drawn once, permanent, themed yellow
// ---------------------------------------------------------------------------

function drawExitMarkers(exitPoints) {
  if (!scene || exitGroup) return; // draw once, never reassigned; no-op if 3D viewer never initialized

  exitGroup = new THREE.Group();
  exitPulseRings = [];

  exitPoints.forEach((pt, i) => {
    const sub = new THREE.Group();
    sub.position.set(pt[0], pt[1], pt[2]);

    const cone = new THREE.Mesh(
      new THREE.ConeGeometry(7, 16, 8),
      new THREE.MeshStandardMaterial({
        color: EXIT_COLOR,
        emissive: EXIT_COLOR,
        emissiveIntensity: 0.5,
      })
    );
    cone.position.y = 14;
    cone.rotation.x = Math.PI;
    sub.add(cone);

    const disc = new THREE.Mesh(
      new THREE.CircleGeometry(9, 24),
      new THREE.MeshBasicMaterial({
        color: EXIT_COLOR,
        transparent: true,
        opacity: 0.35,
        side: THREE.DoubleSide,
      })
    );
    disc.rotation.x = -Math.PI / 2;
    disc.position.y = 1;
    sub.add(disc);

    const groundRing = new THREE.Mesh(
      new THREE.RingGeometry(12, 14, 32),
      new THREE.MeshBasicMaterial({
        color: EXIT_COLOR,
        transparent: true,
        opacity: 0.55,
        side: THREE.DoubleSide,
      })
    );
    groundRing.rotation.x = -Math.PI / 2;
    groundRing.position.y = 0.5;
    sub.add(groundRing);

    exitGroup.add(sub);

    const labelEl = document.createElement('div');
    labelEl.className = 'exit-label';
    labelEl.textContent = `EXIT ${i + 1}`;
    document.getElementById('viewer-canvas').parentElement.appendChild(labelEl);

    exitPulseRings.push({
      ring: groundRing,
      worldPos: new THREE.Vector3(pt[0], pt[1] + 18, pt[2]),
      labelEl,
      baseOpacity: 0.55,
    });
  });

  scene.add(exitGroup);
}

// ---------------------------------------------------------------------------
// Emergency hazard sphere
// ---------------------------------------------------------------------------

function updateHazardSphere(emergency) {
  if (!scene) return;
  if (!emergency.active || !emergency.zone_based || !emergency.zone_center) {
    if (hazardSphere) { scene.remove(hazardSphere); disposeMesh(hazardSphere); hazardSphere = null; }
    if (hazardShell) { scene.remove(hazardShell); disposeMesh(hazardShell); hazardShell = null; }
    if (hazardBeacon) {
      scene.remove(hazardBeacon);
      hazardBeacon.traverse(disposeMesh);
      hazardBeacon = null;
    }
    return;
  }

  const [cx, cy, cz] = emergency.zone_center;
  const radius = emergency.radius || 160;
  const color = parseInt((emergency.color || '#e5533d').replace('#', '0x'));

  // Inner translucent sphere — the hazard volume itself.
  if (!hazardSphere) {
    const geo = new THREE.SphereGeometry(1, 24, 24);
    const mat = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.16,
      side: THREE.DoubleSide,
    });
    hazardSphere = new THREE.Mesh(geo, mat);
    scene.add(hazardSphere);
  }
  hazardSphere.material.color.setHex(color);
  hazardSphere.position.set(cx, cy, cz);
  hazardSphere.userData.radius = radius;

  // Outer wireframe shell — slowly rotates, gives the zone a "force field" look.
  if (!hazardShell) {
    const geo = new THREE.SphereGeometry(1, 18, 18);
    const mat = new THREE.MeshBasicMaterial({
      color,
      wireframe: true,
      transparent: true,
      opacity: 0.3,
    });
    hazardShell = new THREE.Mesh(geo, mat);
    scene.add(hazardShell);
  }
  hazardShell.material.color.setHex(color);
  hazardShell.position.set(cx, cy, cz);
  hazardShell.userData.radius = radius * 1.15;

  // Vertical warning beacon — a translucent beam + glowing flare on top that
  // strobes, visible from across the tunnel even when the zone itself is
  // out of frame.
  if (!hazardBeacon) {
    hazardBeacon = new THREE.Group();
    const beamHeight = 260;
    const beam = new THREE.Mesh(
      new THREE.CylinderGeometry(2.4, 2.4, beamHeight, 12, 1, true),
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.2,
        side: THREE.DoubleSide,
        depthWrite: false,
      })
    );
    beam.position.y = beamHeight / 2;
    hazardBeacon.add(beam);

    const flare = new THREE.Sprite(
      new THREE.SpriteMaterial({ map: makeFlareTexture(), color, transparent: true, depthWrite: false })
    );
    flare.scale.set(80, 80, 1);
    flare.position.y = beamHeight;
    hazardBeacon.add(flare);

    hazardBeacon.userData.beam = beam;
    hazardBeacon.userData.flare = flare;
    scene.add(hazardBeacon);
  }
  hazardBeacon.userData.beam.material.color.setHex(color);
  hazardBeacon.userData.flare.material.color.setHex(color);
  hazardBeacon.position.set(cx, cy, cz);
}

// One-shot expanding rings fired the instant an emergency is declared —
// three staggered rings radiating out from the hazard center, like a
// shockwave. Purely cosmetic; cleaned up automatically once they fade out.
function spawnShockwave(center, colorHex) {
  if (!scene || !clock) return;
  const [cx, cy, cz] = center;
  const now = clock.getElapsedTime();
  for (let i = 0; i < 3; i++) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(1, 2.6, 48),
      new THREE.MeshBasicMaterial({
        color: colorHex,
        transparent: true,
        opacity: 0.9,
        side: THREE.DoubleSide,
        depthWrite: false,
      })
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(cx, cy + 1.5, cz);
    scene.add(ring);
    shockwaves.push({
      mesh: ring,
      start: now + i * 0.18,
      maxRadius: 110 + i * 40,
      duration: 1.3,
    });
  }
}

// ---------------------------------------------------------------------------
// Route drawing
// ---------------------------------------------------------------------------

function disposeGroupChildren(group) {
  while (group.children.length) {
    const c = group.children.pop();
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      if (c.material.map) c.material.map.dispose();
      c.material.dispose();
    }
  }
}

function drawRoutes(rescueRoutes) {
  if (!routeLinesGroup) return;
  disposeGroupChildren(routeLinesGroup);

  Object.values(rescueRoutes || {}).forEach((route) => {
    addLine(route.primary, PRIMARY_ROUTE_COLOR, routeLinesGroup);
    if (route.alternative_available && route.alternative && route.alternative.length) {
      addLine(route.alternative, ALT_ROUTE_COLOR, routeLinesGroup);
    }
  });
}

// Draws the shortest (and, if available, alternative) safe path for
// whichever worker is currently selected in the roster/detail panel. Kept
// in its own group so it doesn't get cleared/overwritten by the emergency
// rescue-route drawing loop, and so it's visible any time a worker is
// selected — not only during an active emergency.
function drawSelectedRoute(route) {
  if (!selectedRouteGroup) return;
  disposeGroupChildren(selectedRouteGroup);
  if (!route || !route.reachable) return;
  addLine(route.route_coordinates, PRIMARY_ROUTE_COLOR, selectedRouteGroup);
  if (
    route.alternative_available &&
    route.alternative_route_coordinates &&
    route.alternative_route_coordinates.length
  ) {
    addLine(route.alternative_route_coordinates, ALT_ROUTE_COLOR, selectedRouteGroup);
  }
}

function clearSelectedRoute() {
  drawSelectedRoute(null);
}

// ---------------------------------------------------------------------------
// Buddy Trigger System — dashed links from a distressed worker to every
// nearby worker the backend has notified as their "buddy"
// ---------------------------------------------------------------------------

function drawBuddyLinks(alerts) {
  if (!buddyLinesGroup) return;
  disposeGroupChildren(buddyLinesGroup);
  (alerts || []).forEach((alert) => {
    (alert.buddies || []).forEach((b) => {
      // route_coordinates is the buddy's real A* walking path over the
      // tunnel graph to the distressed worker (from /api/buddy-alerts) — not
      // a straight line, so it never cuts through rock or a hazard zone.
      if (b.route_coordinates && b.route_coordinates.length >= 2) {
        addDashedRoute(b.route_coordinates, BUDDY_LINE_COLOR, buddyLinesGroup);
      }
    });
  });
}

// Straight-segment curve (not a smoothed spline) so the tube follows the
// exact walkable path and never cuts through rock or a hazard zone.
function buildPolylineCurve(points) {
  const curve = new THREE.CurvePath();
  for (let i = 0; i < points.length - 1; i++) {
    curve.add(new THREE.LineCurve3(points[i], points[i + 1]));
  }
  return curve;
}

// Outer glow halo shared by every neon route/link — wider, unlit,
// additive-blended, drawn beneath the bright core so routes read clearly
// against dark tunnel geometry at any zoom level.
function addGlowHalo(curve, tubularSegments, color, group, renderOrder) {
  const glowGeo = new THREE.TubeGeometry(curve, tubularSegments, 2.6, 8, false);
  const glowMat = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.35,
    depthTest: false,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    side: THREE.DoubleSide,
  });
  const glowMesh = new THREE.Mesh(glowGeo, glowMat);
  glowMesh.renderOrder = renderOrder;
  group.add(glowMesh);
}

// Builds a small tiling texture that is opaque for the first ~60% of its
// width and transparent for the rest, so a tube's UV.x (running along its
// length) reads as a bold, evenly spaced dash pattern instead of the
// single-pixel-wide native line the browser would otherwise render.
let _dashTexture = null;
function getDashTexture() {
  if (_dashTexture) return _dashTexture;
  const canvas = document.createElement('canvas');
  canvas.width = 64;
  canvas.height = 8;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, 64, 8);
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0, 0, 40, 8); // dash
  // 40-64 left transparent as the gap
  _dashTexture = new THREE.CanvasTexture(canvas);
  _dashTexture.wrapS = THREE.RepeatWrapping;
  _dashTexture.wrapT = THREE.RepeatWrapping;
  return _dashTexture;
}

// A real 3D tube (not a 1px THREE.Line) so "thickness" is an actual radius
// that reads clearly at any zoom level, rendered in a saturated neon color,
// with an additive outer glow halo layered underneath for extra visibility
// against dark tunnel walls — this is the buddy-notification path drawn
// when the Buddy Trigger System fires.
function addDashedRoute(coords, color, group) {
  if (!coords || coords.length < 2 || !group) return;
  const points = coords.map((c) => new THREE.Vector3(c[0], c[1] + 3.5, c[2]));
  const curve = buildPolylineCurve(points);
  const pathLength = curve.getLength() || 1;
  const tubularSegments = Math.max(points.length * 6, 24);

  addGlowHalo(curve, tubularSegments, color, group, 2049);

  // Bright dashed core tube on top of the glow.
  const coreGeo = new THREE.TubeGeometry(curve, tubularSegments, 1.1, 8, false);
  const dashTexture = getDashTexture().clone();
  dashTexture.needsUpdate = true;
  const dashPeriod = 14; // world units per dash+gap cycle
  dashTexture.repeat.set(pathLength / dashPeriod, 1);
  const coreMat = new THREE.MeshBasicMaterial({
    color,
    map: dashTexture,
    transparent: true,
    opacity: 1.0,
    depthTest: false,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    side: THREE.DoubleSide,
  });
  const coreMesh = new THREE.Mesh(coreGeo, coreMat);
  coreMesh.renderOrder = 2050;
  group.add(coreMesh);
}

// Same neon tube-with-glow treatment as the buddy links above, but a solid
// (non-dashed) core — used for the shortest/rescue path so it stays bright
// and unmistakable from any camera angle instead of the old 1px line.
function addLine(coords, color, group) {
  if (!coords || coords.length < 2 || !group) return;
  const points = coords.map((c) => new THREE.Vector3(c[0], c[1] + 3.0, c[2]));
  const curve = buildPolylineCurve(points);
  const tubularSegments = Math.max(points.length * 6, 24);

  addGlowHalo(curve, tubularSegments, color, group, 1999);

  const coreGeo = new THREE.TubeGeometry(curve, tubularSegments, 1.1, 8, false);
  const coreMat = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 1.0,
    depthTest: false,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    side: THREE.DoubleSide,
  });
  const coreMesh = new THREE.Mesh(coreGeo, coreMat);
  coreMesh.renderOrder = 2000;
  group.add(coreMesh);
}

// ---------------------------------------------------------------------------
// Interaction
// ---------------------------------------------------------------------------

function onCanvasClick(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(mouse, camera);
  const spheres = Object.values(workerMarkers).map((m) => m.sphere);
  const hits = raycaster.intersectObjects(spheres);
  if (hits.length) {
    const hitMesh = hits[0].object;
    const entry = Object.values(workerMarkers).find((m) => m.sphere === hitMesh);
    if (entry) selectWorker(entry.userId);
  }
}

function selectWorker(id) {
  selectedWorkerId = id;
  document.querySelectorAll('.roster-row').forEach((el) => {
    el.classList.toggle('selected', el.dataset.workerId === id);
  });
  renderDetailPanel();
}

// ---------------------------------------------------------------------------
// Animation loop
// ---------------------------------------------------------------------------

function animate() {
  requestAnimationFrame(animate);
  const t = clock.getElapsedTime();

  if (exitGroup) {
    exitGroup.rotation.y = t * 0.15;
  }
  exitPulseRings.forEach((e, i) => {
    const pulse = 0.35 + 0.25 * Math.sin(t * 1.6 + i);
    e.ring.material.opacity = pulse;
    const s = 1 + 0.12 * Math.sin(t * 1.6 + i);
    e.ring.scale.set(s, s, 1);
  });

  if (hazardSphere) {
    const pulse = 1 + 0.08 * Math.sin(t * 2.4);
    hazardSphere.scale.setScalar(hazardSphere.userData.radius * pulse);
    hazardSphere.material.opacity = 0.14 + 0.08 * Math.sin(t * 2.4);
  }
  if (hazardShell) {
    hazardShell.rotation.y = t * 0.25;
    hazardShell.rotation.x = Math.sin(t * 0.15) * 0.2;
    const pulse = 1 + 0.05 * Math.sin(t * 3.1 + 1);
    hazardShell.scale.setScalar(hazardShell.userData.radius * pulse);
    // Fast strobe, like a rotating hazard beacon, layered on top of the slow pulse.
    hazardShell.material.opacity = 0.22 + 0.18 * Math.max(0, Math.sin(t * 5));
  }
  if (hazardBeacon) {
    const strobe = 0.5 + 0.5 * Math.sin(t * 6);
    hazardBeacon.userData.beam.material.opacity = 0.12 + 0.22 * strobe;
    hazardBeacon.userData.flare.material.opacity = 0.5 + 0.5 * strobe;
    const flareScale = 70 + 30 * strobe;
    hazardBeacon.userData.flare.scale.set(flareScale, flareScale, 1);
    hazardBeacon.rotation.y = t * 0.4;
  }
  if (shockwaves.length) {
    shockwaves = shockwaves.filter((s) => {
      const age = t - s.start;
      if (age < 0) return true; // staggered ring hasn't started yet
      const p = age / s.duration;
      if (p >= 1) {
        scene.remove(s.mesh);
        disposeMesh(s.mesh);
        return false;
      }
      s.mesh.scale.setScalar(2 + p * s.maxRadius);
      s.mesh.material.opacity = 0.9 * (1 - p);
      return true;
    });
  }
  controls.update();

  projectLabels();
  renderer.render(scene, camera);
}

// ---------------------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------------------

function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString([], { hour12: false });
}

function renderClock() {
  const el = document.getElementById('val-clock');
  const now = new Date();
  el.textContent = now.toLocaleTimeString([], { hour12: false });
}

function renderRoster(workers) {
  const list = document.getElementById('roster-list');
  list.innerHTML = '';
  workers
    .slice()
    .sort((a, b) => a.name.localeCompare(b.name))
    .forEach((w) => {
      const row = document.createElement('div');
      row.className = 'roster-row' + (w.worker_id === selectedWorkerId ? ' selected' : '');
      row.dataset.workerId = w.worker_id;
      row.innerHTML = `
        <span class="roster-name">${w.name}</span>
        <span class="roster-badge badge-${w.status}">${w.status}</span>
      `;
      row.addEventListener('click', () => selectWorker(w.worker_id));
      list.appendChild(row);
    });
  document.getElementById('val-worker-count').textContent = workers.length;
}

function renderDetailPanel() {
  const empty = document.getElementById('detail-empty');
  const content = document.getElementById('detail-content');
  const w = latestWorkers[selectedWorkerId];

  if (!w) {
    empty.classList.remove('hidden');
    content.classList.add('hidden');
    clearSelectedRoute();
    return;
  }
  empty.classList.add('hidden');
  content.classList.remove('hidden');

  document.getElementById('detail-name').textContent = w.name;
  const statusPill = document.getElementById('detail-status');
  statusPill.textContent = w.status;
  statusPill.className = `status-pill ${w.status}`;

  document.getElementById('detail-hr').textContent = `${Math.round(w.hr)} bpm`;
  document.getElementById('detail-temp').textContent = `${w.temp.toFixed(1)} °C`;
  document.getElementById('detail-spo2').textContent = `${w.spo2.toFixed(1)}%`;
  document.getElementById('detail-battery').textContent = `${Math.round(w.battery)}%`;
  document.getElementById('detail-risk').textContent = `${w.risk} / 10`;
  document.getElementById('detail-fatigue').textContent = w.fatigue;
  document.getElementById('detail-note').textContent = w.ai_note;

  document.getElementById('detail-dist-exit').textContent =
    w.dist_to_exit != null ? `${w.dist_to_exit.toFixed(1)} m` : 'unreachable';
  document.getElementById('detail-dist-refuge').textContent =
    w.dist_to_refuge != null ? `${w.dist_to_refuge.toFixed(1)} m` : 'unreachable';

  fetch(`/api/routes/${w.worker_id}`)
    .then((r) => r.json())
    .then((route) => {
      document.getElementById('detail-route-status').textContent = route.reachable
        ? 'SAFE'
        : 'NO SAFE ROUTE';
      document.getElementById('detail-route-dest').textContent = route.destination_label || '—';
      document.getElementById('detail-route-distance').textContent = route.distance_m != null
        ? `${route.distance_m} m`
        : '—';
      document.getElementById('detail-route-eta').textContent = route.eta_seconds != null
        ? `${Math.round(route.eta_seconds)} s`
        : '—';
      document.getElementById('detail-route-nodes').textContent = route.nodes_explored ?? '—';
      document.getElementById('detail-route-alt').textContent = route.alternative_available
        ? 'Available'
        : 'None';
      // Only draw an evacuation route when the backend confirms that the
      // destination is one of Exit 1/2/3.
      const validExit =
        route.destination_type === 'exit' &&
        Number.isInteger(route.destination_exit_index) &&
        route.destination_exit_index >= 0 &&
        route.destination_exit_index < 3;
      drawSelectedRoute(validExit ? route : null);
    })
    .catch(() => {});
}

function renderEmergencyBanner(emergency) {
  const banner = document.getElementById('emergency-banner');
  const chip = document.getElementById('chip-emergency');
  const valEl = document.getElementById('val-emergency');

  if (emergency.active) {
    banner.classList.remove('hidden');
    document.getElementById('banner-label').textContent = `${emergency.label.toUpperCase()} DECLARED`;
    document.getElementById('banner-action').textContent = emergency.action || '';
    chip.classList.add('active');
    valEl.textContent = emergency.label.toUpperCase();
  } else {
    banner.classList.add('hidden');
    chip.classList.remove('active');
    valEl.textContent = 'NORMAL';
  }
}

function renderRescuePanel(emergency) {
  const list = document.getElementById('rescue-list');
  if (!emergency.active || !emergency.workers_affected || !emergency.workers_affected.length) {
    list.innerHTML = '<div class="empty-state">No workers currently inside a hazard zone.</div>';
    return;
  }
  list.innerHTML = '';
  emergency.workers_affected.forEach((wid) => {
    const route = emergency.rescue_routes[wid];
    const w = latestWorkers[wid];
    const name = w ? w.name : wid;
    const safe = route.route_status === 'SAFE';
    const card = document.createElement('div');
    card.className = 'rescue-card';
    card.innerHTML = `
      <div class="rescue-card-head">
        <span class="rescue-name">${name}</span>
        <span class="rescue-status ${safe ? 'safe' : 'unsafe'}">${route.route_status}</span>
      </div>
      <div class="rescue-detail-row"><span>Algorithm</span><span>${route.algorithm}</span></div>
      <div class="rescue-detail-row"><span>Destination</span><span>${route.destination_label || '—'}</span></div>
      <div class="rescue-detail-row"><span>Distance</span><span>${route.distance_m != null ? route.distance_m + ' m' : '—'}</span></div>
      <div class="rescue-detail-row"><span>ETA</span><span>${route.eta_seconds != null ? Math.round(route.eta_seconds) + ' s' : '—'}</span></div>
      <div class="rescue-detail-row"><span>Nodes explored</span><span>${route.nodes_explored}</span></div>
      <div class="rescue-detail-row"><span>Alternative</span><span>${route.alternative_available ? 'Yes' : 'No'}</span></div>
    `;
    list.appendChild(card);
  });
}

const BUDDY_REASON_LABELS = { sos: 'SOS', fall: 'FALL', hazard: 'HAZARD ZONE' };

function renderBuddyPanel(alerts) {
  const list = document.getElementById('buddy-list');
  if (!alerts || !alerts.length) {
    list.innerHTML =
      '<div class="empty-state">No active buddy alerts. Nearby workers and the control room are auto-notified the moment someone triggers SOS, falls, or enters a hazard zone.</div>';
    return;
  }
  list.innerHTML = '';
  alerts.forEach((a) => {
    const elapsed = Math.max(0, Math.round(Date.now() / 1000 - a.triggered_at));
    const buddies = a.buddies || [];
    const buddyRows = buddies.length
      ? buddies
          .map(
            (b) => `
        <div class="buddy-notified-row">
          <span class="bd-name">${b.name}</span>
          <span class="bd-dist${b.within_radius ? '' : ' far'}">${Math.round(b.distance_m)} m · ${Math.round(b.eta_seconds)}s</span>
        </div>`
          )
          .join('')
      : '<div class="buddy-notified-row"><span class="bd-name" style="opacity:.6">No worker has a safe walkable route yet</span></div>';

    const card = document.createElement('div');
    card.className = 'buddy-alert-card';
    card.innerHTML = `
      <div class="buddy-alert-head">
        <span class="buddy-alert-name">${a.name}</span>
        <span class="buddy-alert-reason">${BUDDY_REASON_LABELS[a.reason] || a.reason}</span>
      </div>
      <div class="buddy-alert-meta">Triggered ${elapsed}s ago · ${buddies.length} worker${buddies.length === 1 ? '' : 's'} notified · dist. via tunnel route</div>
      ${buddyRows}
      <div class="buddy-control-room"><span class="dot"></span>Control room notified</div>
    `;
    list.appendChild(card);
  });
}

function pollBuddyAlerts() {
  fetch('/api/buddy-alerts')
    .then((r) => r.json())
    .then((data) => {
      renderBuddyPanel(data.alerts);
      drawBuddyLinks(data.alerts);
    })
    .catch(() => {})
    .finally(() => setTimeout(pollBuddyAlerts, 1500));
}

function renderEventLog(events) {
  const log = document.getElementById('event-log');
  log.innerHTML = '';
  events
    .slice()
    .reverse()
    .slice(0, 60)
    .forEach((e) => {
      const row = document.createElement('div');
      row.className = 'event-row' + (e.level === 'critical' ? ' critical' : '');
      row.innerHTML = `<span class="ts">${fmtTime(e.ts)}</span>${e.text}`;
      log.appendChild(row);
    });
}

function buildEmergencyGrid(types) {
  const grid = document.getElementById('emergency-grid');
  grid.innerHTML = '';
  Object.entries(types).forEach(([key, def]) => {
    const btn = document.createElement('button');
    btn.className = 'emergency-btn';
    btn.innerHTML = `<span class="dot" style="background:${def.color}"></span>${def.label}`;
    btn.addEventListener('click', () => triggerEmergency(key));
    grid.appendChild(btn);
  });
}

function triggerEmergency(key) {
  fetch('/api/emergency/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    // Let the backend identify the actual incident worker first instead of
    // forcing the roster's first worker to become the simulated victim.
    body: JSON.stringify({ type: key }),
  }).catch(() => showToast('Failed to start emergency'));
}

function resetEmergency() {
  fetch('/api/emergency/reset', { method: 'POST' }).catch(() =>
    showToast('Failed to reset emergency')
  );
}

// A static list of emergency type metadata mirrors the backend's
// EMERGENCY_TYPES for building the trigger grid without an extra request —
// kept in sync with dashboard_base.py.
const EMERGENCY_TYPES = {
  gas_leak: { label: 'Gas Leak', color: '#f4c542' },
  co_leak: { label: 'CO Leak', color: '#e88a2c' },
  fire: { label: 'Fire', color: '#e5533d' },
  explosion: { label: 'Explosion', color: '#d81e3f' },
  tunnel_collapse: { label: 'Tunnel Collapse', color: '#8a6d3b' },
  rock_fall: { label: 'Rock Fall', color: '#a08262' },
  low_oxygen: { label: 'Low Oxygen', color: '#4c9ed9' },
  water_ingress: { label: 'Water Ingress', color: '#2f7cc9' },
  equipment_failure: { label: 'Equipment Failure', color: '#8a8a8a' },
  manual_sos: { label: 'Manual SOS', color: '#d81e3f' },
};

// ---------------------------------------------------------------------------
// Polling loops — separate setTimeout chains so a slow request can't pile up
// ---------------------------------------------------------------------------

function pollWorkers() {
  fetch('/api/workers')
    .then((r) => r.json())
    .then((data) => {
      document.getElementById('chip-connection').classList.add('online');
      document.getElementById('chip-connection').classList.remove('offline');
      document.getElementById('val-connection').textContent = 'ONLINE';

      latestWorkers = {};
      data.workers.forEach((w) => (latestWorkers[w.worker_id] = w));
      updateWorkerMarkers(data.workers);
      renderRoster(data.workers);
      if (selectedWorkerId) {
        renderDetailPanel();
      } else {
        // Automatically display the shortest exit route for the first worker
        // until the operator selects a different worker.
        const firstWorker = data.workers[0];
        if (firstWorker) {
          selectedWorkerId = firstWorker.worker_id;
          renderRoster(data.workers);
          renderDetailPanel();
        }
      }
    })
    .catch(() => {
      document.getElementById('chip-connection').classList.add('offline');
      document.getElementById('chip-connection').classList.remove('online');
      document.getElementById('val-connection').textContent = 'OFFLINE';
    })
    .finally(() => setTimeout(pollWorkers, 1000));
}

function triggerEmergencyFX(emergency) {
  const color = emergency.color || '#e5533d';
  const rgb = hexToRgbString(color);

  const flash = document.getElementById('emergency-flash');
  if (flash) {
    flash.style.setProperty('--em-rgb', rgb);
    flash.classList.remove('firing');
    void flash.offsetWidth; // force reflow so the animation restarts on back-to-back triggers
    flash.classList.add('firing');
  }

  const banner = document.getElementById('emergency-banner');
  if (banner) {
    banner.classList.remove('firing');
    void banner.offsetWidth;
    banner.classList.add('firing');
  }

  if (emergency.zone_based && emergency.zone_center) {
    spawnShockwave(emergency.zone_center, parseInt(color.replace('#', '0x')));
  }
}

function pollEmergency() {
  fetch('/api/emergency')
    .then((r) => r.json())
    .then((data) => {
      latestEmergency = data;

      const key = data.active ? `${data.type}:${data.started_at}` : null;
      const isNewTrigger = !!data.active && key !== lastEmergencyKey;
      lastEmergencyKey = key;

      renderEmergencyBanner(data);
      renderRescuePanel(data);
      updateHazardSphere(data);

      const wrap = document.getElementById('viewer-wrap');
      if (wrap) {
        wrap.classList.toggle('emergency-active', !!data.active);
        if (data.active) wrap.style.setProperty('--em-rgb', hexToRgbString(data.color));
      }

      if (isNewTrigger) {
        triggerEmergencyFX(data);
        if (data.primary_worker_id && latestWorkers[data.primary_worker_id]) {
          selectedWorkerId = data.primary_worker_id;
          renderRoster(Object.values(latestWorkers));
          renderDetailPanel();
        }
      }

      if (data.active) {
        // Draw routes only for workers actually affected by the current
        // emergency. Unrelated abnormal vitals are not treated as victims.
        drawRoutes(data.rescue_routes);
      } else {
        drawRoutes({});
      }
    })
    .catch(() => {})
    .finally(() => setTimeout(pollEmergency, 1200));
}

function pollEvents() {
  fetch('/api/events')
    .then((r) => r.json())
    .then((data) => renderEventLog(data.events))
    .catch(() => {})
    .finally(() => setTimeout(pollEvents, 1500));
}

function pollRoutingStatus() {
  fetch('/api/routing/status')
    .then((r) => r.json())
    .then((data) => {
      document.getElementById('chip-routing').classList.add('online');
      document.getElementById('val-routing').textContent = data.graph_connected ? 'READY' : 'DEGRADED';
      document.getElementById('health-nodes').textContent = data.num_nodes;
      document.getElementById('health-edges').textContent = data.num_edges;
      document.getElementById('health-connectivity').textContent = data.graph_connected
        ? 'Fully connected'
        : 'Degraded';
      document.getElementById('health-algo').textContent = data.algorithm;
      drawExitMarkers(data.exit_points);
    })
    .catch(() => {
      document.getElementById('chip-routing').classList.add('offline');
      document.getElementById('val-routing').textContent = 'OFFLINE';
    });
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

window.addEventListener('DOMContentLoaded', () => {
  // Wire up everything that does NOT depend on Three.js first, so that a
  // 3D-viewer failure (e.g. a missing/corrupt file under static/vendor/)
  // can never take the rest of the dashboard down with it.
  buildEmergencyGrid(EMERGENCY_TYPES);
  document.getElementById('reset-emergency-btn').addEventListener('click', resetEmergency);
  document.getElementById('banner-details-btn').addEventListener('click', () => {
    document.getElementById('rescue-panel').scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  });

  setInterval(renderClock, 1000);
  renderClock();

  pollWorkers();
  pollEmergency();
  pollEvents();
  pollRoutingStatus();
  pollBuddyAlerts();

  // 3D viewer init is isolated — if THREE/GLTFLoader/OrbitControls failed
  // to load (offline, blocked CDN, older embedded browser engine), show a
  // clear message in the viewer pane instead of silently breaking the app.
  try {
    if (typeof THREE === 'undefined' || !THREE.GLTFLoader || !THREE.OrbitControls) {
      throw new Error('Three.js failed to load from /static/vendor/');
    }
    initScene();
  } catch (err) {
    console.error('3D viewer init failed:', err);
    const wrap = document.getElementById('viewer-canvas').parentElement;
    const msg = document.createElement('div');
    msg.className = 'viewer-error';
    msg.innerHTML =
      '3D viewer unavailable.<br><span>Could not load the bundled Three.js files from ' +
      '<code>static/vendor/</code> — make sure three.min.js, GLTFLoader.js, and ' +
      'OrbitControls.js are present alongside app.js.</span>';
    wrap.appendChild(msg);
  }
});
