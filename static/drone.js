/* Rescue Drone V1 — automatic visual flight along backend graph routes. */
class RescueDrone {
  constructor(THREE) {
    this.THREE = THREE;
    this.group = new THREE.Group();
    this.group.name = 'DRONE-01';
    this.group.visible = false;
    this.group.scale.setScalar(4.0);
    this._buildMesh();

    this.active = false;
    this.route = [];
    this.waypointIndex = 0;
    this.routeRevision = -1;
    this.missionState = 'STANDBY';
    this.terminalState = 'STANDBY';
    this.targetWorkerId = null;
    this.entryExitLabel = null;
    this.routeStatus = 'IDLE';
    this.battery = 100;
    this.signal = 100;
    this.speed = 0;
    this.cruiseSpeed = 48; // world units/second for demo playback
    this.forward = new THREE.Vector3(0, 0, -1);
  }

  _buildMesh() {
    const T = this.THREE;
    const bodyMat = new T.MeshStandardMaterial({
      color: 0x20252a, roughness: 0.5, metalness: 0.55,
    });
    const accentMat = new T.MeshStandardMaterial({
      color: 0x35d6ff, emissive: 0x0b6f84, emissiveIntensity: 0.8,
      roughness: 0.35, metalness: 0.25,
    });
    const armMat = new T.MeshStandardMaterial({ color: 0x343b42, roughness: 0.55, metalness: 0.45 });
    const propMat = new T.MeshBasicMaterial({ color: 0xaeefff, transparent: true, opacity: 0.65 });

    const body = new T.Mesh(new T.BoxGeometry(1.6, 0.55, 1.9), bodyMat);
    this.group.add(body);

    const nose = new T.Mesh(new T.SphereGeometry(0.28, 12, 12), accentMat);
    nose.position.set(0, -0.02, -1.05);
    this.group.add(nose);

    this.propellers = [];
    const offsets = [
      [1.25, 0, -1.25], [-1.25, 0, -1.25],
      [1.25, 0, 1.25], [-1.25, 0, 1.25],
    ];
    const up = new T.Vector3(0, 1, 0);
    offsets.forEach(([x, y, z]) => {
      const arm = new T.Mesh(new T.CylinderGeometry(0.07, 0.07, 1.6, 6), armMat);
      arm.position.set(x * 0.55, 0.04, z * 0.55);
      arm.quaternion.setFromUnitVectors(up, new T.Vector3(x, 0, z).normalize());
      this.group.add(arm);

      const prop = new T.Mesh(new T.BoxGeometry(1.05, 0.035, 0.14), propMat);
      prop.position.set(x, 0.2, z);
      this.group.add(prop);
      this.propellers.push(prop);
    });

    const beacon = new T.Mesh(
      new T.SphereGeometry(0.11, 8, 8),
      new T.MeshStandardMaterial({ color: 0xff4f64, emissive: 0xff1938, emissiveIntensity: 1.3 })
    );
    beacon.position.set(0, 0.35, 0.2);
    this.group.add(beacon);
    this.beacon = beacon;

    const spot = new T.SpotLight(0xe8f8ff, 22, 85, Math.PI / 7, 0.5, 1.3);
    spot.position.set(0, 0, -0.9);
    const target = new T.Object3D();
    target.position.set(0, 0, -8);
    this.group.add(target);
    spot.target = target;
    this.group.add(spot);
    this.spotlight = spot;
  }

  setMission(mission) {
    if (!mission || !mission.active || !Array.isArray(mission.route_coordinates) || !mission.route_coordinates.length) {
      this.standDown();
      return;
    }

    this.routeRevision = mission.route_revision;
    this.route = mission.route_coordinates.map((p) => new this.THREE.Vector3(p[0], p[1], p[2]));
    this.waypointIndex = this.route.length > 1 ? 1 : 0;
    this.group.position.copy(this.route[0]);
    this.group.visible = true;
    this.active = true;
    this.missionState = 'EN_ROUTE';
    this.terminalState = mission.terminal_state || 'WORKER_REACHED';
    this.targetWorkerId = mission.target_worker_id || null;
    this.entryExitLabel = mission.entry_exit_label || null;
    this.routeStatus = mission.route_status || 'SAFE';
    this.battery = Number.isFinite(mission.battery) ? mission.battery : 100;
    this.signal = Number.isFinite(mission.signal) ? mission.signal : 100;
    this.speed = 0;
  }

  standDown() {
    this.active = false;
    this.route = [];
    this.waypointIndex = 0;
    this.missionState = 'STANDBY';
    this.terminalState = 'STANDBY';
    this.targetWorkerId = null;
    this.entryExitLabel = null;
    this.routeStatus = 'IDLE';
    this.speed = 0;
    this.group.visible = false;
  }

  update(dt) {
    const T = this.THREE;
    this.propellers.forEach((p) => (p.rotation.y += 16 * dt));
    this.beacon.material.emissiveIntensity = 0.7 + 0.7 * Math.max(0, Math.sin(performance.now() * 0.007));

    if (!this.active || this.missionState !== 'EN_ROUTE') {
      this.speed = 0;
      return;
    }

    if (this.waypointIndex >= this.route.length) {
      this.missionState = this.terminalState;
      this.speed = 0;
      return;
    }

    const target = this.route[this.waypointIndex];
    const delta = target.clone().sub(this.group.position);
    const dist = delta.length();

    if (dist < 1.2) {
      this.group.position.copy(target);
      this.waypointIndex += 1;
      if (this.waypointIndex >= this.route.length) {
        this.missionState = this.terminalState;
        this.speed = 0;
      }
      return;
    }

    const dir = delta.normalize();
    this.forward.lerp(dir, Math.min(1, dt * 5)).normalize();

    // Smoothly orient local -Z toward travel direction.
    const desiredQ = new T.Quaternion().setFromUnitVectors(new T.Vector3(0, 0, -1), dir);
    this.group.quaternion.slerp(desiredQ, Math.min(1, dt * 4));

    const step = Math.min(this.cruiseSpeed * dt, dist);
    this.group.position.addScaledVector(dir, step);
    this.speed = step / Math.max(dt, 0.001);
    this.battery = Math.max(0, this.battery - 0.035 * dt);
    this.signal = T.MathUtils.clamp(this.signal + (Math.random() - 0.5) * 0.4 * dt, 35, 100);
  }

  get position() {
    return this.group.position;
  }
}
