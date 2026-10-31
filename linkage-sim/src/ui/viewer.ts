// three.js view: bars colored by tag, joint symbols by type, hover names, click to pick points.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Model } from '../model/types';
import type { Pose } from '../solver/system';

const TAG_COLORS = [0x4e79a7, 0xf28e2b, 0x59a14f, 0xe15759, 0x76b7b2, 0xedc948, 0xb07aa1, 0xff9da7, 0x9c755f];
const JOINT_COLORS: Record<string, number> = { pin: 0xdddddd, ball: 0xffcc33, ujoint: 0x33ccff, gear: 0xff66cc, ground: 0x888888 };

export class Viewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private bodyGroups: THREE.Group[] = [];
  private pickables: THREE.Mesh[] = [];
  private model: Model | null = null;
  private raycaster = new THREE.Raycaster();
  private hovered: THREE.Mesh | null = null;
  private tooltip: HTMLDivElement;
  private tagColor = new Map<string, number>();
  onPickPoint: (ref: string) => void = () => {};

  constructor(private container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.scene.background = new THREE.Color(0x1d1f23);
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(2, -3, 5);
    this.scene.add(sun);
    this.tooltip = document.createElement('div');
    this.tooltip.className = 'tooltip';
    container.appendChild(this.tooltip);
    this.renderer.domElement.addEventListener('pointermove', (e) => this.handleHover(e));
    this.renderer.domElement.addEventListener('click', (e) => this.handleClick(e));
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  // Rebuild meshes for a new model. Keeps the camera unless fit is requested.
  setModel(model: Model, poses: Pose[], fit: boolean): void {
    for (const g of this.bodyGroups) this.scene.remove(g);
    this.bodyGroups = [];
    this.pickables = [];
    this.model = model;
    const thickness = barThickness(model);
    for (const [i, body] of model.bodies.entries()) {
      const group = new THREE.Group();
      const color = this.colorFor(body.tags);
      group.add(...bodyMeshes(body, i, color, thickness, this.pickables));
      this.bodyGroups.push(group);
      this.scene.add(group);
    }
    this.addJointSymbols(model, thickness);
    this.setPoses(poses);
    if (fit) this.fitCamera();
  }

  setPoses(poses: Pose[]): void {
    poses.forEach((p, i) => {
      const g = this.bodyGroups[i];
      if (!g) return;
      g.position.set(p.r[0], p.r[1], p.r[2]);
      g.quaternion.set(p.q[1], p.q[2], p.q[3], p.q[0]);
    });
  }

  toPNG(): string {
    this.renderer.render(this.scene, this.camera);
    return this.renderer.domElement.toDataURL('image/png');
  }

  private colorFor(tags: string[]): number {
    const tag = tags[0] ?? '';
    if (!this.tagColor.has(tag)) this.tagColor.set(tag, TAG_COLORS[this.tagColor.size % TAG_COLORS.length]);
    return this.tagColor.get(tag)!;
  }

  private addJointSymbols(model: Model, thickness: number): void {
    const index = new Map(model.bodies.map((b, i) => [b.name, i]));
    const at = (body: string, point: string) => model.bodies[index.get(body)!].points[point];
    for (const j of model.joints) {
      if (j.type === 'pin' || j.type === 'ball' || j.type === 'ujoint') {
        const mesh = jointMesh(j.type, thickness, j.type === 'ball' ? null : j.axisA);
        mesh.position.set(...at(j.a.body, j.a.point));
        this.bodyGroups[index.get(j.a.body)!].add(mesh);
      }
      if (j.type === 'gear') {
        const host = j.about ? j.about[0] : null;
        const mesh = jointMesh('gear', thickness, j.axis);
        if (host) mesh.position.set(...at(host.body, host.point));
        this.bodyGroups[index.get(host ? host.body : j.a)!].add(mesh);
      }
    }
  }

  private fitCamera(): void {
    const box = new THREE.Box3();
    for (const g of this.bodyGroups) box.expandByObject(g);
    if (box.isEmpty()) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.1);
    this.controls.target.copy(center);
    this.camera.position.copy(center).add(new THREE.Vector3(0.5, -1.2, 0.9).multiplyScalar(size));
    this.camera.near = size / 1000;
    this.camera.far = size * 100;
    this.camera.updateProjectionMatrix();
  }

  private resize(): void {
    const w = Math.max(this.container.clientWidth, 1);
    const h = Math.max(this.container.clientHeight, 1);
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  private pick(e: PointerEvent | MouseEvent): THREE.Intersection | null {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    return this.raycaster.intersectObjects(this.pickables, false)[0] ?? null;
  }

  private handleHover(e: PointerEvent): void {
    const hit = this.pick(e);
    const mesh = (hit?.object as THREE.Mesh) ?? null;
    if (this.hovered && this.hovered !== mesh) setEmissive(this.hovered, 0x000000);
    this.hovered = mesh;
    if (!mesh) {
      this.tooltip.style.display = 'none';
      return;
    }
    setEmissive(mesh, 0x444444);
    const body = this.model!.bodies[mesh.userData.bodyIndex];
    this.tooltip.textContent = `${body.name}${body.tags.length ? '  [' + body.tags.join(', ') + ']' : ''}  — click to plot nearest point`;
    const rect = this.container.getBoundingClientRect();
    this.tooltip.style.display = 'block';
    this.tooltip.style.left = `${e.clientX - rect.left + 12}px`;
    this.tooltip.style.top = `${e.clientY - rect.top + 12}px`;
  }

  // Click a body: pick its named point nearest to the hit.
  private handleClick(e: MouseEvent): void {
    const hit = this.pick(e);
    if (!hit || !this.model) return;
    const bodyIndex = hit.object.userData.bodyIndex as number;
    const body = this.model.bodies[bodyIndex];
    const local = this.bodyGroups[bodyIndex].worldToLocal(hit.point.clone());
    let best = '';
    let bestDist = Infinity;
    for (const [name, p] of Object.entries(body.points)) {
      const d = local.distanceTo(new THREE.Vector3(...p));
      if (d < bestDist) {
        bestDist = d;
        best = name;
      }
    }
    if (best) this.onPickPoint(`${body.name}.${best}`);
  }
}

function setEmissive(mesh: THREE.Mesh, color: number): void {
  (mesh.material as THREE.MeshStandardMaterial).emissive.setHex(color);
}

function barThickness(model: Model): number {
  const lengths = model.bodies.filter((b) => b.kind === 'bar').map((b) => b.length).sort((a, b) => a - b);
  const typical = lengths[Math.floor(lengths.length / 2)] ?? 0.5;
  return typical * 0.02;
}

function bodyMeshes(body: Model['bodies'][number], index: number, color: number, t: number, pickables: THREE.Mesh[]): THREE.Object3D[] {
  const material = new THREE.MeshStandardMaterial({ color, roughness: 0.6 });
  const out: THREE.Object3D[] = [];
  if (body.kind === 'bar') {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(body.length, t, t), material);
    mesh.position.set(body.length / 2, 0, 0);
    mesh.userData.bodyIndex = index;
    pickables.push(mesh);
    out.push(mesh);
  }
  const points = Object.values(body.points);
  if (body.kind === 'plate') {
    const line = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p))),
      new THREE.LineBasicMaterial({ color }),
    );
    out.push(line);
    for (const p of points) {
      const dot = new THREE.Mesh(new THREE.BoxGeometry(t * 2, t * 2, t * 2), material);
      dot.position.set(...p);
      dot.userData.bodyIndex = index;
      pickables.push(dot);
      out.push(dot);
    }
  }
  return out;
}

function jointMesh(type: string, t: number, axis: [number, number, number] | null): THREE.Mesh {
  const material = new THREE.MeshStandardMaterial({ color: JOINT_COLORS[type] ?? 0xffffff, roughness: 0.4 });
  let geometry: THREE.BufferGeometry;
  if (type === 'ball') geometry = new THREE.SphereGeometry(t * 1.3, 16, 12);
  else if (type === 'pin') geometry = new THREE.CylinderGeometry(t * 1.1, t * 1.1, t * 3.2, 16);
  else if (type === 'ujoint') geometry = new THREE.TorusGeometry(t * 1.4, t * 0.4, 8, 20);
  else geometry = new THREE.TorusGeometry(t * 3, t * 0.5, 8, 32);
  const mesh = new THREE.Mesh(geometry, material);
  if (axis) {
    // Cylinders run along local y, tori lie around local z.
    const from = type === 'pin' ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(0, 0, 1);
    mesh.quaternion.setFromUnitVectors(from, new THREE.Vector3(...axis).normalize());
  }
  return mesh;
}
