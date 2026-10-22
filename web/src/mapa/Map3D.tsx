// Mapa 3D de ocupacion con three.js. Se carga aparte (React.lazy) para no pesar en el arranque.
import { useEffect, useRef, useState } from 'react';
import { wall } from './maze';
import { spatialPose, spatialSegments } from './spatialPose';
import type { Maze, RobotSample } from '../telemetria/types';
import type { Occupancy } from './occupancy';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
export default function Map3D({
  occupancy,
  trail,
  last,
  maze,
}: {
  occupancy: Occupancy;
  trail: RobotSample[];
  last: RobotSample | null;
  maze: Maze;
}) {
  const host = useRef<HTMLDivElement>(null),
    [error, setError] = useState(false),
    [top, setTop] = useState(false),
    [showTrack, setShowTrack] = useState(true);
  const [showEchoes, setShowEchoes] = useState(false);
  const latest = useRef({ occupancy, trail, last, maze });
  latest.current = { occupancy, trail, last, maze };
  useEffect(() => {
    const maze = latest.current.maze;
    const el = host.current;
    if (!el) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    } catch {
      setError(true);
      return;
    }
    el.append(renderer.domElement);
    const scene = new THREE.Scene(),
      camera = new THREE.PerspectiveCamera(50, 1, 1, 10000);
    camera.position.set(0, top ? 160 : 90, top ? 0.01 : 90);
    const controls = new OrbitControls(camera, renderer.domElement);
    const span = Math.max(maze.W, maze.H) * maze.PITCH * 0.1;
    controls.target.set(maze.W * maze.PITCH * 0.05, 0, -maze.H * maze.PITCH * 0.05);
    camera.position.copy(controls.target).add(new THREE.Vector3(0, span * 1.3, top ? 0.01 : span));
    controls.enableDamping = true;
    scene.add(new THREE.AmbientLight(0xffffff, 2));
    const grid = new THREE.GridHelper(span * 1.2, 40);
    grid.position.copy(controls.target);
    scene.add(grid);
    const mesh = new THREE.InstancedMesh(
      new THREE.BoxGeometry(1, 5, 1),
      new THREE.MeshLambertMaterial({ color: 0xc8a06a }),
      30000,
    );
    mesh.count = 0;
    mesh.visible = showEchoes;
    scene.add(mesh);
    const path = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x2f6fdf }),
    );
    scene.add(path);
    const robot = new THREE.Mesh(
      new THREE.BoxGeometry(8, 3, 10),
      new THREE.MeshLambertMaterial({ color: 0xd9480f }),
    );
    scene.add(robot);
    const group = new THREE.Group();
    scene.add(group);
    const updateWalls = (maze: Maze) => {
      for (const child of [...group.children]) {
        if (child instanceof THREE.Mesh) {
          child.geometry.dispose();
          (Array.isArray(child.material) ? child.material : [child.material]).forEach((m) => m.dispose());
        }
        group.remove(child);
      }
      if (!showTrack) return;
      const material = new THREE.MeshLambertMaterial({ color: 0x9aa3b2, transparent: true, opacity: 0.3 });
      const ox = 0,
        oy = 0,
        thickness = maze.WALL ?? 12;
      for (let x = 0; x < maze.W; x++)
        for (let y = 0; y < maze.H; y++)
          for (const d of [0, 3, ...(y === 0 ? [2] : []), ...(x === maze.W - 1 ? [1] : [])])
            if (wall(maze, x, y, d)) {
              const horizontal = d === 0 || d === 2,
                p = maze.PITCH;
              const m = new THREE.Mesh(
                new THREE.BoxGeometry(
                  (horizontal ? p : thickness) * 0.1,
                  5,
                  (horizontal ? thickness : p) * 0.1,
                ),
                material,
              );
              m.position.set(
                ((x + (d === 1 ? 1 : horizontal ? 0.5 : 0)) * p - ox) * 0.1,
                2.5,
                -((y + (d === 0 ? 1 : horizontal ? 0 : 0.5)) * p - oy) * 0.1,
              );
              group.add(m);
            }
    };
    updateWalls(maze);
    const resize = () => {
      const width = Math.max(1, el.clientWidth),
        height = Math.max(1, el.clientHeight);
      renderer.setSize(width, height);
      renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(el);
    resize();
    let raf = 0,
      previous: Occupancy | undefined,
      previousTrail: RobotSample[] | undefined,
      previousMaze = maze;
    const animate = () => {
      const data = latest.current;
      const changedMaze = previousMaze !== data.maze;
      if (changedMaze) {
        updateWalls(data.maze);
        previousMaze = data.maze;
      }
      // Live NAV is absolute. A missing P/H/G must never change the map origin.
      if (showEchoes && previous !== data.occupancy) {
        let n = 0;
        const matrix = new THREE.Matrix4();
        for (const [key, v] of data.occupancy.grid) {
          if (v <= 0.9 || n >= 30000) continue;
          const [x, y] = key.split(',').map(Number);
          if (
            x < 0 ||
            y < 0 ||
            x * 10 > data.maze.W * data.maze.PITCH ||
            y * 10 > data.maze.H * data.maze.PITCH
          )
            continue;
          mesh.setMatrixAt(n++, matrix.makeTranslation(x, 2.5, -y));
        }
        mesh.count = n;
        mesh.instanceMatrix.needsUpdate = true;
        previous = data.occupancy;
      }
      if (previousTrail !== data.trail || changedMaze) {
        path.geometry.dispose();
        path.geometry = new THREE.BufferGeometry().setFromPoints(
          spatialSegments(data.trail, data.maze).flatMap(([a, b]) => [
            new THREE.Vector3(a[0] * 0.1, 0.3, -a[1] * 0.1),
            new THREE.Vector3(b[0] * 0.1, 0.3, -b[1] * 0.1),
          ]),
        );
        previousTrail = data.trail;
      }
      const pose = spatialPose(data.last, data.maze);
      robot.visible = pose !== null;
      if (data.last && pose) {
        robot.position.set(pose[0] * 0.1, 1.5, -pose[1] * 0.1);
        robot.rotation.y = (data.last.a * Math.PI) / 180;
      }
      controls.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(animate);
    };
    animate();
    return () => {
      cancelAnimationFrame(raf);
      observer.disconnect();
      controls.dispose();
      scene.traverse((obj) => {
        if (obj instanceof THREE.Mesh || obj instanceof THREE.Line) {
          obj.geometry.dispose();
          const materials = Array.isArray(obj.material) ? obj.material : [obj.material];
          materials.forEach((m) => m.dispose());
        }
      });
      renderer.dispose();
      renderer.domElement.remove();
    };
  }, [showTrack, showEchoes, top]);
  return (
    <section className="panel wide">
      <h2>Mapa 3D de lo recorrido</h2>
      <div className="row">
        <button onClick={() => setTop(!top)}>{top ? 'Vista orbital' : 'Vista desde arriba'}</button>
        <label>
          <input type="checkbox" checked={showTrack} onChange={(e) => setShowTrack(e.target.checked)} />{' '}
          Paredes confirmadas
        </label>
        <label>
          <input type="checkbox" checked={showEchoes} onChange={(e) => setShowEchoes(e.target.checked)} />{' '}
          Ecos ToF estimados
        </label>
      </div>
      {showEchoes && (
        <p className="status">Los ecos del cono del sensor son aproximados; no son paredes confirmadas.</p>
      )}
      {error && <p>Mapa 3D no disponible en este dispositivo. Puedes seguir usando las vistas 2D.</p>}
      <div className="map3d" ref={host} />
      <p className="status">Arrastra para girar · rueda para acercar · clic derecho para mover</p>
    </section>
  );
}
