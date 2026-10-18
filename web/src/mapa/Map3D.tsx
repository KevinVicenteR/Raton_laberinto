// Mapa 3D de ocupacion con three.js. Se carga aparte (React.lazy) para no pesar en el arranque.
import { useEffect, useRef, useState } from 'react';
import { wall } from './maze';
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
  const latest = useRef({ occupancy, trail, last });
  latest.current = { occupancy, trail, last };
  useEffect(() => {
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
    controls.enableDamping = true;
    scene.add(new THREE.AmbientLight(0xffffff, 2));
    scene.add(new THREE.GridHelper(500, 50));
    const mesh = new THREE.InstancedMesh(
      new THREE.BoxGeometry(1, 5, 1),
      new THREE.MeshLambertMaterial({ color: 0xc8a06a }),
      30000,
    );
    mesh.count = 0;
    scene.add(mesh);
    const path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x2f6fdf }));
    scene.add(path);
    const robot = new THREE.Mesh(
      new THREE.BoxGeometry(8, 3, 10),
      new THREE.MeshLambertMaterial({ color: 0xd9480f }),
    );
    scene.add(robot);
    if (showTrack) {
      const group = new THREE.Group(),
        material = new THREE.MeshLambertMaterial({ color: 0x9aa3b2, transparent: true, opacity: 0.3 });
      const ox = (maze.start[0] + 0.5) * maze.PITCH,
        oy = (maze.start[1] + 0.5) * maze.PITCH - 33;
      for (let x = 0; x < maze.W; x++)
        for (let y = 0; y < maze.H; y++)
          for (const d of [0, 3, ...(y === 0 ? [2] : []), ...(x === maze.W - 1 ? [1] : [])])
            if (wall(maze, x, y, d)) {
              const horizontal = d === 0 || d === 2,
                p = maze.PITCH;
              const m = new THREE.Mesh(
                new THREE.BoxGeometry((horizontal ? p : 15) * 0.1, 5, (horizontal ? 15 : p) * 0.1),
                material,
              );
              m.position.set(
                ((x + (d === 1 ? 1 : horizontal ? 0.5 : 0)) * p - ox) * 0.1,
                2.5,
                -((y + (d === 0 ? 1 : horizontal ? 0 : 0.5)) * p - oy) * 0.1,
              );
              group.add(m);
            }
      scene.add(group);
    }
    const resize = () => {
      renderer.setSize(el.clientWidth, el.clientHeight);
      renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
      camera.aspect = el.clientWidth / el.clientHeight;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(el);
    resize();
    let raf = 0,
      previous: Occupancy | undefined,
      previousTrail: RobotSample[] | undefined;
    const animate = () => {
      const data = latest.current;
      if (previous !== data.occupancy) {
        let n = 0;
        const matrix = new THREE.Matrix4();
        for (const [key, v] of data.occupancy.grid) {
          if (v <= 0.9 || n >= 30000) continue;
          const [x, y] = key.split(',').map(Number);
          mesh.setMatrixAt(n++, matrix.makeTranslation(x, 2.5, -y));
        }
        mesh.count = n;
        mesh.instanceMatrix.needsUpdate = true;
        previous = data.occupancy;
      }
      if (previousTrail !== data.trail) {
        path.geometry.dispose();
        path.geometry = new THREE.BufferGeometry().setFromPoints(
          data.trail.map((s) => new THREE.Vector3(s.x * 0.1, 0.3, -s.y * 0.1)),
        );
        previousTrail = data.trail;
      }
      if (data.last) {
        robot.position.set(data.last.x * 0.1, 1.5, -data.last.y * 0.1);
        robot.rotation.y = (data.last.a * Math.PI) / 180;
        const delta = robot.position.clone().sub(controls.target).multiplyScalar(0.05);
        controls.target.add(delta);
        camera.position.add(delta);
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
  }, [maze, showTrack, top]);
  return (
    <section className="panel wide">
      <h2>Mapa 3D de lo recorrido</h2>
      <div className="row">
        <button onClick={() => setTop(!top)}>{top ? 'Vista orbital' : 'Vista desde arriba'}</button>
        <label>
          <input type="checkbox" checked={showTrack} onChange={(e) => setShowTrack(e.target.checked)} />{' '}
          Mostrar pista
        </label>
      </div>
      {error && <p>Mapa 3D no disponible en este dispositivo. Puedes seguir usando las vistas 2D.</p>}
      <div className="map3d" ref={host} />
      <p className="status">Arrastra para girar · rueda para acercar · clic derecho para mover</p>
    </section>
  );
}
