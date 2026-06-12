let DEPS = null;

async function loadDependencies() {
  if (DEPS) return DEPS;
  const THREE = await import('three');
  const { OrbitControls } = await import('three/examples/jsm/controls/OrbitControls.js');
  const URDFLoader = (await import('https://cdn.jsdelivr.net/npm/urdf-loader@0.12.6/src/URDFLoader.js')).default;
  DEPS = { THREE, OrbitControls, URDFLoader };
  return DEPS;
}

const VIEWER_HEIGHT = 360;

window.AerialRobotUrdfViewer = async function renderUrdfViewer(container, urdfText, options = {}) {
  const { THREE, OrbitControls, URDFLoader } = await loadDependencies();
  container.replaceChildren();

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d1426);
  const camera = new THREE.PerspectiveCamera(45, (container.clientWidth || 320) / VIEWER_HEIGHT, 0.01, 1000);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(container.clientWidth || 320, VIEWER_HEIGHT);
  container.appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0xffffff, 0.75));
  const light = new THREE.DirectionalLight(0xffffff, 0.9);
  light.position.set(3, 5, 4);
  scene.add(light);
  const grid = new THREE.GridHelper(4, 20, 0x33506c, 0x1f2a44);
  scene.add(grid);

  // STL/DAE meshes load asynchronously through the manager; framing is
  // redone in onLoad once real geometry exists.
  const manager = new THREE.LoadingManager();
  const failedMeshes = [];
  manager.onError = (url) => {
    failedMeshes.push(url);
    options.onMeshError?.(failedMeshes.length);
  };

  const loader = new URDFLoader(manager);
  // Meshes referenced as package://<pkg>/<path> are served by the console
  // HTTP server under /pkg/<pkg>/<path>.
  loader.packages = (pkg) => `/pkg/${pkg}`;
  const robot = loader.parse(urdfText);
  robot.rotation.x = -Math.PI / 2;
  scene.add(robot);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  const frame = () => {
    const box = new THREE.Box3().setFromObject(robot);
    if (box.isEmpty()) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    robot.position.sub(center);
    const maxDim = Math.max(size.x, size.y, size.z, 0.1);
    camera.position.set(maxDim * 1.8, maxDim * 1.6, maxDim * 1.2);
    camera.near = maxDim / 100;
    camera.far = maxDim * 100;
    camera.updateProjectionMatrix();
    grid.position.y = -size.y / 2;
    controls.target.set(0, 0, 0);
    controls.update();
  };
  frame();
  manager.onLoad = frame;

  const resize = () => {
    const width = container.clientWidth || 320;
    camera.aspect = width / VIEWER_HEIGHT;
    camera.updateProjectionMatrix();
    renderer.setSize(width, VIEWER_HEIGHT);
  };
  let resizeObserver = null;
  if (window.ResizeObserver) {
    resizeObserver = new ResizeObserver(resize);
    resizeObserver.observe(container);
  } else {
    window.addEventListener('resize', resize);
  }

  let running = true;
  const animate = () => {
    if (!running) return;
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  };
  animate();

  return {
    setJointValues(values) {
      if (typeof robot.setJointValues === 'function') {
        robot.setJointValues(values);
      } else {
        Object.entries(values).forEach(([name, value]) => robot.joints?.[name]?.setJointValue(value));
      }
    },
    dispose() {
      running = false;
      manager.onLoad = undefined;
      if (resizeObserver) resizeObserver.disconnect();
      else window.removeEventListener('resize', resize);
      controls.dispose();
      scene.traverse((obj) => {
        obj.geometry?.dispose?.();
        const materials = Array.isArray(obj.material) ? obj.material : (obj.material ? [obj.material] : []);
        materials.forEach((material) => {
          Object.values(material).forEach((value) => value?.isTexture && value.dispose());
          material.dispose();
        });
      });
      renderer.dispose();
      container.replaceChildren();
    },
  };
};
