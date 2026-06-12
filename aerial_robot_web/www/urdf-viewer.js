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
  const grid = new THREE.GridHelper(10, 20, 0x33506c, 0x1f2a44);
  scene.add(grid);
  const axes = new THREE.AxesHelper(1.0);
  scene.add(axes);

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
  const robotRoot = new THREE.Group();
  robot.rotation.x = -Math.PI / 2;
  robotRoot.add(robot);
  scene.add(robotRoot);

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
    const distance = Math.max(maxDim * 2.4, 3.0);
    camera.position.set(distance, distance * 0.75, distance * 0.8);
    camera.near = maxDim / 100;
    camera.far = Math.max(maxDim * 100, 200);
    camera.updateProjectionMatrix();
    controls.target.copy(robotRoot.position);
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
  let followRobot = true;
  const animate = () => {
    if (!running) return;
    requestAnimationFrame(animate);
    if (followRobot) controls.target.lerp(robotRoot.position, 0.12);
    controls.update();
    renderer.render(scene, camera);
  };
  animate();

  const rosBasis = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, -1),
    new THREE.Vector3(0, 1, 0),
  );
  const rosBasisInverse = rosBasis.clone().invert();
  const rosRotation = new THREE.Matrix4();
  const threeRotation = new THREE.Matrix4();

  function applyRosPose(pose) {
    const position = pose?.position || {};
    const orientation = pose?.orientation || {};
    robotRoot.position.set(
      Number(position.x) || 0,
      Number(position.z) || 0,
      -(Number(position.y) || 0),
    );
    const q = new THREE.Quaternion(
      Number(orientation.x) || 0,
      Number(orientation.y) || 0,
      Number(orientation.z) || 0,
      Number(orientation.w) || 1,
    ).normalize();
    rosRotation.makeRotationFromQuaternion(q);
    threeRotation.multiplyMatrices(rosBasis, rosRotation).multiply(rosBasisInverse);
    robotRoot.quaternion.setFromRotationMatrix(threeRotation);
  }

  return {
    setJointValues(values) {
      if (typeof robot.setJointValues === 'function') {
        robot.setJointValues(values);
      } else {
        Object.entries(values).forEach(([name, value]) => robot.joints?.[name]?.setJointValue(value));
      }
    },
    setBasePose(pose) {
      applyRosPose(pose);
    },
    setFollowRobot(value) {
      followRobot = Boolean(value);
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
