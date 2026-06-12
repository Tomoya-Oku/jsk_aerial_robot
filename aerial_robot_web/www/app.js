(() => {
  if (!window.React || !window.ReactDOM) {
    const root = document.getElementById('root');
    if (root) {
      root.querySelector('.boot-hint')?.replaceChildren(
        'React assets failed to load. Connect this device to the internet once or pre-cache the CDN assets.',
      );
    }
    return;
  }

  const React = window.React;
  const ReactDOM = window.ReactDOM;
  const e = React.createElement;
  const params = new URLSearchParams(window.location.search);
  const defaultBridgePort = params.get('rosbridge_port') || '9090';
  const initialRobotNs = normalizeNs(params.get('robot_ns') || '');
  const initialPoseTopic = params.get('pose_topic') || nsJoin(initialRobotNs, 'ground_truth');
  const robotType = params.get('robot_type') || 'generic';
  const RECONNECT_DELAY_MS = 3000;
  const PREVIEW_THROTTLE_MS = 500;
  const PREVIEW_MAX_CHARS = 4000;

  function normalizeNs(ns) {
    if (!ns) return '';
    return ns.startsWith('/') ? ns : `/${ns}`;
  }

  function serviceName(name) {
    return name.startsWith('/') ? name : `/${name}`;
  }

  function nsJoin(ns, name) {
    const cleanName = name.startsWith('/') ? name.slice(1) : name;
    if (!ns) return `/${cleanName}`;
    return `${normalizeNs(ns)}/${cleanName}`;
  }

  function describeError(error, fallback) {
    if (!error) return fallback;
    if (error.message) return error.message;
    if (error.reason) return error.reason;
    if (error.type) return `${error.type} event`;
    return String(error);
  }

  function useRosConnection(url) {
    const [attempt, setAttempt] = React.useState(0);
    const [state, setState] = React.useState({ connected: false, error: '', ros: null });
    React.useEffect(() => {
      let alive = true;
      let retryId = 0;
      const scheduleRetry = () => {
        window.clearTimeout(retryId);
        retryId = window.setTimeout(() => setAttempt((value) => value + 1), RECONNECT_DELAY_MS);
      };
      if (!window.ROSLIB) {
        setState({
          connected: false,
          error: 'roslib failed to load; connect this browser to the internet once or pre-cache the CDN asset',
          ros: null,
        });
        scheduleRetry();
        return () => {
          alive = false;
          window.clearTimeout(retryId);
        };
      }

      let ros;
      try {
        ros = new window.ROSLIB.Ros({ url });
      } catch (error) {
        setState({ connected: false, error: describeError(error, 'connection setup failed'), ros: null });
        scheduleRetry();
        return () => {
          alive = false;
          window.clearTimeout(retryId);
        };
      }
      ros.on('connection', () => alive && setState({ connected: true, error: '', ros }));
      ros.on('close', () => {
        if (!alive) return;
        setState((prev) => ({ ...prev, connected: false }));
        scheduleRetry();
      });
      ros.on('error', (error) => {
        if (!alive) return;
        setState({ connected: false, error: describeError(error, 'connection error'), ros });
      });
      const closeRos = () => {
        alive = false;
        window.clearTimeout(retryId);
        try {
          ros.close();
        } catch (error) {
          console.warn(error);
        }
      };
      window.addEventListener('pagehide', closeRos, { once: true });
      return () => {
        window.removeEventListener('pagehide', closeRos);
        closeRos();
      };
    }, [url, attempt]);
    return state;
  }

  function callService(ros, name, type, values = {}) {
    return new Promise((resolve, reject) => {
      if (!window.ROSLIB) {
        reject(new Error('roslib is not loaded'));
        return;
      }
      const service = new window.ROSLIB.Service({ ros, name: serviceName(name), serviceType: type });
      service.callService(new window.ROSLIB.ServiceRequest(values), resolve, reject);
    });
  }

  class ErrorBoundary extends React.Component {
    constructor(props) {
      super(props);
      this.state = { error: null };
    }

    static getDerivedStateFromError(error) {
      return { error };
    }

    componentDidCatch(error, info) {
      console.error(error, info);
    }

    render() {
      if (!this.state.error) return this.props.children;
      return e('main', { className: 'app' },
        e('header', { className: 'app-header' },
          e('h1', null, 'Aerial Robot Web Console'),
          e('div', { className: 'status-row' },
            e(InfoPill, {
              label: 'Interface',
              value: describeError(this.state.error, 'render error'),
              tone: 'bad',
            }),
          ),
        ),
      );
    }
  }

  function App() {
    const [bridgeUrl, setBridgeUrl] = React.useState(`ws://${window.location.hostname}:${defaultBridgePort}`);
    const [robotNs, setRobotNs] = React.useState(initialRobotNs);
    const [poseTopic, setPoseTopic] = React.useState(initialPoseTopic);
    const [filter, setFilter] = React.useState('');
    const [graph, setGraph] = React.useState({ nodes: [], topics: [] });
    const [selected, setSelected] = React.useState(null);
    const [details, setDetails] = React.useState(null);
    const [preview, setPreview] = React.useState(null);
    const [urdf, setUrdf] = React.useState('');
    const [basePose, setBasePose] = React.useState(null);
    const [poseStamp, setPoseStamp] = React.useState('never');
    const [lastUpdate, setLastUpdate] = React.useState('never');
    const { connected, error, ros } = useRosConnection(bridgeUrl);

    const updateRobotNs = React.useCallback((value) => {
      const nextNs = normalizeNs(value);
      setRobotNs(nextNs);
      setPoseTopic(nsJoin(nextNs, 'ground_truth'));
    }, []);

    const refresh = React.useCallback(async () => {
      if (!ros || !connected) return;
      try {
        const [nodes, topics] = await Promise.all([
          callService(ros, '/rosapi/nodes', 'rosapi/Nodes'),
          callService(ros, '/rosapi/topics', 'rosapi/Topics'),
        ]);
        setGraph({ nodes: nodes.nodes || [], topics: topics.topics || [] });
        setLastUpdate(new Date().toLocaleTimeString());
      } catch (err) {
        console.warn(err);
      }
    }, [ros, connected]);

    React.useEffect(() => {
      refresh();
      const id = window.setInterval(refresh, 2000);
      return () => window.clearInterval(id);
    }, [refresh]);

    // The URDF rarely changes, so fetch it once per connection/namespace.
    React.useEffect(() => {
      if (!ros || !connected) return undefined;
      let alive = true;
      callService(ros, '/rosapi/get_param', 'rosapi/GetParam', { name: `${robotNs}/robot_description`, default: '' })
        .then((res) => {
          if (!alive || !res.value) return;
          let text = res.value;
          try { text = JSON.parse(res.value); } catch (ignore) { /* rosapi may already return a plain string */ }
          if (typeof text !== 'string') return;
          setUrdf(text);
        })
        .catch(() => undefined);
      return () => { alive = false; };
    }, [ros, connected, robotNs]);

    React.useEffect(() => {
      if (!ros || !connected || !selected) return;
      const load = async () => {
        if (selected.kind === 'node') {
          setDetails(await callService(ros, '/rosapi/node_details', 'rosapi/NodeDetails', { node: selected.name }));
        } else {
          const [type, publishers, subscribers] = await Promise.all([
            callService(ros, '/rosapi/topic_type', 'rosapi/TopicType', { topic: selected.name }),
            callService(ros, '/rosapi/publishers', 'rosapi/Publishers', { topic: selected.name }),
            callService(ros, '/rosapi/subscribers', 'rosapi/Subscribers', { topic: selected.name }),
          ]);
          setDetails({ type: type.type, publishers: publishers.publishers || [], subscribers: subscribers.subscribers || [] });
        }
      };
      setDetails(null);
      load().catch((err) => setDetails({ error: String(err) }));
    }, [ros, connected, selected]);

    // Live message preview for the selected topic.
    React.useEffect(() => {
      setPreview(null);
      if (!ros || !connected || selected?.kind !== 'topic' || !details?.type) return undefined;
      if (!window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros,
        name: selected.name,
        messageType: details.type,
        throttle_rate: PREVIEW_THROTTLE_MS,
        queue_length: 1,
      });
      topic.subscribe((msg) => setPreview(msg));
      return () => topic.unsubscribe();
    }, [ros, connected, selected, details]);

    // Drive the URDF viewer from live joint states so the model mirrors the
    // actual robot.
    const viewerApiRef = React.useRef(null);
    React.useEffect(() => {
      if (!ros || !connected) return undefined;
      if (!window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros,
        name: `${robotNs}/joint_states`,
        messageType: 'sensor_msgs/JointState',
        throttle_rate: 100,
        queue_length: 1,
      });
      topic.subscribe((msg) => {
        const values = {};
        (msg.name || []).forEach((name, index) => { values[name] = msg.position?.[index] ?? 0; });
        viewerApiRef.current?.setJointValues(values);
      });
      return () => topic.unsubscribe();
    }, [ros, connected, robotNs]);

    // Move the whole URDF model from odometry so the browser view can be used
    // like a lightweight Gazebo pose viewer.
    React.useEffect(() => {
      setBasePose(null);
      setPoseStamp('never');
      if (!ros || !connected || !poseTopic) return undefined;
      if (!window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros,
        name: poseTopic,
        messageType: 'nav_msgs/Odometry',
        throttle_rate: 50,
        queue_length: 1,
      });
      topic.subscribe((msg) => {
        const pose = msg.pose?.pose;
        if (!pose) return;
        setBasePose(pose);
        const stamp = msg.header?.stamp;
        if (stamp && Number.isFinite(stamp.secs)) {
          setPoseStamp(`${stamp.secs}.${String(stamp.nsecs || 0).padStart(9, '0')}`);
        } else {
          setPoseStamp(new Date().toLocaleTimeString());
        }
      });
      return () => topic.unsubscribe();
    }, [ros, connected, poseTopic]);

    const lowerFilter = filter.toLowerCase();
    const filteredNodes = graph.nodes.filter((name) => name.toLowerCase().includes(lowerFilter));
    const filteredTopics = graph.topics.filter((name) => name.toLowerCase().includes(lowerFilter));

    return e('main', { className: 'app' },
      e('header', { className: 'app-header' },
        e('h1', null, 'Aerial Robot Web Console'),
        e('div', { className: 'status-row' },
          e(InfoPill, { label: 'ROS Bridge', value: connected ? 'connected' : (error ? `${error} (retrying...)` : 'connecting...'), tone: connected ? 'ok' : error ? 'bad' : 'warn' }),
          e(InfoPill, { label: 'Robot', value: `${robotType}${robotNs ? ` @ ${robotNs}` : ''}` }),
          e(InfoPill, { label: 'Pose', value: formatPose(basePose), tone: basePose ? 'ok' : 'warn' }),
          e(InfoPill, { label: 'Nodes / Topics', value: `${graph.nodes.length} / ${graph.topics.length}` }),
          e(InfoPill, { label: 'Last update', value: lastUpdate }),
        ),
      ),
      e('section', { className: 'toolbar' },
        e('div', { className: 'two-col' },
          e('label', { className: 'field' },
            e('span', { className: 'label' }, 'Bridge URL'),
            e('input', { value: bridgeUrl, onChange: (ev) => setBridgeUrl(ev.target.value) }),
          ),
          e('label', { className: 'field' },
            e('span', { className: 'label' }, 'Namespace'),
            e('input', { value: robotNs, onChange: (ev) => updateRobotNs(ev.target.value), placeholder: '/robot_ns' }),
          ),
          e('label', { className: 'field' },
            e('span', { className: 'label' }, 'Odometry Topic'),
            e('input', { value: poseTopic, onChange: (ev) => setPoseTopic(serviceName(ev.target.value)), placeholder: '/robot/ground_truth' }),
          ),
          e('label', { className: 'field' },
            e('span', { className: 'label' }, 'Filter'),
            e('input', { value: filter, onChange: (ev) => setFilter(ev.target.value), placeholder: 'Filter nodes/topics...' }),
          ),
        ),
        e('button', { onClick: refresh, disabled: !connected }, 'Refresh'),
      ),
      e('section', { className: 'cards' },
        e(GraphList, { title: 'Nodes', items: filteredNodes, kind: 'node', selected, setSelected, connected, loading: lastUpdate === 'never' }),
        e(GraphList, { title: 'Topics', items: filteredTopics, kind: 'topic', selected, setSelected, connected, loading: lastUpdate === 'never' }),
        e(DetailsCard, { selected, details, preview }),
      ),
      e(UrdfPanel, { urdf, viewerApiRef, basePose, poseTopic, poseStamp, jointTopic: nsJoin(robotNs, 'joint_states') }),
    );
  }

  function InfoPill({ label, value, tone }) {
    return e('div', { className: 'pill' }, e('span', { className: 'label' }, label), e('span', { className: `value ${tone || ''}` }, value));
  }

  function GraphList({ title, items, kind, selected, setSelected, connected, loading }) {
    let body;
    if (items.length) {
      body = items.map((name) => e('button', {
        key: `${kind}:${name}`,
        className: `item ${selected?.kind === kind && selected?.name === name ? 'active' : ''}`,
        onClick: () => setSelected({ kind, name }),
      }, name));
    } else if (!connected) {
      body = e('div', { className: 'empty' }, 'Waiting for rosbridge connection...');
    } else if (loading) {
      body = e('div', { className: 'skeleton', 'aria-hidden': 'true' },
        e('div', { className: 'skeleton-row' }), e('div', { className: 'skeleton-row' }), e('div', { className: 'skeleton-row' }));
    } else {
      body = e('div', { className: 'empty' }, 'No matching entries');
    }
    return e('article', { className: 'card' },
      e('h2', null, `${title} (${items.length})`),
      e('div', { className: 'list' }, body),
    );
  }

  function formatPreview(message) {
    let text;
    try {
      text = JSON.stringify(message, null, 2);
    } catch (ignore) {
      text = String(message);
    }
    if (text.length > PREVIEW_MAX_CHARS) text = `${text.slice(0, PREVIEW_MAX_CHARS)}\n... (truncated)`;
    return text;
  }

  function formatPose(pose) {
    if (!pose) return 'waiting...';
    const p = pose.position || {};
    return `x ${formatNumber(p.x)} y ${formatNumber(p.y)} z ${formatNumber(p.z)}`;
  }

  function formatNumber(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number.toFixed(2) : '0.00';
  }

  function DetailsCard({ selected, details, preview }) {
    const list = (label, values) => e('div', { className: 'section' }, e('span', { className: 'label' }, label), (values || []).map((value) => e('div', { className: 'meta', key: value }, value)));
    return e('article', { className: 'card' },
      e('h2', null, 'Info / Pub-Sub'),
      selected ? e('div', null,
        e('div', { className: 'topic-row' }, e('strong', null, selected.name), e('span', { className: 'badge' }, selected.kind)),
        details?.error && e('p', { className: 'value bad' }, details.error),
        !details && !preview && e('div', { className: 'empty' }, 'Loading details...'),
        selected.kind === 'node' && details && !details.error && e(React.Fragment, null, list('Publications', details.publications), list('Subscriptions', details.subscriptions), list('Services', details.services)),
        selected.kind === 'topic' && details && !details.error && e(React.Fragment, null,
          e('p', { className: 'meta' }, `type: ${details.type || 'unknown'}`),
          list('Publishers', details.publishers),
          list('Subscribers', details.subscribers),
          e('div', { className: 'section' },
            e('span', { className: 'label' }, 'Latest message'),
            preview ? e('pre', { className: 'preview' }, formatPreview(preview)) : e('div', { className: 'meta' }, 'Waiting for a message...'),
          ),
        ),
      ) : e('div', { className: 'empty' }, 'Select a node or topic'),
    );
  }

  function UrdfPanel({ urdf, viewerApiRef, basePose, poseTopic, poseStamp, jointTopic }) {
    const viewerRef = React.useRef(null);
    const [message, setMessage] = React.useState('Waiting for robot_description...');
    const [meshNotice, setMeshNotice] = React.useState('');
    React.useEffect(() => {
      if (!urdf || !viewerRef.current) return undefined;
      if (!window.AerialRobotUrdfViewer) {
        setMessage('URDF viewer module is still loading.');
        return undefined;
      }
      let cancelled = false;
      setMeshNotice('');
      const options = {
        onMeshError: (count) => !cancelled && setMeshNotice(`${count} mesh file(s) failed to load; showing partial model.`),
      };
      window.AerialRobotUrdfViewer(viewerRef.current, urdf, options).then((viewer) => {
        if (cancelled) {
          viewer.dispose();
          return;
        }
        viewerApiRef.current = viewer;
        if (basePose) viewer.setBasePose(basePose);
        setMessage('');
      }).catch((err) => setMessage(`3D viewer unavailable: ${err.message || err}`));
      return () => {
        cancelled = true;
        viewerApiRef.current?.dispose();
        viewerApiRef.current = null;
      };
    }, [urdf, viewerApiRef]);
    React.useEffect(() => {
      viewerApiRef.current?.setBasePose(basePose);
    }, [basePose, viewerApiRef]);
    return e('section', { className: 'card section' },
      e('h2', null, 'URDF Viewer'),
      e('p', { className: 'meta' }, `Touch-drag to orbit. The model follows ${poseTopic} and live ${jointTopic}. Last pose: ${poseStamp}.`),
      e('div', { className: 'viewer' },
        e('div', { className: 'viewer-host', ref: viewerRef }),
        message && e('div', { className: 'viewer-message' }, e('div', { className: 'empty' }, message)),
      ),
      meshNotice && e('p', { className: 'meta' }, meshNotice),
    );
  }

  ReactDOM.createRoot(document.getElementById('root')).render(e(ErrorBoundary, null, e(App)));
})();
