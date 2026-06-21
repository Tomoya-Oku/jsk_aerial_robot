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
          e('h1', null, 'DRAGON Lab Aerial Robot Web Console'),
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
    const [graph, setGraph] = React.useState({ nodes: [], topics: [] });
    const [selected, setSelected] = React.useState(null);
    const [details, setDetails] = React.useState(null);
    const [preview, setPreview] = React.useState(null);
    const [urdf, setUrdf] = React.useState('');
    const [basePose, setBasePose] = React.useState(null);
    const [poseStamp, setPoseStamp] = React.useState('never');
    const [lastUpdate, setLastUpdate] = React.useState('never');
    const { connected, error, ros } = useRosConnection(bridgeUrl);
    const [tab, setTab] = React.useState('console');
    const isDracomancer = robotType === 'dracomancer';

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

    return e('main', { className: 'app' },
      e('header', { className: 'app-header' },
        e('h1', null, 'DRAGON Lab Aerial Robot Web Console'),
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
        ),
        e('button', { onClick: refresh, disabled: !connected }, 'Refresh'),
      ),
      isDracomancer && e('div', { className: 'tab-bar' },
        e('button', { className: `tab-btn${tab === 'console' ? ' active' : ''}`, onClick: () => setTab('console') }, 'Overview'),
        e('button', { className: `tab-btn${tab === 'dracomancer' ? ' active' : ''}`, onClick: () => setTab('dracomancer') }, 'Dracomancer'),
        e('button', { className: `tab-btn${tab === 'servo' ? ' active' : ''}`, onClick: () => setTab('servo') }, 'Servo Monitor'),
      ),
      isDracomancer && tab === 'dracomancer'
        ? e(DracomancerPanel, { ros, connected, robotNs, urdf, basePose, poseTopic, poseStamp })
        : isDracomancer && tab === 'servo'
          ? e(ServoMonitorTab, { ros, connected, robotNs })
          : e(React.Fragment, null,
            e('section', { className: 'cards' },
              e(GraphList, { title: 'Nodes', items: graph.nodes, kind: 'node', selected, setSelected, connected, loading: lastUpdate === 'never' }),
              e(GraphList, { title: 'Topics', items: graph.topics, kind: 'topic', selected, setSelected, connected, loading: lastUpdate === 'never' }),
              e(DetailsCard, { selected, details, preview, ros, connected }),
            ),
            e(UrdfPanel, {
              urdf, viewerApiRef, basePose, poseTopic, poseStamp,
              jointTopic: nsJoin(robotNs, 'joint_states'),
              controls: e('div', { className: 'control-stack' },
                e(RosbagControl, { ros, connected, topics: graph.topics }),
                e(FlightControl, { ros, connected, robotNs }),
              ),
            }),
          ),
    );
  }

  function InfoPill({ label, value, tone }) {
    return e('div', { className: 'pill' }, e('span', { className: 'label' }, label), e('span', { className: `value ${tone || ''}` }, value));
  }

  function GraphList({ title, items, kind, selected, setSelected, connected, loading }) {
    const [filter, setFilter] = React.useState('');
    const [collapsed, setCollapsed] = React.useState(false);
    const lowerFilter = filter.trim().toLowerCase();
    const visible = lowerFilter ? items.filter((name) => name.toLowerCase().includes(lowerFilter)) : items;
    let body;
    if (visible.length) {
      body = visible.map((name) => e('button', {
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
    return e('article', { className: `card${collapsed ? ' collapsed' : ''}` },
      e('div', { className: 'card-head list-head' },
        e('h2', null, `${title} (${lowerFilter ? `${visible.length}/${items.length}` : items.length})`),
        e('button', {
          className: 'secondary toggle-btn',
          onClick: () => setCollapsed((value) => !value),
          'aria-expanded': !collapsed,
          'aria-label': collapsed ? `Expand ${title.toLowerCase()} list` : `Collapse ${title.toLowerCase()} list`,
        }, collapsed ? '▸' : '▾'),
      ),
      !collapsed && e(React.Fragment, null,
        e('input', {
          className: 'list-filter',
          value: filter,
          onChange: (ev) => setFilter(ev.target.value),
          placeholder: `Filter ${title.toLowerCase()}...`,
        }),
        e('div', { className: 'list' }, body),
      ),
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

  function DetailsCard({ selected, details, preview, ros, connected }) {
    const [collapsed, setCollapsed] = React.useState(false);
    const list = (label, values) => e('div', { className: 'section' }, e('span', { className: 'label' }, label), (values || []).map((value) => e('div', { className: 'meta', key: value }, value)));
    return e('article', { className: `card${collapsed ? ' collapsed' : ''}` },
      e('div', { className: 'card-head list-head' },
        e('h2', null, 'Info / Pub-Sub'),
        e('button', {
          className: 'secondary toggle-btn',
          onClick: () => setCollapsed((value) => !value),
          'aria-expanded': !collapsed,
          'aria-label': collapsed ? 'Expand info panel' : 'Collapse info panel',
        }, collapsed ? '▸' : '▾'),
      ),
      !collapsed && e('div', { className: 'details-body' },
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
          e(PublishBox, { ros, connected, topic: selected.name, type: details.type }),
        ),
        ) : e('div', { className: 'empty' }, 'Select a node or topic'),
      ),
    );
  }

  const PRIMITIVE_TYPES = new Set([
    'bool', 'byte', 'char',
    'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
    'float32', 'float64', 'string', 'time', 'duration',
  ]);

  function primitiveTemplate(type) {
    if (type === 'string') return '';
    if (type === 'bool') return false;
    if (type === 'time' || type === 'duration') return { secs: 0, nsecs: 0 };
    return 0;
  }

  // Build an empty message skeleton from rosapi/MessageDetails typedefs so the
  // publish box always starts from a structure that matches the topic type.
  function buildMessageTemplate(typedefs, rootType) {
    const byType = new Map((typedefs || []).map((def) => [def.type, def]));
    const build = (type, depth) => {
      if (PRIMITIVE_TYPES.has(type)) return primitiveTemplate(type);
      const def = byType.get(type);
      if (!def || depth > 12) return {};
      const message = {};
      (def.fieldnames || []).forEach((name, index) => {
        const fieldType = def.fieldtypes?.[index] || 'string';
        const arrayLen = def.fieldarraylen?.[index] ?? -1;
        if (arrayLen < 0) {
          message[name] = build(fieldType, depth + 1);
        } else if (arrayLen === 0) {
          message[name] = [];
        } else {
          message[name] = Array.from({ length: arrayLen }, () => build(fieldType, depth + 1));
        }
      });
      return message;
    };
    return build(rootType, 0);
  }

  function PublishBox({ ros, connected, topic, type }) {
    const [text, setText] = React.useState('');
    const [status, setStatus] = React.useState(null);
    const publisherRef = React.useRef(null);

    React.useEffect(() => {
      setStatus(null);
      setText('');
      if (!ros || !connected || !type || !window.ROSLIB) return undefined;
      let alive = true;
      callService(ros, '/rosapi/message_details', 'rosapi/MessageDetails', { type })
        .then((res) => {
          if (!alive) return;
          setText(JSON.stringify(buildMessageTemplate(res.typedefs, type), null, 2));
        })
        .catch(() => alive && setText('{}'));
      const publisher = new window.ROSLIB.Topic({ ros, name: topic, messageType: type });
      publisherRef.current = publisher;
      return () => {
        alive = false;
        publisherRef.current = null;
        try {
          publisher.unadvertise();
        } catch (error) {
          console.warn(error);
        }
      };
    }, [ros, connected, topic, type]);

    const publish = () => {
      let payload;
      try {
        payload = JSON.parse(text);
      } catch (error) {
        setStatus({ tone: 'bad', text: `Invalid JSON: ${error.message}` });
        return;
      }
      try {
        publisherRef.current?.publish(new window.ROSLIB.Message(payload));
        setStatus({ tone: 'ok', text: `Published at ${new Date().toLocaleTimeString()}` });
      } catch (error) {
        setStatus({ tone: 'bad', text: describeError(error, 'publish failed') });
      }
    };

    return e('div', { className: 'section publish-box' },
      e('span', { className: 'label' }, `Publish (${type || 'unknown type'})`),
      e('textarea', {
        className: 'publish-input',
        value: text,
        rows: 8,
        spellCheck: false,
        onChange: (ev) => setText(ev.target.value),
        placeholder: 'Loading message template...',
      }),
      e('div', { className: 'publish-actions' },
        e('button', { onClick: publish, disabled: !connected || !type || !text }, 'Publish'),
        status && e('span', { className: `value ${status.tone}` }, status.text),
      ),
    );
  }

  // Talks to the RosbagRecorder in aerial_robot_web_server.py over the
  // /aerial_robot_web/rosbag/{start,stop,status} topics.
  const ROSBAG_NS = '/aerial_robot_web/rosbag';

  function publishOnce(ros, name, type, payload) {
    const topic = new window.ROSLIB.Topic({ ros, name, messageType: type });
    topic.publish(new window.ROSLIB.Message(payload));
  }

  function loadRosbagSettings() {
    try {
      return JSON.parse(window.localStorage.getItem('rosbagSettings')) || {};
    } catch (ignore) {
      return {};
    }
  }

  function RosbagControl({ ros, connected, topics }) {
    const [status, setStatus] = React.useState(null);
    const [open, setOpen] = React.useState(false);
    const [recordAll, setRecordAll] = React.useState(() => loadRosbagSettings().all !== false);
    const [picked, setPicked] = React.useState(() => new Set(loadRosbagSettings().topics || []));
    const [bagDir, setBagDir] = React.useState(() => loadRosbagSettings().bagDir || '');
    const [pickFilter, setPickFilter] = React.useState('');

    React.useEffect(() => {
      try {
        window.localStorage.setItem('rosbagSettings', JSON.stringify({ all: recordAll, topics: [...picked], bagDir }));
      } catch (ignore) { /* private mode etc.; settings just stop persisting */ }
    }, [recordAll, picked, bagDir]);

    React.useEffect(() => {
      setStatus(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const sub = new window.ROSLIB.Topic({ ros, name: `${ROSBAG_NS}/status`, messageType: 'std_msgs/String' });
      sub.subscribe((msg) => {
        try {
          setStatus(JSON.parse(msg.data));
        } catch (ignore) {
          setStatus(null);
        }
      });
      return () => sub.unsubscribe();
    }, [ros, connected]);

    const recording = Boolean(status?.recording);
    const start = () => publishOnce(ros, `${ROSBAG_NS}/start`, 'std_msgs/String', {
      data: JSON.stringify({ all: recordAll, topics: recordAll ? [] : [...picked], bag_dir: bagDir.trim() }),
    });
    const stop = () => publishOnce(ros, `${ROSBAG_NS}/stop`, 'std_msgs/Empty', {});

    let statusText = recordAll ? 'ready (all topics)' : `ready (${picked.size} topic(s))`;
    if (status?.error) statusText = status.error;
    else if (recording) statusText = `recording ${status.all ? 'all topics' : `${status.topics.length} topic(s)`}`;
    else if (status?.bag_path) statusText = `saved: ${status.bag_path.split('/').pop()}`;

    const lowerPickFilter = pickFilter.trim().toLowerCase();
    const pickable = lowerPickFilter ? topics.filter((name) => name.toLowerCase().includes(lowerPickFilter)) : topics;
    const togglePick = (name) => setPicked((prev) => {
      const next = new Set(prev);
      if (next.has(name)) next.delete(name);
      else next.add(name);
      return next;
    });

    return e('div', { className: 'flight-control' },
      e('span', { className: 'label' }, 'Rosbag'),
      e('div', { className: 'fc-row rosbag-row' },
        recording
          ? e('button', { className: 'fc-btn danger', onClick: stop }, '■ Stop recording')
          : e('button', {
            className: 'fc-btn',
            onClick: start,
            disabled: !connected || (!recordAll && !picked.size),
          }, '● Record rosbag'),
        e('button', {
          className: 'secondary fc-btn gear-btn',
          onClick: () => setOpen(true),
          'aria-label': 'Rosbag settings',
        }, '⚙'),
      ),
      e('p', { className: `meta fc-status${status?.error ? ' bad' : ''}` }, statusText),
      open && e('div', { className: 'modal-overlay', onClick: () => setOpen(false) },
        e('div', { className: 'modal', onClick: (ev) => ev.stopPropagation() },
          e('h3', null, 'Rosbag settings'),
          e('label', { className: 'field modal-field' },
            e('span', { className: 'label' }, 'Bag folder (on the robot)'),
            e('input', {
              className: 'list-filter',
              value: bagDir,
              onChange: (ev) => setBagDir(ev.target.value),
              placeholder: 'server default (~/rosbags)',
            }),
          ),
          e('label', { className: 'check-row all-row' },
            e('input', { type: 'checkbox', checked: recordAll, onChange: (ev) => setRecordAll(ev.target.checked) }),
            e('span', null, 'Record all topics (rosbag record -a)'),
          ),
          !recordAll && e(React.Fragment, null,
            e('input', {
              className: 'list-filter',
              value: pickFilter,
              onChange: (ev) => setPickFilter(ev.target.value),
              placeholder: 'Filter topics...',
            }),
            e('div', { className: 'modal-tools' },
              e('button', { className: 'secondary', onClick: () => setPicked(new Set([...picked, ...pickable])) }, 'Select shown'),
              e('button', { className: 'secondary', onClick: () => setPicked(new Set()) }, 'Clear'),
            ),
            e('div', { className: 'modal-list' },
              pickable.length
                ? pickable.map((name) => e('label', { className: 'check-row', key: name },
                  e('input', { type: 'checkbox', checked: picked.has(name), onChange: () => togglePick(name) }),
                  e('span', null, name)))
                : e('div', { className: 'empty' }, 'No matching topics'),
            ),
          ),
          e('div', { className: 'modal-actions' },
            e('button', { onClick: () => setOpen(false) }, recordAll ? 'Done (all topics)' : `Done (${picked.size} selected)`),
          ),
        ),
      ),
    );
  }

  // Mirrors aerial_robot_base/scripts/keyboard_command.py: Empty teleop
  // commands plus FlightNav velocity nudges.
  const FLIGHT_NAV = { VEL_MODE: 1, WORLD_FRAME: 0, COG: 1 };
  const TELEOP_XY_VEL = 0.2;
  const TELEOP_Z_VEL = 0.2;
  const TELEOP_YAW_VEL = 0.2;

  function FlightControl({ ros, connected, robotNs }) {
    const [status, setStatus] = React.useState('');
    const send = (topicName, type, payload, label) => {
      if (!ros || !connected || !window.ROSLIB) return;
      try {
        const topic = new window.ROSLIB.Topic({ ros, name: topicName, messageType: type });
        topic.publish(new window.ROSLIB.Message(payload));
        setStatus(`sent ${label}`);
      } catch (error) {
        setStatus(describeError(error, 'publish failed'));
      }
    };
    const teleop = (command, label) => send(nsJoin(robotNs, `teleop_command/${command}`), 'std_msgs/Empty', {}, label);
    const nav = (fields, label) => send(nsJoin(robotNs, 'uav/nav'), 'aerial_robot_msgs/FlightNav', {
      control_frame: FLIGHT_NAV.WORLD_FRAME,
      target: FLIGHT_NAV.COG,
      ...fields,
    }, label);
    const btn = (label, onClick, extra) => e('button', {
      className: `fc-btn${extra ? ` ${extra}` : ''}`,
      onClick,
      disabled: !connected,
    }, label);
    return e('div', { className: 'flight-control' },
      e('span', { className: 'label' }, 'Flight Control'),
      e('div', { className: 'fc-row fc-row-2' },
        btn('Arm', () => teleop('start', 'arm')),
        btn('Takeoff', () => teleop('takeoff', 'takeoff')),
      ),
      e('div', { className: 'fc-row fc-row-3' },
        btn('↺ Yaw', () => nav({ yaw_nav_mode: FLIGHT_NAV.VEL_MODE, target_omega_z: TELEOP_YAW_VEL }, '+yaw vel')),
        btn('↑ Fwd', () => nav({ pos_xy_nav_mode: FLIGHT_NAV.VEL_MODE, target_vel_x: TELEOP_XY_VEL }, '+x vel')),
        btn('↻ Yaw', () => nav({ yaw_nav_mode: FLIGHT_NAV.VEL_MODE, target_omega_z: -TELEOP_YAW_VEL }, '-yaw vel')),
      ),
      e('div', { className: 'fc-row fc-row-3' },
        btn('← Left', () => nav({ pos_xy_nav_mode: FLIGHT_NAV.VEL_MODE, target_vel_y: TELEOP_XY_VEL }, '+y vel')),
        btn('↓ Back', () => nav({ pos_xy_nav_mode: FLIGHT_NAV.VEL_MODE, target_vel_x: -TELEOP_XY_VEL }, '-x vel')),
        btn('→ Right', () => nav({ pos_xy_nav_mode: FLIGHT_NAV.VEL_MODE, target_vel_y: -TELEOP_XY_VEL }, '-y vel')),
      ),
      e('div', { className: 'fc-row fc-row-2' },
        btn('▲ Up', () => nav({ pos_z_nav_mode: FLIGHT_NAV.VEL_MODE, target_vel_z: TELEOP_Z_VEL }, '+z vel')),
        btn('▼ Down', () => nav({ pos_z_nav_mode: FLIGHT_NAV.VEL_MODE, target_vel_z: -TELEOP_Z_VEL }, '-z vel')),
      ),
      e('div', { className: 'fc-row fc-row-3' },
        btn('Land', () => teleop('land', 'land')),
        btn('F.Land', () => teleop('force_landing', 'force landing'), 'danger'),
        btn('Halt', () => teleop('halt', 'halt'), 'danger'),
      ),
      e('p', { className: 'meta fc-status' }, status || `vel step: ${TELEOP_XY_VEL} m/s, ${TELEOP_YAW_VEL} rad/s`),
    );
  }

  function UrdfPanel({ urdf, viewerApiRef, basePose, poseTopic, poseStamp, jointTopic, controls }) {
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
      e('h2', null, 'Live Robot Model (URDF + Odometry)'),
      e('p', { className: 'meta' }, `Touch-drag to orbit. The model follows ${poseTopic} and live ${jointTopic}. Last pose: ${poseStamp}.`),
      e('div', { className: 'viewer-row' },
        e('div', { className: 'viewer' },
          e('div', { className: 'viewer-host', ref: viewerRef }),
          message && e('div', { className: 'viewer-message' }, e('div', { className: 'empty' }, message)),
        ),
        controls,
      ),
      meshNotice && e('p', { className: 'meta' }, meshNotice),
    );
  }

  // ---- Dracomancer tab ----

  const DRACOMANCER_JOINT_NAMES = [
    'shoulder_abduction_adduction_joint',
    'shoulder_flexion_extension_joint',
    'upper_arm_external_internal_rotation_joint',
    'elbow_flexion_extension_joint',
    'wrist_supination_joint',
    'wrist_flexion_extension_joint',
    'wrist_abduction_adduction_joint',
  ];

  const DRACOMANCER_JOINT_LABELS = {
    shoulder_abduction_adduction_joint: 'Shoulder Abd/Add',
    shoulder_flexion_extension_joint: 'Shoulder Flex/Ext',
    upper_arm_external_internal_rotation_joint: 'Upper Arm Rot',
    elbow_flexion_extension_joint: 'Elbow Flex/Ext',
    wrist_supination_joint: 'Wrist Supination',
    wrist_flexion_extension_joint: 'Wrist Flex/Ext',
    wrist_abduction_adduction_joint: 'Wrist Abd/Add',
  };

  const JOINT_LIMIT_RAD = Math.PI / 2;

  function JointControlCard({ connected, deviceNs, names, positions, onSetJoint, onResetAll }) {
    return e('div', { className: 'flight-control' },
      e('span', { className: 'label' }, 'Joint Control'),
      e('p', { className: 'meta' }, `→ ${nsJoin(deviceNs, 'joint_cmd')}`),
      e('div', { className: 'fc-row' },
        e('button', { className: 'secondary', onClick: onResetAll, disabled: !connected }, 'Reset All to 0°'),
      ),
      e('div', { className: 'drac-joints', style: { marginTop: '10px' } },
        names.map((name, i) => {
          const val = positions[i] ?? 0;
          const label = DRACOMANCER_JOINT_LABELS[name] || name;
          return e('div', { className: 'drac-joint-row', key: name },
            e('div', { className: 'drac-joint-head' },
              e('span', { className: 'drac-joint-name' }, label),
              e('span', { className: 'meta' }, `${(val * 180 / Math.PI).toFixed(1)}°`),
            ),
            e('input', {
              type: 'range',
              min: -JOINT_LIMIT_RAD,
              max: JOINT_LIMIT_RAD,
              step: 0.01,
              value: val,
              disabled: !connected,
              onChange: (ev) => onSetJoint(i, Number(ev.target.value)),
            }),
          );
        }),
      ),
    );
  }

  function DracomancerPanel({ ros, connected, robotNs, urdf, basePose, poseTopic, poseStamp }) {
    const deviceNs = robotNs;
    const dracomancerViewerApiRef = React.useRef(null);
    const [dragonNs, setDragonNs] = React.useState('/dragon');
    const [flightState, setFlightState] = React.useState(null);
    const [forceRadius, setForceRadius] = React.useState(null);
    const [torqueRadius, setTorqueRadius] = React.useState(null);
    const [safetyScale, setSafetyScale] = React.useState(null);
    const [joyAxes, setJoyAxes] = React.useState(null);
    const [deviceJoints, setDeviceJoints] = React.useState(null);
    const [dragonJoints, setDragonJoints] = React.useState(null);
    const [recaptureStatus, setRecaptureStatus] = React.useState('');
    const [jointPositions, setJointPositions] = React.useState(() => new Array(DRACOMANCER_JOINT_NAMES.length).fill(0));
    const jointPubRef = React.useRef(null);

    React.useEffect(() => {
      setFlightState(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(dragonNs, 'flight_state'),
        messageType: 'std_msgs/UInt8', throttle_rate: 250, queue_length: 1,
      });
      topic.subscribe((msg) => setFlightState(msg.data));
      return () => topic.unsubscribe();
    }, [ros, connected, dragonNs]);

    // Inradius comes straight from the controlled robot (dragon); dracomancer
    // does not republish it. Only the dracomancer-computed safety scale is taken
    // from the device namespace.
    React.useEffect(() => {
      setForceRadius(null);
      setTorqueRadius(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const forceTopic = new window.ROSLIB.Topic({
        ros, name: nsJoin(dragonNs, 'debug/fc_f_min'),
        messageType: 'std_msgs/Float64', throttle_rate: 100, queue_length: 1,
      });
      const torqueTopic = new window.ROSLIB.Topic({
        ros, name: nsJoin(dragonNs, 'debug/fc_t_min'),
        messageType: 'std_msgs/Float64', throttle_rate: 100, queue_length: 1,
      });
      forceTopic.subscribe((msg) => setForceRadius(Number(msg.data)));
      torqueTopic.subscribe((msg) => setTorqueRadius(Number(msg.data)));
      return () => { forceTopic.unsubscribe(); torqueTopic.unsubscribe(); };
    }, [ros, connected, dragonNs]);

    React.useEffect(() => {
      setSafetyScale(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(deviceNs, 'dragon_shape_safety_scale'),
        messageType: 'std_msgs/Float64', throttle_rate: 100, queue_length: 1,
      });
      topic.subscribe((msg) => setSafetyScale(Number(msg.data)));
      return () => topic.unsubscribe();
    }, [ros, connected, deviceNs]);

    React.useEffect(() => {
      setJoyAxes(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(deviceNs, 'joystick/calibrated'),
        messageType: 'std_msgs/Float32MultiArray', throttle_rate: 100, queue_length: 1,
      });
      topic.subscribe((msg) => setJoyAxes(msg.data || []));
      return () => topic.unsubscribe();
    }, [ros, connected, deviceNs]);

    React.useEffect(() => {
      setDeviceJoints(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(deviceNs, 'joint_states'),
        messageType: 'sensor_msgs/JointState', throttle_rate: 100, queue_length: 1,
      });
      topic.subscribe((msg) => {
        setDeviceJoints(msg);
        const values = {};
        (msg.name || []).forEach((name, i) => { values[name] = msg.position?.[i] ?? 0; });
        dracomancerViewerApiRef.current?.setJointValues(values);
      });
      return () => topic.unsubscribe();
    }, [ros, connected, deviceNs]);

    React.useEffect(() => {
      setDragonJoints(null);
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(dragonNs, 'joint_states'),
        messageType: 'sensor_msgs/JointState', throttle_rate: 100, queue_length: 1,
      });
      topic.subscribe((msg) => setDragonJoints(msg));
      return () => topic.unsubscribe();
    }, [ros, connected, dragonNs]);

    const jointNames = deviceJoints && (deviceJoints.name || []).length > 0
      ? deviceJoints.name
      : DRACOMANCER_JOINT_NAMES;

    React.useEffect(() => {
      setJointPositions((prev) => prev.length === jointNames.length ? prev : new Array(jointNames.length).fill(0));
    }, [jointNames.length]);

    React.useEffect(() => {
      if (!ros || !connected || !window.ROSLIB) { jointPubRef.current = null; return undefined; }
      const pub = new window.ROSLIB.Topic({ ros, name: nsJoin(deviceNs, 'joint_cmd'), messageType: 'sensor_msgs/JointState' });
      jointPubRef.current = pub;
      return () => { jointPubRef.current = null; try { pub.unadvertise(); } catch (err) { console.warn(err); } };
    }, [ros, connected, deviceNs]);

    const publishJointCmd = (pos) => {
      if (!jointPubRef.current || !window.ROSLIB) return;
      jointPubRef.current.publish(new window.ROSLIB.Message({
        header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
        name: jointNames,
        position: pos,
        velocity: [],
        effort: [],
      }));
    };

    const setJointAt = (i, val) => {
      setJointPositions((prev) => {
        const next = prev.slice();
        next[i] = val;
        publishJointCmd(next);
        return next;
      });
    };

    const resetJoints = () => {
      const zeros = new Array(jointNames.length).fill(0);
      setJointPositions(zeros);
      publishJointCmd(zeros);
    };

    const recaptureNeutral = () => {
      if (!ros || !connected || !window.ROSLIB) return;
      const ns = deviceNs.replace(/^\//, '');
      try {
        publishOnce(ros, `/${ns}_control_pose/recapture_neutral`, 'std_msgs/Empty', {});
        setRecaptureStatus(`sent at ${new Date().toLocaleTimeString()}`);
      } catch (err) {
        setRecaptureStatus(describeError(err, 'publish failed'));
      }
    };

    const isHovering = flightState != null && flightState >= 4;
    const isLanding = flightState === 6;
    const stateText = flightState == null ? '—'
      : isLanding ? `${flightState} (landing)`
      : isHovering ? `${flightState} (hovering)`
      : `${flightState} (grounded)`;

    const safetyCls = safetyScale == null ? '' : safetyScale >= 0.7 ? 'ok' : safetyScale >= 0.3 ? 'warn' : 'bad';

    const jointRow = (name, pos, maxRad) => {
      const clamped = Math.max(-maxRad, Math.min(maxRad, pos));
      const pct = Math.abs(clamped) / maxRad * 50;
      const fill = clamped >= 0
        ? e('div', { className: 'gauge-fill', style: { left: '50%', width: `${pct.toFixed(1)}%` } })
        : e('div', { className: 'gauge-fill', style: { left: `${(50 - pct).toFixed(1)}%`, width: `${pct.toFixed(1)}%` } });
      return e('div', { className: 'drac-joint-row', key: name },
        e('div', { className: 'drac-joint-head' },
          e('span', { className: 'drac-joint-name' }, name),
          e('span', { className: 'meta' }, `${(pos * 180 / Math.PI).toFixed(1)}°`),
        ),
        e('div', { className: 'gauge-track bipolar' }, fill),
      );
    };

    return e('div', { className: 'drac-panel' },
      e('div', { className: 'drac-ns-bar' },
        e('label', { className: 'field drac-ns-field' },
          e('span', { className: 'label' }, 'Dragon Namespace'),
          e('input', {
            value: dragonNs,
            onChange: (ev) => setDragonNs(normalizeNs(ev.target.value)),
            placeholder: '/dragon',
          }),
        ),
      ),
      e('div', { className: 'drac-cards' },
        e('article', { className: 'card' },
          e('h2', null, 'Dragon Status'),
          e('div', { className: 'drac-metrics' },
            e('div', { className: 'drac-metric' },
              e('span', { className: 'label' }, 'Flight State'),
              e('span', { className: `value ${isHovering ? 'ok' : 'warn'}` }, stateText),
            ),
            e('div', { className: 'drac-metric' },
              e('span', { className: 'label' }, 'Safety Scale'),
              e('span', { className: `value ${safetyCls}` }, safetyScale != null ? `${(safetyScale * 100).toFixed(0)}%` : '—'),
            ),
            e('div', { className: 'drac-metric' },
              e('span', { className: 'label' }, 'Force Inradius'),
              e('span', { className: 'value' }, forceRadius != null ? formatNumber(forceRadius) : '—'),
            ),
            e('div', { className: 'drac-metric' },
              e('span', { className: 'label' }, 'Torque Inradius'),
              e('span', { className: 'value' }, torqueRadius != null ? formatNumber(torqueRadius) : '—'),
            ),
          ),
          safetyScale != null && e('div', { className: 'drac-gauge-row' },
            e('span', { className: 'label' }, 'Shape Safety'),
            e('div', { className: 'gauge-track' },
              e('div', { className: `gauge-fill ${safetyCls}`, style: { width: `${(safetyScale * 100).toFixed(1)}%` } }),
            ),
          ),
        ),
        e('article', { className: 'card' },
          e('h2', null, 'Joystick'),
          joyAxes
            ? e('div', { className: 'drac-axes' },
                ['X (fwd/back)', 'Y (left/right)', 'Z (up/down)'].map((label, i) => {
                  const val = Number(joyAxes[i] ?? 0);
                  const pct = Math.abs(val) * 50;
                  const fill = val >= 0
                    ? e('div', { className: 'gauge-fill', style: { left: '50%', width: `${pct.toFixed(1)}%` } })
                    : e('div', { className: 'gauge-fill', style: { left: `${(50 - pct).toFixed(1)}%`, width: `${pct.toFixed(1)}%` } });
                  return e('div', { className: 'drac-axis-row', key: label },
                    e('div', { className: 'drac-axis-head' },
                      e('span', { className: 'label' }, label),
                      e('span', { className: 'meta' }, formatNumber(val)),
                    ),
                    e('div', { className: 'gauge-track bipolar' }, fill),
                  );
                }),
              )
            : e('div', { className: 'empty' }, `Waiting for ${nsJoin(deviceNs, 'joystick/calibrated')}...`),
        ),
      ),
      e(UrdfPanel, {
        urdf,
        viewerApiRef: dracomancerViewerApiRef,
        basePose,
        poseTopic,
        poseStamp,
        jointTopic: nsJoin(deviceNs, 'joint_states'),
        controls: e('div', { className: 'control-stack' },
          e(JointControlCard, { connected, deviceNs, names: jointNames, positions: jointPositions, onSetJoint: setJointAt, onResetAll: resetJoints }),
        ),
      }),
      e('article', { className: 'card drac-action-card' },
        e('h2', null, 'IMU Heading Recapture'),
        e('p', { className: 'meta' }, `Resets the neutral heading reference for IMU-relative motion (direction_mode). Publishes to /${deviceNs.replace(/^\//, '')}_control_pose/recapture_neutral.`),
        e('div', null,
          e('button', { onClick: recaptureNeutral, disabled: !connected }, 'Recapture Neutral Heading'),
        ),
        recaptureStatus && e('p', { className: 'meta fc-status' }, recaptureStatus),
      ),
      e('article', { className: 'card' },
        e('h2', null, `Device Joints (${deviceNs})`),
        deviceJoints && (deviceJoints.name || []).length > 0
          ? e('div', { className: 'drac-joints' },
              (deviceJoints.name || []).map((name, i) =>
                jointRow(name, (deviceJoints.position || [])[i] ?? 0, Math.PI / 2),
              ),
            )
          : e('div', { className: 'empty' }, `Waiting for ${nsJoin(deviceNs, 'joint_states')}...`),
      ),
      e('article', { className: 'card' },
        e('h2', null, `Dragon Joints (${dragonNs})`),
        dragonJoints && (dragonJoints.name || []).length > 0
          ? e('div', { className: 'drac-joints' },
              (dragonJoints.name || []).map((name, i) =>
                jointRow(name, (dragonJoints.position || [])[i] ?? 0, Math.PI / 2),
              ),
            )
          : e('div', { className: 'empty' }, `Waiting for ${nsJoin(dragonNs, 'joint_states')}...`),
      ),
    );
  }

  // ---- Servo Monitor tab ----

  function ServoMonitorTab({ ros, connected, robotNs }) {
    const [servoNs, setServoNs] = React.useState(robotNs || '');
    const [boardData, setBoardData] = React.useState([]);
    const [servoStates, setServoStates] = React.useState({});
    const [torqueStates, setTorqueStates] = React.useState({});
    const [fetchStatus, setFetchStatus] = React.useState('');
    const torquePubRef = React.useRef(null);

    React.useEffect(() => {
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(servoNs, 'servo/states'),
        messageType: 'spinal/ServoStates', throttle_rate: 200, queue_length: 1,
      });
      topic.subscribe((msg) => {
        setServoStates((prev) => {
          const next = { ...prev };
          (msg.servos || []).forEach((s) => { next[s.index] = { angle: s.angle, temp: s.temp, load: s.load, error: s.error }; });
          return next;
        });
      });
      return () => topic.unsubscribe();
    }, [ros, connected, servoNs]);

    React.useEffect(() => {
      if (!ros || !connected || !window.ROSLIB) return undefined;
      const topic = new window.ROSLIB.Topic({
        ros, name: nsJoin(servoNs, 'servo/torque_states'),
        messageType: 'spinal/ServoTorqueStates', throttle_rate: 500, queue_length: 1,
      });
      topic.subscribe((msg) => {
        const next = {};
        (msg.torque_enable || []).forEach((v, i) => { next[i] = !!v; });
        setTorqueStates(next);
      });
      return () => topic.unsubscribe();
    }, [ros, connected, servoNs]);

    React.useEffect(() => {
      if (!ros || !connected || !window.ROSLIB) { torquePubRef.current = null; return undefined; }
      const pub = new window.ROSLIB.Topic({ ros, name: nsJoin(servoNs, 'servo/torque_enable'), messageType: 'spinal/ServoTorqueCmd' });
      torquePubRef.current = pub;
      return () => { torquePubRef.current = null; try { pub.unadvertise(); } catch (err) { console.warn(err); } };
    }, [ros, connected, servoNs]);

    const fetchBoardInfo = React.useCallback(() => {
      if (!ros || !connected) { setFetchStatus('Not connected'); return; }
      setFetchStatus('Fetching...');
      callService(ros, nsJoin(servoNs, 'get_board_info'), 'spinal/GetBoardInfo', {})
        .then((res) => {
          let idx = 0;
          const rows = [];
          (res.boards || []).forEach((b) => {
            (b.servos || []).forEach((s, i) => {
              rows.push({ index: idx++, boardId: b.slave_id, servoIdx: i, servoId: s.id, pGain: s.p_gain, iGain: s.i_gain, dGain: s.d_gain, profileVelocity: s.profile_velocity, currentLimit: s.current_limit, sendDataFlag: !!s.send_data_flag });
            });
          });
          setBoardData(rows);
          setFetchStatus(`Updated ${new Date().toLocaleTimeString()}`);
        })
        .catch((err) => setFetchStatus(describeError(err, 'get_board_info failed')));
    }, [ros, connected, servoNs]);

    React.useEffect(() => { if (connected) fetchBoardInfo(); }, [connected]);

    const publishTorque = (indices, enable) => {
      if (!torquePubRef.current || !window.ROSLIB) return;
      torquePubRef.current.publish(new window.ROSLIB.Message({
        index: indices.map(Number),
        torque_enable: indices.map(() => enable ? 1 : 0),
      }));
    };

    const decodeError = (error) => {
      if (!error) return '—';
      const flags = [[0x80,'Enc.Conn'],[0x40,'Resol'],[0x20,'Overload'],[0x10,'Elec.Shock'],[0x08,'MotorEnc'],[0x04,'Overheat'],[0x02,'PulleySkip'],[0x01,'VoltIn']];
      const errs = flags.filter(([m]) => error & m).map(([,n]) => n);
      return errs.length ? errs.join(' | ') : '—';
    };

    const COLS = ['Torque','Index','Board','Srv.Idx','ID','Angle (tick)','Temp (°C)','Load','Error','P / I / D','Prof.Vel','Cur.Lim','Send Flag'];

    return e('div', { className: 'servo-panel' },
      e('div', { className: 'servo-toolbar' },
        e('label', { className: 'field' },
          e('span', { className: 'label' }, 'Servo Namespace'),
          e('input', { value: servoNs, onChange: (ev) => setServoNs(normalizeNs(ev.target.value)), placeholder: robotNs }),
        ),
        e('button', { onClick: fetchBoardInfo, disabled: !connected }, 'Update Board Info'),
        e('button', { onClick: () => publishTorque(boardData.map((r) => r.index), true), disabled: !connected || !boardData.length }, 'All Torque ON'),
        e('button', { onClick: () => publishTorque(boardData.map((r) => r.index), false), disabled: !connected || !boardData.length }, 'All Torque OFF'),
        fetchStatus && e('span', { className: 'meta' }, fetchStatus),
      ),
      boardData.length === 0
        ? e('div', { className: 'empty' }, connected ? 'No board data — click "Update Board Info" (requires rm:=true and servo bridge running).' : 'Waiting for rosbridge connection...')
        : e('div', { className: 'servo-table-wrap' },
            e('table', { className: 'servo-table' },
              e('thead', null, e('tr', null, COLS.map((h) => e('th', { key: h }, h)))),
              e('tbody', null,
                boardData.map((row) => {
                  const st = servoStates[row.index] || {};
                  const torque = torqueStates[row.index];
                  const hasError = st.error != null && st.error !== 0;
                  const hot = st.temp != null && st.temp > 60;
                  return e('tr', { key: row.index, className: hasError ? 'servo-row-err' : '' },
                    e('td', null,
                      e('button', {
                        className: `servo-trq-btn${torque === true ? ' on' : torque === false ? ' off' : ''}`,
                        onClick: () => publishTorque([row.index], !torque),
                        disabled: !connected,
                      }, torque == null ? '?' : torque ? 'ON' : 'OFF'),
                    ),
                    e('td', null, row.index),
                    e('td', null, row.boardId),
                    e('td', null, row.servoIdx),
                    e('td', null, row.servoId),
                    e('td', null, st.angle != null ? st.angle : '—'),
                    e('td', { className: hot ? 'warn' : '' }, st.temp != null ? st.temp : '—'),
                    e('td', null, st.load != null ? st.load : '—'),
                    e('td', { className: hasError ? 'bad' : '' }, decodeError(st.error)),
                    e('td', { className: 'meta' }, `${row.pGain} / ${row.iGain} / ${row.dGain}`),
                    e('td', null, row.profileVelocity),
                    e('td', null, row.currentLimit),
                    e('td', null, String(row.sendDataFlag)),
                  );
                }),
              ),
            ),
          ),
    );
  }

  ReactDOM.createRoot(document.getElementById('root')).render(e(ErrorBoundary, null, e(App)));
})();
