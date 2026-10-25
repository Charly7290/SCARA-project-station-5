const rclnodejs = require('rclnodejs');
const mqtt = require('mqtt');

const MQTT_URL   = 'mqtt://localhost:1883';
const MQTT_USER  = 'scara';
const MQTT_PASS  = 'KMComp111805.';
const MQTT_TOPIC = 'scara/telemetry';
const ROS_TOPIC  = 'scara_telemetry';

/*
It sends a value only when it changes by more than its deadband, 
plus a periodic heartbeat. That keeps the database from filling with 
millions of rows. It drops nan values and the 180 marker from defineAngles.
*/

// Same order as the CSV from the Pico. deadband = minimum change to send,
// heartbeat = resend at least this often (ms) so Grafana has no gaps.
const VARS = [
  { id: 1, name: 'state',      deadband: 0.5,  heartbeat: 10000 },
  { id: 2, name: 'pcbs',       deadband: 0.5,  heartbeat: 10000 },
  { id: 3, name: 'm1_angle',   deadband: 0.2,  heartbeat: 1000 },
  { id: 4, name: 'm2_angle',   deadband: 0.2,  heartbeat: 1000 },
  { id: 5, name: 'theta1',     deadband: 0.01, heartbeat: 10000 },
  { id: 6, name: 'theta2',     deadband: 0.01, heartbeat: 10000 },
  { id: 7, name: 'theta_prism', deadband: 0.01, heartbeat: 10000 },
];
const last = {};   // id -> { value, t }

const client = mqtt.connect(MQTT_URL, { username: MQTT_USER, password: MQTT_PASS });
client.on('connect', () => console.log('MQTT connected'));
client.on('error', (e) => console.error('MQTT error:', e.message));

function shouldSend(v, value, now) {
  const prev = last[v.id];
  return !prev || Math.abs(value - prev.value) >= v.deadband || now - prev.t >= v.heartbeat;
}

function handleTelemetry(msg) {
  if (!client.connected) return;
  const parts = msg.data.split(',');
  if (parts.length !== VARS.length) {
    console.warn('Bad telemetry line:', msg.data);
    return;
  }
  const now = Date.now();
  VARS.forEach((v, i) => {
    const value = parseFloat(parts[i]);
    if (!Number.isFinite(value)) return;                       // nan / inf
    if ((v.id === 5 || v.id === 6) && value === 180) return;   // IK "out of limits" marker
    if (!shouldSend(v, value, now)) return;
    last[v.id] = { value, t: now };
    client.publish(MQTT_TOPIC, JSON.stringify({ numeric_value: value, source_id: v.id }));
  });
}

async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('scara_mqtt_bridge');
  node.createSubscription('std_msgs/msg/String', ROS_TOPIC,
    { qos: rclnodejs.QoS.profileSensorData }, handleTelemetry);   // best effort, matches the Pico
  rclnodejs.spin(node);
  console.log(`Bridge running: /${ROS_TOPIC} -> ${MQTT_TOPIC}`);
}
main().catch(console.error);