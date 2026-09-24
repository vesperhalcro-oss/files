const canvas = document.querySelector("#lidar-map");
const context = canvas.getContext("2d");
const toggleButton = document.querySelector("#toggle-simulation");
const resetButton = document.querySelector("#reset-simulation");
const speedInput = document.querySelector("#simulation-speed");
const speedValue = document.querySelector("#speed-value");
const statusValue = document.querySelector("#telemetry-status");
const frameValue = document.querySelector("#telemetry-frame");
const positionValue = document.querySelector("#telemetry-position");
const yawValue = document.querySelector("#telemetry-yaw");
const pointsValue = document.querySelector("#telemetry-points");
const ids = (prefix) => Object.fromEntries(["drivable", "nondrivable", "static", "dynamic", "near", "mid", "far", "cells", "fps", "latency", "point-rate", "queue"].map((name) => [name, document.querySelector(`#${prefix}-${name}`)]));
const semanticValues = ids("semantic");
const zoneValues = ids("zone");
const metricValues = ids("metric");
const cellsValue = document.querySelector("#telemetry-cells");

const state = {
  running: true,
  speed: 1,
  frame: 0,
  time: 0,
  pose: { x: 0, y: 0, yaw: 0 },
  trail: [],
  lastFrameTime: 0,
  fps: 0,
  latency: 0,
  frameInterval: 16.7,
  queue: 0,
};

function resizeCanvas() {
  const ratio = window.devicePixelRatio || 1;
  const bounds = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(bounds.width * ratio));
  canvas.height = Math.max(1, Math.floor(bounds.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  draw();
}

function updateTelemetry() {
  statusValue.textContent = state.running ? "Running" : "Paused";
  frameValue.textContent = state.frame.toLocaleString();
  positionValue.textContent = `${state.pose.x.toFixed(1)}, ${state.pose.y.toFixed(1)}`;
  yawValue.textContent = `${(state.pose.yaw * 180 / Math.PI).toFixed(1)}°`;
  pointsValue.textContent = "240";
  cellsValue.textContent = Math.round(state.frame * 18 + 240).toLocaleString();
  semanticValues.drivable.textContent = Math.round(70 + Math.sin(state.time) * 12);
  semanticValues.nondrivable.textContent = Math.round(82 + Math.cos(state.time * 0.8) * 10);
  semanticValues.static.textContent = Math.round(48 + Math.sin(state.time * 0.6) * 8);
  semanticValues.dynamic.textContent = Math.round(40 + Math.cos(state.time * 0.5) * 7);
  zoneValues.near.textContent = Math.round(120 + Math.sin(state.time) * 15);
  zoneValues.mid.textContent = Math.round(78 + Math.cos(state.time) * 12);
  zoneValues.far.textContent = Math.round(42 + Math.sin(state.time * 0.7) * 8);
  metricValues.fps.textContent = state.fps.toFixed(1);
  metricValues.latency.textContent = `${state.latency.toFixed(2)} ms`;
  metricValues["point-rate"].textContent = `${Math.round(240 / Math.max(state.frameInterval / 1000, 0.001)).toLocaleString()}/s`;
  metricValues.queue.textContent = `demo · ${state.queue} pending`;
}

function draw() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const scale = Math.min(width, height) / 70;
  const centerX = width / 2;
  const centerY = height / 2;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#0c161e";
  context.fillRect(0, 0, width, height);

  context.strokeStyle = "rgba(148, 163, 184, 0.1)";
  context.lineWidth = 1;
  for (let grid = -30; grid <= 30; grid += 1) {
    const offset = grid * scale;
    context.beginPath();
    context.moveTo(centerX + offset, centerY - 30 * scale);
    context.lineTo(centerX + offset, centerY + 30 * scale);
    context.moveTo(centerX - 30 * scale, centerY + offset);
    context.lineTo(centerX + 30 * scale, centerY + offset);
    context.stroke();
  }

  context.strokeStyle = "rgba(34, 197, 94, 0.75)";
  context.lineWidth = 2;
  context.beginPath();
  state.trail.forEach(([x, y], index) => {
    const drawX = centerX + x * scale;
    const drawY = centerY - y * scale;
    if (index === 0) context.moveTo(drawX, drawY);
    else context.lineTo(drawX, drawY);
  });
  context.stroke();

  for (let index = 0; index < 240; index += 1) {
    const angle = index * Math.PI * 2 / 240;
    const range = 8 + Math.sin(angle * 3 + state.time) * 2 + Math.abs(Math.cos(angle * 11 - state.time * 0.7)) * 1.5;
    const worldX = state.pose.x + range * Math.cos(angle + state.pose.yaw);
    const worldY = state.pose.y + range * Math.sin(angle + state.pose.yaw);
    const intensity = 0.5 + 0.5 * Math.sin(angle * 5 + state.time);
    const color = range <= 10 ? "#4ade80" : range <= 30 ? "#fbbf24" : "#fb7185";
    context.fillStyle = color;
    context.globalAlpha = 0.35 + intensity * 0.6;
    context.fillRect(centerX + worldX * scale, centerY - worldY * scale, 2, 2);
  }
  context.globalAlpha = 1;

  context.save();
  context.translate(centerX + state.pose.x * scale, centerY - state.pose.y * scale);
  context.rotate(-state.pose.yaw);
  context.fillStyle = "#4ade80";
  context.shadowColor = "rgba(74, 222, 128, 0.75)";
  context.shadowBlur = 18;
  context.beginPath();
  context.moveTo(0, -10);
  context.lineTo(7, 8);
  context.lineTo(0, 5);
  context.lineTo(-7, 8);
  context.closePath();
  context.fill();
  context.restore();
}

function step(timestamp) {
  if (!state.lastTimestamp) state.lastTimestamp = timestamp;
  const delta = Math.min((timestamp - state.lastTimestamp) / 1000, 0.1);
  state.lastTimestamp = timestamp;
  if (state.running) {
    const frameStart = performance.now();
    const simulationDelta = delta * state.speed;
    state.time += simulationDelta;
    state.pose.yaw += 0.01 * simulationDelta;
    state.pose.x += 0.5 * simulationDelta * Math.cos(state.pose.yaw);
    state.pose.y += 0.5 * simulationDelta * Math.sin(state.pose.yaw);
    state.trail.push([state.pose.x, state.pose.y]);
    if (state.trail.length > 2048) state.trail.shift();
    state.frame += 1;
    state.latency = performance.now() - frameStart;
    state.frameInterval = delta * 1000;
    state.fps = delta > 0 ? 1 / delta : 0;
    state.queue = Math.max(0, Math.round(state.queue + 0.4 - (state.fps > 30 ? 0.7 : 0.1)));
    updateTelemetry();
  }
  draw();
  requestAnimationFrame(step);
}

toggleButton.addEventListener("click", () => {
  state.running = !state.running;
  toggleButton.textContent = state.running ? "Pause" : "Resume";
  updateTelemetry();
});

resetButton.addEventListener("click", () => {
  state.frame = 0;
  state.time = 0;
  state.pose = { x: 0, y: 0, yaw: 0 };
  state.trail = [];
  updateTelemetry();
});

speedInput.addEventListener("input", () => {
  state.speed = Number(speedInput.value);
  speedValue.textContent = `${state.speed}x`;
});

window.addEventListener("resize", resizeCanvas);
updateTelemetry();
resizeCanvas();
requestAnimationFrame(step);
