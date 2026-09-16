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

const state = {
  running: true,
  speed: 1,
  frame: 0,
  time: 0,
  pose: { x: 0, y: 0, yaw: 0 },
  trail: [],
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
    context.fillStyle = `rgba(125, 211, 252, ${0.25 + intensity * 0.7})`;
    context.fillRect(centerX + worldX * scale, centerY - worldY * scale, 2, 2);
  }

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
    const simulationDelta = delta * state.speed;
    state.time += simulationDelta;
    state.pose.yaw += 0.01 * simulationDelta;
    state.pose.x += 0.5 * simulationDelta * Math.cos(state.pose.yaw);
    state.pose.y += 0.5 * simulationDelta * Math.sin(state.pose.yaw);
    state.trail.push([state.pose.x, state.pose.y]);
    if (state.trail.length > 2048) state.trail.shift();
    state.frame += 1;
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
