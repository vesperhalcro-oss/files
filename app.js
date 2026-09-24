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
const objectsValue = document.querySelector("#telemetry-objects");
const replayButton = document.querySelector("#start-replay");
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
  replayFrames: window.TACTICAL_MAPPER_REPLAY || [],
  replayIndex: 0,
  replayElapsed: 0,
  replayLoaded: false,
  currentFrame: { raw_point_count: 0, pose: { x: 0, y: 0, yaw: 0 }, cells: [], objects: [] },
};

function loadReplay() {
  if (!state.replayFrames.length) {
    throw new Error("Replay data is unavailable");
  }
  state.replayLoaded = true;
  state.replayIndex = 0;
  state.replayElapsed = 0;
  state.currentFrame = state.replayFrames[0];
  state.frame = 1;
  state.pose = { ...state.currentFrame.pose };
  updateTelemetry();
  draw();
}

function applyReplayFrame() {
  const frame = state.replayFrames[state.replayIndex % state.replayFrames.length];
  state.currentFrame = frame;
  state.pose = { ...frame.pose };
  state.frame = state.replayIndex + 1;
  state.trail.push([state.pose.x, state.pose.y]);
  if (state.trail.length > 2048) state.trail.shift();
}

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
  const cells = state.currentFrame.cells;
  const semanticCount = (className) => cells.filter((cell) => cell.semantic_class === className).length;
  pointsValue.textContent = (state.currentFrame.raw_point_count || 0).toLocaleString();
  objectsValue.textContent = state.currentFrame.objects.length.toLocaleString();
  cellsValue.textContent = cells.length.toLocaleString();
  semanticValues.drivable.textContent = semanticCount("drivable_terrain");
  semanticValues.nondrivable.textContent = cells.filter((cell) => cell.terrain === "non_drivable").length;
  semanticValues.static.textContent = cells.filter((cell) => ["wall", "barrier", "static_obstacle"].includes(cell.semantic_class)).length;
  semanticValues.dynamic.textContent = cells.filter((cell) => ["pedestrian", "vehicle", "cyclist", "other_dynamic"].includes(cell.semantic_class)).length;
  zoneValues.near.textContent = cells.filter((cell) => cell.resolution === 0.05).length;
  zoneValues.mid.textContent = cells.filter((cell) => cell.resolution === 0.1).length;
  zoneValues.far.textContent = cells.filter((cell) => cell.resolution === 0.5).length;
  metricValues.fps.textContent = state.fps.toFixed(1);
  metricValues.latency.textContent = `${state.latency.toFixed(2)} ms`;
  metricValues["point-rate"].textContent = `${Math.round(state.currentFrame.raw_point_count / Math.max(state.frameInterval / 1000, 0.001)).toLocaleString()}/s`;
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

  for (const cell of state.currentFrame.cells) {
    const worldX = cell.grid_x * cell.resolution;
    const worldY = cell.grid_y * cell.resolution;
    const color = cell.terrain === "drivable" ? "#4ade80" : "#fbbf24";
    const alpha = Math.min(0.95, 0.35 + cell.confidence * 0.6);
    context.fillStyle = color;
    context.globalAlpha = alpha;
    const cellSize = Math.max(cell.resolution * scale, 3);
    context.fillRect(centerX + worldX * scale, centerY - worldY * scale - cellSize, cellSize, cellSize);
    if (["wall", "barrier", "static_obstacle"].includes(cell.semantic_class)) {
      context.strokeStyle = "#fb7185";
      context.strokeRect(centerX + worldX * scale, centerY - worldY * scale - cellSize, cellSize, cellSize);
    }
  }
  context.globalAlpha = 1;

  for (const object of state.currentFrame.objects) {
    context.strokeStyle = object.dynamic ? "#c084fc" : "#fb7185";
    context.lineWidth = 2;
    context.strokeRect(
      centerX + (object.x - object.width / 2) * scale,
      centerY - (object.y + object.length / 2) * scale,
      object.width * scale,
      object.length * scale,
    );
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
    const frameStart = performance.now();
    const simulationDelta = delta * state.speed;
    state.time += simulationDelta;
    if (state.replayLoaded) {
      state.replayElapsed += simulationDelta;
      const nextFrame = state.replayFrames[(state.replayIndex + 1) % state.replayFrames.length];
      const currentFrame = state.replayFrames[state.replayIndex];
      const frameDuration = Math.max(
        nextFrame.timestamp > currentFrame.timestamp
          ? nextFrame.timestamp - currentFrame.timestamp
          : currentFrame.timestamp - (state.replayFrames[state.replayIndex - 1]?.timestamp || 0),
        0.001,
      );
      if (state.replayElapsed >= frameDuration) {
        state.replayElapsed %= frameDuration;
        state.replayIndex = (state.replayIndex + 1) % state.replayFrames.length;
        applyReplayFrame();
      }
    }
    state.latency = performance.now() - frameStart;
    state.frameInterval = delta * 1000;
    state.fps = delta > 0 ? 1 / delta : 0;
    state.queue = 0;
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
  state.replayIndex = 0;
  state.replayElapsed = 0;
  if (state.replayLoaded) {
    state.currentFrame = state.replayFrames[0];
    state.pose = { ...state.currentFrame.pose };
  }
  updateTelemetry();
});

replayButton.addEventListener("click", () => {
  replayButton.disabled = true;
  replayButton.textContent = "Loading replay…";
  try {
    if (!state.replayLoaded) loadReplay();
    state.running = true;
    toggleButton.textContent = "Pause";
    replayButton.textContent = "Replay loaded";
  } catch (error) {
    replayButton.textContent = "Replay unavailable";
    statusValue.textContent = error.message;
  } finally {
    replayButton.disabled = false;
  }
});

speedInput.addEventListener("input", () => {
  state.speed = Number(speedInput.value);
  speedValue.textContent = `${state.speed}x`;
});

window.addEventListener("resize", resizeCanvas);
updateTelemetry();
resizeCanvas();
try {
  loadReplay();
  replayButton.textContent = "Replay running";
} catch {
  replayButton.textContent = "Replay unavailable";
  statusValue.textContent = "Replay unavailable";
}
requestAnimationFrame(step);
