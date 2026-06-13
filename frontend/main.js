// ============================================================
// King Phoenix GCS — Main Application
// ============================================================

// --- CONFIGURATION ---
const COMPANION_COMPUTER_IP = "192.168.10.179";
const DATA_STALE_THRESHOLD_MS = 5000;

// --- WAYPOINT STATE ---
const savedWaypoints = localStorage.getItem("kingPhoenixWaypoints");
let recordedWaypoints = savedWaypoints ? JSON.parse(savedWaypoints) : [];
if (savedWaypoints) {
  console.log("Loaded waypoints from local storage:", recordedWaypoints);
}

const WAYPOINT_CONFIG = [
  { label: "Logistic 1", alt: 1.0 },
  { label: "Logistic 2", alt: 1.0 },
  { label: "Barrel Drop", alt: 1.0 },
  { label: "Before Exit", alt: 1.4 },
  { label: "After Exit", alt: 1.4 },
  { label: "Outdoor Drop 1", alt: 1.0 },
  { label: "Outdoor Drop 2", alt: 1.0 },
  { label: "Final Land", alt: 1.0 },
];
const WAYPOINT_LABELS = WAYPOINT_CONFIG.map((wp) => wp.label);

let currentVehicleState = {};
let lastDataTimestamp = null;
let dataAgeInterval = null;

// --- MAP INITIALIZATION ---
const map = L.map("map").setView([0, 0], 2);
L.tileLayer(
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  {
    maxZoom: 22,
  },
).addTo(map);

const droneIcon = L.icon({
  iconUrl: "assets/navigation.png",
  iconSize: [32, 32],
  iconAnchor: [16, 16],
});
let vehicleMarker = null;
let mapInitialized = false;

// --- UI ELEMENT REFERENCES ---
const cameraFeedElem = document.getElementById("camera-feed");
const cameraOverlay = document.getElementById("camera-overlay");
const cameraBadge = document.getElementById("camera-badge");
const connectionStatusElem = document.getElementById("connection-status");
const connectionIcon = document.getElementById("connection-icon");
const flightModeElem = document.getElementById("flight-mode");
const batteryLevelElem = document.getElementById("battery-level");
const logContainer = document.getElementById("log-container");
const logCountElem = document.getElementById("log-count");
const clearLogBtn = document.getElementById("clear-log-btn");
const armStatusElem = document.getElementById("arm-status");
const armIcon = document.getElementById("arm-icon");
const dataAgeElem = document.getElementById("data-age");
const pitchElem = document.getElementById("data-pitch");
const rollElem = document.getElementById("data-roll");
const headingElem = document.getElementById("data-heading");
const altElem = document.getElementById("data-alt");
const longitudeElem = document.getElementById("data-longitude");
const latitudeElem = document.getElementById("data-latitude");
const groundspeedElem = document.getElementById("data-groundspeed");
const climbElem = document.getElementById("data-climb");
const distanceElem = document.getElementById("data-rangefinder");
const startMissionBtn = document.getElementById("start-mission-btn");
const stopMissionBtn = document.getElementById("stop-mission-btn");
const loadingOverlay = document.getElementById("loading-overlay");

// --- WAYPOINT UI REFERENCES ---
const recordWpBtn = document.getElementById("record-wp-btn");
const clearWpBtn = document.getElementById("clear-wp-btn");
const wpDisplay = document.getElementById("wp-display");
const wpBadge = document.getElementById("wp-badge");

// --- MODAL REFERENCES ---
const confirmModal = document.getElementById("confirm-modal");
const modalTitle = document.getElementById("modal-title");
const modalMessage = document.getElementById("modal-message");
const modalConfirmBtn = document.getElementById("modal-confirm");
const modalCancelBtn = document.getElementById("modal-cancel");
const keyboardHelp = document.getElementById("keyboard-help");
const closeHelpBtn = document.getElementById("close-help");

// --- TOAST SYSTEM ---
const toastContainer = document.getElementById("toast-container");

function showToast(message, type = "info") {
  const toast = document.createElement("div");
  toast.className = `toast ${type}`;
  toast.textContent = message;
  toastContainer.appendChild(toast);

  // Remove from DOM after animation finishes
  toast.addEventListener("animationend", (e) => {
    if (e.animationName === "toast-out") {
      toast.remove();
    }
  });
}

// --- MODAL SYSTEM ---
let pendingConfirmation = null;

function showConfirm(title, message, onConfirm) {
  modalTitle.textContent = title;
  modalMessage.textContent = message;
  pendingConfirmation = onConfirm;
  confirmModal.setAttribute("aria-hidden", "false");
}

function hideConfirm() {
  confirmModal.setAttribute("aria-hidden", "true");
  pendingConfirmation = null;
}

modalConfirmBtn.addEventListener("click", () => {
  if (pendingConfirmation) pendingConfirmation();
  hideConfirm();
});

modalCancelBtn.addEventListener("click", hideConfirm);

confirmModal.addEventListener("click", (e) => {
  if (e.target === confirmModal) hideConfirm();
});

// --- KEYBOARD SHORTCUTS ---
document.addEventListener("keydown", (e) => {
  // Help toggle
  if (e.key === "?" && !e.ctrlKey && !e.metaKey) {
    e.preventDefault();
    const isHidden = keyboardHelp.getAttribute("aria-hidden") !== "false";
    keyboardHelp.setAttribute("aria-hidden", isHidden ? "false" : "true");
    return;
  }

  // Close modals with Escape
  if (e.key === "Escape") {
    keyboardHelp.setAttribute("aria-hidden", "true");
    hideConfirm();
    return;
  }

  // Ctrl+Shift shortcuts
  if (!e.ctrlKey || !e.shiftKey) return;

  switch (e.key.toLowerCase()) {
    case "r":
      e.preventDefault();
      if (!recordWpBtn.disabled) recordWpBtn.click();
      break;
    case "s":
      e.preventDefault();
      if (!startMissionBtn.disabled) startMissionBtn.click();
      break;
    case "x":
      e.preventDefault();
      if (!stopMissionBtn.disabled) stopMissionBtn.click();
      break;
    case "c":
      e.preventDefault();
      if (!clearWpBtn.disabled) clearWpBtn.click();
      break;
  }
});

closeHelpBtn.addEventListener("click", () => {
  keyboardHelp.setAttribute("aria-hidden", "true");
});

// --- DATA AGE TRACKING ---
function updateDataAge() {
  if (!lastDataTimestamp) {
    dataAgeElem.textContent = "No data";
    dataAgeElem.classList.remove("stale");
    return;
  }

  const elapsed = Date.now() - lastDataTimestamp;
  const seconds = Math.floor(elapsed / 1000);

  if (elapsed > DATA_STALE_THRESHOLD_MS) {
    dataAgeElem.textContent = `Stale: ${seconds}s ago`;
    dataAgeElem.classList.add("stale");
  } else {
    dataAgeElem.textContent = `Live (${seconds}s ago)`;
    dataAgeElem.classList.remove("stale");
  }
}

// --- CAMERA FEED HANDLING ---
function setupCameraFeed() {
  const url = `http://${COMPANION_COMPUTER_IP}:5001/video_feed`;
  cameraFeedElem.src = url;
  cameraBadge.textContent = "Loading";
  cameraBadge.style.color = "var(--tertiary-color)";

  cameraFeedElem.onload = () => {
    cameraOverlay.classList.add("hidden");
    cameraBadge.textContent = "Live";
    cameraBadge.style.color = "var(--secondary-color)";
  };

  cameraFeedElem.onerror = () => {
    cameraOverlay.classList.remove("hidden");
    cameraBadge.textContent = "Offline";
    cameraBadge.style.color = "var(--error-color)";
  };
}

// --- WEBSOCKET LOGIC ---
let wsConnection = null;

function connectWebSocket() {
  const ws = new WebSocket(
    `ws://${window.location.hostname || "localhost"}:8765`,
  );

  ws.onopen = () => {
    console.log("Connected to MAVLink WebSocket server!");
    connectionStatusElem.textContent = "CONNECTED";
    connectionStatusElem.classList.remove("disconnected");
    connectionStatusElem.classList.add("connected");
    connectionIcon.classList.remove("disconnected");
    connectionIcon.classList.add("connected");
    wsConnection = ws;
    loadingOverlay.classList.add("hidden");
    updateUIState();
    showToast("Connected to vehicle", "success");
  };

  ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "log") {
      updateLog(message);
    } else if (message.type === "state") {
      currentVehicleState = message.data;
      lastDataTimestamp = Date.now();
      updateAll(currentVehicleState);
    }
  };

  ws.onclose = () => {
    console.log("Disconnected. Reconnecting in 3s...");
    connectionStatusElem.textContent = "DISCONNECTED";
    connectionStatusElem.classList.remove("connected");
    connectionStatusElem.classList.add("disconnected");
    connectionIcon.classList.remove("connected");
    connectionIcon.classList.add("disconnected");
    wsConnection = null;
    updateUIState();
    showToast("Connection lost. Retrying...", "error");
    setTimeout(connectWebSocket, 3000);
  };

  ws.onerror = (error) => {
    console.error("WebSocket Error:", error);
    ws.close();
  };
}

// --- WAYPOINT MANAGEMENT ---
function updateWpButton() {
  const nextIndex = recordedWaypoints.length;
  if (nextIndex < WAYPOINT_LABELS.length) {
    const btnText = recordWpBtn.querySelector(".btn-icon")
      ? `Record ${WAYPOINT_LABELS[nextIndex]}`
      : `Record ${WAYPOINT_LABELS[nextIndex]}`;
    // Preserve icon if present
    const iconSpan = recordWpBtn.querySelector(".btn-icon");
    if (iconSpan) {
      recordWpBtn.innerHTML = "";
      recordWpBtn.appendChild(iconSpan);
      recordWpBtn.appendChild(
        document.createTextNode(" " + WAYPOINT_LABELS[nextIndex]),
      );
    } else {
      recordWpBtn.textContent = `Record ${WAYPOINT_LABELS[nextIndex]}`;
    }
    recordWpBtn.disabled = false;
  } else {
    const iconSpan = recordWpBtn.querySelector(".btn-icon");
    if (iconSpan) {
      recordWpBtn.innerHTML = "";
      recordWpBtn.appendChild(iconSpan);
      recordWpBtn.appendChild(document.createTextNode(" All Set"));
    } else {
      recordWpBtn.textContent = "All Waypoints Set";
    }
    recordWpBtn.disabled = true;
  }
  wpBadge.textContent = `${recordedWaypoints.length} / ${WAYPOINT_LABELS.length}`;
}

function updateWpDisplay() {
  if (recordedWaypoints.length === 0) {
    wpDisplay.innerHTML = '<p class="empty-state">No waypoints recorded.</p>';
  } else {
    let listHtml = "<ol>";
    recordedWaypoints.forEach((wp, index) => {
      const config = WAYPOINT_CONFIG[index];
      listHtml += `<li><b>${config.label}:</b> ${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)} <span style="color:var(--text-muted)">@ ${wp.alt}m</span></li>`;
    });
    listHtml += "</ol>";
    wpDisplay.innerHTML = listHtml;
  }
}

function updateUIState() {
  const isConnected =
    wsConnection && wsConnection.readyState === WebSocket.OPEN;
  stopMissionBtn.disabled = !isConnected;

  if (!isConnected) {
    recordWpBtn.disabled = true;
    clearWpBtn.disabled = true;
    startMissionBtn.disabled = true;
  } else {
    clearWpBtn.disabled = recordedWaypoints.length === 0;
    const allWaypointsSet = recordedWaypoints.length === WAYPOINT_LABELS.length;
    startMissionBtn.disabled = !allWaypointsSet;
    recordWpBtn.disabled = allWaypointsSet;
  }
}

// --- EVENT LISTENERS ---
recordWpBtn.addEventListener("click", () => {
  if (
    typeof currentVehicleState.lat !== "number" ||
    typeof currentVehicleState.lon !== "number"
  ) {
    showToast(
      "Cannot record waypoint: current drone location is unknown.",
      "error",
    );
    return;
  }

  const nextIndex = recordedWaypoints.length;
  if (nextIndex < WAYPOINT_LABELS.length) {
    const waypointConfig = WAYPOINT_CONFIG[nextIndex];
    const newWaypoint = {
      lat: currentVehicleState.lat,
      lon: currentVehicleState.lon,
      alt: waypointConfig.alt,
    };
    recordedWaypoints.push(newWaypoint);
    localStorage.setItem(
      "kingPhoenixWaypoints",
      JSON.stringify(recordedWaypoints),
    );
    console.log(
      `Recorded waypoint ${nextIndex + 1} (${waypointConfig.label}):`,
      newWaypoint,
    );
    showToast(`Recorded ${waypointConfig.label}`, "success");
    updateWpButton();
    updateWpDisplay();
    updateUIState();
  }
});

clearWpBtn.addEventListener("click", () => {
  showConfirm(
    "Clear Waypoints",
    "Are you sure you want to clear all recorded waypoints? This cannot be undone.",
    () => {
      recordedWaypoints = [];
      localStorage.removeItem("kingPhoenixWaypoints");
      console.log("Waypoints cleared.");
      showToast("All waypoints cleared", "warning");
      updateWpButton();
      updateWpDisplay();
      updateUIState();
    },
  );
});

startMissionBtn.addEventListener("click", () => {
  if (wsConnection && wsConnection.readyState === WebSocket.OPEN) {
    if (recordedWaypoints.length !== WAYPOINT_LABELS.length) {
      showToast(
        `Please record all ${WAYPOINT_LABELS.length} waypoints before starting.`,
        "warning",
      );
      return;
    }
    showConfirm(
      "Start Mission",
      `Start autonomous mission with ${recordedWaypoints.length} waypoints?`,
      () => {
        const command = {
          action: "start_mission",
          waypoints: recordedWaypoints,
        };
        wsConnection.send(JSON.stringify(command));
        console.log(
          'Sent "start_mission" command with waypoints:',
          recordedWaypoints,
        );
        showToast("Mission start command sent!", "success");
      },
    );
  } else {
    showToast("Cannot send command: not connected to the drone.", "error");
  }
});

stopMissionBtn.addEventListener("click", () => {
  if (wsConnection && wsConnection.readyState === WebSocket.OPEN) {
    showConfirm(
      "Stop Mission",
      "Stop the current mission and command the drone to land?",
      () => {
        const command = { action: "stop_mission" };
        wsConnection.send(JSON.stringify(command));
        console.log('Sent "stop_mission" command to backend.');
        showToast("Mission stop command sent — drone landing.", "warning");
      },
    );
  } else {
    showToast("Cannot send command: not connected to the drone.", "error");
  }
});

// --- LOG MANAGEMENT ---
let logMessageCount = 0;

function updateLog(log) {
  const logMessage = document.createElement("div");
  logMessage.className = `log-message severity-${log.severity}`;
  const timestamp = new Date().toLocaleTimeString("id-ID", { hour12: false });
  logMessage.textContent = `[${timestamp}] ${log.text}`;
  logContainer.appendChild(logMessage);
  logContainer.scrollTop = logContainer.scrollHeight;

  logMessageCount++;
  logCountElem.textContent = logMessageCount;
}

clearLogBtn.addEventListener("click", () => {
  logContainer.innerHTML = "";
  logMessageCount = 0;
  logCountElem.textContent = "0";
  showToast("Log cleared", "info");
});

// --- UPDATE FUNCTIONS ---
function updateAll(state) {
  if (state.pitch != null) pitchElem.textContent = state.pitch.toFixed(2);
  if (state.roll != null) rollElem.textContent = state.roll.toFixed(2);
  if (state.heading != null) headingElem.textContent = state.heading.toFixed(2);
  if (state.alt_rel != null) altElem.textContent = state.alt_rel.toFixed(2);
  if (typeof state.lat === "number")
    latitudeElem.textContent = `${state.lat.toFixed(7)}`;
  if (typeof state.lon === "number")
    longitudeElem.textContent = `${state.lon.toFixed(7)}`;
  if (state.flight_mode !== undefined)
    flightModeElem.textContent = state.flight_mode;
  if (state.level !== undefined)
    batteryLevelElem.textContent = `${state.level}%`;
  if (state.groundspeed != null)
    groundspeedElem.textContent = state.groundspeed.toFixed(2);
  if (state.climb != null) climbElem.textContent = state.climb.toFixed(2);
  if (state.distance != null)
    distanceElem.textContent = state.distance.toFixed(2);

  if (state.armed !== undefined) {
    if (state.armed) {
      armStatusElem.textContent = "ARMED";
      armStatusElem.classList.remove("disarmed");
      armStatusElem.classList.add("armed");
      armIcon.classList.remove("disarmed");
      armIcon.classList.add("armed");
    } else {
      armStatusElem.textContent = "DISARMED";
      armStatusElem.classList.remove("armed");
      armStatusElem.classList.add("disarmed");
      armIcon.classList.remove("armed");
      armIcon.classList.add("disarmed");
    }
  }

  if (typeof state.lat === "number" && typeof state.lon === "number") {
    const latlng = [state.lat, state.lon];
    if (!mapInitialized) {
      map.setView(latlng, 19);
      vehicleMarker = L.marker(latlng, {
        icon: droneIcon,
        rotationAngle: 0,
      }).addTo(map);
      mapInitialized = true;
    } else {
      vehicleMarker.setLatLng(latlng);
    }
    if (typeof state.heading === "number") {
      vehicleMarker.setRotationAngle(state.heading);
    }
  }
}

// --- START THE APP ---
setupCameraFeed();
updateUIState();
updateWpButton();
updateWpDisplay();
connectWebSocket();

// Start data age timer
dataAgeInterval = setInterval(updateDataAge, 1000);
