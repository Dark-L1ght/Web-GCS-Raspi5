// --- CONFIGURATION ---
const COMPANION_COMPUTER_IP = '192.168.10.212';
const MISSION_ALTITUDE = 2.0; // Default altitude for waypoints

// --- WAYPOINT STATE ---
// Check localStorage for saved waypoints when the page loads
const savedWaypoints = localStorage.getItem('kingPhoenixWaypoints');
let recordedWaypoints = savedWaypoints ? JSON.parse(savedWaypoints) : [];
if (savedWaypoints) {
    console.log('Loaded waypoints from local storage:', recordedWaypoints);
}

const WAYPOINT_LABELS = ['Logistics 1', 'Logistics 2', 'Barrel', 'Final Land'];
let currentVehicleState = {}; // Store the latest state globally

// --- MAP INITIALIZATION ---
const map = L.map('map').setView([0, 0], 2);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 22,
}).addTo(map);

const droneIcon = L.icon({
    iconUrl: 'assets/navigation.png',
    iconSize: [32, 32],
    iconAnchor: [16, 16]
});
let vehicleMarker = null;
let mapInitialized = false;

// --- UI ELEMENT REFERENCES ---
const cameraFeedElem = document.getElementById('camera-feed');
const connectionStatusElem = document.getElementById('connection-status');
const flightModeElem = document.getElementById('flight-mode');
const batteryLevelElem = document.getElementById('battery-level');
const logContainer = document.getElementById('log-container');
const armStatusElem = document.getElementById('arm-status');
const pitchElem = document.getElementById('data-pitch');
const rollElem = document.getElementById('data-roll');
const headingElem = document.getElementById('data-heading');
const altElem = document.getElementById('data-alt');
const longitudeElem = document.getElementById('data-longitude');
const latitudeElem = document.getElementById('data-latitude');
const groundspeedElem = document.getElementById('data-groundspeed');
const climbElem = document.getElementById('data-climb');
const distanceElem = document.getElementById('data-rangefinder');
const startMissionBtn = document.getElementById('start-mission-btn');
const stopMissionBtn = document.getElementById('stop-mission-btn');

// --- WAYPOINT UI REFERENCES ---
const recordWpBtn = document.getElementById('record-wp-btn');
const clearWpBtn = document.getElementById('clear-wp-btn');
const wpDisplay = document.getElementById('wp-display');


// --- WEBSOCKET LOGIC ---
let wsConnection = null;

function connectWebSocket() {
    const ws = new WebSocket(`ws://${window.location.hostname || 'localhost'}:8765`);
    ws.onopen = () => {
        console.log('Connected to MAVLink WebSocket server!');
        connectionStatusElem.textContent = 'CONNECTED';
        connectionStatusElem.style.backgroundColor = 'var(--secondary-color)';
        wsConnection = ws;
        updateUIState();
    };
    ws.onmessage = (event) => {
        const message = JSON.parse(event.data);
        if (message.type === 'log') {
            updateLog(message);
        } else if (message.type === 'state') {
            currentVehicleState = message.data;
            updateAll(currentVehicleState);
        }
    };
    ws.onclose = () => {
        console.log('Disconnected. Reconnecting in 3s...');
        connectionStatusElem.textContent = 'DISCONNECTED';
        connectionStatusElem.style.backgroundColor = 'var(--error-color)';
        wsConnection = null;
        updateUIState();
        setTimeout(connectWebSocket, 3000);
    };
    ws.onerror = (error) => {
        console.error('WebSocket Error:', error);
        ws.close();
    };
}

// --- WAYPOINT MANAGEMENT FUNCTIONS ---
function updateWpButton() {
    const nextIndex = recordedWaypoints.length;
    if (nextIndex < WAYPOINT_LABELS.length) {
        recordWpBtn.textContent = `Record ${WAYPOINT_LABELS[nextIndex]}`;
        recordWpBtn.disabled = false;
    } else {
        recordWpBtn.textContent = 'All Waypoints Set';
        recordWpBtn.disabled = true;
    }
}

function updateWpDisplay() {
    if (recordedWaypoints.length === 0) {
        wpDisplay.innerHTML = '<p>No waypoints recorded.</p>';
    } else {
        let listHtml = '<ol>';
        recordedWaypoints.forEach((wp, index) => {
            listHtml += `<li><b>${WAYPOINT_LABELS[index]}:</b> ${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}</li>`;
        });
        listHtml += '</ol>';
        wpDisplay.innerHTML = listHtml;
    }
}

function updateUIState() {
    const isConnected = wsConnection && wsConnection.readyState === WebSocket.OPEN;
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
recordWpBtn.addEventListener('click', () => {
    if (typeof currentVehicleState.lat !== 'number' || typeof currentVehicleState.lon !== 'number') {
        alert("Cannot record waypoint: Current drone location is unknown.");
        return;
    }

    if (recordedWaypoints.length < WAYPOINT_LABELS.length) {
        const newWaypoint = {
            lat: currentVehicleState.lat,
            lon: currentVehicleState.lon,
            alt: MISSION_ALTITUDE
        };
        recordedWaypoints.push(newWaypoint);
        localStorage.setItem('kingPhoenixWaypoints', JSON.stringify(recordedWaypoints)); // SAVE
        console.log(`Recorded waypoint ${recordedWaypoints.length}:`, newWaypoint);
        updateWpButton();
        updateWpDisplay();
        updateUIState();
    }
});

clearWpBtn.addEventListener('click', () => {
    if (confirm("Are you sure you want to clear all recorded waypoints?")) {
        recordedWaypoints = [];
        localStorage.removeItem('kingPhoenixWaypoints'); // CLEAR
        console.log("Waypoints cleared.");
        updateWpButton();
        updateWpDisplay();
        updateUIState();
    }
});

startMissionBtn.addEventListener('click', () => {
    if (wsConnection && wsConnection.readyState === WebSocket.OPEN) {
        if (recordedWaypoints.length !== WAYPOINT_LABELS.length) {
            alert('Error: Please record all 4 waypoints before starting the mission.');
            return;
        }
        const command = {
            action: 'start_mission',
            waypoints: recordedWaypoints
        };
        wsConnection.send(JSON.stringify(command));
        console.log('Sent "start_mission" command with waypoints:', recordedWaypoints);
        alert('Mission start command sent with recorded waypoints!');
    } else {
        alert('Error: Cannot send command. Not connected to the drone.');
    }
});

stopMissionBtn.addEventListener('click', () => {
    if (wsConnection && wsConnection.readyState === WebSocket.OPEN) {
        const command = { action: 'stop_mission' };
        wsConnection.send(JSON.stringify(command));
        console.log('Sent "stop_mission" command to backend.');
        alert('Mission stop command sent!');
    } else {
        alert('Error: Cannot send command. Not connected to the drone.');
    }
});


// --- UPDATE FUNCTIONS ---
function updateAll(state) {
    if (state.pitch != null) pitchElem.textContent = `${state.pitch.toFixed(2)} °`;
    if (state.roll != null) rollElem.textContent = `${state.roll.toFixed(2)} °`;
    if (state.heading != null) headingElem.textContent = `${state.heading.toFixed(2)} °`;
    if (state.alt_rel != null) altElem.textContent = `${state.alt_rel.toFixed(2)} m`;
    if (typeof state.lat === 'number') latitudeElem.textContent = `${state.lat.toFixed(7)}`;
    if (typeof state.lon === 'number') longitudeElem.textContent = `${state.lon.toFixed(7)}`;
    if (state.flight_mode !== undefined) flightModeElem.textContent = state.flight_mode;
    if (state.level !== undefined) batteryLevelElem.textContent = `${state.level}%`;
    if (state.groundspeed != null) groundspeedElem.textContent = `${state.groundspeed.toFixed(2)} m/s`;
    if (state.climb != null) climbElem.textContent = `${state.climb.toFixed(2)} m/s`;
    if (state.distance != null) distanceElem.textContent = `${state.distance.toFixed(2)} m`;

    if (state.armed !== undefined) {
        if (state.armed) {
            armStatusElem.textContent = 'ARMED';
            armStatusElem.classList.remove('disarmed');
            armStatusElem.classList.add('armed');
        } else {
            armStatusElem.textContent = 'DISARMED';
            armStatusElem.classList.remove('armed');
            armStatusElem.classList.add('disarmed');
        }
    }

    if (typeof state.lat === 'number' && typeof state.lon === 'number') {
        const latlng = [state.lat, state.lon];
        if (!mapInitialized) {
            map.setView(latlng, 19);
            vehicleMarker = L.marker(latlng, { icon: droneIcon, rotationAngle: 0 }).addTo(map);
            mapInitialized = true;
        } else {
            vehicleMarker.setLatLng(latlng);
        }
        if (typeof state.heading === 'number') {
            vehicleMarker.setRotationAngle(state.heading);
        }
    }
}

function updateLog(log) {
    const logMessage = document.createElement('div');
    logMessage.className = `log-message severity-${log.severity}`;
    const timestamp = new Date().toLocaleTimeString('id-ID', { hour12: false });
    logMessage.textContent = `[${timestamp}] ${log.text}`;
    logContainer.appendChild(logMessage);
    logContainer.scrollTop = logContainer.scrollHeight;
}

// --- START THE APP ---
cameraFeedElem.src = `http://${COMPANION_COMPUTER_IP}:5001/video_feed`;
updateUIState();
updateWpButton(); // Renders button text correctly on load
updateWpDisplay(); // Renders saved waypoints on load
connectWebSocket();