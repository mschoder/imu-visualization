import { WebSocketServer } from "ws";
import { AttitudeEstimates } from './attitudeEstimates';
import * as THREE from 'three';

// WebSocket server port
const SERVER_PORT = 8089;

// Create a WebSocket server
const wss = new WebSocketServer({ port: SERVER_PORT }, () => {
    console.log(`WebSocket server running at ws://localhost:${SERVER_PORT}`);
});

var att1 = new THREE.Vector3(0, 0, 0);
var att2 = new THREE.Vector3(0, 0, 0);

const delta = new THREE.Vector3(0.01, 0.02, 0.03);

// Returns an equivalent angle on the interval [0, 2*PI)
function wrapAngleRad(angle: number): number {
    const twoPi = 2 * Math.PI;
    return ((angle % twoPi) + twoPi) % twoPi;
}

// Function to send rotation data to connected WebSocket clients
const sendRotationData = () => {
    // Compute quaternion outputs
    const euler1 = new THREE.Euler(att1.x, att1.y, att1.z);
    const euler2 = new THREE.Euler(att2.x, att2.y, att2.z);

    const q1 = new THREE.Quaternion().setFromEuler(euler1);
    const q2 = new THREE.Quaternion().setFromEuler(euler2);

    const estimates: AttitudeEstimates = {};
    estimates.accel_mag = q1;
    estimates.complementary = q2;

    // Prepare rotation data as JSON
    const rotationData = JSON.stringify(estimates);
    console.log(rotationData);

    // Broadcast rotation data to all connected clients
    wss.clients.forEach(client => {
        if (client.readyState === 1) { // Ensure the client connection is open
            client.send(rotationData);
        }
    });

    // Update rotation for slow rotation effect
    att1.add(delta);
    att2.add(delta);
};

// Send rotation data every 50ms (20Hz)
setInterval(sendRotationData, 50);

// WebSocket event handlers
wss.on("connection", ws => {
    console.log("New WebSocket client connected");

    ws.on("close", () => {
        console.log("WebSocket client disconnected");
    });
});
