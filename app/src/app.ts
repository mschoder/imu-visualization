import * as THREE from 'three';
import { parseAttitudeEstimates } from '../attitudeEstimates';
import "./styles.css";
import { Euler, Quaternion } from 'three';

// Function to convert quaternion to Euler angles in degrees
const quaternionToEulerAngles = (q: Quaternion) => {
    const euler = new Euler().setFromQuaternion(q, 'XYZ');
    return {
        x: THREE.MathUtils.radToDeg(euler.x),
        y: THREE.MathUtils.radToDeg(euler.y),
        z: THREE.MathUtils.radToDeg(euler.z)
    };
};

// Table for displaying Euler angles
const eulerAngleTable = document.createElement('table');
eulerAngleTable.innerHTML = `
    <thead>
        <tr>
            <th>Roll(X)</th>
            <th>Pitch(Y)</th>
            <th>Yaw(Z)</th>
        </tr>
    </thead>
    <tbody>
        <tr id="accel-mag-row">
            <td id="accel-mag-roll">0</td>
            <td id="accel-mag-pitch">0</td>
            <td id="accel-mag-yaw">0</td>
        </tr>
        <tr id="complementary-row">
            <td id="complementary-roll">0</td>
            <td id="complementary-pitch">0</td>
            <td id="complementary-yaw">0</td>
        </tr>
        <tr id="ekf-row">
            <td id="ekf-roll">0</td>
            <td id="ekf-pitch">0</td>
            <td id="ekf-yaw">0</td>
        </tr>
        <tr id="madgwick-row">
            <td id="madgwick-roll">0</td>
            <td id="madgwick-pitch">0</td>
            <td id="madgwick-yaw">0</td>
        </tr>
    </tbody>
`;
document.body.appendChild(eulerAngleTable);

// Update table with Euler angles
const updateTable = (idPrefix: string, quaternion: Quaternion) => {
    const angles = quaternionToEulerAngles(quaternion);
    document.getElementById(`${idPrefix}-roll`)!.textContent = angles.x.toFixed(2);
    document.getElementById(`${idPrefix}-pitch`)!.textContent = angles.y.toFixed(2);
    document.getElementById(`${idPrefix}-yaw`)!.textContent = angles.z.toFixed(2);
};

// Set up the scene, camera, and renderer
//
// Rotate the scene root so that it aligns with NED coordinate system where X is
// forward, into the screen
const scene = new THREE.Scene();
scene.rotation.set(Math.PI / 2, 0, -Math.PI / 2);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Add wireframe cubes to the scene
const geometry = new THREE.BoxGeometry(2, 1, 0.5);
const material1 = new THREE.MeshBasicMaterial({ color: 0x00ff00, wireframe: true });
const material2 = new THREE.MeshBasicMaterial({ color: 0x0000ff, wireframe: true });
const material3 = new THREE.MeshBasicMaterial({ color: 0xff0000, wireframe: true });
const material4 = new THREE.MeshBasicMaterial({ color: 0xff00ff, wireframe: true });
const cube1 = new THREE.Mesh(geometry, material1);
const cube2 = new THREE.Mesh(geometry, material2);
const cube3 = new THREE.Mesh(geometry, material3);
const cube4 = new THREE.Mesh(geometry, material4);

// Add various cubes to represent different IMU filters
cube1.position.set(0, 0, 0);
cube2.position.set(0, 0, 0);
cube3.position.set(0, 0, 0);
cube4.position.set(0, 0, 0);

const axesHelper1 = new THREE.AxesHelper(0.5);
const axesHelper2 = axesHelper1.clone();
const axesHelper3 = axesHelper1.clone();
const axesHelper4 = axesHelper1.clone();
cube1.add(axesHelper1);
cube2.add(axesHelper2);
cube3.add(axesHelper3);
cube4.add(axesHelper4);

scene.add(cube1);
scene.add(cube2);
scene.add(cube3);
scene.add(cube4);

// Visualize global axes
var globalAxesHelper = new THREE.AxesHelper(0.5);
globalAxesHelper.position.set(6, 2, 2);
scene.add(globalAxesHelper);

// Set camera position in the scene.
camera.position.x = 1;
camera.position.y = 1;
camera.position.z = 5;
camera.lookAt(0, 0, 0);

// Function to animate the cube
const animate = () => {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
};
animate();

// WebSocket setup
console.log("About to setup websocket")
const socket = new WebSocket(`ws://${location.hostname}:8089`);
console.log("Socket established")

socket.onmessage = event => {
    try {
        const estimates = parseAttitudeEstimates(JSON.parse(event.data));

        if (estimates.accel_mag) {
            cube1.quaternion.copy(estimates.accel_mag);
        }
        if (estimates.complementary) {
            cube2.quaternion.copy(estimates.complementary);
        }
        if (estimates.ekf) {
            cube3.quaternion.copy(estimates.ekf);
        }
        if (estimates.madgwick) {
            cube4.quaternion.copy(estimates.madgwick);
        }

    } catch (err) {
        console.error("Invalid data received:", event.data);
    }
};

socket.onmessage = event => {
    try {
        const estimates = parseAttitudeEstimates(JSON.parse(event.data));

        if (estimates.accel_mag) {
            cube1.quaternion.copy(estimates.accel_mag);
            updateTable('accel-mag', estimates.accel_mag);
        }
        if (estimates.complementary) {
            cube2.quaternion.copy(estimates.complementary);
            updateTable('complementary', estimates.complementary);
        }
        if (estimates.ekf) {
            cube3.quaternion.copy(estimates.ekf);
            updateTable('ekf', estimates.ekf);
        }
        if (estimates.madgwick) {
            cube4.quaternion.copy(estimates.madgwick);
            updateTable('madgwick', estimates.madgwick);
        }

    } catch (err) {
        console.error("Invalid data received:", event.data);
    }
};

// Get the cube toggle elements
const cube1Toggle = document.getElementById("cube1-toggle") as HTMLInputElement;
const cube2Toggle = document.getElementById("cube2-toggle") as HTMLInputElement;
const cube3Toggle = document.getElementById("cube3-toggle") as HTMLInputElement;
const cube4Toggle = document.getElementById("cube4-toggle") as HTMLInputElement;

// Handle visibility toggling
cube1Toggle?.addEventListener("change", (event: Event) => {
    const target = event.target as HTMLInputElement;
    cube1.visible = target.checked;
});
cube2Toggle?.addEventListener("change", (event: Event) => {
    const target = event.target as HTMLInputElement;
    cube2.visible = target.checked;
});
cube3Toggle?.addEventListener("change", (event: Event) => {
    const target = event.target as HTMLInputElement;
    cube3.visible = target.checked;
});
cube4Toggle?.addEventListener("change", (event: Event) => {
    const target = event.target as HTMLInputElement;
    cube4.visible = target.checked;
});

// Handle window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});