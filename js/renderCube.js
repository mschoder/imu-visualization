/*
Process serial data from IMU, perform filtering, and visualize using Three.js lib
*/

'use strict';

// let dataAccelx = 0, dataAccely = 0, dataAccelz = 0;  // m/s^2
// let dataOmegax = 0, dataOmegay = 0, dataOmegaz = 0;  // rad/s
// let dataMagx = 0, dataMagy = 0, dataMagz = 0;        // uT
let dataTemp = 0;                                    // degC
let rotx = 0, roty = 0, rotz = 0;                    // rad
let vx = 0.0, vy = 0.0, vz = 0.0;                    // m/s
let px = 0.0, py = 0.0, pz = 0.0;                    // m



const sigFig = 6;
const degToRad = (Math.PI/180);
const radToDeg = (180/Math.PI);
const updateFreq = 100;  // hertz

let container, ms, viz;
let camera, scene, renderer;
let cube, plane;

//Connect to socket.io
const serverIP = "localhost";
const socket = io.connect(serverIP + ':5004');
console.log('socket connected to: ' + serverIP);

processData();
createCube();
animate();

function processData() {
    socket.on('position_update', function(data) {
        let dataArray = data.split(", ");
        console.log(dataArray);

        if (dataArray[0].substring(0,3) != 'ypr (deg)') {
            console.log('Cannot parse serial data as number')
        } else {

            // // crappy position update
            // vx += (dataAccelx / updateFreq);
            // vy += (dataAccely / updateFreq);
            // vz += (dataAccelz / updateFreq);
            
            // px += vx / updateFreq;
            // py += vy / updateFreq;
            // pz += vz / updateFreq;

            rotz = parseFloat(dataArray[0].substring(5)) * degToRad;
            rotx = parseFloat(dataArray[1]) * degToRad;
            roty = parseFloat(dataArray[2]) * degToRad;
            console.log('ypr: ', roty, rotx, rotz);

            // console.log("Rotation x-y-z: ", rotx + "," + roty + "," + rotz);
        }
    });
}

function createCube() {
    container = document.createElement( 'div' );
    container.style.position = 'relative';
    container.style.top = '10px';
    container.style.width = '100%';
    viz = document.getElementById("viz");
    // document.body.appendChild(container);
    viz.appendChild(container);

    var info = document.createElement( 'div' );
    info.style.position = 'absolute';
    info.style.top = '10px';
    info.style.width = '100%';
    info.style.textAlign = 'center';
    info.innerHTML = 'Visualize IMU';
    info.setAttribute('id', 'pourHeading');
    container.appendChild( info );

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera( 75, viz.offsetWidth / viz.offsetHeight, 0.1, 1000 );

    renderer = new THREE.WebGLRenderer();
    renderer.setSize( viz.offsetWidth, viz.offsetHeight);
    container.appendChild( renderer.domElement );

    const geometry = new THREE.BoxGeometry(1, 2, 0.5);
    for ( var i = 0; i < geometry.faces.length; i += 2 ) {

        var hex = Math.random() * 0xffffff;
        geometry.faces[ i ].color.setHex( hex );
        geometry.faces[ i + 1 ].color.setHex( hex );

    }
    const material = new THREE.MeshBasicMaterial( { vertexColors: THREE.FaceColors, overdraw: 0.5 } );
    // TODO: Fix overdraw warnings above

    cube = new THREE.Mesh( geometry, material );
    scene.add( cube );

    const axesHelper = new THREE.AxesHelper( 5 );
    cube.add( axesHelper );

    camera.position.z = 5;

    window.addEventListener( 'resize', onWindowResize, false );

}

function animate () {
    requestAnimationFrame( animate );
    cube.rotation.x = rotx;
    cube.rotation.y = roty;
    cube.rotation.z = rotz;

    cube.position.x = 0 //px;
    cube.position.y = 0 //py;
    cube.position.z = 0 //pz;

    renderer.render( scene, camera );
}

function onWindowResize() {
    camera.aspect = viz.offsetWidth / viz.offsetHeight;
    camera.updateProjectionMatrix();
    renderer.setSize( viz.offsetWidth, viz.offsetHeight );
}


