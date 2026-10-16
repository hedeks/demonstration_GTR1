// Инициализация 3D сцены (Three.js)
const container = document.getElementById('three-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff); // Белый фон для дашборда
const camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

// Настройка орбитальной камеры
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
camera.position.set(3, 3, 3);
controls.target.set(0, 0, 0);

// Добавляем красивую сетку (Пол)
const grid = new THREE.GridHelper(10, 20, 0x45f3ff, 0x222222);
grid.position.y = -0.01; // Уровень пола
grid.material.opacity = 0.2;
grid.material.transparent = true;
scene.add(grid);

// Оси координат мира (RGB = XYZ)
scene.add(new THREE.AxesHelper(1.5));

// Базовая геометрия для вокселей карты
const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);

// Выделенный маркер активной цели MTV1 (заменяет случайный зеленый кубик)
const targetMarkerGeom = new THREE.SphereGeometry(0.08, 16, 16);
const targetMarkerMat = new THREE.MeshStandardMaterial({
    color: 0xff8c42,
    emissive: 0xff4500,
    emissiveIntensity: 0.6,
    roughness: 0.3
});
const targetMarker = new THREE.Mesh(targetMarkerGeom, targetMarkerMat);
targetMarker.visible = false;
scene.add(targetMarker);

// --- 3D Target Highlight System (Активная цель MTV1) ---
const targetObjectGroup = new THREE.Group();
targetObjectGroup.visible = false;
scene.add(targetObjectGroup);

// 1. Wireframe 3D Bounding Box
const targetBoxGeom = new THREE.BoxGeometry(1, 1, 1);
const targetBoxMat = new THREE.MeshBasicMaterial({ 
    color: 0xff8c42, 
    wireframe: true,
    transparent: true,
    opacity: 0.95
});
const targetBoxMesh = new THREE.Mesh(targetBoxGeom, targetBoxMat);
targetObjectGroup.add(targetBoxMesh);

// 2. Vertical Light Beacon (Holo Beam)
const targetBeaconGeom = new THREE.CylinderGeometry(0.04, 0.16, 1.4, 16, 1, true);
const targetBeaconMat = new THREE.MeshBasicMaterial({
    color: 0xff8c42,
    transparent: true,
    opacity: 0.45,
    side: THREE.DoubleSide,
    blending: THREE.AdditiveBlending
});
const targetBeaconMesh = new THREE.Mesh(targetBeaconGeom, targetBeaconMat);
targetBeaconMesh.position.y = 0.7;
targetObjectGroup.add(targetBeaconMesh);

// 3. Pulsing Ground Halo Rings
const targetRingGeom = new THREE.RingGeometry(0.12, 0.22, 32);
const targetRingMat = new THREE.MeshBasicMaterial({
    color: 0xff8c42,
    transparent: true,
    opacity: 0.8,
    side: THREE.DoubleSide
});
const targetRingMesh = new THREE.Mesh(targetRingGeom, targetRingMat);
targetRingMesh.rotation.x = Math.PI / 2;
targetRingMesh.position.y = -0.15;
targetObjectGroup.add(targetRingMesh);

// 4. Central Pulsing Goal Sphere
const targetCoreGeom = new THREE.SphereGeometry(0.06, 16, 16);
const targetCoreMat = new THREE.MeshStandardMaterial({
    color: 0xff8c42,
    emissive: 0xff4500,
    emissiveIntensity: 0.8
});
const targetCoreMesh = new THREE.Mesh(targetCoreGeom, targetCoreMat);
targetObjectGroup.add(targetCoreMesh);

// --- 3D OPRN Splice Chain Visualization Group (MP12 directed graph) ---
const oprnGroup = new THREE.Group();
scene.add(oprnGroup);

function updateOprn3DVisualization(nodes) {
    while (oprnGroup.children.length > 0) {
        const obj = oprnGroup.children[0];
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) obj.material.dispose();
        oprnGroup.remove(obj);
    }

    if (!nodes || nodes.length === 0) return;

    nodes.forEach(node => {
        if (!node.start || !node.end) return;

        // Three.js coords (Z-up to Y-up: x, z, -y)
        const p1 = new THREE.Vector3(node.start.x, node.start.z, -node.start.y);
        const p2 = new THREE.Vector3(node.end.x, node.end.z, -node.end.y);
        const v = new THREE.Vector3().subVectors(p2, p1);
        const len = v.length();
        if (len < 0.001) return;

        // Color based on OPRN FSM status
        let hexColor = 0x00ff88; // VALIDATED
        if (node.status === 'COLLISION') hexColor = 0xff3366;
        else if (node.status === 'EXECUTED') hexColor = 0x00d2d3;
        else if (node.status === 'PENDING') hexColor = 0xffaa00;

        // 1. Splice Link 3D Tube (Cylinder)
        const cylGeom = new THREE.CylinderGeometry(0.018, 0.018, len, 8);
        const cylMat = new THREE.MeshStandardMaterial({
            color: hexColor,
            emissive: hexColor,
            emissiveIntensity: 0.6,
            roughness: 0.3
        });
        const cylMesh = new THREE.Mesh(cylGeom, cylMat);
        const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);
        cylMesh.position.copy(mid);
        cylMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v.clone().normalize());
        oprnGroup.add(cylMesh);

        // 2. Waypoint Node Sphere
        const nodeGeom = new THREE.SphereGeometry(0.045, 16, 16);
        const nodeMat = new THREE.MeshStandardMaterial({
            color: hexColor,
            emissive: hexColor,
            emissiveIntensity: 0.8
        });
        const nodeMesh = new THREE.Mesh(nodeGeom, nodeMat);
        nodeMesh.position.copy(p2);
        oprnGroup.add(nodeMesh);

        // 3. Direction Arrow Cone (at 75% along the segment)
        const arrowGeom = new THREE.ConeGeometry(0.038, 0.08, 8);
        const arrowMat = new THREE.MeshBasicMaterial({ color: hexColor });
        const arrowMesh = new THREE.Mesh(arrowGeom, arrowMat);
        const arrowPos = new THREE.Vector3().lerpVectors(p1, p2, 0.75);
        arrowMesh.position.copy(arrowPos);
        arrowMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v.clone().normalize());
        oprnGroup.add(arrowMesh);
    });
}

// Для массового высокоскоростного рендеринга вокселей карты (30 FPS)
let maxVoxelCapacity = 20000;
const voxelFillMat = new THREE.MeshBasicMaterial({
    color: 0xcccccc, 
    polygonOffset: true, 
    polygonOffsetFactor: 1, 
    polygonOffsetUnits: 1
});
const voxelWireMat = new THREE.MeshBasicMaterial({
    color: 0x000000, 
    wireframe: true
});
let instancedVoxelsFill = new THREE.InstancedMesh(geometry, voxelFillMat, maxVoxelCapacity);
let instancedVoxelsWire = new THREE.InstancedMesh(geometry, voxelWireMat, maxVoxelCapacity);
instancedVoxelsFill.count = 0;
instancedVoxelsWire.count = 0;
scene.add(instancedVoxelsFill);
scene.add(instancedVoxelsWire);

let currentMode = 'SIM';
let datasetViewInitialized = false;

// Единая группа Агента
const agentMesh = new THREE.Group();
scene.add(agentMesh);

// 1. Модель наземного колесного робота (режим SIM): корпус стоит строго на полу (Y=0 .. 0.75м)
const simRobotGroup = new THREE.Group();
const robotRadius = 0.22;
const robotHeight = 0.75;
const agentGeometry = new THREE.CylinderGeometry(robotRadius, robotRadius, robotHeight, 24);
agentGeometry.translate(0, robotHeight / 2, 0); // Нижнее основание на полу Y=0, верх на Y=0.75м

const agentMaterial = new THREE.MeshStandardMaterial({ 
    color: 0x00e676, 
    roughness: 0.35, 
    metalness: 0.7,
    transparent: true, 
    opacity: 0.88 
});
const robotCylinder = new THREE.Mesh(agentGeometry, agentMaterial);
simRobotGroup.add(robotCylinder);

const wireGeo = new THREE.WireframeGeometry(agentGeometry);
const wireMat = new THREE.LineBasicMaterial({ color: 0x00f0ff, linewidth: 1.5 });
const agentWire = new THREE.LineSegments(wireGeo, wireMat);
simRobotGroup.add(agentWire);

// Носовой указатель направления движения (курсовая стрелка робота вперед вдоль +X)
const fwdIndicatorGeo = new THREE.ConeGeometry(0.06, 0.16, 16);
fwdIndicatorGeo.rotateZ(-Math.PI / 2);
fwdIndicatorGeo.translate(robotRadius + 0.08, robotHeight * 0.7, 0);
const fwdIndicatorMat = new THREE.MeshBasicMaterial({ color: 0xffff00 });
const fwdIndicator = new THREE.Mesh(fwdIndicatorGeo, fwdIndicatorMat);
simRobotGroup.add(fwdIndicator);

// Камера сенсора на верхушке робота (высота 0.75м, наклон -25 градусов вниз)
const simCamMount = new THREE.Group();
simCamMount.position.set(0, robotHeight, 0);
simCamMount.rotation.z = THREE.MathUtils.degToRad(-25); // Наклон оптической оси вниз на 25 град

const simCamBox = new THREE.Mesh(
    new THREE.BoxGeometry(0.08, 0.06, 0.12),
    new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.2 })
);
simCamMount.add(simCamBox);
const simLens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.025, 0.025, 0.03, 16),
    new THREE.MeshStandardMaterial({ color: 0x00ffff, roughness: 0.1, emissive: 0x005577 })
);
simLens.rotateZ(Math.PI / 2);
simLens.position.set(0.05, 0, 0);
simCamMount.add(simLens);
simRobotGroup.add(simCamMount);

agentMesh.add(simRobotGroup);

// 2. Модель ручной камеры / оптического сенсора с пирамидой обзора (режим DATASET)
const datasetCameraGroup = new THREE.Group();

// Корпус оптического сенсора (Kinect)
const camBodyGeo = new THREE.BoxGeometry(0.24, 0.08, 0.10);
const camBodyMat = new THREE.MeshStandardMaterial({ color: 0x1a2332, roughness: 0.4, metalness: 0.8 });
const camBodyMesh = new THREE.Mesh(camBodyGeo, camBodyMat);
datasetCameraGroup.add(camBodyMesh);

// Линза объектива (направлена вперед по оптической оси +Z)
const lensGeo = new THREE.CylinderGeometry(0.035, 0.035, 0.04, 16);
lensGeo.rotateX(Math.PI / 2);
lensGeo.translate(0, 0, 0.06);
const lensMat = new THREE.MeshStandardMaterial({ color: 0x00ffff, roughness: 0.2, metalness: 0.9 });
datasetCameraGroup.add(new THREE.Mesh(lensGeo, lensMat));

// Пирамида поля зрения камеры (Frustum FOV) с наклоном
const frustumPoints = [];
const fovW = 0.30;
const fovH = 0.22;
const fovDist = 0.55;
frustumPoints.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(-fovW, -fovH, fovDist));
frustumPoints.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(fovW, -fovH, fovDist));
frustumPoints.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(fovW, fovH, fovDist));
frustumPoints.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(-fovW, fovH, fovDist));
frustumPoints.push(new THREE.Vector3(-fovW, -fovH, fovDist), new THREE.Vector3(fovW, -fovH, fovDist));
frustumPoints.push(new THREE.Vector3(fovW, -fovH, fovDist), new THREE.Vector3(fovW, fovH, fovDist));
frustumPoints.push(new THREE.Vector3(fovW, fovH, fovDist), new THREE.Vector3(-fovW, fovH, fovDist));
frustumPoints.push(new THREE.Vector3(-fovW, fovH, fovDist), new THREE.Vector3(-fovW, -fovH, fovDist));

const frustumGeo = new THREE.BufferGeometry().setFromPoints(frustumPoints);
const frustumMat = new THREE.LineBasicMaterial({ color: 0x00ffcc, linewidth: 2 });
const frustumMesh = new THREE.LineSegments(frustumGeo, frustumMat);
datasetCameraGroup.add(frustumMesh);
datasetCameraGroup.visible = false;
agentMesh.add(datasetCameraGroup);

// Оси ориентации: Red = Right (+X), Green = Down (+Y), Blue = Optical Forward (+Z)
const dirHelper = new THREE.AxesHelper(0.35);
agentMesh.add(dirHelper);

// Демонстрационный маршрут A*
const points = [];
points.push(new THREE.Vector3(0, 0, 0));
points.push(new THREE.Vector3(0.5, 0, 0));
points.push(new THREE.Vector3(0.5, 0.5, 0));
points.push(new THREE.Vector3(0.5, 0.5, 0.5));
const lineMaterial = new THREE.LineBasicMaterial({ color: 0xff3366, linewidth: 2 });
const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
const routeLine = new THREE.Line(lineGeometry, lineMaterial);
scene.add(routeLine);

// Обработка изменения размера
window.addEventListener('resize', () => {
    if(!container) return;
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
});

// Логика следования за агентом
let isFollowingAgent = false;
const btnFollow = document.getElementById('follow-agent-btn');
const btnReset = document.getElementById('reset-view-btn');

if (btnFollow) {
    btnFollow.addEventListener('click', () => {
        isFollowingAgent = !isFollowingAgent;
        btnFollow.style.background = isFollowingAgent ? '#ff3366' : 'rgba(255, 255, 255, 0.1)';
    });
}
if (btnReset) {
    btnReset.addEventListener('click', () => {
        isFollowingAgent = false;
        if(btnFollow) btnFollow.style.background = 'rgba(255, 255, 255, 0.1)';
        if (currentMode === 'DATASET') {
            camera.position.set(2.4, 2.3, 0.5);
            controls.target.set(0.6, 0.7, -1.2);
        } else {
            camera.position.set(3, 3, 3);
            controls.target.set(0, 0, 0);
        }
    });
}

// Анимация 3D сцены
let renderFrameCount = 0;
let lastFpsTimestamp = performance.now();

function animate() {
    requestAnimationFrame(animate);

    // Live FPS meter for 3D Viewport
    renderFrameCount++;
    const now = performance.now();
    if (now - lastFpsTimestamp >= 500) {
        const currentFps = Math.round((renderFrameCount * 1000) / (now - lastFpsTimestamp));
        const fpsElem = document.getElementById('fps-val');
        if (fpsElem) fpsElem.innerText = currentFps;
        renderFrameCount = 0;
        lastFpsTimestamp = now;
    }

    // Active Goal 3D Beacon Animations
    if (targetObjectGroup && targetObjectGroup.visible) {
        const t = now * 0.003;
        targetRingMesh.scale.set(1.0 + 0.25 * Math.sin(t), 1.0 + 0.25 * Math.sin(t), 1);
        targetBeaconMesh.material.opacity = 0.35 + 0.15 * Math.sin(t * 1.5);
        const sCore = 1.0 + 0.18 * Math.sin(t * 2);
        targetCoreMesh.scale.set(sCore, sCore, sCore);
    }
    
    if (isFollowingAgent && agentMesh) {
        controls.target.copy(agentMesh.position);
    }
    
    controls.update();
    renderer.render(scene, camera);
}
animate();

// --- 2D Канвас ---
const cvCanvas = document.getElementById('cv-canvas');
const ctx = cvCanvas.getContext('2d');
// Подгоняем размер (в реальном приложении нужно синхронизировать с потоком)
setTimeout(() => {
    cvCanvas.width = cvCanvas.parentElement.clientWidth;
    cvCanvas.height = cvCanvas.parentElement.clientHeight;
}, 100);

// --- WebSockets Real-Time Integration ---
let ws;

// --- Функция эмуляции логов для UI ---
function addLog(source, msg) {
    const term = document.getElementById('terminal');
    const div = document.createElement('div');
    div.className = `log-line ${source.toLowerCase()}`;
    div.innerText = `[${source}] ${msg}`;
    term.appendChild(div);
    term.scrollTop = term.scrollHeight;
}

function connectWS() {
    const host = window.location.hostname || '127.0.0.1';
    ws = new WebSocket(`ws://${host}:8081`);

    ws.onopen = () => {
        document.querySelector('.status-indicator').style.background = '#00ff88';
        document.querySelector('.status-indicator').style.boxShadow = '0 0 10px #00ff88';
        addLog('SYS', 'WebSocket Bridge Connected. Awaiting real-time telemetry...');
    };

    ws.onclose = () => {
        document.querySelector('.status-indicator').style.background = '#ff3366';
        document.querySelector('.status-indicator').style.boxShadow = '0 0 10px #ff3366';
        addLog('SYS', 'WebSocket Bridge Disconnected. Reconnecting in 2s...');
        setTimeout(connectWS, 2000);
    };

    ws.onerror = (err) => {
        console.error('WebSocket Error:', err);
    };


    function applyVoxels(voxels) {
        const count = voxels ? voxels.length : 0;
        if (count > 0) {
            const voxelCounter = document.getElementById('voxel-val');
            if (voxelCounter) voxelCounter.innerText = count.toLocaleString();
            
            if (count > maxVoxelCapacity) {
                maxVoxelCapacity = Math.max(count * 2, maxVoxelCapacity * 2);
                scene.remove(instancedVoxelsFill);
                scene.remove(instancedVoxelsWire);
                instancedVoxelsFill.dispose();
                instancedVoxelsWire.dispose();
                instancedVoxelsFill = new THREE.InstancedMesh(geometry, voxelFillMat, maxVoxelCapacity);
                instancedVoxelsWire = new THREE.InstancedMesh(geometry, voxelWireMat, maxVoxelCapacity);
                scene.add(instancedVoxelsFill);
                scene.add(instancedVoxelsWire);
            }

            instancedVoxelsFill.count = count;
            instancedVoxelsWire.count = count;
            
            const dummy = new THREE.Object3D();
            for (let i = 0; i < count; i++) {
                dummy.position.set(voxels[i].x, voxels[i].z, -voxels[i].y);
                dummy.updateMatrix();
                instancedVoxelsFill.setMatrixAt(i, dummy.matrix);
                instancedVoxelsWire.setMatrixAt(i, dummy.matrix);
            }
            instancedVoxelsFill.instanceMatrix.needsUpdate = true;
            instancedVoxelsWire.instanceMatrix.needsUpdate = true;
        }
    }

    function applyPath(points) {
        if (!points || points.length === 0) return;
        const newPoints = points.map(p => new THREE.Vector3(p.x, p.z, -p.y));
        const newGeom = new THREE.BufferGeometry().setFromPoints(newPoints);
        routeLine.geometry.dispose();
        routeLine.geometry = newGeom;
    }

    function applyPose(data, isSim) {
        simRobotGroup.visible = isSim;
        datasetCameraGroup.visible = !isSim;

        if (!datasetViewInitialized && !isSim) {
            datasetViewInitialized = true;
            camera.position.set(2.4, 2.3, 0.5);
            controls.target.set(0.6, 0.7, -1.2);
            controls.update();
        }

        const q_w2t = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
        const posX = data.x;
        const posY = data.y;
        const posZ = data.z;
        const qx = data.qx || 0, qy = data.qy || 0, qz = data.qz || 0, qw = (data.qw !== undefined ? data.qw : 1);

        let yawDeg = 0, pitchDeg = 0, rollDeg = 0;
        if (isSim && data.yaw !== undefined) {
            yawDeg = Math.round(data.yaw * 180 / Math.PI);
            pitchDeg = 0;
            rollDeg = 0;
        } else {
            const siny_cosp = 2 * (qw * qz + qx * qy);
            const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            yawDeg = Math.round(Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI);

            const sinp = 2 * (qw * qy - qz * qx);
            if (Math.abs(sinp) >= 1) {
                pitchDeg = Math.round(Math.sign(sinp) * 90);
            } else {
                pitchDeg = Math.round(Math.asin(sinp) * 180 / Math.PI);
            }

            const sinr_cosp = 2 * (qw * qx + qy * qz);
            const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
            rollDeg = Math.round(Math.atan2(sinr_cosp, cosr_cosp) * 180 / Math.PI);
        }

        const robotPosElem = document.getElementById('robot-pos-val');
        if (robotPosElem) {
            robotPosElem.innerText = `[${posX.toFixed(2)}, ${posY.toFixed(2)}, ${posZ.toFixed(2)}]`;
        }
        const robotYawElem = document.getElementById('robot-yaw-val');
        if (robotYawElem) {
            robotYawElem.innerText = `${yawDeg}°`;
        }

        const hudPos = document.getElementById('hud-pos');
        if (hudPos) {
            hudPos.innerText = `X: ${posX >= 0 ? '+' : ''}${posX.toFixed(3)}m  Y: ${posY >= 0 ? '+' : ''}${posY.toFixed(3)}m  Z: ${posZ >= 0 ? '+' : ''}${posZ.toFixed(3)}m`;
        }
        const hudRot = document.getElementById('hud-rot');
        if (hudRot) {
            hudRot.innerText = `Yaw: ${yawDeg}°  Pitch: ${pitchDeg}°  Roll: ${rollDeg}°`;
        }
        const hudQuat = document.getElementById('hud-quat');
        if (hudQuat) {
            hudQuat.innerText = `[${qx.toFixed(2)}, ${qy.toFixed(2)}, ${qz.toFixed(2)}, ${qw.toFixed(2)}]`;
        }

        if (isSim) {
            agentMesh.position.set(data.x, 0.0, -data.y);
            if (data.yaw !== undefined) {
                agentMesh.rotation.set(0, data.yaw, 0);
                const gyroElem = document.getElementById('gyro-val');
                if (gyroElem) gyroElem.innerText = `Cam P: -25° | Yaw: ${Math.round(data.yaw * 180 / Math.PI)}°`;
            }
        } else {
            agentMesh.position.set(data.x, data.z, -data.y);
            const qRaw = new THREE.Quaternion(data.qx, data.qy, data.qz, data.qw);
            const qThree = q_w2t.clone().multiply(qRaw);
            agentMesh.quaternion.copy(qThree);

            const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(qRaw);
            const horiz = Math.sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
            const pDeg = Math.round(Math.atan2(fwd.z, horiz) * 180 / Math.PI);
            const yDeg = Math.round(Math.atan2(fwd.y, fwd.x) * 180 / Math.PI);

            const gyroElem = document.getElementById('gyro-val');
            if (gyroElem) {
                gyroElem.innerText = `P: ${pDeg}° | Y: ${yDeg}°`;
            }
        }
    }

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            
            if (data.type === 'TRACE_LOG') {
                addLog(data.source, data.message);
            } 
            else if (data.type === 'FRAME') {
                if (!window.latestFrameTs || data.ts === undefined || data.ts >= window.latestFrameTs || (window.latestFrameTs - data.ts > 3.0)) {
                    if (data.ts !== undefined) window.latestFrameTs = data.ts;
                    const img = document.getElementById('video-img');
                    const placeholder = document.getElementById('stream-placeholder');
                    if (placeholder) placeholder.style.display = 'none';
                    img.src = data.data;
                }
            }
            else if (data.type === 'BBOXES' || data.type === 'BBOX') {
                // Update canvas dimensions dynamically
                const img = document.getElementById('video-img');
                if (img && img.clientWidth > 0) {
                    cvCanvas.width = img.clientWidth;
                    cvCanvas.height = img.clientHeight;
                }
                ctx.clearRect(0, 0, cvCanvas.width, cvCanvas.height);
                const boxes = data.type === 'BBOXES' ? (data.boxes || []) : [data];
                for (const box of boxes) {
                    ctx.strokeStyle = '#00ffcc';
                    ctx.lineWidth = 2.5;
                    const x = box.x * cvCanvas.width;
                    const y = box.y * cvCanvas.height;
                    const w = box.w * cvCanvas.width;
                    const h = box.h * cvCanvas.height;
                    ctx.strokeRect(x - w/2, y - h/2, w, h);
                    ctx.fillStyle = '#00ffcc';
                    ctx.font = 'bold 13px Outfit, sans-serif';
                    ctx.fillText(`${box.label} (${Math.round(box.confidence * 100)}%)`, x - w/2 + 2, y - h/2 - 4);
                }
            }
            else if (data.type === 'TICK_BUNDLE') {
                const isSim = (data.mode === 'SIM' || data.mode !== 'DATASET');
                currentMode = isSim ? 'SIM' : 'DATASET';
                const modeElem = document.getElementById('mode-val');
                if (modeElem) modeElem.innerText = currentMode;

                const tickElem = document.getElementById('tick-val');
                if (tickElem && data.tick_id !== undefined) {
                    tickElem.innerText = `#${data.tick_id}`;
                }

                if (data.pose) {
                    applyPose(data.pose, isSim);
                }
                if (data.voxels && data.voxels.length > 0) {
                    applyVoxels(data.voxels);
                }
                if (data.path && data.path.length > 0) {
                    applyPath(data.path);
                }
                updateRTRDebug(data);
            }
            else if (data.type === 'VOXEL') {
                if (targetMarker) {
                    targetMarker.position.set(data.x, data.z, -data.y);
                    targetMarker.visible = true;
                }
            }
            else if (data.type === 'MAP') {
                applyVoxels(data.voxels);
            }
            else if (data.type === 'PATH') {
                applyPath(data.points);
            }
            else if (data.type === 'POSE') {
                const isSim = (data.mode === 'SIM' || data.mode !== 'DATASET');
                currentMode = isSim ? 'SIM' : 'DATASET';
                const modeElem = document.getElementById('mode-val');
                if (modeElem) modeElem.innerText = currentMode;
                applyPose(data, isSim);
            }
            else if (data.type === 'RTR_DEBUG') {
                updateRTRDebug(data);
            }
        } catch(e) {
            console.error("Failed to parse telemetry:", e);
        }
    };
}
connectWS();

// --- Handle Goal Setting ---
document.getElementById('set-goal-btn').addEventListener('click', () => {
    const newGoal = document.getElementById('goal-input').value.trim();
    if (newGoal && ws && ws.readyState === WebSocket.OPEN) {
        ws.send("TARGET:" + newGoal);
        document.getElementById('goal-display').innerText = `FIND_${newGoal.toUpperCase()} (Manual)`;
        document.getElementById('goal-input').value = '';
    }
});


// ==========================================
// NEURO-SYMBOLIC INSPECTOR (OPRN & GOALS TREE)
// ==========================================
const inspectorOverlay = document.getElementById('ns-inspector-overlay');
const toggleInspectorBtn = document.getElementById('toggle-inspector-btn');
const closeInspectorBtn = document.getElementById('close-inspector-btn');

if (toggleInspectorBtn) {
    toggleInspectorBtn.addEventListener('click', () => {
        inspectorOverlay.classList.add('open');
    });
}
if (closeInspectorBtn) {
    closeInspectorBtn.addEventListener('click', () => {
        inspectorOverlay.classList.remove('open');
    });
}
if (inspectorOverlay) {
    inspectorOverlay.addEventListener('click', (e) => {
        if (e.target === inspectorOverlay) {
            inspectorOverlay.classList.remove('open');
        }
    });
}

// Tab Switching
document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
        btn.classList.add('active');
        const targetTab = document.getElementById(btn.dataset.tab);
        if (targetTab) targetTab.classList.add('active');
    });
});

let latestRtrData = null;

function updateRTRDebug(data) {
    latestRtrData = data;
    
    // 1. Update Mode badge
    const modeBadge = document.getElementById('rtr-mode-badge');
    if (modeBadge) modeBadge.innerText = (data.mode || 'SIM') + ' LIVE';

    // 2. Update Active Goal Hero Metrics
    const activeGoal = data.active_goal || {};
    const goalClassElem = document.getElementById('active-goal-class');
    if (goalClassElem) goalClassElem.innerText = activeGoal.target_class || 'None';

    const goalDistElem = document.getElementById('active-goal-dist');
    if (goalDistElem) {
        goalDistElem.innerText = (activeGoal.distance !== undefined && activeGoal.distance >= 0) 
            ? activeGoal.distance.toFixed(2) + ' m' 
            : '-- m';
    }

    // 3. Update BLOM Hero Card
    const blom = data.active_blom;
    const blomUuidElem = document.getElementById('active-blom-uuid');
    const blomStatusElem = document.getElementById('active-blom-status');
    const outcomeValElem = document.getElementById('outcome-marker-val');
    const outcomeBarElem = document.getElementById('outcome-marker-bar');
    const oprnCountElem = document.getElementById('active-oprn-count');
    const inspectorBadge = document.getElementById('inspector-badge');

    if (blom) {
        if (blomUuidElem) blomUuidElem.innerText = blom.uuid;
        if (blomStatusElem) {
            blomStatusElem.innerText = blom.is_complete ? 'VALIDATED' : 'PLANNING';
            blomStatusElem.className = 'blom-status-pill ' + (blom.is_complete ? 'status-VALIDATED' : 'status-PENDING');
        }
        const outcome = blom.outcome_marker || 0.0;
        if (outcomeValElem) outcomeValElem.innerText = outcome.toFixed(2);
        if (outcomeBarElem) outcomeBarElem.style.width = Math.round(outcome * 100) + '%';
        const numOprns = (blom.oprn_nodes ? blom.oprn_nodes.length : (blom.oprn_count || 0));
        if (oprnCountElem) oprnCountElem.innerText = numOprns + ' splices';
        if (inspectorBadge) inspectorBadge.innerText = numOprns + ' OPRN';

        // Render OPRN Tree
        renderOprnTree(blom.oprn_nodes || []);
    } else {
        if (blomUuidElem) blomUuidElem.innerText = 'No Active Plan';
        if (blomStatusElem) {
            blomStatusElem.innerText = 'SEARCHING';
            blomStatusElem.className = 'blom-status-pill status-PENDING';
        }
        if (outcomeValElem) outcomeValElem.innerText = '0.00';
        if (outcomeBarElem) outcomeBarElem.style.width = '0%';
        if (oprnCountElem) oprnCountElem.innerText = '0 splices';
        if (inspectorBadge) inspectorBadge.innerText = '0 OPRN';
        const oprnList = document.getElementById('oprn-tree-list');
        if (oprnList) oprnList.innerHTML = '<div class="tree-empty-state">No active BLOM route. Searching for domain invariants...</div>';
    }

    // 4. Render Goals & Invariants Tree
    renderGoalsTree(data.goals_tree || [], activeGoal.target_class || '');

    // 5. Render Cached TRL1 History
    renderTrl1History(data.cached_bloms || []);

    // 6. Highlight Active Goal on 3D Voxel Model
    if (activeGoal.pos && activeGoal.target_class) {
        targetObjectGroup.visible = true;
        targetObjectGroup.position.set(activeGoal.pos.x, activeGoal.pos.z, -activeGoal.pos.y);
        if (activeGoal.dims) {
            targetBoxMesh.scale.set(
                Math.max(activeGoal.dims.dx, 0.15),
                Math.max(activeGoal.dims.dz, 0.15),
                Math.max(activeGoal.dims.dy, 0.15)
            );
        }
        if (activeGoal.goal_reached) {
            targetBoxMat.color.setHex(0x00ff88);
            targetBeaconMat.color.setHex(0x00ff88);
            targetRingMat.color.setHex(0x00ff88);
            targetCoreMat.color.setHex(0x00ff88);
            targetCoreMat.emissive.setHex(0x00ff88);
        } else {
            targetBoxMat.color.setHex(0xff8c42);
            targetBeaconMat.color.setHex(0xff8c42);
            targetRingMat.color.setHex(0xff8c42);
            targetCoreMat.color.setHex(0xff8c42);
            targetCoreMat.emissive.setHex(0xff4500);
        }
    } else {
        targetObjectGroup.visible = false;
    }

    // 7. Visualize all OPRN splices on 3D Voxel Map
    if (blom && blom.oprn_nodes) {
        updateOprn3DVisualization(blom.oprn_nodes);
    } else {
        updateOprn3DVisualization([]);
    }
}

function renderOprnTree(nodes) {
    const container = document.getElementById('oprn-tree-list');
    if (!container) return;

    if (!nodes || nodes.length === 0) {
        container.innerHTML = '<div class="tree-empty-state">BLOM sequence is empty.</div>';
        return;
    }

    let html = '';
    nodes.forEach(node => {
        const statusClass = 'status-' + (node.status || 'PENDING');
        const dx = node.b_vector ? node.b_vector.x : 0;
        const dy = node.b_vector ? node.b_vector.y : 0;
        const dz = node.b_vector ? node.b_vector.z : 0;
        const bx = (dx >= 0 ? '+' : '') + dx.toFixed(3);
        const by = (dy >= 0 ? '+' : '') + dy.toFixed(3);
        const bz = (dz >= 0 ? '+' : '') + dz.toFixed(3);
        const bnorm = (node.b_length !== undefined ? node.b_length : Math.hypot(dx, dy, dz)).toFixed(3);

        // Classify Kinematic Action Primitive based on b-vector orientation
        const len = parseFloat(bnorm);
        const deg = Math.round(Math.atan2(dy, dx) * 180 / Math.PI);
        let actionDesc = '⬆️ Direct Forward';
        let actionType = 'action-forward';

        if (len < 0.05) {
            actionDesc = '🛑 Station Hold / Arrived';
            actionType = 'action-hold';
        } else if (Math.abs(deg) <= 18) {
            actionDesc = `⬆️ Straight Forward (${len.toFixed(2)}m)`;
            actionType = 'action-forward';
        } else if (deg > 18 && deg <= 65) {
            actionDesc = `↗️ Arc Turn Left (+${deg}° / ${len.toFixed(2)}m)`;
            actionType = 'action-turn';
        } else if (deg > 65 && deg <= 115) {
            actionDesc = `⬅️ Turn Left (+${deg}° / ${len.toFixed(2)}m)`;
            actionType = 'action-turn';
        } else if (deg > 115) {
            actionDesc = `🔄 Sharp Turn Left / Reverse (+${deg}°)`;
            actionType = 'action-reverse';
        } else if (deg < -18 && deg >= -65) {
            actionDesc = `↘️ Arc Turn Right (${deg}° / ${len.toFixed(2)}m)`;
            actionType = 'action-turn';
        } else if (deg < -65 && deg >= -115) {
            actionDesc = `➡️ Turn Right (${deg}° / ${len.toFixed(2)}m)`;
            actionType = 'action-turn';
        } else if (deg < -115) {
            actionDesc = `🔄 Sharp Turn Right / Reverse (${deg}°)`;
            actionType = 'action-reverse';
        }

        // Exact 3D coordinates for start and end
        const sx = node.start ? node.start.x.toFixed(2) : '0.00';
        const sy = node.start ? node.start.y.toFixed(2) : '0.00';
        const sz = node.start ? node.start.z.toFixed(2) : '0.20';
        const ex = node.end ? node.end.x.toFixed(2) : '0.00';
        const ey = node.end ? node.end.y.toFixed(2) : '0.00';
        const ez = node.end ? node.end.z.toFixed(2) : '0.20';

        const subTooltip = `Origin [${node.subject_ref || 'agent'}]: X=${sx}m, Y=${sy}m, Z=${sz}m`;
        const tgtTooltip = `Waypoint [${node.target_ref || 'target'}]: X=${ex}m, Y=${ey}m, Z=${ez}m (Click to Focus 3D View)`;

        let statusIcon = '<i class="fas fa-clock"></i>';
        if (node.status === 'EXECUTED') statusIcon = '<i class="fas fa-check-circle"></i>';
        else if (node.status === 'VALIDATED') statusIcon = '<i class="fas fa-bolt"></i>';
        else if (node.status === 'COLLISION') statusIcon = '<i class="fas fa-exclamation-triangle"></i>';

        html += `
            <div class="oprn-node-card oprn-card-${node.status || 'PENDING'}" onclick="focusWaypoint(${ex}, ${ez}, -${ey}, '${node.target_ref}')">
                <div class="oprn-node-top">
                    <div>
                        <span class="oprn-step-tag">Step #${node.index || 1}</span>
                        <span class="oprn-step-uuid">${node.uuid}</span>
                    </div>
                    <span class="oprn-status ${statusClass}">${statusIcon} ${node.status || 'PENDING'}</span>
                </div>
                <div class="oprn-valence-row">
                    <span class="valence-badge" title="${subTooltip}"><i class="fas fa-cube"></i> ${node.subject_ref || 'agent'}</span>
                    <span class="valence-arrow">──( b-vector )──►</span>
                    <span class="valence-badge valence-target-badge" title="${tgtTooltip}"><i class="fas fa-map-marker-alt"></i> ${node.target_ref || 'target'}</span>
                </div>
                <div class="oprn-coords-row">
                    <span class="wp-coord-chip" title="3D World Coordinates [X, Y, Z]">
                        <i class="fas fa-crosshairs"></i> <b>${node.target_ref}</b>: [X: ${ex}, Y: ${ey}, Z: ${ez}]
                    </span>
                </div>
                <div class="oprn-action-row">
                    <span class="action-badge ${actionType}"><i class="fas fa-arrows-alt"></i> Action: <b>${actionDesc}</b></span>
                </div>
                <div class="b-vector-chips">
                    <span class="b-chip bx">Δx: ${bx}m</span>
                    <span class="b-chip by">Δy: ${by}m</span>
                    <span class="b-chip bz">Δz: ${bz}m</span>
                    <span class="b-chip bnorm">‖b‖: ${bnorm}m</span>
                </div>
            </div>
        `;
    });
    container.innerHTML = html;
}

function renderGoalsTree(goals, activeTargetClass) {
    const container = document.getElementById('invariants-tree-list');
    const badge = document.getElementById('invariants-count-badge');
    if (badge) badge.innerText = goals.length + ' Invariants';
    if (!container) return;

    if (!goals || goals.length === 0) {
        container.innerHTML = '<div class="tree-empty-state">No domain invariants discovered in MP11 yet.</div>';
        return;
    }

    let html = '';
    goals.forEach(goal => {
        const isTarget = (goal.class_label === activeTargetClass || goal.is_target);
        const cardClass = isTarget ? 'invariant-card active-target' : 'invariant-card';
        const distStr = goal.distance !== undefined ? goal.distance.toFixed(2) + 'm' : '--';
        const px = goal.pos ? goal.pos.x.toFixed(2) : '0';
        const py = goal.pos ? goal.pos.y.toFixed(2) : '0';
        const pz = goal.pos ? goal.pos.z.toFixed(2) : '0';
        const sx = goal.dims ? goal.dims.dx.toFixed(2) : '0';
        const sy = goal.dims ? goal.dims.dy.toFixed(2) : '0';
        const sz = goal.dims ? goal.dims.dz.toFixed(2) : '0';

        html += `
            <div class="${cardClass}">
                <div class="inv-main">
                    <div class="inv-header">
                        <span class="inv-name">${goal.class_label.toUpperCase()}</span>
                        ${isTarget ? '<span class="blom-status-pill status-VALIDATED">ACTIVE TARGET</span>' : ''}
                        <span class="inv-uuid">${goal.uuid}</span>
                    </div>
                    <div class="inv-coords">
                        Pos: [${px}, ${py}, ${pz}] | BBox: [${sx} × ${sy} × ${sz}] | Dist: ${distStr}
                    </div>
                </div>
                <div>
                    <button class="btn btn-sm" onclick="setGoalFromTree('${goal.class_label}')" style="font-size:0.75rem; padding:4px 8px;">
                        🎯 Set Goal
                    </button>
                </div>
            </div>
        `;
    });
    container.innerHTML = html;
}

window.setGoalFromTree = function(label) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send("TARGET:" + label);
        addLog('MTV1', 'Dynamic Goal Set via Inspector: ' + label);
        const input = document.getElementById('goal-input');
        if (input) input.value = label;
        const display = document.getElementById('goal-display');
        if (display) display.innerText = label.toUpperCase();
    }
};

function renderTrl1History(cached) {
    const container = document.getElementById('trl1-history-list');
    if (!container) return;

    if (!cached || cached.length === 0) {
        container.innerHTML = '<div class="tree-empty-state">No past BLOM trajectories logged in TRL1 yet.</div>';
        return;
    }

    let html = '';
    cached.forEach((item, idx) => {
        html += `
            <div class="invariant-card">
                <div class="inv-main">
                    <div class="inv-header">
                        <span class="inv-name">TRL1 Route #${idx + 1}</span>
                        <span class="inv-uuid">${item.uuid}</span>
                    </div>
                    <div class="inv-coords">
                        Splices: ${item.oprn_count} | Outcome Marker: ${(item.outcome_marker || 0).toFixed(2)} | Complete: ${item.is_complete ? 'YES' : 'NO'}
                    </div>
                </div>
            </div>
        `;
    });
    container.innerHTML = html;
}


// ==========================================
// MOTION CONTROLS & COORDINATE NAVIGATION
// ==========================================
let isRobotStopped = false;
const stopBtn = document.getElementById('robot-stop-btn');
if (stopBtn) {
    stopBtn.addEventListener('click', () => {
        isRobotStopped = !isRobotStopped;
        if (isRobotStopped) {
            stopBtn.innerText = '▶ RESUME';
            stopBtn.className = 'btn btn-sm btn-stop btn-resume';
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send("CMD:STOP");
                addLog('OP13', 'EMERGENCY STOP / HOLD POSITION sent to robot.');
            }
        } else {
            stopBtn.innerText = '🛑 STOP';
            stopBtn.className = 'btn btn-sm btn-stop';
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send("CMD:RESUME");
                addLog('OP13', 'RESUME NAVIGATION sent to robot.');
            }
        }
    });
}

const gotoCoordBtn = document.getElementById('btn-goto-coord');
if (gotoCoordBtn) {
    gotoCoordBtn.addEventListener('click', () => {
        const nx = parseFloat(document.getElementById('nav-x').value) || 0.0;
        const ny = parseFloat(document.getElementById('nav-y').value) || 0.0;
        const nyaw = parseFloat(document.getElementById('nav-yaw').value) || 0.0;
        
        if (ws && ws.readyState === WebSocket.OPEN) {
            const cmd = `NAV_POSE:${nx},${ny},${nyaw}`;
            ws.send(cmd);
            addLog('MTV1', `Navigating to manual coordinate [${nx.toFixed(2)}, ${ny.toFixed(2)}] Yaw: ${nyaw}°`);
            
            // Highlight target in 3D scene
            if (targetObjectGroup) {
                targetObjectGroup.position.set(nx, 0.0, -ny);
                targetObjectGroup.visible = true;
            }
        }
    });
}

// Click-to-Navigate on 3D Ground Plane
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

renderer.domElement.addEventListener('dblclick', (event) => {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const targetPoint = new THREE.Vector3();
    if (raycaster.ray.intersectPlane(groundPlane, targetPoint)) {
        // Three.js coords (x, z, -y) -> World coords (x, y)
        const worldX = parseFloat(targetPoint.x.toFixed(2));
        const worldY = parseFloat((-targetPoint.z).toFixed(2));
        
        const inputX = document.getElementById('nav-x');
        const inputY = document.getElementById('nav-y');
        if (inputX) inputX.value = worldX;
        if (inputY) inputY.value = worldY;

        const currentYaw = parseFloat(document.getElementById('nav-yaw').value) || 0.0;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(`NAV_POSE:${worldX},${worldY},${currentYaw}`);
            addLog('MTV1', `3D Map Click-to-Navigate: Target set to [${worldX}, ${worldY}]`);
            if (targetObjectGroup) {
                targetObjectGroup.position.set(targetPoint.x, 0.0, targetPoint.z);
                targetObjectGroup.visible = true;
            }
        }
    }
});


// --- Right-Click 3D Context Menu (Set Goal / OPRN Triplet / Navigation) ---
const contextMenu = document.getElementById('scene-context-menu');
let ctxTargetCoords = { worldX: 0, worldY: 0, worldZ: 0, threeX: 0, threeY: 0, threeZ: 0, bX: 0, bY: 0, bZ: 0, bLen: 0 };

renderer.domElement.addEventListener('contextmenu', (event) => {
    event.preventDefault();

    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const targetPoint = new THREE.Vector3();
    if (raycaster.ray.intersectPlane(groundPlane, targetPoint)) {
        const worldX = parseFloat(targetPoint.x.toFixed(2));
        const worldY = parseFloat((-targetPoint.z).toFixed(2));
        const worldZ = parseFloat(targetPoint.y.toFixed(2));

        const robX = agentMesh ? agentMesh.position.x : 0.0;
        const robY = agentMesh ? -agentMesh.position.z : 0.0;
        const robZ = agentMesh ? agentMesh.position.y : 0.0;

        const bX = parseFloat((worldX - robX).toFixed(2));
        const bY = parseFloat((worldY - robY).toFixed(2));
        const bZ = parseFloat((worldZ - robZ).toFixed(2));
        const bLen = Math.hypot(bX, bY, bZ).toFixed(2);

        ctxTargetCoords = {
            worldX, worldY, worldZ,
            threeX: targetPoint.x, threeY: targetPoint.y, threeZ: targetPoint.z,
            bX, bY, bZ, bLen
        };

        // Populate context menu info
        const coordsElem = document.getElementById('ctx-coords');
        const distElem = document.getElementById('ctx-dist');
        if (coordsElem) coordsElem.innerText = `X: ${worldX >= 0 ? '+' : ''}${worldX}m  Y: ${worldY >= 0 ? '+' : ''}${worldY}m  Z: ${worldZ}m`;
        if (distElem) distElem.innerText = `${bLen}m`;

        // Position context menu within window boundaries
        if (contextMenu) {
            contextMenu.style.display = 'block';
            const menuWidth = 280;
            const menuHeight = 220;
            let posX = event.clientX + 5;
            let posY = event.clientY + 5;

            if (posX + menuWidth > window.innerWidth) posX = window.innerWidth - menuWidth - 10;
            if (posY + menuHeight > window.innerHeight) posY = window.innerHeight - menuHeight - 10;

            contextMenu.style.left = `${posX}px`;
            contextMenu.style.top = `${posY}px`;
        }
    }
});

function hideContextMenu() {
    if (contextMenu) contextMenu.style.display = 'none';
}

window.addEventListener('click', (e) => {
    if (contextMenu && !contextMenu.contains(e.target)) {
        hideContextMenu();
    }
});

window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') hideContextMenu();
});

// Context Menu Action 1: Set as Goal OPRN
const ctxSetTargetBtn = document.getElementById('ctx-set-target');
if (ctxSetTargetBtn) {
    ctxSetTargetBtn.addEventListener('click', () => {
        hideContextMenu();
        const { worldX, worldY, bX, bY, bZ, bLen, threeX, threeZ } = ctxTargetCoords;
        const targetYaw = Math.round(Math.atan2(bY, bX) * 180 / Math.PI);

        const inputX = document.getElementById('nav-x');
        const inputY = document.getElementById('nav-y');
        const inputYaw = document.getElementById('nav-yaw');
        if (inputX) inputX.value = worldX;
        if (inputY) inputY.value = worldY;
        if (inputYaw) inputYaw.value = targetYaw;

        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(`NAV_POSE:${worldX},${worldY},${targetYaw}`);
            addLog('MTV1', `[MTV1] User Set Goal via Context Menu: [agent_base] ──(b: ${bLen}m)──► target[${worldX}, ${worldY}]`);
        }

        // Highlight 3D beacon
        if (targetObjectGroup) {
            targetObjectGroup.position.set(threeX, 0.0, threeZ);
            targetObjectGroup.visible = true;
        }

        // Create and render interactive OPRN Triplet: Subject -> b-vector -> Target
        const robX = agentMesh ? agentMesh.position.x : 0.0;
        const robY = agentMesh ? -agentMesh.position.z : 0.0;
        const robZ = agentMesh ? agentMesh.position.y : 0.0;

        const customOprn = {
            index: 1,
            uuid: `OPRN-INTERACTIVE-` + Math.floor(Math.random() * 89999 + 10000),
            subject_ref: 'agent_base',
            target_ref: `target_[${worldX}, ${worldY}]`,
            status: 'VALIDATED',
            b_length: parseFloat(bLen),
            b_vector: { x: bX, y: bY, z: bZ },
            start: { x: robX, y: robY, z: robZ },
            end: { x: worldX, y: worldY, z: worldZ }
        };
        renderOprnTree([customOprn]);
        updateOprn3DVisualization([customOprn]);
    });
}

// Context Menu Action 2: Navigate Here
const ctxNavHereBtn = document.getElementById('ctx-nav-here');
if (ctxNavHereBtn) {
    ctxNavHereBtn.addEventListener('click', () => {
        hideContextMenu();
        const { worldX, worldY, bX, bY, bZ, bLen, threeX, threeZ } = ctxTargetCoords;
        const targetYaw = Math.round(Math.atan2(bY, bX) * 180 / Math.PI);

        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(`NAV_POSE:${worldX},${worldY},${targetYaw}`);
            addLog('OP13', `Navigating to [${worldX}, ${worldY}] via Context Menu.`);
        }

        if (targetObjectGroup) {
            targetObjectGroup.position.set(threeX, 0.0, threeZ);
            targetObjectGroup.visible = true;
        }
    });
}

// Context Menu Action 3: Focus Camera
const ctxFocusCamBtn = document.getElementById('ctx-focus-cam');
if (ctxFocusCamBtn) {
    ctxFocusCamBtn.addEventListener('click', () => {
        hideContextMenu();
        controls.target.set(ctxTargetCoords.threeX, ctxTargetCoords.threeY, ctxTargetCoords.threeZ);
        controls.update();
        addLog('SYS', `Camera focused on [${ctxTargetCoords.worldX}, ${ctxTargetCoords.worldY}].`);
    });
}

// Context Menu Action 4: Emergency Stop
const ctxStopRobotBtn = document.getElementById('ctx-stop-robot');
if (ctxStopRobotBtn) {
    ctxStopRobotBtn.addEventListener('click', () => {
        hideContextMenu();
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send('CMD:STOP');
            addLog('OP13', '🛑 EMERGENCY STOP activated via 3D Context Menu.');
        }
    });
}

window.focusWaypoint = function(wx, wy, wz, label) {
    if (typeof controls !== 'undefined' && controls && controls.target) {
        controls.target.set(wx, wy, wz);
        controls.update();
        console.log(`[Dashboard] Focused 3D view on ${label} at [${wx}, ${wy}, ${wz}]`);
    }
};
