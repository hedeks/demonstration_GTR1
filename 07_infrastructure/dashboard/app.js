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
grid.position.y = -2.0; // Смещаем сетку глубже (-2.0), чтобы она была гарантированно ниже всех вокселей пола
grid.material.opacity = 0.2;
grid.material.transparent = true;
scene.add(grid);

// Оси координат мира (RGB = XYZ)
scene.add(new THREE.AxesHelper(1.5));

// Демонстрационный Воксель (OctoMap эмуляция)
const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
const material = new THREE.MeshBasicMaterial({ 
    color: 0x00ff88, 
    wireframe: false, 
    transparent: true, 
    opacity: 0.6 
});
const voxel = new THREE.Mesh(geometry, material);
voxel.position.set(0.5, 0.5, 0.5);
scene.add(voxel);

// Для массового рендеринга вокселей карты
let instancedVoxelsFill = null;
let instancedVoxelsWire = null;

// Отображение камеры (Агента)
const agentGeometry = new THREE.BoxGeometry(0.15, 0.15, 0.3);
const agentMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const agentMesh = new THREE.Mesh(agentGeometry, agentMaterial);
scene.add(agentMesh);
const dirHelper = new THREE.AxesHelper(0.3);
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
        camera.position.set(3, 3, 3);
        controls.target.set(0, 0, 0);
    });
}

// Анимация 3D сцены
function animate() {
    requestAnimationFrame(animate);
    
    if (isFollowingAgent && agentMesh) {
        controls.target.copy(agentMesh.position);
    }
    
    controls.update();
    voxel.rotation.x += 0.01;
    voxel.rotation.y += 0.01;
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

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            
            if (data.type === 'TRACE_LOG') {
                addLog(data.source, data.message);
            } 
            else if (data.type === 'FRAME') {
                const img = document.getElementById('video-img');
                const placeholder = document.getElementById('stream-placeholder');
                if (placeholder) placeholder.style.display = 'none';
                img.src = data.data;
            }
            else if (data.type === 'BBOX') {
                // Update canvas dimensions dynamically
                const img = document.getElementById('video-img');
                if (img && img.clientWidth > 0) {
                    cvCanvas.width = img.clientWidth;
                    cvCanvas.height = img.clientHeight;
                }
                // Draw on canvas
                ctx.clearRect(0, 0, cvCanvas.width, cvCanvas.height);
                ctx.strokeStyle = '#b366ff';
                ctx.lineWidth = 3;
                const x = data.x * cvCanvas.width;
                const y = data.y * cvCanvas.height;
                const w = data.w * cvCanvas.width;
                const h = data.h * cvCanvas.height;
                ctx.strokeRect(x - w/2, y - h/2, w, h);
                ctx.fillStyle = '#b366ff';
                ctx.font = '14px Outfit';
                ctx.fillText(`${data.label} (${data.confidence})`, x - w/2, y - h/2 - 5);
            }
            else if (data.type === 'VOXEL') {
                const v = new THREE.Mesh(geometry, material);
                v.position.set(data.x, -data.y, -data.z); // CV to Three.js coords
                scene.add(v);
            }
            else if (data.type === 'MAP') {
                if (instancedVoxelsFill) {
                    scene.remove(instancedVoxelsFill);
                    instancedVoxelsFill.dispose();
                }
                if (instancedVoxelsWire) {
                    scene.remove(instancedVoxelsWire);
                    instancedVoxelsWire.dispose();
                }
                
                const count = data.voxels.length;
                if (count > 0) {
                    const voxelCounter = document.getElementById('voxel-val');
                    if (voxelCounter) voxelCounter.innerText = count.toLocaleString();
                    
                    // Материал для серых вокселей (Opaque, с оффсетом чтобы сетка не мерцала)
                    const fillMat = new THREE.MeshBasicMaterial({
                        color: 0xcccccc, 
                        polygonOffset: true, 
                        polygonOffsetFactor: 1, 
                        polygonOffsetUnits: 1
                    });
                    
                    // Материал для черной сетки
                    const wireMat = new THREE.MeshBasicMaterial({
                        color: 0x000000, 
                        wireframe: true
                    });
                    
                    instancedVoxelsFill = new THREE.InstancedMesh(geometry, fillMat, count);
                    instancedVoxelsWire = new THREE.InstancedMesh(geometry, wireMat, count);
                    
                    const dummy = new THREE.Object3D();
                    for (let i = 0; i < count; i++) {
                        // CV to Three.js coordinates: X=X, Y=-Y, Z=-Z
                        dummy.position.set(data.voxels[i].x, -data.voxels[i].y, -data.voxels[i].z);
                        dummy.updateMatrix();
                        instancedVoxelsFill.setMatrixAt(i, dummy.matrix);
                        instancedVoxelsWire.setMatrixAt(i, dummy.matrix);
                    }
                    scene.add(instancedVoxelsFill);
                    scene.add(instancedVoxelsWire);
                }
            }
            else if (data.type === 'PATH') {
                const newPoints = data.points.map(p => new THREE.Vector3(p.x, -p.y, -p.z)); // CV to Three.js coords
                const newGeom = new THREE.BufferGeometry().setFromPoints(newPoints);
                routeLine.geometry.dispose();
                routeLine.geometry = newGeom;
            }
            else if (data.type === 'POSE') {
                agentMesh.position.set(data.x, -data.y, -data.z); // CV to Three.js coords
                
                // Преобразование кватерниона из CV в Three.js
                // Поворот вокруг X на 180 градусов (инверсия Y и Z осей)
                const cvQuat = new THREE.Quaternion(data.qx, data.qy, data.qz, data.qw);
                const fixQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
                cvQuat.premultiply(fixQuat);
                agentMesh.quaternion.copy(cvQuat);
            }
        } catch(e) {
            console.error("Failed to parse telemetry:", e);
        }
    };
}
connectWS();
