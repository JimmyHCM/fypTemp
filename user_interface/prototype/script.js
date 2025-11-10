const screens = document.querySelectorAll('.screen');
const navButtons = document.querySelectorAll('.nav-btn');
const modeTiles = document.querySelectorAll('.mode-tile[data-target]');
const activeModeLabel = document.getElementById('active-mode-label');
const toastTemplate = document.getElementById('toast-template');
const generateBtn = document.getElementById('generate-btn');
const endReturnBtn = document.getElementById('end-return-btn');
const confirmDialog = document.getElementById('confirm-dialog');
const recordBtn = document.getElementById('record-btn');
const savePresetBtn = document.getElementById('save-preset-btn');
const connectivityPill = document.getElementById('connectivity-pill');
const connectivityMetric = document.getElementById('connectivity-metric');
const connectivityDetail = document.getElementById('connectivity-detail');
const connectivityCard = document.getElementById('connectivity-card');
const liveTiles = document.querySelectorAll('.live-tile');

const ROS_CONFIG = {
    url: (() => {
        const searchParams = new URLSearchParams(window.location.search);
        const queryUrl = searchParams.get('rosbridge');
        const storedUrl = window.localStorage?.getItem('rosbridgeUrl');
        const chosen = queryUrl || storedUrl || 'ws://localhost:9090';
        if (queryUrl && window.localStorage) {
            window.localStorage.setItem('rosbridgeUrl', queryUrl);
        }
        return chosen;
    })(),
    reconnectDelayMs: 4000,
    autoReconnect: true,
};

let ros = null;
let reconnectTimer = null;
let missionStateTopic = null;
let suppressNextReconnect = false;

function showScreen(id) {
    screens.forEach(section => {
        section.classList.toggle('active', section.id === id);
        if (section.classList.contains('active')) {
            activeModeLabel.textContent = section.dataset.screen;
        }
    });
    navButtons.forEach(btn => {
        btn.classList.toggle('active', btn.dataset.target === id);
    });
    window.scrollTo({ top: 0, behavior: 'smooth' });
}

navButtons.forEach(btn => {
    btn.addEventListener('click', () => showScreen(btn.dataset.target));
});

modeTiles.forEach(tile => {
    tile.addEventListener('click', () => showScreen(tile.dataset.target));
});

// Update range outputs live.
document.querySelectorAll('input[type="range"]').forEach(range => {
    const outputId = range.dataset.output;
    const output = document.getElementById(outputId);
    if (!output) return;
    const updateOutput = () => {
        output.textContent = range.id === 'speed' ? `${range.value} m/s` : `${range.value}`;
        if (outputId === 'overlap-output') {
            output.textContent = `${range.value}%`;
        }
        if (outputId === 'lane-output') {
            output.textContent = `${Number(range.value).toFixed(1)} m`;
        }
        if (outputId === 'speed-output') {
            output.textContent = `${Number(range.value).toFixed(1)} m/s`;
        }
    };
    updateOutput();
    range.addEventListener('input', updateOutput);
});

function spawnToast(message) {
    const fragment = toastTemplate.content.cloneNode(true);
    const toast = fragment.querySelector('.toast');
    toast.textContent = message;
    document.body.appendChild(toast);
    setTimeout(() => toast.remove(), 2600);
}

generateBtn?.addEventListener('click', () => {
    spawnToast('Optimized path generated (94% coverage)');
});

endReturnBtn?.addEventListener('click', () => {
    if (typeof confirmDialog.showModal === 'function') {
        confirmDialog.showModal();
    } else {
        spawnToast('Confirm return in final build');
    }
});

confirmDialog?.addEventListener('close', () => {
    if (confirmDialog.returnValue === 'confirm') {
        spawnToast('AquaSweep returning to dock');
    }
});

let isRecording = false;
recordBtn?.addEventListener('click', () => {
    isRecording = !isRecording;
    recordBtn.textContent = isRecording ? 'Stop recording' : 'Start recording';
    recordBtn.classList.toggle('danger', isRecording);
    savePresetBtn.toggleAttribute('disabled', !isRecording);
    spawnToast(isRecording ? 'Manual run recording started' : 'Manual run saved');
    if (!isRecording) {
        savePresetBtn.textContent = 'Save as preset';
    } else {
        savePresetBtn.textContent = 'Recording…';
    }
});

savePresetBtn?.addEventListener('click', () => {
    if (savePresetBtn.disabled) return;
    spawnToast('Preset draft ready to name');
    savePresetBtn.textContent = 'Saved';
    savePresetBtn.disabled = true;
});

// Allow linking cards to manual screen.
document.querySelectorAll('[data-target="manual-screen"]').forEach(btn => {
    btn.addEventListener('click', () => showScreen('manual-screen'));
});

// Toggle light mode through keyboard shortcut to preview contrast.
document.addEventListener('keydown', evt => {
    if (evt.key.toLowerCase() === 'l' && evt.metaKey) {
        document.body.classList.toggle('light');
        spawnToast(document.body.classList.contains('light') ? 'Light mode preview' : 'Dark mode preview');
    }
});

function setConnectivityState(state, detail) {
    if (!connectivityPill || !connectivityMetric || !connectivityDetail) {
        return;
    }
    const descriptors = {
        connecting: {
            pillText: 'Bridge linking…',
            pillState: 'info',
            metric: 'Linking',
            detail: detail || `Connecting to ${ROS_CONFIG.url}`,
        },
        connected: {
            pillText: 'Connected',
            pillState: 'good',
            metric: 'Online',
            detail: detail || `Connected to ${ROS_CONFIG.url}`,
        },
        error: {
            pillText: 'Disconnected',
            pillState: 'caution',
            metric: 'Offline',
            detail: detail || 'Retrying shortly…',
        },
    };
    const descriptor = descriptors[state] || descriptors.error;
    connectivityPill.dataset.state = descriptor.pillState;
    connectivityPill.textContent = descriptor.pillText;
    connectivityMetric.textContent = descriptor.metric;
    connectivityDetail.textContent = descriptor.detail;
}

function scheduleReconnect() {
    if (!ROS_CONFIG.autoReconnect) {
        return;
    }
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(() => {
        connectToRosbridge();
    }, ROS_CONFIG.reconnectDelayMs);
}

function updateLiveTileState(key) {
    if (!liveTiles?.length) {
        return;
    }
    liveTiles.forEach(tile => {
        tile.classList.toggle('live-active', tile.dataset.state === key);
    });
}

function interpretMissionState(state) {
    const value = (state || '').toLowerCase();
    if (value.includes('return')) {
        return 'returning';
    }
    if (value.includes('pause') || value.includes('idle') || value.includes('dock') || value.includes('manual')) {
        return 'paused';
    }
    return 'active';
}

function attachMissionStateSubscription() {
    if (!ros || !window.ROSLIB) {
        return;
    }
    missionStateTopic = new ROSLIB.Topic({
        ros,
        name: '/mission_state',
        messageType: 'std_msgs/msg/String',
    });
    missionStateTopic.subscribe(message => {
        const missionState = message?.data || 'Unknown';
        const tileKey = interpretMissionState(missionState);
        updateLiveTileState(tileKey);
        connectivityDetail.textContent = `Mission state: ${missionState}`;
    });
}

function teardownMissionStateSubscription() {
    if (missionStateTopic) {
        missionStateTopic.unsubscribe();
        missionStateTopic = null;
    }
}

function connectToRosbridge() {
    if (!window.ROSLIB) {
        console.warn('roslib.js not loaded; skipping ROS connection');
        setConnectivityState('error', 'roslib.js missing');
        return;
    }
    clearTimeout(reconnectTimer);
    if (ros) {
        suppressNextReconnect = true;
        try {
            ros.close();
        } catch (error) {
            console.warn('Failed to close previous ROS connection', error);
        }
    }
    ros = new ROSLIB.Ros({ url: ROS_CONFIG.url });
    setConnectivityState('connecting', `Connecting to ${ROS_CONFIG.url}`);

    ros.on('connection', () => {
        setConnectivityState('connected', `Connected to ${ROS_CONFIG.url}`);
        spawnToast('Connected to ROS 2');
        teardownMissionStateSubscription();
        attachMissionStateSubscription();
        window.ros = ros;
    });

    ros.on('error', error => {
        console.error('rosbridge error', error);
        setConnectivityState('error', 'Connection error. Retrying…');
        teardownMissionStateSubscription();
        ros = null;
        scheduleReconnect();
    });

    ros.on('close', () => {
        setConnectivityState('error', 'Disconnected. Retrying…');
        teardownMissionStateSubscription();
        ros = null;
        if (suppressNextReconnect) {
            suppressNextReconnect = false;
            return;
        }
        scheduleReconnect();
    });
}

if (connectivityCard) {
    connectivityCard.style.cursor = 'pointer';
    connectivityCard.setAttribute('title', 'Tap to change rosbridge URL');
}

connectivityCard?.addEventListener('click', () => {
    const nextUrl = window.prompt('ROS bridge WebSocket URL', ROS_CONFIG.url);
    if (!nextUrl || nextUrl === ROS_CONFIG.url) {
        return;
    }
    ROS_CONFIG.url = nextUrl;
    try {
        window.localStorage?.setItem('rosbridgeUrl', nextUrl);
    } catch (error) {
        console.warn('Unable to persist rosbridge URL', error);
    }
    connectToRosbridge();
});

if (connectivityPill) {
    setConnectivityState('connecting', `Connecting to ${ROS_CONFIG.url}`);
    connectToRosbridge();
}
