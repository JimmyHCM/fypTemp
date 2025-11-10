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
