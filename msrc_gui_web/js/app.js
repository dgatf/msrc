/**
 * app.js — Main application entry point.
 *          Orchestrates serial communication, UI mapping, circuit diagram, and PWA.
 */

const serial = new SerialConnection();
let currentConfig = null;
let debugAutoScroll = true;

document.addEventListener('DOMContentLoaded', async () => {
    /* Check Web Serial support */
    if (!SerialConnection.isSupported()) {
        $('unsupportedBanner').style.display = 'block';
        $('btConnect').disabled = true;
        return;
    }

    /* Preload circuit images */
    await preloadCircuitImages();

    /* Bind UI events */
    bindUIEvents();
    initCircuit();

    /* Set initial visibility */
    updateVisibility();
    updateEscOptions();
    updateCurrentSensorUI();
    updateToggleables();
    generateCircuit();

    /* ── Connection ────────────────────────────────── */

    $('btConnect').addEventListener('click', async () => {
        if (serial.connected) {
            await serial.disconnect();
            setDisconnected();
            return;
        }
        try {
            await serial.requestPort();
            await serial.connect();
            setConnected();

            /* Disable debug first, wait, then request config — same as Qt GUI */
            await serial.disableDebug();
            setStatus('Requesting config...');
            setTimeout(async () => {
                try { await serial.requestConfig(); }
                catch (e) { setStatus('Error requesting config: ' + e.message); }
            }, 2000);
        } catch (e) {
            setStatus('Connection failed: ' + e.message);
        }
    });

    serial.onConfigReceived = (bytes) => {
        try {
            currentConfig = configDecode(bytes);
            if (currentConfig.version > CONFIG_VERSION) {
                setStatus('Config version mismatch! Board: ' + currentConfig.version + ', App: ' + CONFIG_VERSION);
            } else if (currentConfig.version < CONFIG_VERSION) {
                setStatus('Older config version, forcing current version.');
                currentConfig.version = CONFIG_VERSION;
            }
            setUiFromConfig(currentConfig);
            setStatus('Config loaded (v' + currentConfig.version + ', ' + bytes.length + ' bytes)');
        } catch (e) {
            setStatus('Error decoding config: ' + e.message);
        }
    };

    serial.onDebugData = (text) => {
        const el = $('ptDebug');
        el.textContent += text;
        if (debugAutoScroll) el.scrollTop = el.scrollHeight;
    };

    serial.onDisconnect = () => setDisconnected();

    /* ── Update (write config) ─────────────────────── */

    $('btUpdate').addEventListener('click', async () => {
        if (!serial.connected) return;
        try {
            const cfg = getConfigFromUi();
            const bytes = configEncode(cfg);
            await serial.writeConfig(bytes);
            setStatus('Config written. Reset RP2040 to apply.');
        } catch (e) {
            setStatus('Error writing config: ' + e.message);
        }
    });

    /* ── Default Config ────────────────────────────── */

    $('btDefaultConfig').addEventListener('click', async () => {
        if (!serial.connected) return;
        if (!confirm('Write default config to flash? This will overwrite current settings.')) return;
        try {
            await serial.writeDefaultConfig();
            setStatus('Default config written. Reset RP2040.');
        } catch (e) {
            setStatus('Error: ' + e.message);
        }
    });

    /* ── Tabs ──────────────────────────────────────── */

    document.querySelectorAll('.tab').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
            document.querySelectorAll('.tab-content').forEach(s => s.classList.remove('active'));
            btn.classList.add('active');
            $('tab-' + btn.dataset.tab).classList.add('active');
            if (btn.dataset.tab === 'circuit') generateCircuit();
        });
    });

    /* ── Debug ─────────────────────────────────────── */

    $('btDebug').addEventListener('click', async () => {
        if (!serial.connected) return;
        if (serial.debugMode) {
            await serial.disableDebug();
            $('btDebug').textContent = 'Enable Log';
            setStatus('Debug disabled');
        } else {
            await serial.enableDebug();
            $('btDebug').textContent = 'Disable Log';
            setStatus('Debug enabled');
        }
    });

    $('btClearDebug').addEventListener('click', () => {
        $('ptDebug').textContent = '';
    });

    $('btScroll').addEventListener('click', () => {
        debugAutoScroll = !debugAutoScroll;
        $('btScroll').textContent = debugAutoScroll ? 'No scroll' : 'Auto scroll';
    });

    /* ── File Open/Save ────────────────────────────── */

    $('btOpenConfig').addEventListener('click', async () => {
        const input = document.createElement('input');
        input.type = 'file';
        input.accept = '.cfg';
        input.onchange = async () => {
            const file = input.files[0];
            if (!file) return;
            const buf = await file.arrayBuffer();
            const bytes = new Uint8Array(buf);
            if (bytes.length >= CONFIG_SIZE) {
                try {
                    currentConfig = configDecode(bytes);
                    setUiFromConfig(currentConfig);
                    setStatus('Config loaded from file: ' + file.name);
                } catch (e) {
                    setStatus('Error reading file: ' + e.message);
                }
            } else {
                setStatus('File too small (' + bytes.length + ' bytes, need ' + CONFIG_SIZE + ')');
            }
        };
        input.click();
    });

    $('btSaveConfig').addEventListener('click', () => {
        const cfg = getConfigFromUi();
        const bytes = configEncode(cfg);
        const blob = new Blob([bytes], { type: 'application/octet-stream' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'msrc_config.cfg';
        a.click();
        URL.revokeObjectURL(a.href);
        setStatus('Config saved to msrc_config.cfg');
    });

    /* ── Register Service Worker ───────────────────── */
    if ('serviceWorker' in navigator) {
        navigator.serviceWorker.register('sw.js').catch(e =>
            console.warn('SW registration failed:', e)
        );
    }
});

/* ── UI state helpers ────────────────────────────────── */

function setConnected() {
    $('btConnect').textContent = 'Disconnect';
    $('btConnect').classList.add('btn-danger');
    $('btConnect').classList.remove('btn-primary');
    $('btUpdate').disabled = false;
    $('btDefaultConfig').disabled = false;
    $('btDebug').disabled = false;
    setStatus('Connected');
}

function setDisconnected() {
    $('btConnect').textContent = 'Connect';
    $('btConnect').classList.remove('btn-danger');
    $('btConnect').classList.add('btn-primary');
    $('btUpdate').disabled = true;
    $('btDefaultConfig').disabled = true;
    $('btDebug').disabled = true;
    if (serial.debugMode) {
        serial.debugMode = false;
        $('btDebug').textContent = 'Enable Log';
    }
    setStatus('Disconnected');
}

function setStatus(msg) {
    $('statusBar').textContent = msg;
}
