from .base import render_template, BASE_CSS, BASE_JS

_EXTRA_CSS = '''
.distance-badge {
    display: inline-block;
    padding: 4px 12px;
    border-radius: 12px;
    font-size: 13px;
    font-weight: 700;
    letter-spacing: 0.5px;
    text-transform: uppercase;
}
.distance-badge.LOST      { background: rgba(248,81,73,0.15);  border: 1px solid var(--accent-red);    color: var(--accent-red); }
.distance-badge.TOO_CLOSE { background: rgba(248,81,73,0.15);  border: 1px solid var(--accent-red);    color: var(--accent-red); }
.distance-badge.CLOSE     { background: rgba(210,153,34,0.15); border: 1px solid var(--accent-orange); color: var(--accent-orange); }
.distance-badge.GOOD      { background: rgba(63,185,80,0.15);  border: 1px solid var(--accent-green);  color: var(--accent-green); }
.distance-badge.FAR       { background: rgba(31,111,235,0.15); border: 1px solid var(--accent-blue);   color: var(--accent-blue); }

.speed-bar-row {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 6px;
    font-size: 12px;
}
.speed-bar-label { width: 62px; color: var(--text-muted); flex-shrink: 0; }
.speed-bar-track {
    flex: 1;
    height: 8px;
    background: var(--bg-sidebar);
    border-radius: 4px;
    overflow: hidden;
    border: 1px solid var(--border-color);
}
.speed-bar-fill {
    height: 100%;
    border-radius: 4px;
    background: var(--accent-blue);
    transition: width 0.15s;
}
.speed-bar-value { width: 46px; text-align: right; color: var(--text-secondary); flex-shrink: 0; font-variant-numeric: tabular-nums; }

.model-status { padding: 6px 10px; border-radius: 4px; font-size: 12px; margin-bottom: 10px; }
.model-status.ok  { background: rgba(63,185,80,0.1);  border: 1px solid rgba(63,185,80,0.3);  color: var(--accent-green); }
.model-status.err { background: rgba(248,81,73,0.1);  border: 1px solid rgba(248,81,73,0.3);  color: var(--accent-red); }
.model-status.loading { background: rgba(210,153,34,0.1); border: 1px solid rgba(210,153,34,0.3); color: #d6a63a; }

.reason-text { font-size: 11px; color: var(--text-muted); margin-top: 4px; font-style: italic; }

.key-display {
    display: grid;
    grid-template-areas: ". up ." "left down right";
    gap: 6px;
    justify-content: center;
    margin: 10px 0 6px;
}
.key-box {
    width: 32px;
    height: 32px;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid var(--border-color);
    border-radius: 4px;
    font-size: 13px;
    font-weight: 700;
    color: var(--text-muted);
    background: var(--bg-sidebar);
    transition: background 0.1s, border-color 0.1s, color 0.1s;
}
.key-box.active {
    background: rgba(63,185,80,0.2);
    border-color: var(--accent-green);
    color: var(--accent-green);
}
.key-up    { grid-area: up; }
.key-down  { grid-area: down; }
.key-left  { grid-area: left; }
.key-right { grid-area: right; }
.drive-row {
    display: flex;
    align-items: stretch;
    gap: 8px;
    margin-bottom: 8px;
}
.drive-row .button {
    flex: 1;
}

'''

_CONTENT = '''
    <div class="container">
        <div class="video-section">
            <img src="/video" class="stream">
        </div>

        <div class="controls-section">

            <!-- Control Panel -->
            <div class="card">
                <div class="card-header">Control Panel</div>
                <div style="display:flex;align-items:center;gap:12px;margin-bottom:12px">
                    <span id="run-indicator" style="width:14px;height:14px;border-radius:50%;background:#e74c3c;flex-shrink:0;display:inline-block"></span>
                    <span id="run-label" style="font-size:14px;font-weight:600;color:var(--text-secondary)">STOPPED</span>
                </div>
                <div class="drive-row">
                    <button class="button success" onclick="post('/start')">Start Run</button>
                    <button class="button danger"  onclick="post('/stop')">Stop Run</button>
                </div>
                <div class="drive-row">
                    <button class="button" id="mode-btn" onclick="toggleMode()" style="background:#555">Manual Mode</button>
                    <button class="button" onclick="post('/reset')" style="background:#444">Reset State</button>
                </div>
                <div id="key-panel" style="display:none">
                    <div class="key-display">
                        <div class="key-box key-up"    id="key-up">&#9650;</div>
                        <div class="key-box key-left"  id="key-left">&#9664;</div>
                        <div class="key-box key-down"  id="key-down">&#9660;</div>
                        <div class="key-box key-right" id="key-right">&#9654;</div>
                    </div>
                    <p style="text-align:center;font-size:11px;color:var(--text-muted);margin:4px 0 0">Use arrow keys or WASD</p>
                </div>
                <div id="ctrl-status" class="status"></div>
            </div>

            <!-- Model Status -->
            <div class="card">
                <div class="card-header">System Info</div>
                <div id="model-status" class="model-status loading">Loading model...</div>
                <div class="config-item">
                    <span class="config-label">Server</span>
                    <span class="config-value">{{ hostname }}</span>
                </div>
                <div class="config-item">
                    <span class="config-label">Lane Frames</span>
                    <span class="config-value" id="lane-frames">0</span>
                </div>
            </div>

            <!-- Target Status -->
            <div class="card">
                <div class="card-header">Lead Truck</div>
                <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:10px">
                    <span style="font-size:13px;color:var(--text-secondary)">Distance</span>
                    <span id="distance-badge" class="distance-badge LOST">LOST</span>
                </div>
                <div class="stats-grid" style="grid-template-columns:1fr 1fr 1fr;">
                    <div class="stat-box">
                        <div class="stat-value" id="target-score">—</div>
                        <div class="stat-label">Confidence</div>
                    </div>
                    <div class="stat-box">
                        <div class="stat-value" id="target-area">—</div>
                        <div class="stat-label">Box Area</div>
                    </div>
                    <div class="stat-box">
                        <div class="stat-value" id="target-bottom">—</div>
                        <div class="stat-label">Bottom</div>
                    </div>
                </div>
                <div class="reason-text" id="target-reason">waiting for target...</div>
            </div>

            <!-- Speeds -->
            <div class="card">
                <div class="card-header">
                    Wheel Output
                    <span id="multiplier-badge" style="font-size:11px;font-weight:400;color:var(--text-muted)">×1.00</span>
                </div>
                <div class="speed-bar-row">
                    <span class="speed-bar-label">Lane Left</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-lane-l" style="width:0%"></div></div>
                    <span class="speed-bar-value" id="val-lane-l">0.000</span>
                </div>
                <div class="speed-bar-row">
                    <span class="speed-bar-label">Lane Right</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-lane-r" style="width:0%"></div></div>
                    <span class="speed-bar-value" id="val-lane-r">0.000</span>
                </div>
                <div class="speed-bar-row" style="margin-top:6px">
                    <span class="speed-bar-label">Output Left</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-final-l" style="width:0%;background:var(--accent-green)"></div></div>
                    <span class="speed-bar-value" id="val-final-l">0.000</span>
                </div>
                <div class="speed-bar-row">
                    <span class="speed-bar-label">Output Right</span>
                    <div class="speed-bar-track"><div class="speed-bar-fill" id="bar-final-r" style="width:0%;background:var(--accent-green)"></div></div>
                    <span class="speed-bar-value" id="val-final-r">0.000</span>
                </div>
                <div class="reason-text" id="command-reason">—</div>
            </div>

            <!-- Tuning -->
            <div class="card">
                <div class="card-header">Speed Tuning</div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Close Target</span>
                        <span id="close-val">0.40</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="close-slider" min="0" max="1" step="0.05" value="0.40"
                               oninput="document.getElementById('close-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Good Distance</span>
                        <span id="good-val">1.00</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="good-slider" min="0" max="1.5" step="0.05" value="1.00"
                               oninput="document.getElementById('good-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Far Target</span>
                        <span id="far-val">1.20</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="far-slider" min="0" max="2" step="0.05" value="1.20"
                               oninput="document.getElementById('far-val').textContent=parseFloat(this.value).toFixed(2); sendConfig()">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Leader Speed</span>
                        <span id="leader-speed-val">0.140</span>
                    </div>
                    <div class="slider-controls">
                        <input type="range" class="slider" id="leader-speed-slider" min="0" max="0.22" step="0.005" value="0.140"
                               oninput="document.getElementById('leader-speed-val').textContent=parseFloat(this.value).toFixed(3); sendConfig()">
                    </div>
                </div>
                <div id="tune-status" class="status"></div>
            </div>

        </div>
    </div>
'''

_EXTRA_JS = '''
function post(path) {
    fetch(path, {method: 'POST'})
        .then(r => r.json())
        .then(data => {
            const msg = data.status || JSON.stringify(data);
            showStatus('ctrl-status', msg, 'success');
        })
        .catch(() => showStatus('ctrl-status', 'Request failed', 'error'));
}


let _manualMode = false;
const keyState = {up: false, down: false, left: false, right: false};
const keyMap = {
    'ArrowUp': 'up', 'w': 'up', 'W': 'up',
    'ArrowDown': 'down', 's': 'down', 'S': 'down',
    'ArrowLeft': 'left', 'a': 'left', 'A': 'left',
    'ArrowRight': 'right', 'd': 'right', 'D': 'right',
};

function clearLocalKeys() {
    Object.keys(keyState).forEach(k => keyState[k] = false);
    updateKeyDisplay();
}

function updateKeyDisplay() {
    for (const [key, active] of Object.entries(keyState)) {
        const el = document.getElementById('key-' + key);
        if (el) {
            el.classList.toggle('active', active);
        }
    }
}

function sendKeys() {
    fetch('/keys', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(keyState)
    }).catch(() => {});
}

function setManualUI(enabled) {
    _manualMode = !!enabled;

    const btn = document.getElementById('mode-btn');
    const panel = document.getElementById('key-panel');

    if (btn) {
        btn.textContent = _manualMode ? 'Auto Mode' : 'Manual Mode';
    }

    if (panel) {
        panel.style.display = _manualMode ? 'block' : 'none';
    }

    if (!_manualMode) {
        clearLocalKeys();
    }
}

function toggleMode() {
    const nextMode = !_manualMode;

    postJSON('/set_mode', {mode: nextMode ? 'manual' : 'auto'})
        .then(data => {
            setManualUI(!!data.manual_mode);
            showStatus('ctrl-status', data.status || (data.manual_mode ? 'Manual mode active' : 'Auto mode active'), 'success');
            if (!data.manual_mode) {
                sendKeys();
            }
        })
        .catch(() => showStatus('ctrl-status', 'Mode switch failed', 'error'));
}

document.addEventListener('keydown', e => {
    const dir = keyMap[e.key];

    if (dir && !keyState[dir]) {
        e.preventDefault();
        keyState[dir] = true;
        updateKeyDisplay();

        if (_manualMode) {
            sendKeys();
        }
    }
});

document.addEventListener('keyup', e => {
    const dir = keyMap[e.key];

    if (dir && keyState[dir]) {
        e.preventDefault();
        keyState[dir] = false;
        updateKeyDisplay();

        if (_manualMode) {
            sendKeys();
        }
    }
});

window.addEventListener('blur', () => {
    clearLocalKeys();

    if (_manualMode) {
        sendKeys();
    }
});

setInterval(() => {
    if (_manualMode && Object.values(keyState).some(Boolean)) {
        sendKeys();
    }
}, 150);

function setBar(barId, valId, value, maxVal) {
    const pct = Math.min(100, Math.max(0, (value / maxVal) * 100));
    document.getElementById(barId).style.width = pct + '%';
    document.getElementById(valId).textContent = value.toFixed(3);
}

function updateStatus() {
    fetch('/status')
        .then(r => r.json())
        .then(data => {
            // Running / mode indicator
            const running = data.running;
            const manual = !!data.manual_mode;
            setManualUI(manual);
            document.getElementById('run-indicator').style.background = running ? '#3fb950' : '#e74c3c';
            document.getElementById('run-label').textContent = running
                ? (manual ? 'RUNNING · MANUAL' : 'RUNNING')
                : (manual ? 'STOPPED · MANUAL' : 'STOPPED');

            // Model
            const modelEl = document.getElementById('model-status');
            if (data.model_loaded) {
                modelEl.className = 'model-status ok';
                modelEl.textContent = '✓ Model loaded';
            } else if (data.model_load_error) {
                modelEl.className = 'model-status err';
                modelEl.textContent = '✗ ' + data.model_load_error;
            } else {
                modelEl.className = 'model-status loading';
                modelEl.textContent = 'Loading model...';
            }

            document.getElementById('lane-frames').textContent = data.lane_frame_count || 0;

            // Target
            const t = data.target;
            const state = t ? t.distance_state : 'LOST';
            const badge = document.getElementById('distance-badge');
            badge.textContent = state;
            badge.className = 'distance-badge ' + state;

            document.getElementById('target-score').textContent  = t && t.found ? t.score.toFixed(2) : '—';
            document.getElementById('target-area').textContent   = t && t.found ? t.area : '—';
            document.getElementById('target-bottom').textContent = t && t.found ? t.bottom_y : '—';
            document.getElementById('target-reason').textContent = t ? t.reason : '—';

            // Speeds
            const laneMax = 1.5;
            setBar('bar-lane-l',  'val-lane-l',  data.lane_left  || 0, laneMax);
            setBar('bar-lane-r',  'val-lane-r',  data.lane_right || 0, laneMax);

            const cmd = data.command;
            const mult = cmd ? cmd.speed_multiplier : 0;
            document.getElementById('multiplier-badge').textContent = '×' + mult.toFixed(2);
            document.getElementById('command-reason').textContent = cmd ? cmd.reason : '—';

            setBar('bar-final-l', 'val-final-l', cmd ? cmd.left_speed  : 0, laneMax);
            setBar('bar-final-r', 'val-final-r', cmd ? cmd.right_speed : 0, laneMax);

            if (typeof data.leader_speed === 'number') {
                const slider = document.getElementById('leader-speed-slider');
                if (document.activeElement !== slider) {
                    slider.value = data.leader_speed.toFixed(3);
                    document.getElementById('leader-speed-val').textContent = data.leader_speed.toFixed(3);
                }
            }
        })
        .catch(() => {});
}

function sendConfig() {
    const data = {
        close_multiplier: parseFloat(document.getElementById('close-slider').value),
        good_multiplier:  parseFloat(document.getElementById('good-slider').value),
        far_multiplier:   parseFloat(document.getElementById('far-slider').value),
        leader_speed:     parseFloat(document.getElementById('leader-speed-slider').value),
    };
    postJSON('/update_config', data)
        .then(() => showStatus('tune-status', 'Tuning updated', 'success'))
        .catch(() => showStatus('tune-status', 'Tuning update failed', 'error'));
}

setInterval(updateStatus, 300);
updateStatus();
'''


CONVOYING_TEMPLATE = render_template(
    title    = "Convoy Control — Lead Truck Tracking",
    subtitle = "Lane steering stays active · speed adapts to the lead truck distance",
    content_html = _CONTENT,
    extra_css    = _EXTRA_CSS,
    extra_js     = _EXTRA_JS,
)