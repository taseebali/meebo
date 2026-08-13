// ~*~ 2000s Retro Meebo Teleop JavaScript Controller ~*~

let currentLinear = 0.0;
let currentAngular = 0.0;
let lastSentLinear = null;
let lastSentAngular = null;
let activeKeys = {};
let isServerShutdown = false;
let isFetchPending = false;

const dispSpeed = document.getElementById('disp-speed');
const dispSteer = document.getElementById('disp-steer');
const dispMode = document.getElementById('disp-mode');
const gpStatus = document.getElementById('gp-status');
const gpName = document.getElementById('gp-name');
const btnEstop = document.getElementById('btn-estop');
const btnShutdown = document.getElementById('btn-shutdown');
const sysStatus = document.getElementById('sys-status');

// WASD Keyboard Listeners
const KEY_MAP = {
    'KeyW': 'w',
    'KeyS': 's',
    'KeyA': 'a',
    'KeyD': 'd'
};

window.addEventListener('keydown', (e) => {
    if (isServerShutdown) return;
    if (KEY_MAP[e.code]) {
        activeKeys[KEY_MAP[e.code]] = true;
        const btn = document.getElementById(`key-${KEY_MAP[e.code]}`);
        if (btn) btn.classList.add('active');
        sendVelocityInstant();
    } else if (e.code === 'Space') {
        triggerEstop();
    }
});

window.addEventListener('keyup', (e) => {
    if (KEY_MAP[e.code]) {
        activeKeys[KEY_MAP[e.code]] = false;
        const btn = document.getElementById(`key-${KEY_MAP[e.code]}`);
        if (btn) btn.classList.remove('active');
        sendVelocityInstant();
    }
});

btnEstop.addEventListener('click', triggerEstop);

if (btnShutdown) {
    btnShutdown.addEventListener('click', () => {
        if (confirm("Are you sure you want to shut down the Meebo Teleop Server and release the camera?")) {
            isServerShutdown = true;
            sysStatus.innerText = "SERVER SHUTTING DOWN...";
            sysStatus.className = "text-yellow";
            fetch('/api/shutdown', { method: 'POST', keepalive: true })
            .then(() => {
                sysStatus.innerText = "SERVER OFF / CAMERA RELEASED";
                sysStatus.className = "text-red";
            })
            .catch(() => {
                sysStatus.innerText = "SERVER OFF / CAMERA RELEASED";
                sysStatus.className = "text-red";
            });
        }
    });
}

function triggerEstop() {
    currentLinear = 0.0;
    currentAngular = 0.0;
    lastSentLinear = 0.0;
    lastSentAngular = 0.0;
    updateDisplay();
    fetch('/api/estop', { method: 'POST', keepalive: true }).catch(() => {});
}

// WASD Velocity Calculation
function computeKeyboardVelocity() {
    let lin = 0.0;
    let ang = 0.0;

    if (activeKeys['w']) lin += 1.0;
    if (activeKeys['s']) lin -= 1.0;
    if (activeKeys['a']) ang += 1.0;   // Turn Left
    if (activeKeys['d']) ang -= 1.0;   // Turn Right

    return { linear: lin, angular: ang };
}

// HTML5 Gamepad / XInput API Poller
function pollGamepad() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    let gp = null;

    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
            gp = gamepads[i];
            break;
        }
    }

    if (gp) {
        gpStatus.innerText = "CONNECTED";
        gpStatus.className = "text-green";
        gpName.innerText = gp.id;

        let rawLin = -gp.axes[1];  // Left Stick Y (Inverted)
        let rawAng = -gp.axes[0];  // Left Stick X (Inverted)

        // Triggers support (RT / LT)
        if (gp.buttons[7] && gp.buttons[7].value > 0.1) rawLin = gp.buttons[7].value;
        if (gp.buttons[6] && gp.buttons[6].value > 0.1) rawLin = -gp.buttons[6].value;

        // Apply 5% Deadzone Filter
        let lin = Math.abs(rawLin) > 0.05 ? rawLin : 0.0;
        let ang = Math.abs(rawAng) > 0.05 ? rawAng : 0.0;

        if (Math.abs(lin) > 0.05 || Math.abs(ang) > 0.05) {
            dispMode.innerText = "GAMEPAD (XINPUT)";
            return { linear: lin, angular: ang };
        }
    } else {
        gpStatus.innerText = "DISCONNECTED";
        gpStatus.className = "text-red";
        gpName.innerText = "No Gamepad Connected (Plug in Xbox / USB Controller)";
    }

    dispMode.innerText = "KEYBOARD (WASD)";
    return computeKeyboardVelocity();
}

function updateDisplay() {
    dispSpeed.innerText = currentLinear.toFixed(2);
    dispSteer.innerText = currentAngular.toFixed(2);
}

function sendVelocityInstant() {
    if (isServerShutdown) return;
    const cmd = pollGamepad();
    currentLinear = cmd.linear;
    currentAngular = cmd.angular;
    updateDisplay();

    // Skip redundant network calls if values haven't changed and a request is in-flight
    if (isFetchPending && currentLinear === lastSentLinear && currentAngular === lastSentAngular) {
        return;
    }

    lastSentLinear = currentLinear;
    lastSentAngular = currentAngular;
    isFetchPending = true;

    fetch('/api/cmd_vel', {
        method: 'POST',
        headers: { 
            'Content-Type': 'application/json',
            'Connection': 'keep-alive'
        },
        body: JSON.stringify({ linear: currentLinear, angular: currentAngular }),
        keepalive: true
    })
    .then(res => {
        isFetchPending = false;
        if (res.ok) {
            sysStatus.innerText = "ONLINE (LOW LATENCY)";
            sysStatus.className = "text-green";
        }
    })
    .catch(() => {
        isFetchPending = false;
        if (!isServerShutdown) {
            sysStatus.innerText = "OFFLINE (PI DOWN)";
            sysStatus.className = "text-red";
        }
    });
}

// Send velocity updates at 25 Hz (40 ms)
setInterval(sendVelocityInstant, 40);
