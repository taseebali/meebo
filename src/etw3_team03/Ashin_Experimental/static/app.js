// ~*~ 2000s Retro Meebo Teleop JavaScript Controller (with Mobile HUD & G-Pad) ~*~

let currentLinear = 0.0;
let currentAngular = 0.0;
let lastSentLinear = null;
let lastSentAngular = null;
let activeKeys = {};
let touchState = { up: false, down: false, left: false, right: false };
let isServerShutdown = false;
let isFetchPending = false;
let isMobileMode = false;

const dispSpeed = document.getElementById('disp-speed');
const dispSteer = document.getElementById('disp-steer');
const dispMode = document.getElementById('disp-mode');
const gpStatus = document.getElementById('gp-status');
const gpName = document.getElementById('gp-name');
const btnEstop = document.getElementById('btn-estop');
const btnShutdown = document.getElementById('btn-shutdown');
const sysStatus = document.getElementById('sys-status');

// Mobile HUD Elements
const btnMobileHud = document.getElementById('btn-mobile-hud');
const btnExitMobile = document.getElementById('btn-exit-mobile');
const desktopView = document.getElementById('desktop-view');
const mobileHud = document.getElementById('mobile-hud');
const mSysStatus = document.getElementById('m-sys-status');
const mDispSpeed = document.getElementById('m-disp-speed');
const mDispSteer = document.getElementById('m-disp-steer');
const btnMobileEstop = document.getElementById('btn-mobile-estop');

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
if (btnMobileEstop) btnMobileEstop.addEventListener('click', triggerEstop);

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

// Mobile Fullscreen HUD Handlers
if (btnMobileHud) {
    btnMobileHud.addEventListener('click', enterMobileHud);
}

if (btnExitMobile) {
    btnExitMobile.addEventListener('click', exitMobileHud);
}

function enterMobileHud() {
    isMobileMode = true;
    mobileHud.classList.remove('hidden');
    
    // Request browser fullscreen if available
    if (document.documentElement.requestFullscreen) {
        document.documentElement.requestFullscreen().catch(() => {});
    }
}

function exitMobileHud() {
    isMobileMode = false;
    mobileHud.classList.add('hidden');
    
    if (document.exitFullscreen) {
        document.exitFullscreen().catch(() => {});
    }
}

// Touch G-Pad Controls Wire-up
const gPadButtons = [
    { id: 'pad-up', dir: 'up' },
    { id: 'pad-down', dir: 'down' },
    { id: 'pad-left', dir: 'left' },
    { id: 'pad-right', dir: 'right' }
];

gPadButtons.forEach(item => {
    const btn = document.getElementById(item.id);
    if (btn) {
        const startHandler = (e) => {
            e.preventDefault();
            touchState[item.dir] = true;
            btn.classList.add('active');
            sendVelocityInstant();
        };

        const endHandler = (e) => {
            e.preventDefault();
            touchState[item.dir] = false;
            btn.classList.remove('active');
            sendVelocityInstant();
        };

        btn.addEventListener('touchstart', startHandler, { passive: false });
        btn.addEventListener('touchend', endHandler, { passive: false });
        btn.addEventListener('touchcancel', endHandler, { passive: false });
        btn.addEventListener('mousedown', startHandler);
        btn.addEventListener('mouseup', endHandler);
    }
});

function triggerEstop() {
    currentLinear = 0.0;
    currentAngular = 0.0;
    lastSentLinear = 0.0;
    lastSentAngular = 0.0;
    touchState = { up: false, down: false, left: false, right: false };
    activeKeys = {};
    updateDisplay();
    fetch('/api/estop', { method: 'POST', keepalive: true }).catch(() => {});
}

// WASD & Touch Velocity Calculation
function computeInputVelocity() {
    let lin = 0.0;
    let ang = 0.0;

    // Check Keyboard
    if (activeKeys['w']) lin += 1.0;
    if (activeKeys['s']) lin -= 1.0;
    if (activeKeys['a']) ang += 1.0;   // Turn Left
    if (activeKeys['d']) ang -= 1.0;   // Turn Right

    // Check Touch G-Pad
    if (touchState.up) lin += 1.0;
    if (touchState.down) lin -= 1.0;
    if (touchState.left) ang += 1.0;
    if (touchState.right) ang -= 1.0;

    // Clamp input bounds
    lin = Math.max(-1.0, Math.min(1.0, lin));
    ang = Math.max(-1.0, Math.min(1.0, ang));

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

    dispMode.innerText = (touchState.up || touchState.down || touchState.left || touchState.right) 
        ? "MOBILE G-PAD (TOUCH)" 
        : "KEYBOARD (WASD)";

    return computeInputVelocity();
}

function updateDisplay() {
    const spdStr = currentLinear.toFixed(2);
    const strStr = currentAngular.toFixed(2);

    if (dispSpeed) dispSpeed.innerText = spdStr;
    if (dispSteer) dispSteer.innerText = strStr;

    if (mDispSpeed) mDispSpeed.innerText = spdStr;
    if (mDispSteer) mDispSteer.innerText = strStr;
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
            if (sysStatus) {
                sysStatus.innerText = "ONLINE (LOW LATENCY)";
                sysStatus.className = "text-green";
            }
            if (mSysStatus) {
                mSysStatus.innerText = "ONLINE";
                mSysStatus.className = "text-green";
            }
        }
    })
    .catch(() => {
        isFetchPending = false;
        if (!isServerShutdown) {
            if (sysStatus) {
                sysStatus.innerText = "OFFLINE (PI DOWN)";
                sysStatus.className = "text-red";
            }
            if (mSysStatus) {
                mSysStatus.innerText = "OFFLINE";
                mSysStatus.className = "text-red";
            }
        }
    });
}

// Auto-detect mobile devices on page load
if (window.innerWidth < 768 || 'ontouchstart' in window) {
    if (dispMode) dispMode.innerText = "TOUCH / MOBILE DETECTED";
}

// Send velocity updates at 25 Hz (40 ms)
setInterval(sendVelocityInstant, 40);
