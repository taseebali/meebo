// Teleop Controller JS
let currentLinear = 0.0;
let currentAngular = 0.0;
let activeKeys = {};
let sendInterval = null;

const dispSpeed = document.getElementById('disp-speed');
const dispSteer = document.getElementById('disp-steer');
const dispMode = document.getElementById('disp-mode');
const gpStatus = document.getElementById('gamepad-status');
const gpName = document.getElementById('gp-name');
const stickDot = document.getElementById('stick-dot');
const btnEstop = document.getElementById('btn-estop');

// Key state mapping
const KEY_MAP = {
    'KeyW': 'w',
    'KeyS': 's',
    'KeyA': 'a',
    'KeyD': 'd'
};

window.addEventListener('keydown', (e) => {
    if (KEY_MAP[e.code]) {
        activeKeys[KEY_MAP[e.code]] = true;
        document.getElementById(`key-${KEY_MAP[e.code]}`).classList.add('active');
    } else if (e.code === 'Space') {
        triggerEstop();
        document.getElementById('key-space').style.background = '#da3633';
    }
});

window.addEventListener('keyup', (e) => {
    if (KEY_MAP[e.code]) {
        activeKeys[KEY_MAP[e.code]] = false;
        document.getElementById(`key-${KEY_MAP[e.code]}`).classList.remove('active');
    } else if (e.code === 'Space') {
        document.getElementById('key-space').style.background = '#161b22';
    }
});

btnEstop.addEventListener('click', triggerEstop);

function triggerEstop() {
    currentLinear = 0.0;
    currentAngular = 0.0;
    updateDisplay();
    fetch('/api/estop', { method: 'POST' });
}

// Compute velocity from active keyboard keys
function computeKeyboardVelocity() {
    let lin = 0.0;
    let ang = 0.0;

    if (activeKeys['w']) lin += 1.0;
    if (activeKeys['s']) lin -= 1.0;
    if (activeKeys['a']) ang += 1.0;   // Turn left
    if (activeKeys['d']) ang -= 1.0;   // Turn right

    return { linear: lin, angular: ang };
}

// Gamepad Polling Loop (XInput / HTML5 Gamepad API)
function pollGamepad() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    let gp = null;
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) { gp = gamepads[i]; break; }
    }

    if (gp) {
        gpStatus.innerHTML = `<span class="dot cyan"></span> GAMEPAD: CONNECTED`;
        gpName.innerText = gp.id.substring(0, 30);
        
        // Left Stick Y (Axis 1) for Linear, Left Stick X (Axis 0) or Right Stick X (Axis 2/3) for Angular
        let rawLin = -gp.axes[1];  // Invert Y axis
        let rawAng = -gp.axes[0];  // Invert X axis for standard steer

        // Triggers (RT = axis 5 or button 7, LT = axis 2 or button 6)
        if (gp.buttons[7] && gp.buttons[7].value > 0.1) rawLin = gp.buttons[7].value;
        if (gp.buttons[6] && gp.buttons[6].value > 0.1) rawLin = -gp.buttons[6].value;

        // Apply Deadzone (5%)
        let lin = Math.abs(rawLin) > 0.05 ? rawLin : 0.0;
        let ang = Math.abs(rawAng) > 0.05 ? rawAng : 0.0;

        // Visualizer Update
        const offsetX = ang * 30;
        const offsetY = -lin * 30;
        stickDot.style.transform = `translate(calc(-50% + ${offsetX}px), calc(-50% + ${offsetY}px))`;

        if (Math.abs(lin) > 0.05 || Math.abs(ang) > 0.05) {
            dispMode.innerText = "GAMEPAD (XINPUT)";
            return { linear: lin, angular: ang };
        }
    } else {
        gpStatus.innerHTML = `<span class="dot gray"></span> GAMEPAD: NONE`;
        gpName.innerText = "Plug in any Xbox / USB Controller";
        stickDot.style.transform = `translate(-50%, -50%)`;
    }

    dispMode.innerText = "KEYBOARD (WASD)";
    return computeKeyboardVelocity();
}

function updateDisplay() {
    dispSpeed.innerText = currentLinear.toFixed(2);
    dispSteer.innerText = currentAngular.toFixed(2);
}

// Send velocity updates at 25 Hz (40 ms interval)
setInterval(() => {
    const cmd = pollGamepad();
    currentLinear = cmd.linear;
    currentAngular = cmd.angular;
    updateDisplay();

    fetch('/api/cmd_vel', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ linear: currentLinear, angular: currentAngular })
    }).catch(err => console.error("Telemetry send error", err));
}, 40);
