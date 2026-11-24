Build a web app called "TerrainX Driver": a landscape phone controller that drives an RC car through an ESP32 over Bluetooth Low Energy, using the Web Bluetooth API. React + TypeScript, no backend, no extra libraries beyond what you need for React. It must work in Chrome on Android. Give me every file in full, no TODOs and no placeholder code.

## 1. Bluetooth link (must match the ESP32 exactly)
- Device name prefix: "TerrainX". Service UUID: `7a1c0001-5e2b-4c8f-9f3a-1d2e3c4b5a60`.
- Control characteristic `7a1c0002-5e2b-4c8f-9f3a-1d2e3c4b5a60`: write with `writeValueWithoutResponse`.
- Telemetry characteristic `7a1c0003-5e2b-4c8f-9f3a-1d2e3c4b5a60`: subscribe with `startNotifications`.
- Connect: a big "Connect car" button calls `navigator.bluetooth.requestDevice({ filters: [{ namePrefix: "TerrainX" }], optionalServices: [SERVICE_UUID] })`. This needs a user tap, so never call it automatically.
- Auto-reconnect: on `gattserverdisconnected`, call `device.gatt.connect()` again every 1 s, forever, until it works. No new tap is needed for the same device object. Show the state in the status bar: green "Connected", amber "Reconnecting…", red "Disconnected".
- If `navigator.bluetooth` is missing, show a full-screen message: "Open this app in Chrome on Android (HTTPS). iPhone Safari does not support Web Bluetooth."
- Never send two writes at once. Keep one async send loop that awaits each write before the next.

## 2. Control packet (6 bytes, sent every 50 ms while connected, even when nothing changes)
`[0xA5, throttle, steer, flags, maxPct, checksum]`
- throttle and steer: signed byte, -100..100 (forward and right are positive). Store negatives as two's complement (e.g. -100 = 0x9C).
- flags: bit0 (0x01) = SLOW mode, bit1 (0x02) = STOP, bit2 (0x04) = FULL mode, bit3 (0x08) = LIGHT. NORMAL mode = neither bit0 nor bit2.
- maxPct: 20..100, the MAX POWER slider (top power of FULL mode; the car scales NORMAL to 75% and SLOW to 40% of it). Send 0 until the slider value is known (0 = "keep the value saved on the car").
- checksum = byte0 XOR byte1 XOR byte2 XOR byte3 XOR byte4.
- Examples: full forward in NORMAL, max 100 = `A5 64 00 00 64 A5`; full forward in FULL, max 80 = `A5 64 00 04 50 95`; STOP, max unknown = `A5 00 00 02 00 A7`; full left in SLOW with light, max 60 = `A5 00 9C 09 3C 0C`.
- The car stops by itself if it gets no packet for 300 ms, so the 50 ms loop must never pause while connected.
- When the page is hidden (`visibilitychange`) or the phone locks, send throttle 0 and steer 0.

## 3. Telemetry (7 bytes, notified 5 times a second)
`[0x5A, driveOut, steerOut, flagsEcho, status, packetsPerSecond, maxPct]`
- maxPct is the MAX POWER value saved on the car. When the app connects, set the slider to it (until the user moves the slider), then send the slider value in every packet. This way the car remembers the setting between runs.
- driveOut and steerOut: signed byte, -100..100, the car's actual motor output after ramping.
- status bit0 = 1 means the car is in failsafe (no recent packets).
- Show driveOut and steerOut as two centred bar meters at the bottom. Show packetsPerSecond as "NN pkt/s" (should be about 20). If status bit0 is set while connected, show an amber "Failsafe" badge.

## 4. Layout (landscape, dark theme, readable in sunlight)
- Colours: background #0B1120, cards #131C30, borders #26324D, text #F2F5FA, muted #93A0BC, accent cyan #38C6F4.
- Top bar: status dot, "TERRAINX" wordmark, status text, then on the right a MAX POWER slider, a "pkt/s" pill and a "Connect / Disconnect" button.
- MAX POWER slider: label "MAX", range 20–100 in steps of 5, orange (#F07B1A) thumb and track, the value shown beside it as "85%" in bold orange. At least 160 px wide with a 44 px tall touch area. A light haptic tick on each step. Dragging it must never move the throttle or steering pads.
- Three columns below:
  - Left: THROTTLE pad. A vertical track with a glowing cyan knob. Drag up/down for -100..100. It springs back to centre on release.
  - Middle: a big STOP button (red outline; when active it's filled red, text changes to "RESUME", and "STOPPED" shows across the whole screen). Below it, three mode buttons stacked: SLOW (blue #1A73E8, subtitle "bridge · narrows · descent"), NORMAL (green #1FB86A, subtitle "default"), FULL (orange #F07B1A, subtitle "climb · sand · leap"). The active mode is filled and glows; the others are outlined. Under them, a LIGHT toggle (yellow when on).
  - Right: STEERING pad. A horizontal track with the same knob. Drag left/right for -100..100. It springs back to centre.
- Both pads are multi-touch at the same time: use Pointer Events with `setPointerCapture`, one pointer per pad.
- Dead zone 6% at the centre of each pad.
- The app starts in NORMAL with STOP off and LIGHT off.
- Short haptic tap (`navigator.vibrate`) on mode change, STOP and LIGHT.
- In portrait, show a "Turn your phone sideways" screen.
- A "Full screen" icon button that calls `requestFullscreen()` then `screen.orientation.lock("landscape")` (ignore errors).
- Keep the screen awake while connected with `navigator.wakeLock.request("screen")`. Request it again when the page becomes visible.

## 5. Other inputs
- Keyboard (for testing on a laptop): W/S throttle, A/D steer, 1/2/3 = SLOW/NORMAL/FULL, Space = STOP toggle, L = light.
- Gamepad API: if a gamepad is connected, left stick Y (inverted) = throttle, right stick X = steer (fall back to left stick X if there's only one stick). Dead zone 0.08. Buttons: X = SLOW, A = NORMAL, Y = FULL, Start = STOP toggle. Show the gamepad name in the top bar.
- Whichever input moved last controls throttle and steer.

## 6. Debug line
- A small line at the very bottom shows the last packet sent in hex (e.g. "A5 64 00 04 C5"), the mode and the throttle/steer values. This is for checking against the ESP32 Serial Monitor.

## 7. Code structure (keep it simple)
- `ble.ts`: connect, reconnect loop, send loop, telemetry parsing.
- `packet.ts`: builds the 5-byte packet with the checksum. Include a small self-test that checks the four example packets above and logs the result to the console on startup.
- `state.ts`: one state store (throttle, steer, mode, stop, light, connection status, telemetry).
- `App.tsx` and components for the pads, buttons, meters and status bar.
- No analytics, no network calls, no login.
