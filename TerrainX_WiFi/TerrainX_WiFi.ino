// TerrainX car: WiFi control. The ESP32 makes its own WiFi network and serves the controller page.
// Board: ESP32-CAM (AI Thinker), esp32 core 3.x. Library: "WebSockets" by Markus Sattler (Links2004).
//
// Phone: join WiFi "TerrainX-Car" (password below), open http://192.168.4.1, turn the phone sideways.
//
// Wiring (ESP32-CAM header -> motor driver inputs, 220 ohm in series on each):
//   GPIO13 -> drive forward    GPIO2  -> drive reverse
//   GPIO14 -> steer left       GPIO15 -> steer right
//   GPIO4  =  onboard flash LED, used as a headlight
//   5V / GND from the buck (via 1N5819 + 1000uF), GND shared with the driver.
//
// Messages over the WebSocket (port 81), text:
//   phone -> car  "thr,str,flags,max" ~20 times a second; "p<number>" ping
//   car -> phone  "q<number>" ping reply; "t,driveOut%,steerOut%,flags,failsafe,packetsPerSec,max" 5 times a second
//   flags: bit0 SLOW, bit1 STOP, bit2 FULL, bit3 LIGHT  (neither SLOW nor FULL = NORMAL)
//   max: 20..100, the page's MAX slider = top power of FULL mode (NORMAL and SLOW scale with it).
//        Saved on the ESP32, so it survives a reset. 0 = keep the saved value.

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>

// ---------- WiFi ----------
const char* AP_SSID = "TerrainX-Car";
const char* AP_PASS = "terrainx2026";     // at least 8 characters; change it before the event
const int   WIFI_CHANNEL = 6;             // if the venue is crowded, try 1 or 11

// ---------- pins ----------
const int DRIVE_A = 13, DRIVE_B = 2;
const int STEER_A = 14, STEER_B = 15;
const int LIGHT_PIN = 4;

// ---------- tuning ----------
const int  PWM_FREQ = 2000;            // 1000 for the car's own transistor bridge; MX1508 is fine up to 20000
const int  PWM_BITS = 8;
const bool ACTIVE_LOW = false;         // true if the driver inputs idle HIGH
const bool INVERT_DRIVE = false, INVERT_STEER = false;

const int MAX_FULL = 255;              // hardware ceiling; the page's MAX slider picks a % of this for FULL
const int NORMAL_PCT = 75;             // NORMAL = 75% of FULL  (default driving)
const int SLOW_PCT = 40;               // SLOW   = 40% of FULL  (bridge, narrows, lining up the leap, descent)
const int MIN_DRIVE = 70;             // toy motors don't turn below ~25%: jump straight past it
const int MAX_STEER = 200;
const int MIN_STEER = 90;
const int LIGHT_DUTY = 40;
const int RAMP_STEP = 15;
const uint32_t REVERSE_PAUSE_MS = 60;
const uint32_t FAILSAFE_MS = 300;

const uint8_t F_SLOW = 0x01, F_STOP = 0x02, F_FULL = 0x04, F_LIGHT = 0x08;

WebServer server(80);
WebSocketsServer ws(81);
int activeClient = -1;                 // only the newest phone drives
int targetDrive = 0, targetSteer = 0, curDrive = 0, curSteer = 0;
uint8_t flags = 0;
uint32_t lastCmd = 0;
uint16_t rxCount = 0;
bool failsafe = true;
int maxPct = 100;                      // FULL mode power, set from the page, saved in flash
bool maxDirty = false;
uint32_t maxChangedAt = 0;
Preferences prefs;

const char PAGE[] = R"html(<!doctype html><html lang=en><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name=theme-color content="#0B1120"><title>TerrainX</title>
<style>
:root{--bg:#0B1120;--card:#131C30;--line:#26324D;--text:#F2F5FA;--sub:#93A0BC;--cyan:#38C6F4;--blue:#1A73E8;--green:#1FB86A;--orange:#F07B1A;--red:#FF4D4D}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;overflow:hidden;touch-action:none;user-select:none;-webkit-user-select:none}
.app{display:grid;grid-template-columns:1fr minmax(260px,32%) 1fr;grid-template-rows:auto 1fr auto;height:100%;padding:max(10px,env(safe-area-inset-top)) max(12px,env(safe-area-inset-right)) 10px max(12px,env(safe-area-inset-left));gap:10px}
.top{grid-column:1/4;display:flex;align-items:center;gap:14px;font-size:14px;color:var(--sub)}
.brand{font-weight:700;color:var(--text);font-size:17px;letter-spacing:1px}
.dot{width:10px;height:10px;border-radius:50%;background:var(--red);box-shadow:0 0 8px var(--red)}
.dot.ok{background:var(--green);box-shadow:0 0 8px var(--green)}.dot.wait{background:#FFB020;box-shadow:0 0 8px #FFB020}
.maxc{display:flex;align-items:center;gap:8px;color:var(--text);font-weight:700;font-size:13px;letter-spacing:1px}
.maxc input{width:150px;accent-color:var(--orange);touch-action:pan-x}
.maxc b{min-width:42px;color:var(--orange);font-variant-numeric:tabular-nums}
.sp{flex:1}.pill{border:1px solid var(--line);border-radius:20px;padding:3px 10px;font-variant-numeric:tabular-nums}
.pad{position:relative;background:var(--card);border:1px solid var(--line);border-radius:22px;overflow:hidden}
.pad .lab{position:absolute;left:14px;top:10px;font-size:12px;letter-spacing:2px;color:var(--sub)}
.track{position:absolute;background:#0E1628;border-radius:40px}
#padL .track{left:50%;top:12%;bottom:12%;width:84px;transform:translateX(-50%)}
#padR .track{top:50%;left:10%;right:10%;height:84px;transform:translateY(-50%)}
.knob{position:absolute;width:84px;height:84px;border-radius:50%;background:radial-gradient(circle at 35% 30%,#7FDCFF,var(--cyan));box-shadow:0 0 22px rgba(56,198,244,.55);left:50%;top:50%;transform:translate(-50%,-50%)}
.mid{display:flex;flex-direction:column;gap:10px;min-height:0}
#stop{flex:0 0 auto;height:72px;border-radius:18px;border:2px solid var(--red);background:transparent;color:var(--red);font-size:24px;font-weight:800;letter-spacing:3px}
#stop.on{background:var(--red);color:#fff;box-shadow:0 0 26px rgba(255,77,77,.6)}
.modes{display:flex;flex-direction:column;gap:8px;flex:1;min-height:0}
.mode{flex:1;border-radius:16px;border:2px solid;background:transparent;font-size:18px;font-weight:800;letter-spacing:1px;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:44px}
.mode small{font-size:11px;font-weight:500;opacity:.8;letter-spacing:0}
.mode[data-m="0"]{border-color:var(--blue);color:var(--blue)}.mode[data-m="1"]{border-color:var(--green);color:var(--green)}.mode[data-m="2"]{border-color:var(--orange);color:var(--orange)}
.mode.on{color:#fff}.mode[data-m="0"].on{background:var(--blue);box-shadow:0 0 18px rgba(26,115,232,.6)}
.mode[data-m="1"].on{background:var(--green);box-shadow:0 0 18px rgba(31,184,106,.6)}.mode[data-m="2"].on{background:var(--orange);box-shadow:0 0 18px rgba(240,123,26,.6)}
#light{height:44px;border-radius:14px;border:2px solid var(--line);background:transparent;color:var(--sub);font-weight:700;letter-spacing:2px}
#light.on{border-color:#FFE27A;color:#FFE27A}
.bot{grid-column:1/4;display:flex;gap:16px;align-items:center;font-size:12px;color:var(--sub)}
.bar{flex:1;height:8px;background:#0E1628;border-radius:4px;position:relative;overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;left:50%;width:0;background:var(--cyan)}
#banner{position:fixed;inset:0;display:none;align-items:center;justify-content:center;font-size:56px;font-weight:900;letter-spacing:8px;color:var(--red);pointer-events:none;text-shadow:0 0 30px rgba(255,77,77,.7)}
#rot{position:fixed;inset:0;background:var(--bg);display:none;align-items:center;justify-content:center;text-align:center;font-size:20px;color:var(--sub);padding:30px}
@media (orientation:portrait) and (max-width:700px){#rot{display:flex}}
</style></head><body>
<div class=app>
 <div class=top><span class=dot id=dot></span><span class=brand>TERRAINX</span><span id=st>Connecting…</span><span class=sp></span>
  <label class=maxc>MAX <input type=range id=max min=20 max=100 step=5 value=100><b id=maxv>--%</b></label>
  <span class=pill id=ping>-- ms</span><span class=pill id=rate>-- pkt/s</span></div>
 <div class=pad id=padL><span class=lab>THROTTLE</span><div class=track></div><div class=knob id=kL></div></div>
 <div class=mid>
  <button id=stop>STOP</button>
  <div class=modes>
   <button class=mode data-m=0>SLOW<small>bridge · narrows · descent</small></button>
   <button class="mode on" data-m=1>NORMAL<small>default</small></button>
   <button class=mode data-m=2>FULL<small>climb · sand · leap</small></button>
  </div>
  <button id=light>LIGHT</button>
 </div>
 <div class=pad id=padR><span class=lab>STEERING</span><div class=track></div><div class=knob id=kR></div></div>
 <div class=bot>DRIVE<div class=bar><i id=bD></i></div>STEER<div class=bar><i id=bS></i></div><span id=keys>Keys: W A S D · 1 2 3 · Space · L</span></div>
</div>
<div id=banner>STOPPED</div>
<div id=rot>Turn your phone sideways to drive.</div>
<script>
const $=id=>document.getElementById(id);
const S={thr:0,str:0,mode:1,stop:false,light:false,max:0};let ws,ok=false,sent=0,maxTouched=false;
const buzz=ms=>{try{navigator.vibrate&&navigator.vibrate(ms)}catch(e){}};
function flags(){let f=0;if(S.mode==0)f|=1;if(S.stop)f|=2;if(S.mode==2)f|=4;if(S.light)f|=8;return f}
function setStatus(c,t){$('dot').className='dot '+c;$('st').textContent=t}
function connect(){
 setStatus('wait','Connecting…');
 ws=new WebSocket('ws://'+(location.hostname||'192.168.4.1')+':81/');
 ws.onopen=()=>{ok=true;setStatus('ok','Connected')};
 ws.onclose=()=>{ok=false;setStatus('','Disconnected · retrying');setTimeout(connect,400)};
 ws.onerror=()=>ws.close();
 ws.onmessage=e=>{const d=e.data;
  if(d[0]=='q'){$('ping').textContent=(Date.now()-Number(d.slice(1)))+' ms'}
  else if(d[0]=='t'){const v=d.split(',');bar('bD',+v[1]);bar('bS',+v[2]);$('rate').textContent=v[5]+' pkt/s';
   if(!maxTouched&&+v[6]){$('max').value=v[6];$('maxv').textContent=v[6]+'%'}
   if(+v[4]&&!S.stop&&ok)setStatus('wait','Failsafe');else if(ok)setStatus('ok','Connected')}};
}
function bar(id,p){const el=$(id);el.style.width=Math.abs(p)/2+'%';el.style.left=(p<0?50-Math.abs(p)/2:50)+'%'}
connect();
setInterval(()=>{if(ok&&ws.readyState==1){ws.send(S.thr+','+S.str+','+flags()+','+S.max);sent++}},50);
$('max').addEventListener('input',e=>{maxTouched=true;S.max=+e.target.value;$('maxv').textContent=S.max+'%';buzz(5)});
setInterval(()=>{if(ok&&ws.readyState==1)ws.send('p'+Date.now())},1000);
function pad(el,knob,vertical,set){let id=null;
 const mv=e=>{const r=el.getBoundingClientRect();
  let v=vertical?-(e.clientY-r.top-r.height/2)/(r.height*0.38):(e.clientX-r.left-r.width/2)/(r.width*0.40);
  v=Math.max(-1,Math.min(1,v));if(Math.abs(v)<0.06)v=0;set(Math.round(v*100));
  if(vertical)knob.style.top=(50-v*38)+'%';else knob.style.left=(50+v*40)+'%'};
 el.addEventListener('pointerdown',e=>{id=e.pointerId;el.setPointerCapture(id);mv(e);buzz(8)});
 el.addEventListener('pointermove',e=>{if(e.pointerId===id)mv(e)});
 const up=e=>{if(e.pointerId!==id)return;id=null;set(0);knob.style.top='50%';knob.style.left='50%'};
 el.addEventListener('pointerup',up);el.addEventListener('pointercancel',up);
}
pad($('padL'),$('kL'),true,v=>S.thr=v);pad($('padR'),$('kR'),false,v=>S.str=v);
function setMode(m){S.mode=m;document.querySelectorAll('.mode').forEach(b=>b.classList.toggle('on',+b.dataset.m==m));buzz(15)}
document.querySelectorAll('.mode').forEach(b=>b.addEventListener('pointerdown',()=>setMode(+b.dataset.m)));
function toggleStop(){S.stop=!S.stop;$('stop').classList.toggle('on',S.stop);$('stop').textContent=S.stop?'RESUME':'STOP';$('banner').style.display=S.stop?'flex':'none';buzz(S.stop?60:15)}
$('stop').addEventListener('pointerdown',toggleStop);
$('light').addEventListener('pointerdown',()=>{S.light=!S.light;$('light').classList.toggle('on',S.light);buzz(10)});
const K={};function keys(){S.thr=(K.w?100:0)-(K.s?100:0);S.str=(K.d?100:0)-(K.a?100:0)}
addEventListener('keydown',e=>{const k=e.key.toLowerCase();if('wasd'.includes(k)&&k.length==1){K[k]=1;keys()}
 if(k=='1')setMode(0);if(k=='2')setMode(1);if(k=='3')setMode(2);if(k==' '){e.preventDefault();if(!e.repeat)toggleStop()}if(k=='l'&&!e.repeat)$('light').dispatchEvent(new Event('pointerdown'))});
addEventListener('keyup',e=>{const k=e.key.toLowerCase();if('wasd'.includes(k)&&k.length==1){K[k]=0;keys()}});
document.addEventListener('visibilitychange',()=>{if(document.hidden){S.thr=S.str=0}});
</script></body></html>)html";

// ---------- motors ----------
void writePin(int pin, int duty) { ledcWrite(pin, ACTIVE_LOW ? 255 - duty : duty); }

void writeMotor(int pinA, int pinB, int v, bool invert) {
  if (invert) v = -v;
  writePin(pinA, v > 0 ? v : 0);
  writePin(pinB, v < 0 ? -v : 0);
}

int scale(int v, int minPwm, int top) {
  v = constrain(v, -100, 100);
  if (abs(v) < 4) return 0;
  int m = map(abs(v), 4, 100, minPwm, top);
  return v > 0 ? m : -m;
}

void applyCommand(int thr, int str, uint8_t f, int m) {
  flags = f;
  if (m >= 20 && m <= 100 && m != maxPct) {    // the page moved the MAX slider
    maxPct = m;
    maxDirty = true;
    maxChangedAt = millis();
  }
  if (f & F_STOP) thr = str = 0;
  int full = MAX_FULL * maxPct / 100;
  int top = (f & F_FULL) ? full : (f & F_SLOW) ? full * SLOW_PCT / 100 : full * NORMAL_PCT / 100;
  top = max(top, MIN_DRIVE + 5);               // never below the speed where the motor still turns
  targetDrive = scale(thr, MIN_DRIVE, top);
  targetSteer = scale(str, MIN_STEER, MAX_STEER);
  lastCmd = millis();
  rxCount++;
}

void updateMotors() {
  static uint32_t stoppedAt = 0;
  int want = targetDrive;
  if ((want > 0 && curDrive < 0) || (want < 0 && curDrive > 0)) want = 0;
  if (curDrive == 0 && millis() - stoppedAt < REVERSE_PAUSE_MS) want = 0;
  int prev = curDrive;
  if (abs(want) < abs(curDrive)) curDrive = want;
  else if (want > curDrive) curDrive = min(want, curDrive + RAMP_STEP);
  else if (want < curDrive) curDrive = max(want, curDrive - RAMP_STEP);
  if (curDrive == 0 && prev != 0) stoppedAt = millis();
  curSteer = targetSteer;

  writeMotor(DRIVE_A, DRIVE_B, curDrive, INVERT_DRIVE);
  writeMotor(STEER_A, STEER_B, curSteer, INVERT_STEER);
  ledcWrite(LIGHT_PIN, (flags & F_LIGHT) ? LIGHT_DUTY : 0);
}

// ---------- network ----------
void onWs(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    activeClient = num;                 // the newest phone takes over
    Serial.printf("Phone %u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    if (num == activeClient) {
      activeClient = -1;
      targetDrive = targetSteer = 0;
    }
  } else if (type == WStype_TEXT && num == activeClient) {
    char buf[32];
    len = min(len, sizeof(buf) - 1);
    memcpy(buf, payload, len);
    buf[len] = 0;
    if (buf[0] == 'p') {                // ping: send it straight back
      buf[0] = 'q';
      ws.sendTXT(num, buf);
      return;
    }
    char* p = buf;
    long thr = strtol(p, &p, 10);
    if (*p != ',') return;
    long str = strtol(p + 1, &p, 10);
    if (*p != ',') return;
    long f = strtol(p + 1, &p, 10);
    long m = *p == ',' ? strtol(p + 1, &p, 10) : 0;   // older pages send no max
    applyCommand(thr, str, (uint8_t)f, m);
  }
}

void sendTelemetry() {
  static uint32_t last = 0, lastCountAt = 0;
  static uint16_t lastCount = 0;
  static int rate = 0;
  uint32_t now = millis();
  if (now - lastCountAt >= 1000) {
    rate = (uint16_t)(rxCount - lastCount);
    lastCount = rxCount;
    lastCountAt = now;
  }
  if (activeClient < 0 || now - last < 200) return;
  last = now;
  char t[48];
  snprintf(t, sizeof(t), "t,%d,%d,%u,%d,%d,%d", curDrive * 100 / 255, curSteer * 100 / 255, flags, failsafe ? 1 : 0, rate, maxPct);
  ws.sendTXT(activeClient, t);
}

void setup() {
  for (int p : {DRIVE_A, DRIVE_B, STEER_A, STEER_B, LIGHT_PIN}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, (ACTIVE_LOW && p != LIGHT_PIN) ? HIGH : LOW);
  }
  for (int p : {DRIVE_A, DRIVE_B, STEER_A, STEER_B, LIGHT_PIN}) ledcAttach(p, PWM_FREQ, PWM_BITS);
  writeMotor(DRIVE_A, DRIVE_B, 0, false);
  writeMotor(STEER_A, STEER_B, 0, false);
  ledcWrite(LIGHT_PIN, 0);

  Serial.begin(115200);
  prefs.begin("terrainx", false);
  maxPct = constrain(prefs.getUChar("max", 100), 20, 100);
  Serial.printf("MAX power %d%%\n", maxPct);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL, 0, 2);   // max 2 phones: driver + a spare
  WiFi.setSleep(false);                                // lower lag
  server.on("/", [] { server.send(200, "text/html", PAGE); });
  server.onNotFound([] { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();
  ws.begin();
  ws.onEvent(onWs);
  Serial.printf("WiFi %s  ->  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
  server.handleClient();
  ws.loop();
  failsafe = activeClient < 0 || millis() - lastCmd > FAILSAFE_MS;
  if (failsafe) targetDrive = targetSteer = 0;
  if (maxDirty && millis() - maxChangedAt > 1500) {   // save once the slider has settled (spares the flash)
    prefs.putUChar("max", maxPct);
    maxDirty = false;
    Serial.printf("Saved MAX power %d%%\n", maxPct);
  }
  static uint32_t tick = 0;
  if (millis() - tick >= 10) {
    tick = millis();
    updateMotors();
  }
  sendTelemetry();
}
