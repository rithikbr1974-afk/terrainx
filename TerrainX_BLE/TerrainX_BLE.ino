// TerrainX car: Bluetooth Low Energy control (phone web app via Web Bluetooth)
// Board: ESP32-CAM (AI Thinker), esp32 core 3.x. No extra libraries needed.
//
// Wiring (ESP32-CAM header -> motor driver inputs, 220 ohm in series on each):
//   GPIO13 -> drive forward    GPIO2  -> drive reverse
//   GPIO14 -> steer left       GPIO15 -> steer right
//   GPIO4  =  onboard flash LED, used as a headlight
//   5V / GND from the buck (via 1N5819 + 1000uF), GND shared with the driver.
//
// Packet from the app (6 bytes, written ~20 times a second):
//   [0xA5, throttle, steer, flags, maxPct, checksum]
//   throttle, steer: signed -100..100 (forward and right positive)
//   flags: bit0 SLOW, bit1 STOP, bit2 FULL, bit3 LIGHT  (neither SLOW nor FULL = NORMAL)
//   maxPct: 20..100, the app's MAX slider = top power of FULL mode (NORMAL and SLOW scale with it).
//           It is saved on the ESP32, so it survives a reset. 0 = keep the saved value.
//   checksum = XOR of all the bytes before it
//   (The older 5-byte packet without maxPct still works.)
// Telemetry notified back 5 times a second (7 bytes):
//   [0x5A, driveOut%, steerOut%, flags echo, status (bit0 failsafe), packets per second, maxPct]

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Preferences.h>

// ---------- names and UUIDs (the app must use the same) ----------
const char* BLE_NAME = "TerrainX-BLE";
#define SERVICE_UUID "7a1c0001-5e2b-4c8f-9f3a-1d2e3c4b5a60"
#define CTRL_UUID    "7a1c0002-5e2b-4c8f-9f3a-1d2e3c4b5a60"   // write without response
#define TELEM_UUID   "7a1c0003-5e2b-4c8f-9f3a-1d2e3c4b5a60"   // notify

// ---------- pins ----------
const int DRIVE_A = 13, DRIVE_B = 2;
const int STEER_A = 14, STEER_B = 15;
const int LIGHT_PIN = 4;

// ---------- tuning ----------
const int  PWM_FREQ = 2000;            // 1000 for the car's own transistor bridge; MX1508 is fine up to 20000
const int  PWM_BITS = 8;               // duty 0..255
const bool ACTIVE_LOW = false;         // true if the driver inputs idle HIGH
const bool INVERT_DRIVE = false, INVERT_STEER = false;

const int MAX_FULL = 255;              // hardware ceiling; the app's MAX slider picks a % of this for FULL
const int NORMAL_PCT = 75;             // NORMAL = 75% of FULL  (default driving)
const int SLOW_PCT = 40;               // SLOW   = 40% of FULL  (bridge, narrows, lining up the leap, descent)
const int MIN_DRIVE = 70;             // toy motors don't turn below ~25%: jump straight past it
const int MAX_STEER = 200;             // the steering motor stalls at its end stop; don't cook it
const int MIN_STEER = 90;
const int LIGHT_DUTY = 40;             // the flash LED is very bright and gets hot; keep it low
const int RAMP_STEP = 15;              // per 10 ms when speeding up: 0 -> full in ~0.17 s
const uint32_t REVERSE_PAUSE_MS = 60;
const uint32_t FAILSAFE_MS = 300;      // no packet for this long = stop

const uint8_t F_SLOW = 0x01, F_STOP = 0x02, F_FULL = 0x04, F_LIGHT = 0x08;

// ---------- state shared with the BLE task ----------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool havePacket = false, bleConnected = false;
volatile int8_t rxThr = 0, rxStr = 0;
volatile uint8_t rxFlags = 0, rxMax = 0;
volatile uint16_t rxCount = 0;

// ---------- motor state (loop only) ----------
int targetDrive = 0, targetSteer = 0, curDrive = 0, curSteer = 0;
uint8_t flags = 0;
uint32_t lastCmd = 0;
bool failsafe = true;
int maxPct = 100;                      // FULL mode power, set from the app, saved in flash
bool maxDirty = false;
uint32_t maxChangedAt = 0;
Preferences prefs;

BLECharacteristic* telem = nullptr;

// ---------- motors ----------
void writePin(int pin, int duty) { ledcWrite(pin, ACTIVE_LOW ? 255 - duty : duty); }

void writeMotor(int pinA, int pinB, int v, bool invert) {
  if (invert) v = -v;
  writePin(pinA, v > 0 ? v : 0);      // both off = coast; never both on (some bridges short)
  writePin(pinB, v < 0 ? -v : 0);
}

int scale(int v, int minPwm, int top) {
  v = constrain(v, -100, 100);
  if (abs(v) < 4) return 0;
  int m = map(abs(v), 4, 100, minPwm, top);
  return v > 0 ? m : -m;
}

void applyCommand(int8_t thr, int8_t str, uint8_t f, uint8_t m) {
  flags = f;
  if (m >= 20 && m <= 100 && m != maxPct) {    // the app moved the MAX slider
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
}

void updateMotors() {
  static uint32_t stoppedAt = 0;
  int want = targetDrive;
  if ((want > 0 && curDrive < 0) || (want < 0 && curDrive > 0)) want = 0;   // reverse via a stop
  if (curDrive == 0 && millis() - stoppedAt < REVERSE_PAUSE_MS) want = 0;
  int prev = curDrive;
  if (abs(want) < abs(curDrive)) curDrive = want;                          // slowing down: instant
  else if (want > curDrive) curDrive = min(want, curDrive + RAMP_STEP);    // speeding up: ramp
  else if (want < curDrive) curDrive = max(want, curDrive - RAMP_STEP);
  if (curDrive == 0 && prev != 0) stoppedAt = millis();
  curSteer = targetSteer;

  writeMotor(DRIVE_A, DRIVE_B, curDrive, INVERT_DRIVE);
  writeMotor(STEER_A, STEER_B, curSteer, INVERT_STEER);
  ledcWrite(LIGHT_PIN, (flags & F_LIGHT) ? LIGHT_DUTY : 0);
}

// ---------- BLE ----------
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) override {
    bleConnected = true;
    // ask for a 7.5-15 ms connection interval: lower lag than the phone's default
    s->updateConnParams(param->connect.remote_bda, 6, 12, 0, 400);
    Serial.println("Phone connected");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    Serial.println("Phone disconnected");
    BLEDevice::startAdvertising();       // let the app reconnect
  }
};

class CtrlCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    int n = v.length();
    if (n != 5 && n != 6) return;              // 6 bytes with maxPct, or the older 5 without
    uint8_t b[6], x = 0;
    for (int i = 0; i < n; i++) b[i] = (uint8_t)v[i];
    for (int i = 0; i < n - 1; i++) x ^= b[i];
    if (b[0] != 0xA5 || x != b[n - 1]) return;
    portENTER_CRITICAL(&mux);
    rxThr = (int8_t)b[1];
    rxStr = (int8_t)b[2];
    rxFlags = b[3];
    rxMax = n == 6 ? b[4] : 0;
    havePacket = true;
    rxCount++;
    portEXIT_CRITICAL(&mux);
  }
};

void sendTelemetry() {
  static uint32_t last = 0, lastCountAt = 0;
  static uint16_t lastCount = 0;
  static uint8_t rate = 0;
  uint32_t now = millis();
  if (now - lastCountAt >= 1000) {
    uint16_t c = rxCount;
    rate = (uint8_t)min(255, (int)(uint16_t)(c - lastCount));
    lastCount = c;
    lastCountAt = now;
  }
  if (!bleConnected || now - last < 200) return;
  last = now;
  uint8_t t[7] = {
    0x5A,
    (uint8_t)(int8_t)(curDrive * 100 / 255),
    (uint8_t)(int8_t)(curSteer * 100 / 255),
    flags,
    (uint8_t)(failsafe ? 1 : 0),
    rate,
    (uint8_t)maxPct,
  };
  telem->setValue(t, sizeof(t));
  telem->notify();
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
  BLEDevice::init(BLE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService* svc = server->createService(SERVICE_UUID);

  BLECharacteristic* ctrl = svc->createCharacteristic(
      CTRL_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  ctrl->setCallbacks(new CtrlCB());

  telem = svc->createCharacteristic(TELEM_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  telem->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("Advertising as %s\n", BLE_NAME);
}

void loop() {
  if (havePacket) {
    portENTER_CRITICAL(&mux);
    int8_t t = rxThr, s = rxStr;
    uint8_t f = rxFlags, m = rxMax;
    havePacket = false;
    portEXIT_CRITICAL(&mux);
    applyCommand(t, s, f, m);
  }

  if (maxDirty && millis() - maxChangedAt > 1500) {   // save once the slider has settled (spares the flash)
    prefs.putUChar("max", maxPct);
    maxDirty = false;
    Serial.printf("Saved MAX power %d%%\n", maxPct);
  }

  failsafe = !bleConnected || millis() - lastCmd > FAILSAFE_MS;
  if (failsafe) targetDrive = targetSteer = 0;

  static uint32_t tick = 0;
  if (millis() - tick >= 10) {
    tick = millis();
    updateMotors();
  }
  sendTelemetry();
}
