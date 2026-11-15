// DMM v1 stage B firmware: Arduino Mega + ADS124S08 + ADR4525 + 8 latching relays (TQ2-L2-5V via ULN2803A).
// Design: dmm/stageB.md. Build steps: dmm/stageB-build.md. ADC values: TI SBAS660C (ADS124S08 datasheet).
//
// Serial 115200 baud, one command per line (Arduino Serial Monitor, "Newline"). Type "help".
//
// Everything is measured against VCOM (about 2.45 V, the LO terminal), never against GND.
// The ADC reference is the ADR4525 on REFP0 by default ("ref int" switches to the internal 2.5 V).
//
// Every result line starts with a keyword and is comma separated (key=value), so tools/dmmlog.py
// can record it as CSV (for example: dmmlog.py repeat COM5 "meas 10" --every 10 --count 60).

#include <SPI.h>
#include <EEPROM.h>

#define USE_OLED 0  // 1 once the OLED is wired and the Adafruit SSD1306 library is installed (not built yet)
#if USE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;
#endif

// ---------- pins (dmm/stageB.md, sheet 3 of stageB.schjson) ----------

const uint8_t PIN_CS = 53;
const uint8_t PIN_DRDY = 2;
const uint8_t PIN_START = 4;
const uint8_t PIN_RESET = 5;
// Relay k (1..8): set coil on D(20 + 2k), reset coil on D(21 + 2k). K1 = D22/D23 ... K8 = D36/D37.
const uint8_t RELAY_FIRST_PIN = 22;
const uint8_t RELAY_COUNT = 8;

// ---------- ADC (same register values as stage A) ----------

const uint8_t CMD_RESET = 0x06;
const uint8_t CMD_START = 0x08;
const uint8_t CMD_RDATA = 0x12;
const uint8_t CMD_RREG = 0x20;
const uint8_t CMD_WREG = 0x40;

const uint8_t REG_ID = 0x00;
const uint8_t REG_STATUS = 0x01;
const uint8_t REG_INPMUX = 0x02;
const uint8_t REG_PGA = 0x03;
const uint8_t REG_DATARATE = 0x04;
const uint8_t REG_REF = 0x05;
const uint8_t REG_SYS = 0x09;

const uint8_t REF_INTERNAL = 0x39;  // internal 2.5 V, reference buffers off
const uint8_t REF_EXTERNAL = 0x11;  // REFP0/REFN0 (ADR4525), REFP buffer on, internal reference kept on
const uint8_t SYS_DEFAULT = 0x10;
const uint8_t MUX_COM = 12;  // AINCOM: not connected on the stage B board

const SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

// Analog inputs (stageB.md, "アナログ入力の割り当て")
const uint8_t AIN_DIRECT = 0, AIN_DIV_A = 1, AIN_DIV_B = 2, AIN_VCOM = 3;
const uint8_t AIN_HI_S = 4, AIN_LO_S = 5, AIN_RTOP = 6, AIN_RBOT = 7, AIN_ADRREF = 8, AIN_REFOUT = 9;

const int32_t CODE_OVER = 8380000;  // |code| above this is treated as over range (full scale 8388607)

float vref = 2.5;
bool refExternal = true;
uint8_t rateCode = 4;  // 20 SPS, low-latency filter: rejects 50 Hz and 60 Hz

// ---------- measurement modes ----------

enum Kind : uint8_t { VOLTS, OHMS };

struct Mode {
  const char *name;
  Kind kind;
  uint8_t relays;  // bit k-1 = relay k set
  uint8_t p;       // volts: ADC input against VCOM
  uint8_t gain;    // volts: PGA gain
  float rrefNominal;  // ohms: reference resistor
};

// One reading in a mode (declared up here: the Arduino tools put function prototypes above the code)
struct Reading {
  double value;   // volts or ohms
  double sd;      // same unit
  double a, b;    // volts: ADC volts and the offset-corrected tap volts; ohms: V(Rx) and V(Rref)
  bool over;
};

// Relay bits
const uint8_t K1 = 1, K2 = 2, K3 = 4, K4 = 8, K5 = 16, K6 = 32, K7 = 64, K8 = 128;

const Mode MODES[] = {
  {"v100m", VOLTS, K7, AIN_DIRECT, 16, 0},
  {"v1", VOLTS, K7, AIN_DIRECT, 2, 0},
  {"v10", VOLTS, K6, AIN_DIV_A, 2, 0},
  {"v30", VOLTS, K6, AIN_DIV_B, 8, 0},
  {"r100", OHMS, K1, 0, 1, 1e3},
  {"r1k", OHMS, K1, 0, 1, 1e3},
  {"r10k", OHMS, K2, 0, 1, 1e4},
  {"r100k", OHMS, K3, 0, 1, 1e5},
  {"r1m", OHMS, K4, 0, 1, 1e6},
  {"r10m", OHMS, K5, 0, 1, 1e7},
};
const uint8_t MODE_COUNT = sizeof(MODES) / sizeof(MODES[0]);
const uint8_t VOLT_MODES = 4;  // MODES[0..3]
const uint8_t OHM_MODES = 6;   // MODES[4..9]
const uint8_t MODE_NONE = 255;

uint8_t modeIndex = MODE_NONE;
bool fourWire = false;
uint8_t relayState = 0;      // what the firmware last pulsed (bit = set)
bool relayManual = false;    // a relay was moved by hand: the mode is no longer trusted
uint16_t pulseMs = 10;       // coil pulse; TQ2-L2 needs a few ms
uint16_t settleMs = 100;     // wait after the relays move before measuring

// ---------- calibration (EEPROM) ----------

// Divider ratios from the resistors: (10M + 560k + 100k) / (560k + 100k) and (10M + 560k + 100k) / 100k
const float RATIO_A_NOMINAL = 10.66e6 / 0.66e6;
const float RATIO_B_NOMINAL = 10.66e6 / 0.1e6;

struct Cal {
  uint16_t magic;
  uint8_t version;
  float vzero[VOLT_MODES];       // ADC volts read with HI-LO shorted, per volts range
  float ratioA, ratioB;          // input / tap for the 10 V and 30 V ranges
  float rref[OHM_MODES];         // reference resistor values (ohms)
  float rzero[OHM_MODES][2];     // ohms read with HI-LO shorted, [range][0 = 2-wire, 1 = 4-wire]
  float ropen[OHM_MODES];        // leakage seen as a resistance across the input (0 = none)
  uint8_t sum;
};

const uint16_t CAL_MAGIC = 0x4C42;  // "LB"
const uint8_t CAL_VERSION = 1;
Cal cal;

uint8_t calSum(const Cal &c) {
  const uint8_t *b = (const uint8_t *)&c;
  uint8_t s = 0;
  for (size_t i = 0; i < sizeof(Cal) - 1; i++) s += b[i];
  return s;
}

void calDefaults() {
  memset(&cal, 0, sizeof(cal));
  cal.magic = CAL_MAGIC;
  cal.version = CAL_VERSION;
  cal.ratioA = RATIO_A_NOMINAL;
  cal.ratioB = RATIO_B_NOMINAL;
  for (uint8_t i = 0; i < OHM_MODES; i++) cal.rref[i] = MODES[VOLT_MODES + i].rrefNominal;
}

bool calLoad() {
  Cal c;
  EEPROM.get(0, c);
  if (c.magic != CAL_MAGIC || c.version != CAL_VERSION || c.sum != calSum(c)) return false;
  cal = c;
  return true;
}

void calSave() {
  cal.sum = calSum(cal);
  EEPROM.put(0, cal);
}

// ---------- low level ----------

void spiBegin() {
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
}

void spiEnd() {
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void sendCommand(uint8_t cmd) {
  spiBegin();
  SPI.transfer(cmd);
  spiEnd();
}

uint8_t readReg(uint8_t addr) {
  spiBegin();
  SPI.transfer(CMD_RREG | addr);
  SPI.transfer(0x00);
  uint8_t v = SPI.transfer(0x00);
  spiEnd();
  return v;
}

void writeReg(uint8_t addr, uint8_t value) {
  spiBegin();
  SPI.transfer(CMD_WREG | addr);
  SPI.transfer(0x00);
  SPI.transfer(value);
  spiEnd();
}

bool waitDrdy(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (millis() - t0 > timeoutMs) return false;
  }
  return true;
}

bool convert(int32_t &code) {
  sendCommand(CMD_START);
  delayMicroseconds(50);
  if (!waitDrdy(1500)) return false;
  spiBegin();
  SPI.transfer(CMD_RDATA);
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 3; i++) raw = (raw << 8) | SPI.transfer(0x00);
  spiEnd();
  if (raw & 0x800000) raw |= 0xFF000000;
  code = (int32_t)raw;
  return true;
}

uint8_t gainCode(uint16_t gain) {
  uint8_t c = 0;
  while ((1u << c) < gain && c < 7) c++;
  return c;
}

void setInput(uint8_t p, uint8_t n, uint16_t gain, bool forcePga) {
  writeReg(REG_INPMUX, (p << 4) | n);
  if (gain <= 1 && !forcePga) writeReg(REG_PGA, 0x00);
  else writeReg(REG_PGA, 0x08 | gainCode(gain));
}

double codeToVolts(double code, uint16_t gain) {
  return code * vref / ((double)gain * 8388608.0);
}

// Mean and standard deviation of `count` conversions (integer sums: AVR double is 32 bit).
// `over` is set when any conversion is near full scale.
bool measureCodes(uint16_t count, double &mean, double &sd, bool &over) {
  int64_t sum = 0, sumSq = 0;
  int32_t first = 0;
  over = false;
  for (uint16_t i = 0; i < count; i++) {
    int32_t c;
    if (!convert(c)) return false;
    if (c > CODE_OVER || c < -CODE_OVER) over = true;
    if (i == 0) first = c;
    int64_t d = (int64_t)c - first;
    sum += d;
    sumSq += d * d;
  }
  double m = (double)sum / count;
  mean = first + m;
  sd = count > 1 ? sqrt(((double)sumSq - m * (double)sum) / (count - 1)) : 0;
  return true;
}

// Volts between two inputs, mean and sd.
bool measureVolts(uint8_t p, uint8_t n, uint16_t gain, uint16_t count, double &v, double &sdv, bool &over) {
  setInput(p, n, gain, false);
  double mean, sd;
  if (!measureCodes(count, mean, sd, over)) return false;
  v = codeToVolts(mean, gain);
  sdv = codeToVolts(sd, gain);
  return true;
}

void setReference(bool external) {
  refExternal = external;
  writeReg(REG_REF, external ? REF_EXTERNAL : REF_INTERNAL);
  delay(10);
}

bool initAdc() {
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_RESET, HIGH);
  delay(5);
  sendCommand(CMD_RESET);
  delay(5);
  uint8_t id = readReg(REG_ID);
  writeReg(REG_STATUS, 0x00);
  writeReg(REG_DATARATE, 0x20 | 0x10 | rateCode);
  writeReg(REG_SYS, SYS_DEFAULT);
  setReference(true);
  return (id & 0x07) == 0x00;
}

// ---------- relays ----------

// One coil at a time, so the 5 V rail only ever carries one coil current (about 33-40 mA).
void pulseCoil(uint8_t relay, bool set) {
  uint8_t pin = RELAY_FIRST_PIN + 2 * (relay - 1) + (set ? 0 : 1);
  digitalWrite(pin, HIGH);
  delay(pulseMs);
  digitalWrite(pin, LOW);
  delay(pulseMs);  // let the coil current die before the next pulse
}

void relayMove(uint8_t relay, bool set) {
  pulseCoil(relay, set);
  if (set) relayState |= (1 << (relay - 1));
  else relayState &= ~(1 << (relay - 1));
}

// All relays to reset, whatever the firmware believes (power-up state is unknown).
void relaysInit() {
  for (uint8_t k = 1; k <= RELAY_COUNT; k++) relayMove(k, false);
  relayManual = false;
}

// Break before make: every relay that must open is reset first, then the ones that must close are set.
void relaysApply(uint8_t want) {
  for (uint8_t k = 1; k <= RELAY_COUNT; k++) {
    uint8_t bit = 1 << (k - 1);
    if (!(want & bit) && (relayState & bit || relayManual)) relayMove(k, false);
  }
  for (uint8_t k = 1; k <= RELAY_COUNT; k++) {
    uint8_t bit = 1 << (k - 1);
    if ((want & bit) && (!(relayState & bit) || relayManual)) relayMove(k, true);
  }
  relayManual = false;
  delay(settleMs);
}

// ---------- output ----------

void show(const String &line1, const String &line2) {
  (void)line1;
  (void)line2;
#if USE_OLED
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(line1);
  oled.setTextSize(2);
  oled.setCursor(0, 24);
  oled.print(line2);
  oled.display();
#endif
}

void fail(const __FlashStringHelper *what) {
  Serial.print(F("error,"));
  Serial.println(what);
}

String inputName(uint8_t ch) {
  return ch == MUX_COM ? String("COM") : String("AIN") + ch;
}

// ---------- commands: ADC (as in stage A) ----------

void cmdId() {
  uint8_t id = readReg(REG_ID);
  Serial.print(F("id,0x"));
  Serial.print(id, HEX);
  Serial.println((id & 0x07) == 0 ? F(",ADS124S08") : F(",unexpected (check wiring, SPI mode 1, RESET high)"));
}

void cmdRegs() {
  for (uint8_t a = 0; a <= 0x11; a++) {
    Serial.print(F("reg,0x"));
    if (a < 16) Serial.print('0');
    Serial.print(a, HEX);
    Serial.print(F(",0x"));
    uint8_t v = readReg(a);
    if (v < 16) Serial.print('0');
    Serial.println(v, HEX);
  }
}

void cmdMon() {
  bool wasExternal = refExternal;
  setReference(false);
  double mean, sd;
  bool over;
  struct { uint8_t mon; const char *name; } items[] = {{3, "AVDD"}, {4, "DVDD"}};
  for (auto &it : items) {
    writeReg(REG_SYS, (it.mon << 5) | SYS_DEFAULT);
    if (!measureCodes(4, mean, sd, over)) { fail(F("timeout (DRDY)")); break; }
    Serial.print(F("mon,"));
    Serial.print(it.name);
    Serial.print(',');
    Serial.println(4.0 * codeToVolts(mean, 1), 4);
  }
  setInput(0, 1, 1, true);
  writeReg(REG_SYS, (2 << 5) | SYS_DEFAULT);
  if (measureCodes(4, mean, sd, over)) {
    double v = codeToVolts(mean, 1);
    Serial.print(F("mon,TEMP_C,"));
    Serial.println(25.0 + (v - 0.129) / 0.000403, 2);
  }
  writeReg(REG_SYS, SYS_DEFAULT);
  setReference(wasExternal);
}

bool parseInput(const String &s, uint8_t &ch) {
  if (s.equalsIgnoreCase("com")) { ch = MUX_COM; return true; }
  if (s.equalsIgnoreCase("vcom")) { ch = AIN_VCOM; return true; }
  if (s.length() == 0 || !isDigit(s[0])) return false;
  long v = s.toInt();
  if (v < 0 || v > 11) return false;
  ch = v;
  return true;
}

bool parseGain(const String &s, uint16_t &gain) {
  long g = s.toInt();
  if (g < 1 || g > 128 || (g & (g - 1))) { fail(F("gain must be 1, 2, 4 ... 128")); return false; }
  gain = g;
  return true;
}

void printRaw(const char *tag, uint8_t p, uint8_t n, uint16_t gain, uint16_t count, double v, double sdv, bool over) {
  Serial.print(tag);
  Serial.print(',');
  Serial.print(inputName(p));
  Serial.print('-');
  Serial.print(inputName(n));
  Serial.print(F(",gain="));
  Serial.print(gain);
  Serial.print(F(",ref="));
  Serial.print(refExternal ? F("ext") : F("int"));
  Serial.print(F(",n="));
  Serial.print(count);
  Serial.print(F(",V="));
  Serial.print(v, 7);
  Serial.print(F(",uV="));
  Serial.print(v * 1e6, 3);
  Serial.print(F(",sd_uV="));
  Serial.print(sdv * 1e6, 2);
  Serial.println(over ? F(",OVER") : F(""));
}

void cmdV(const String args[], uint8_t argc) {
  uint8_t p, n;
  if (argc < 2 || !parseInput(args[0], p) || !parseInput(args[1], n)) {
    fail(F("usage: v <p> <n> [gain] [count]   (p, n = 0..11, vcom = 3)"));
    return;
  }
  uint16_t gain = 1;
  if (argc > 2 && !parseGain(args[2], gain)) return;
  uint16_t count = argc > 3 ? max(1L, args[3].toInt()) : 10;
  double v, sdv;
  bool over;
  if (!measureVolts(p, n, gain, count, v, sdv, over)) { fail(F("timeout (DRDY)")); return; }
  printRaw("v", p, n, gain, count, v, sdv, over);
}

// ADR4525 (AIN8) - internal REFOUT (AIN9) at gain 128. Limit: 0.05 % + 0.02 % of 2.5 V = 1.75 mV.
void cmdRefdiff(const String args[], uint8_t argc) {
  uint16_t count = argc > 0 ? max(1L, args[0].toInt()) : 20;
  double v, sdv;
  bool over;
  if (!measureVolts(AIN_ADRREF, AIN_REFOUT, 128, count, v, sdv, over)) { fail(F("timeout (DRDY)")); return; }
  printRaw("refdiff", AIN_ADRREF, AIN_REFOUT, 128, count, v, sdv, over);
  Serial.print(F("refdiff,ppm_of_2V5="));
  Serial.print(v / 2.5 * 1e6, 1);
  Serial.println(fabs(v) <= 1.75e-3 ? F(",within both datasheet limits") : F(",OUTSIDE the combined limit of 1.75 mV"));
}

void cmdNoise(const String args[], uint8_t argc) {
  uint16_t count = argc > 0 ? max(2L, args[0].toInt()) : 100;
  uint16_t gain = 1;
  if (argc > 1 && !parseGain(args[1], gain)) return;
  setInput(0, 1, gain, true);
  writeReg(REG_SYS, (1 << 5) | SYS_DEFAULT);
  double mean, sd;
  bool over;
  bool ok = measureCodes(count, mean, sd, over);
  writeReg(REG_SYS, SYS_DEFAULT);
  if (!ok) { fail(F("timeout (DRDY)")); return; }
  Serial.print(F("noise,gain="));
  Serial.print(gain);
  Serial.print(F(",n="));
  Serial.print(count);
  Serial.print(F(",offset_uV="));
  Serial.print(codeToVolts(mean, gain) * 1e6, 2);
  Serial.print(F(",sd_uV="));
  Serial.print(codeToVolts(sd, gain) * 1e6, 3);
  Serial.print(F(",sd_code="));
  Serial.println(sd, 2);
}

void cmdStream(const String args[], uint8_t argc) {
  uint8_t p, n;
  if (argc < 2 || !parseInput(args[0], p) || !parseInput(args[1], n)) {
    fail(F("usage: stream <p> <n> [gain]"));
    return;
  }
  uint16_t gain = 1;
  if (argc > 2 && !parseGain(args[2], gain)) return;
  setInput(p, n, gain, false);
  Serial.println(F("stream,ms,code,V   (send any key to stop)"));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    int32_t c;
    if (!convert(c)) { fail(F("timeout (DRDY)")); return; }
    Serial.print(F("s,"));
    Serial.print(millis());
    Serial.print(',');
    Serial.print(c);
    Serial.print(',');
    Serial.println(codeToVolts(c, gain), 7);
  }
  while (Serial.available()) Serial.read();
}

// ---------- commands: relays and modes ----------

void printRelays() {
  Serial.print(F("relay,state="));
  for (uint8_t k = 1; k <= RELAY_COUNT; k++) Serial.print(relayState & (1 << (k - 1)) ? 'S' : 'r');
  Serial.print(F(",mode="));
  Serial.print(modeIndex == MODE_NONE ? "none" : MODES[modeIndex].name);
  if (modeIndex != MODE_NONE && MODES[modeIndex].kind == OHMS) Serial.print(fourWire ? F(",wire=4w") : F(",wire=2w"));
  Serial.println(relayManual ? F(",MANUAL (run a mode command to return to a known state)") : F(""));
}

// relay                 show the state (S = set, r = reset; K1 first)
// relay init            reset all eight
// relay <1..8> set|reset
// relay pulse <ms>      coil pulse length
void cmdRelay(const String args[], uint8_t argc) {
  if (argc == 0) { printRelays(); return; }
  if (args[0] == "init") {
    relaysInit();
    modeIndex = MODE_NONE;
    printRelays();
    return;
  }
  if (args[0] == "pulse") {
    long ms = argc > 1 ? args[1].toInt() : 0;
    if (ms < 2 || ms > 100) { fail(F("usage: relay pulse <2..100 ms>")); return; }
    pulseMs = ms;
    Serial.print(F("relay,pulse_ms="));
    Serial.println(pulseMs);
    return;
  }
  long k = args[0].toInt();
  if (k < 1 || k > RELAY_COUNT || argc < 2 || (args[1] != "set" && args[1] != "reset")) {
    fail(F("usage: relay | relay init | relay <1..8> set|reset | relay pulse <ms>"));
    return;
  }
  relayMove(k, args[1] == "set");
  relayManual = true;
  printRelays();
}

int8_t findMode(const String &name) {
  for (uint8_t i = 0; i < MODE_COUNT; i++)
    if (name.equalsIgnoreCase(MODES[i].name)) return i;
  return -1;
}

// mode <v100m|v1|v10|v30|r100|r1k|r10k|r100k|r1m|r10m> [2w|4w]
void cmdMode(const String args[], uint8_t argc) {
  int8_t m = argc > 0 ? findMode(args[0]) : -1;
  if (m < 0) {
    fail(F("usage: mode v100m|v1|v10|v30|r100|r1k|r10k|r100k|r1m|r10m [2w|4w]"));
    return;
  }
  const Mode &md = MODES[m];
  bool four = fourWire;
  if (argc > 1) {
    if (args[1] == "4w") four = true;
    else if (args[1] == "2w") four = false;
    else { fail(F("wire must be 2w or 4w")); return; }
  }
  uint8_t want = md.relays;
  if (md.kind == OHMS && !four) want |= K8;  // K8 ties the sense lines to HI and LO
  if (!refExternal) setReference(true);
  relaysApply(want);
  modeIndex = m;
  fourWire = four;
  printRelays();
  if (md.kind == OHMS) Serial.println(F("warn,ohms mode: do not apply a voltage to the terminals"));
}

// ---------- measuring ----------

bool readVolts(uint8_t m, uint16_t count, Reading &r) {
  const Mode &md = MODES[m];
  double v, sdv;
  if (!measureVolts(md.p, AIN_VCOM, md.gain, count, v, sdv, r.over)) return false;
  double ratio = m == 2 ? cal.ratioA : m == 3 ? cal.ratioB : 1.0;
  r.a = v;
  r.b = v - cal.vzero[m];
  r.value = r.b * ratio;
  r.sd = sdv * ratio;
  return true;
}

// Rx = Rref * V(Rx) / V(Rref), V(Rx) taken before and after V(Rref) (A-B-A) to cancel slow drift.
// Both without the PGA and with the same reference, so the reference and the supply cancel.
// Corrections: the open-input leakage as a resistance in parallel with Rx, then the short (leads, EMF).
bool readOhmsRaw(uint8_t m, uint16_t count, Reading &r) {
  double vx1, vx2, vr, sx1, sx2, sr;
  bool o1, o2, o3;
  if (!measureVolts(AIN_HI_S, AIN_LO_S, 1, count, vx1, sx1, o1)) return false;
  if (!measureVolts(AIN_RTOP, AIN_RBOT, 1, count, vr, sr, o2)) return false;
  if (!measureVolts(AIN_HI_S, AIN_LO_S, 1, count, vx2, sx2, o3)) return false;
  double vx = (vx1 + vx2) / 2;
  r.a = vx;
  r.b = vr;
  r.over = o1 || o2 || o3;
  double rref = cal.rref[m - VOLT_MODES];
  // No (or almost no) current through Rref: input open, or Rx above 11 x Rref (beyond the range)
  if (vr <= 0 || r.over || vx > 11 * vr) {
    r.over = true;
    r.value = 0;
    r.sd = 0;
    return true;
  }
  r.value = rref * vx / vr;
  r.sd = rref * sx1 / vr;
  return true;
}

bool readOhms(uint8_t m, uint16_t count, Reading &r) {
  if (!readOhmsRaw(m, count, r)) return false;
  if (r.over) return true;
  uint8_t i = m - VOLT_MODES;
  if (cal.ropen[i] > 0) {
    double g = 1.0 / r.value - 1.0 / cal.ropen[i];
    if (g <= 0) { r.over = true; return true; }
    r.value = 1.0 / g;
  }
  r.value -= cal.rzero[i][fourWire ? 1 : 0];
  return true;
}

bool readMode(uint16_t count, Reading &r) {
  return MODES[modeIndex].kind == VOLTS ? readVolts(modeIndex, count, r) : readOhms(modeIndex, count, r);
}

bool needMode() {
  if (modeIndex == MODE_NONE) { fail(F("no mode yet (type: mode v1, mode r10k 4w ...)")); return false; }
  if (relayManual) { fail(F("a relay was moved by hand: run a mode command first")); return false; }
  return true;
}

void printReading(uint16_t count, const Reading &r) {
  const Mode &md = MODES[modeIndex];
  bool volts = md.kind == VOLTS;
  Serial.print(F("meas,mode="));
  Serial.print(md.name);
  if (!volts) Serial.print(fourWire ? F(",wire=4w") : F(",wire=2w"));
  Serial.print(F(",n="));
  Serial.print(count);
  if (r.over) {
    Serial.println(F(",value=OL"));
    show(md.name, "OL");
    return;
  }
  Serial.print(F(",value="));
  Serial.print(r.value, volts ? 7 : 4);
  Serial.print(volts ? F(",unit=V,sd=") : F(",unit=ohm,sd="));
  Serial.print(r.sd, volts ? 7 : 4);
  if (volts) {
    Serial.print(F(",adc_V="));
    Serial.println(r.a, 7);
  } else {
    Serial.print(F(",vx_V="));
    Serial.print(r.a, 7);
    Serial.print(F(",vref_V="));
    Serial.println(r.b, 7);
  }
  show(String(md.name), String(r.value, volts ? 6 : 3));
}

// meas [count]   one reading in the present mode (count conversions per step; ohms take three steps)
void cmdMeas(const String args[], uint8_t argc) {
  if (!needMode()) return;
  uint16_t count = argc > 0 ? max(1L, args[0].toInt()) : 10;
  Reading r;
  if (!readMode(count, r)) { fail(F("timeout (DRDY)")); return; }
  printReading(count, r);
}

// ---------- calibration commands ----------

void printCal() {
  Serial.print(F("cal,ratioA="));
  Serial.print(cal.ratioA, 6);
  Serial.print(F(",ratioB="));
  Serial.println(cal.ratioB, 5);
  for (uint8_t i = 0; i < VOLT_MODES; i++) {
    Serial.print(F("cal,"));
    Serial.print(MODES[i].name);
    Serial.print(F(",vzero_uV="));
    Serial.println(cal.vzero[i] * 1e6, 2);
  }
  for (uint8_t i = 0; i < OHM_MODES; i++) {
    Serial.print(F("cal,"));
    Serial.print(MODES[VOLT_MODES + i].name);
    Serial.print(F(",rref="));
    Serial.print(cal.rref[i], 3);
    Serial.print(F(",rzero2w="));
    Serial.print(cal.rzero[i][0], 5);
    Serial.print(F(",rzero4w="));
    Serial.print(cal.rzero[i][1], 5);
    Serial.print(F(",ropen="));
    Serial.println(cal.ropen[i], 0);
  }
}

// Divider ratio: the same voltage measured on two ranges.
//   cal ratio a: apply 0.5..1.2 V. Reads it on v1 (direct), then the DIV_A tap on v10.
//   cal ratio b: apply 5..19 V. Reads it on v10 (with ratio a), then the DIV_B tap on v30.
// Zero the ranges first (cal zero with the input shorted on each range).
void cmdCalRatio(const String &which, uint16_t count) {
  bool a = which == "a";
  if (!a && which != "b") { fail(F("usage: cal ratio a|b")); return; }
  uint8_t low = a ? 1 : 2, high = a ? 2 : 3;
  if (!refExternal) setReference(true);
  Reading r1, r2;
  relaysApply(MODES[low].relays);
  modeIndex = low;
  if (!readVolts(low, count, r1)) { fail(F("timeout (DRDY)")); return; }
  relaysApply(MODES[high].relays);
  modeIndex = high;
  if (!readVolts(high, count, r2)) { fail(F("timeout (DRDY)")); return; }
  if (r1.over || r2.over || fabs(r2.b) < 1e-3) { fail(F("over range or no voltage: check the source")); return; }
  double ratio = r1.value / r2.b;
  if (a) cal.ratioA = ratio;
  else cal.ratioB = ratio;
  Serial.print(F("cal,ratio"));
  Serial.print(a ? 'A' : 'B');
  Serial.print('=');
  Serial.print(ratio, 6);
  Serial.print(F(",input_V="));
  Serial.print(r1.value, 7);
  Serial.print(F(",tap_V="));
  Serial.print(r2.b, 7);
  Serial.print(F(",nominal="));
  Serial.println(a ? RATIO_A_NOMINAL : RATIO_B_NOMINAL, 4);
  Serial.println(F("cal,not saved yet (cal save)"));
}

// cal                       show
// cal zero [count]          present mode, input shorted (volts: HI-LO; ohms 2-wire: HI-LO; 4-wire: all four joined)
// cal open [count]          present ohms mode, input open (the leakage across the input; use on r1m and r10m)
// cal ratio a|b [count]     divider ratios (see cmdCalRatio)
// cal rref <mode> <ohms>    value of a reference resistor (from an outside calibration)
// cal save | load | clear   EEPROM
void cmdCal(const String args[], uint8_t argc) {
  if (argc == 0) { printCal(); return; }
  const String &sub = args[0];
  if (sub == "save") { calSave(); Serial.println(F("cal,saved")); return; }
  if (sub == "load") { Serial.println(calLoad() ? F("cal,loaded") : F("cal,nothing valid in EEPROM")); return; }
  if (sub == "clear") { calDefaults(); Serial.println(F("cal,defaults (not saved yet)")); return; }
  if (sub == "rref") {
    int8_t m = argc > 1 ? findMode(args[1]) : -1;
    double ohms = argc > 2 ? args[2].toDouble() : 0;
    if (m < VOLT_MODES || ohms <= 0) { fail(F("usage: cal rref <r100..r10m> <ohms>")); return; }
    cal.rref[m - VOLT_MODES] = ohms;
    if (m == 4 || m == 5) cal.rref[(m == 4 ? 5 : 4) - VOLT_MODES] = ohms;  // r100 and r1k share K1
    printCal();
    return;
  }
  if (sub == "ratio") {
    uint16_t count = argc > 2 ? max(1L, args[2].toInt()) : 20;
    cmdCalRatio(argc > 1 ? args[1] : String(""), count);
    return;
  }
  uint16_t count = argc > 1 ? max(1L, args[1].toInt()) : 20;
  if (sub == "zero") {
    if (!needMode()) return;
    Reading r;
    if (MODES[modeIndex].kind == VOLTS) {
      if (!readVolts(modeIndex, count, r) || r.over) { fail(F("timeout or over range: is the input shorted?")); return; }
      cal.vzero[modeIndex] = r.a;
      Serial.print(F("cal,vzero_uV="));
      Serial.println(r.a * 1e6, 2);
    } else {
      uint8_t i = modeIndex - VOLT_MODES;
      float keep = cal.rzero[i][fourWire ? 1 : 0];
      cal.rzero[i][fourWire ? 1 : 0] = 0;
      if (!readOhms(modeIndex, count, r) || r.over) {
        cal.rzero[i][fourWire ? 1 : 0] = keep;
        fail(F("timeout or over range: is the input shorted?"));
        return;
      }
      cal.rzero[i][fourWire ? 1 : 0] = r.value;
      Serial.print(F("cal,rzero_ohm="));
      Serial.println(r.value, 5);
    }
    Serial.println(F("cal,not saved yet (cal save)"));
    return;
  }
  if (sub == "open") {
    if (!needMode()) return;
    if (MODES[modeIndex].kind != OHMS) { fail(F("cal open is for the ohms modes")); return; }
    Reading r;
    if (!readOhmsRaw(modeIndex, count, r)) { fail(F("timeout (DRDY)")); return; }
    uint8_t i = modeIndex - VOLT_MODES;
    // With the input open, the only current through Rref is the leakage. Below a few noise steps
    // nothing is seen, and no correction is stored.
    if (r.over || r.b < 20e-6) {
      cal.ropen[i] = 0;
      Serial.print(F("cal,ropen=none,vref_uV="));
      Serial.println(r.b * 1e6, 2);
    } else {
      cal.ropen[i] = r.value;
      Serial.print(F("cal,ropen="));
      Serial.print(r.value, 0);
      Serial.print(F(",vref_uV="));
      Serial.println(r.b * 1e6, 2);
    }
    Serial.println(F("cal,not saved yet (cal save)"));
    return;
  }
  fail(F("usage: cal | cal zero | cal open | cal ratio a|b | cal rref <mode> <ohms> | cal save|load|clear"));
}

// ---------- command line ----------

void cmdHelp() {
  Serial.println(F("adc:    id | regs | mon | ref int|ext | rate <0..13> | v <p> <n> [gain] [count] |"));
  Serial.println(F("        refdiff [count] | noise [count] [gain] | stream <p> <n> [gain]"));
  Serial.println(F("relays: relay | relay init | relay <1..8> set|reset | relay pulse <ms> | settle <ms>"));
  Serial.println(F("dmm:    mode v100m|v1|v10|v30|r100|r1k|r10k|r100k|r1m|r10m [2w|4w] | meas [count]"));
  Serial.println(F("cal:    cal | cal zero | cal open | cal ratio a|b | cal rref <mode> <ohms> | cal save|load|clear"));
  Serial.println(F("inputs: 0..11 (3 = VCOM = LO). examples: v 8 3   refdiff   mode v1   meas 20"));
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  String args[6];
  uint8_t argc = 0;
  String cmd;
  int start = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ' ') {
      if (i > start) {
        String tok = line.substring(start, i);
        if (cmd.length() == 0) cmd = tok;
        else if (argc < 6) args[argc++] = tok;
      }
      start = i + 1;
    }
  }
  cmd.toLowerCase();
  for (uint8_t i = 0; i < argc; i++) args[i].toLowerCase();
  if (cmd == "help") cmdHelp();
  else if (cmd == "id") cmdId();
  else if (cmd == "regs") cmdRegs();
  else if (cmd == "mon") cmdMon();
  else if (cmd == "ref") {
    if (argc == 1 && (args[0] == "int" || args[0] == "ext")) {
      setReference(args[0] == "ext");
      Serial.print(F("ref,"));
      Serial.println(args[0]);
    } else fail(F("usage: ref int|ext"));
  } else if (cmd == "rate") {
    long r = argc ? args[0].toInt() : -1;
    if (r < 0 || r > 13) { fail(F("usage: rate <0..13>")); return; }
    rateCode = r;
    writeReg(REG_DATARATE, 0x20 | 0x10 | rateCode);
    Serial.print(F("rate,"));
    Serial.println(rateCode);
  } else if (cmd == "settle") {
    long ms = argc ? args[0].toInt() : -1;
    if (ms < 0 || ms > 10000) { fail(F("usage: settle <0..10000 ms>")); return; }
    settleMs = ms;
    Serial.print(F("settle,ms="));
    Serial.println(settleMs);
  } else if (cmd == "v") cmdV(args, argc);
  else if (cmd == "refdiff") cmdRefdiff(args, argc);
  else if (cmd == "noise") cmdNoise(args, argc);
  else if (cmd == "stream") cmdStream(args, argc);
  else if (cmd == "relay") cmdRelay(args, argc);
  else if (cmd == "mode") cmdMode(args, argc);
  else if (cmd == "meas") cmdMeas(args, argc);
  else if (cmd == "cal") cmdCal(args, argc);
  else fail(F("unknown command (type help)"));
}

void setup() {
  // Relay drivers first: all ULN2803A inputs low, so no coil is driven while the rest starts.
  for (uint8_t i = 0; i < 2 * RELAY_COUNT; i++) {
    digitalWrite(RELAY_FIRST_PIN + i, LOW);
    pinMode(RELAY_FIRST_PIN + i, OUTPUT);
  }
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_START, OUTPUT);
  digitalWrite(PIN_START, LOW);
  pinMode(PIN_DRDY, INPUT);
  SPI.begin();
  delay(10);

#if USE_OLED
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println(F("warn,OLED not found at 0x3C (try 0x3D or USE_OLED 0)"));
#endif

  Serial.println(F("dmm stage B"));
  bool ok = initAdc();
  cmdId();
  if (!ok) Serial.println(F("warn,ADC did not answer: check power, CS/SCLK/DIN/DOUT, RESET"));
  if (calLoad()) Serial.println(F("cal,loaded from EEPROM"));
  else {
    calDefaults();
    Serial.println(F("cal,defaults (nominal resistor values, no zero)"));
  }
  relaysInit();  // the latching relays keep whatever state they had: start from all reset
  printRelays();
  cmdHelp();
  show("dmm stage B", ok ? "ready" : "no ADC");
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) handleLine(buf);
      buf = "";
    } else if (buf.length() < 80) {
      buf += c;
    }
  }
}
