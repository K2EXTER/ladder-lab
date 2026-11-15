// DMM v1 stage A firmware: Arduino Mega + ADS124S08 (+ ADR4525, SSD1306 OLED).
// Wiring: dmm/stageA.md. Register and command values: TI SBAS660C (ADS124S08 datasheet).
//
// Serial 115200 baud, one command per line (Arduino Serial Monitor, "Newline"):
//   help                      list the commands
//   id                        read the ID register (low 3 bits 000 = ADS124S08)
//   regs                      dump registers 00h-11h
//   mon                       AVDD, DVDD and the die temperature from the internal monitors
//   ref int|ext               ADC reference: internal 2.5 V, or REFP0/REFN0 (ADR4525)
//   rate <0..13>              data rate code (0 = 2.5 SPS ... 4 = 20 SPS ... 13 = 4000 SPS)
//   v <p> <n> [gain] [count]  measure AINp - AINn (0..11 or "com"), e.g. "v 0 com" or "v 4 5 128 20"
//   refdiff [count]           ADR4525 (AIN4) - internal REFOUT (AIN5) at gain 128
//   noise [count] [gain]      inputs shorted inside the ADC: offset and noise
//   stream <p> <n> [gain]     one reading per line until any key is sent (for logging on the PC)
//
// Every result line starts with a keyword and is comma separated, so a PC log can be read as CSV.

#include <SPI.h>

#define USE_OLED 0  // 1 once the OLED is wired and the Adafruit SSD1306 library is installed (that build has not been tried yet)
#if USE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;
#endif

// Mega pins (dmm/stageA.md)
const uint8_t PIN_CS = 53;
const uint8_t PIN_DRDY = 2;
const uint8_t PIN_START = 4;
const uint8_t PIN_RESET = 5;

// Commands (datasheet Table 24)
const uint8_t CMD_RESET = 0x06;
const uint8_t CMD_START = 0x08;
const uint8_t CMD_STOP = 0x0A;
const uint8_t CMD_RDATA = 0x12;
const uint8_t CMD_SFOCAL = 0x19;
const uint8_t CMD_RREG = 0x20;
const uint8_t CMD_WREG = 0x40;

// Registers
const uint8_t REG_ID = 0x00;
const uint8_t REG_STATUS = 0x01;
const uint8_t REG_INPMUX = 0x02;
const uint8_t REG_PGA = 0x03;
const uint8_t REG_DATARATE = 0x04;
const uint8_t REG_REF = 0x05;
const uint8_t REG_SYS = 0x09;

// REF register values (Table 31): bits FL_REF_EN[7:6] REFP_BUF[5] REFN_BUF[4] REFSEL[3:2] REFCON[1:0]
// Internal: reference buffers off (datasheet note 1), REFSEL = internal, REFCON = on.
const uint8_t REF_INTERNAL = 0x39;  // 00 1 1 10 01
// External REFP0/REFN0: REFP buffer on, REFN buffer off (REFN0 is at AVSS), internal reference
// kept on so REFOUT (AIN5) can still be measured against the ADR4525.
const uint8_t REF_EXTERNAL = 0x11;  // 00 0 1 00 01

const uint8_t SYS_DEFAULT = 0x10;  // CAL_SAMP = 8 samples, no status byte, no CRC
const uint8_t MUX_COM = 12;        // AINCOM (tied to GND on the board)

const SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);  // only mode 1 is supported

float vref = 2.5;  // volts; replace with a calibrated value later
bool refExternal = false;
uint8_t rateCode = 4;  // 20 SPS: low-latency filter rejects 50 Hz and 60 Hz together
String lastLine = "";

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
  SPI.transfer(0x00);  // read 1 register (n - 1 = 0)
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

// Waits for DRDY to go low; false on timeout.
bool waitDrdy(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (millis() - t0 > timeoutMs) return false;
  }
  return true;
}

// One single-shot conversion: START, wait for DRDY, RDATA. Returns the signed 24-bit code.
bool convert(int32_t &code) {
  sendCommand(CMD_START);
  delayMicroseconds(50);  // let DRDY go high for the new conversion
  // 2.5 SPS is the slowest rate (400 ms); allow for the first-conversion delay
  if (!waitDrdy(1500)) return false;
  spiBegin();
  SPI.transfer(CMD_RDATA);
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 3; i++) raw = (raw << 8) | SPI.transfer(0x00);
  spiEnd();
  if (raw & 0x800000) raw |= 0xFF000000;  // two's complement (7FFFFFh = +FS, 800000h = -FS)
  code = (int32_t)raw;
  return true;
}

uint8_t gainCode(uint16_t gain) {
  uint8_t c = 0;
  while ((1u << c) < gain && c < 7) c++;
  return c;
}

// Sets the input multiplexer and the PGA. gain 1 bypasses the PGA (widest input range,
// single-ended to ground works) unless the PGA is needed (temperature sensor).
void setInput(uint8_t p, uint8_t n, uint16_t gain, bool forcePga) {
  writeReg(REG_INPMUX, (p << 4) | n);
  if (gain <= 1 && !forcePga) {
    writeReg(REG_PGA, 0x00);  // PGA bypassed, gain must be 000
  } else {
    writeReg(REG_PGA, 0x08 | gainCode(gain));  // PGA_EN = 01
  }
}

double codeToVolts(double code, uint16_t gain) {
  return code * vref / ((double)gain * 8388608.0);  // 2^23
}

// Averages `count` conversions; returns mean and standard deviation of the codes.
// On the AVR `double` is a 32-bit float (24-bit mantissa), so the sum is kept in integers
// and the spread is taken around the first reading to keep the numbers small.
bool measureCodes(uint16_t count, double &mean, double &sd) {
  int64_t sum = 0, sumSq = 0;
  int32_t first = 0;
  for (uint16_t i = 0; i < count; i++) {
    int32_t c;
    if (!convert(c)) return false;
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

void setReference(bool external) {
  refExternal = external;
  writeReg(REG_REF, external ? REF_EXTERNAL : REF_INTERNAL);
  delay(10);  // internal reference settles to 0.001 % in 5.9 ms with 1 uF (Table 10)
}

bool initAdc() {
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(10);  // >= 4 tCLK
  digitalWrite(PIN_RESET, HIGH);
  delay(5);  // >= 4096 tCLK (1 ms) before the first command
  sendCommand(CMD_RESET);
  delay(5);
  uint8_t id = readReg(REG_ID);
  writeReg(REG_STATUS, 0x00);                        // clear FL_POR
  writeReg(REG_DATARATE, 0x20 | 0x10 | rateCode);    // single-shot, low-latency filter
  writeReg(REG_SYS, SYS_DEFAULT);
  setReference(false);
  return (id & 0x07) == 0x00;
}

// ---------- output ----------

void show(const String &line1, const String &line2) {
  lastLine = line1;
  (void)line2;  // unused without the OLED
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

void fail(const char *what) {
  Serial.print(F("error,"));
  Serial.println(what);
}

String inputName(uint8_t ch) {
  return ch == MUX_COM ? String("COM") : String("AIN") + ch;
}

// ---------- commands ----------

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

// Monitors (Table 35): 011 = (AVDD - AVSS) / 4, 100 = DVDD / 4, 010 = temperature (PGA on).
// They are measured against the internal reference, so it is switched in for the duration.
void cmdMon() {
  bool wasExternal = refExternal;
  setReference(false);
  double mean, sd;
  struct { uint8_t mon; const char *name; } items[] = {{3, "AVDD"}, {4, "DVDD"}};
  for (auto &it : items) {
    writeReg(REG_SYS, (it.mon << 5) | SYS_DEFAULT);
    if (!measureCodes(4, mean, sd)) { fail("timeout (DRDY)"); break; }
    Serial.print(F("mon,"));
    Serial.print(it.name);
    Serial.print(',');
    Serial.println(4.0 * codeToVolts(mean, 1), 4);
  }
  setInput(0, 1, 1, true);  // the temperature sensor needs the PGA enabled
  writeReg(REG_SYS, (2 << 5) | SYS_DEFAULT);
  if (measureCodes(4, mean, sd)) {
    double v = codeToVolts(mean, 1);
    // 129 mV at 25 C, 403 uV/C (electrical characteristics)
    Serial.print(F("mon,TEMP_C,"));
    Serial.println(25.0 + (v - 0.129) / 0.000403, 2);
  }
  writeReg(REG_SYS, SYS_DEFAULT);
  setReference(wasExternal);
}

bool parseInput(const String &s, uint8_t &ch) {
  if (s.equalsIgnoreCase("com")) { ch = MUX_COM; return true; }
  if (s.length() == 0 || !isDigit(s[0])) return false;
  long v = s.toInt();
  if (v < 0 || v > 11) return false;
  ch = v;
  return true;
}

void printMeasurement(const char *tag, uint8_t p, uint8_t n, uint16_t gain, uint16_t count, double mean, double sd) {
  double volts = codeToVolts(mean, gain);
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
  Serial.print(F(",code="));
  Serial.print(mean, 1);
  Serial.print(F(",V="));
  Serial.print(volts, 7);
  Serial.print(F(",uV="));  // more digits for small values (refdiff, gain 128)
  Serial.print(volts * 1e6, 3);
  Serial.print(F(",sd_uV="));
  Serial.println(codeToVolts(sd, gain) * 1e6, 2);
  show(inputName(p) + "-" + inputName(n) + " g" + gain, String(volts, 6) + "V");
}

void cmdV(const String args[], uint8_t argc) {
  uint8_t p, n;
  if (argc < 2 || !parseInput(args[0], p) || !parseInput(args[1], n)) {
    fail("usage: v <p> <n> [gain] [count]   (p, n = 0..11 or com)");
    return;
  }
  uint16_t gain = argc > 2 ? args[2].toInt() : 1;
  uint16_t count = argc > 3 ? args[3].toInt() : 10;
  if (gain < 1 || gain > 128 || (gain & (gain - 1))) { fail("gain must be 1, 2, 4 ... 128"); return; }
  if (count < 1) count = 1;
  setInput(p, n, gain, false);
  double mean, sd;
  if (!measureCodes(count, mean, sd)) { fail("timeout (DRDY)"); return; }
  printMeasurement("v", p, n, gain, count, mean, sd);
}

// ADR4525 (AIN4) - internal REFOUT (AIN5). The difference is a few mV, so gain 128
// (full scale +-19.5 mV) is used; the error of the ruler itself hardly matters at that size.
// Datasheet limits: internal +-0.05 % (1.25 mV), ADR4525 B grade +-0.02 % (0.5 mV).
void cmdRefdiff(const String args[], uint8_t argc) {
  uint16_t count = argc > 0 ? args[0].toInt() : 20;
  if (count < 1) count = 1;
  setInput(4, 5, 128, false);
  double mean, sd;
  if (!measureCodes(count, mean, sd)) { fail("timeout (DRDY)"); return; }
  double dv = codeToVolts(mean, 128);
  printMeasurement("refdiff", 4, 5, 128, count, mean, sd);
  Serial.print(F("refdiff,ppm_of_2V5,"));
  Serial.print(dv / 2.5 * 1e6, 1);
  Serial.println(fabs(dv) <= 1.75e-3 ? F(",within both datasheet limits") : F(",OUTSIDE the combined limit of 1.75 mV"));
}

// SYS_MON = 001: PGA inputs shorted to mid-supply inside the ADC.
void cmdNoise(const String args[], uint8_t argc) {
  uint16_t count = argc > 0 ? args[0].toInt() : 100;
  uint16_t gain = argc > 1 ? args[1].toInt() : 1;
  if (count < 2) count = 2;
  if (gain < 1 || gain > 128 || (gain & (gain - 1))) { fail("gain must be 1, 2, 4 ... 128"); return; }
  setInput(0, 1, gain, true);
  writeReg(REG_SYS, (1 << 5) | SYS_DEFAULT);
  double mean, sd;
  bool ok = measureCodes(count, mean, sd);
  writeReg(REG_SYS, SYS_DEFAULT);
  if (!ok) { fail("timeout (DRDY)"); return; }
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
    fail("usage: stream <p> <n> [gain]");
    return;
  }
  uint16_t gain = argc > 2 ? args[2].toInt() : 1;
  if (gain < 1 || gain > 128 || (gain & (gain - 1))) { fail("gain must be 1, 2, 4 ... 128"); return; }
  setInput(p, n, gain, false);
  Serial.println(F("stream,ms,code,V   (send any key to stop)"));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    int32_t c;
    if (!convert(c)) { fail("timeout (DRDY)"); return; }
    Serial.print(F("s,"));
    Serial.print(millis());
    Serial.print(',');
    Serial.print(c);
    Serial.print(',');
    Serial.println(codeToVolts(c, gain), 7);
  }
  while (Serial.available()) Serial.read();
}

void cmdHelp() {
  Serial.println(F("commands: id | regs | mon | ref int|ext | rate <0..13> | v <p> <n> [gain] [count] |"));
  Serial.println(F("          refdiff [count] | noise [count] [gain] | stream <p> <n> [gain]"));
  Serial.println(F("inputs: 0..11 or com (AINCOM = GND). examples: v 0 com   v 4 5 128 20   noise 100"));
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
  if (cmd == "help") cmdHelp();
  else if (cmd == "id") cmdId();
  else if (cmd == "regs") cmdRegs();
  else if (cmd == "mon") cmdMon();
  else if (cmd == "ref") {
    if (argc == 1 && (args[0] == "int" || args[0] == "ext")) {
      setReference(args[0] == "ext");
      Serial.print(F("ref,"));
      Serial.println(args[0]);
    } else fail("usage: ref int|ext");
  } else if (cmd == "rate") {
    long r = argc ? args[0].toInt() : -1;
    if (r < 0 || r > 13) { fail("usage: rate <0..13>"); return; }
    rateCode = r;
    writeReg(REG_DATARATE, 0x20 | 0x10 | rateCode);
    Serial.print(F("rate,"));
    Serial.println(rateCode);
  } else if (cmd == "v") cmdV(args, argc);
  else if (cmd == "refdiff") cmdRefdiff(args, argc);
  else if (cmd == "noise") cmdNoise(args, argc);
  else if (cmd == "stream") cmdStream(args, argc);
  else fail("unknown command (type help)");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);  // pin 53 must be an output for the Mega to stay SPI master
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_START, OUTPUT);
  digitalWrite(PIN_START, LOW);  // conversions are started by the START command
  pinMode(PIN_DRDY, INPUT);
  SPI.begin();
  delay(10);  // >= 2.2 ms after power-up

#if USE_OLED
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // some modules use 0x3D
  if (!oledOk) Serial.println(F("warn,OLED not found at 0x3C (try 0x3D or USE_OLED 0)"));
#endif

  bool ok = initAdc();
  Serial.println(F("dmm stage A"));
  cmdId();
  if (!ok) Serial.println(F("warn,ADC did not answer: check power, CS/SCLK/DIN/DOUT, RESET"));
  cmdHelp();
  show("dmm stage A", ok ? "ready" : "no ADC");
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
