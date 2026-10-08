// Six-servo calibration: find each leg's zero (arm level) and its direction.
//
// Wiring (STATUS.md, wiring diagram)
//   leg 1..6 signal (orange) -> D2..D7
//   servo +V (red)           -> 4x AA +.  NOT the Arduino 5V / USB.
//   servo GND (brown)        -> 4x AA -  AND  Arduino GND (common ground)
//
// Serial Monitor: 115200 baud.  Any line ending works.
//
// The zeros live in EEPROM, so they survive a reset - and opening the Serial
// Monitor resets an Uno.  Send `save` after calibrating.
//
// Commands (act on the selected leg unless noted)
//   leg 3         select leg 3 (1-6)
//   1520          go to 1520 us
//   + / -         move by the step size (repeat by sending again)
//   s 1           set step size to 1 us
//   c             centre (1500 us)
//   z             zero = current position (the arm is level here)
//   test          go to zero + 50 us for 1.5 s, then back: watch the tip
//   up / down     record what `test` did: +us raised / lowered the tip
//   home          all legs to their zeros
//   centre        all legs to 1500 us
//   off / on      stop driving all legs (limp) / drive them again
//   off 6         stop driving leg 6 only (`on` brings it back)
//   table         print the calibration, and the lines for the PC side
//   save / load   write / re-read the calibration in EEPROM
//   forget        reset the calibration to 1500, direction unknown
//   p             status
//   h             this help
//
// Procedure, per leg, arms on and NO rods:
//   1. `leg N`, `c`.  Fit the arm at the spline tooth nearest level and
//      pointing at the base-plate centre.
//   2. `s 1`, then `+` / `-` until the rod-end hole is 30.0 mm above the
//      plate (arm level).  `z`.
//   3. `test`, watch the tip, then `up` or `down`.
// When all six are done: `save`, `table`, and copy the table into STATUS.md.
//
// Every move ramps at RAMP_US per RAMP_MS (~140 deg/s), so no command makes
// all six servos jump at once.

#include <Servo.h>
#include <EEPROM.h>

const int N_LEGS = 6;
const int PINS[N_LEGS] = {2, 3, 4, 5, 6, 7};  // leg 1 on D2 ... leg 6 on D7
const int US_MIN = 550, US_MAX = 2460;        // common working limits (STATUS.md)
const int CENTRE = 1500;
const int TEST_US = 50;
const float DEG_PER_US = 0.087;               // servo 1, bench test 2026-09-16

const int RAMP_US = 8;                        // max step per tick, per leg
const unsigned long RAMP_MS = 5;

const uint16_t MAGIC = 0xCA11;
struct Cal {
  uint16_t magic;
  int16_t zero[N_LEGS];
  int8_t sign[N_LEGS];                        // +1: +us raises the tip; -1: lowers; 0: unknown
};

Servo servos[N_LEGS];
Cal cal;
int pos[N_LEGS];                              // what is being written now
int target[N_LEGS];                           // where each leg is ramping to
bool driving = false;
int sel = 0;                                  // selected leg, 0-indexed
int stepUs = 10;
unsigned long rampLast = 0;

// test state: go to zero + TEST_US, hold, come back
bool testing = false;
int testLeg = 0;
unsigned long testStart = 0;

char line[48];
int lineLen = 0;
unsigned long lastCharMs = 0;

void forget() {
  cal.magic = MAGIC;
  for (int i = 0; i < N_LEGS; i++) {
    cal.zero[i] = CENTRE;
    cal.sign[i] = 0;
  }
}

bool load() {
  Cal c;
  EEPROM.get(0, c);
  if (c.magic != MAGIC) return false;
  for (int i = 0; i < N_LEGS; i++) {
    if (c.zero[i] < US_MIN || c.zero[i] > US_MAX || c.sign[i] < -1 || c.sign[i] > 1)
      return false;
  }
  cal = c;
  return true;
}

void setTarget(int leg, int us) {
  if (us < US_MIN || us > US_MAX) {
    Serial.print(F("refused: "));
    Serial.print(us);
    Serial.print(F(" us is outside "));
    Serial.print(US_MIN);
    Serial.print('-');
    Serial.println(US_MAX);
    return;
  }
  target[leg] = us;
}

// Attach one leg at a time, so the six don't all start together.
void driveAll() {
  for (int i = 0; i < N_LEGS; i++) {
    if (!servos[i].attached()) {
      servos[i].attach(PINS[i], 500, 2500);
      servos[i].writeMicroseconds(pos[i]);
      delay(250);
    }
  }
  driving = true;
}

void stopAll() {
  for (int i = 0; i < N_LEGS; i++) servos[i].detach();
  driving = false;
  testing = false;
}

void ramp() {
  if (millis() - rampLast < RAMP_MS) return;
  rampLast = millis();
  for (int i = 0; i < N_LEGS; i++) {
    int e = target[i] - pos[i];
    if (e == 0) continue;
    pos[i] += constrain(e, -RAMP_US, RAMP_US);
    if (driving) servos[i].writeMicroseconds(pos[i]);
  }
}

void printLeg(int i) {
  Serial.print(F("leg "));
  Serial.print(i + 1);
  Serial.print(F(": "));
  Serial.print(target[i]);
  Serial.print(F(" us, zero "));
  Serial.print(cal.zero[i]);
  int off = target[i] - cal.zero[i];
  Serial.print(F(" ("));
  if (off >= 0) Serial.print('+');
  Serial.print(off);
  Serial.print(F(" us = "));
  if (off >= 0) Serial.print('+');
  Serial.print(off * DEG_PER_US, 1);
  Serial.print(F(" deg)"));
  Serial.print(F(", +us "));
  if (cal.sign[i] > 0) Serial.println(F("raises the tip"));
  else if (cal.sign[i] < 0) Serial.println(F("lowers the tip"));
  else Serial.println(F("direction unknown"));
}

void printStatus() {
  Serial.print(F("selected leg "));
  Serial.print(sel + 1);
  Serial.print(F(" (D"));
  Serial.print(PINS[sel]);
  Serial.print(F("), step "));
  Serial.print(stepUs);
  Serial.println(driving ? F(" us, driving") : F(" us, limp (send `on`)"));
  for (int i = 0; i < N_LEGS; i++) {
    Serial.print(i == sel ? F("> ") : F("  "));
    if (!servos[i].attached()) Serial.print(F("[limp] "));
    printLeg(i);
  }
}

void printTable() {
  Serial.println(F("| servo | zero us (arm level) | +us raises tip? |"));
  Serial.println(F("|---|---|---|"));
  for (int i = 0; i < N_LEGS; i++) {
    Serial.print(F("| "));
    Serial.print(i + 1);
    Serial.print(F(" | "));
    Serial.print(cal.zero[i]);
    Serial.print(F(" | "));
    Serial.print(cal.sign[i] > 0 ? F("yes") : cal.sign[i] < 0 ? F("no") : F("?"));
    Serial.println(F(" |"));
  }
  Serial.println();
  Serial.print(F("ZERO_US = ["));
  for (int i = 0; i < N_LEGS; i++) {
    if (i) Serial.print(F(", "));
    Serial.print(cal.zero[i]);
  }
  Serial.println(']');
  Serial.print(F("SIGN = ["));   // +1: +us raises the tip, i.e. +alpha
  for (int i = 0; i < N_LEGS; i++) {
    if (i) Serial.print(F(", "));
    if (cal.sign[i] > 0) Serial.print('+');
    Serial.print(cal.sign[i]);
  }
  Serial.println(']');
  for (int i = 0; i < N_LEGS; i++) {
    if (cal.sign[i] == 0) {
      Serial.println(F("note: some legs have no direction yet (`test`, then `up` / `down`)"));
      break;
    }
  }
}

void printHelp() {
  Serial.println(F("commands: leg N  <us>  + -  s N  c  z  test  up down  home  centre  off on  table  save load forget  p  h"));
}

void handle(char *cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') return;

  int n;
  if (isdigit(cmd[0])) {
    setTarget(sel, atoi(cmd));
    printLeg(sel);
  } else if (sscanf(cmd, "leg %d", &n) == 1) {
    if (n < 1 || n > N_LEGS) {
      Serial.println(F("leg must be 1-6"));
      return;
    }
    sel = n - 1;
    printLeg(sel);
  } else if (strcmp(cmd, "+") == 0) {
    setTarget(sel, target[sel] + stepUs);
    printLeg(sel);
  } else if (strcmp(cmd, "-") == 0) {
    setTarget(sel, target[sel] - stepUs);
    printLeg(sel);
  } else if (sscanf(cmd, "s %d", &n) == 1) {
    if (n > 0) stepUs = n;
    printStatus();
  } else if (strcmp(cmd, "c") == 0) {
    setTarget(sel, CENTRE);
    printLeg(sel);
  } else if (strcmp(cmd, "z") == 0) {
    cal.zero[sel] = target[sel];
    Serial.print(F("zero set (not saved yet): "));
    printLeg(sel);
  } else if (strcmp(cmd, "test") == 0) {
    if (!driving) {
      Serial.println(F("limp: send `on` first"));
      return;
    }
    setTarget(sel, cal.zero[sel] + TEST_US);
    testing = true;
    testLeg = sel;
    testStart = millis();
    Serial.print(F("leg "));
    Serial.print(sel + 1);
    Serial.println(F(": +50 us for 1.5 s - watch the tip, then send `up` or `down`"));
  } else if (strcmp(cmd, "up") == 0) {
    cal.sign[sel] = 1;
    printLeg(sel);
  } else if (strcmp(cmd, "down") == 0) {
    cal.sign[sel] = -1;
    printLeg(sel);
  } else if (strcmp(cmd, "home") == 0) {
    for (int i = 0; i < N_LEGS; i++) setTarget(i, cal.zero[i]);
    Serial.println(F("all legs to their zeros"));
  } else if (strcmp(cmd, "centre") == 0 || strcmp(cmd, "center") == 0) {
    for (int i = 0; i < N_LEGS; i++) setTarget(i, CENTRE);
    Serial.println(F("all legs to 1500 us"));
  } else if (sscanf(cmd, "off %d", &n) == 1) {
    if (n < 1 || n > N_LEGS) {
      Serial.println(F("leg must be 1-6"));
      return;
    }
    servos[n - 1].detach();
    if (testing && testLeg == n - 1) testing = false;
    Serial.print(F("leg "));
    Serial.print(n);
    Serial.println(F(" limp"));
  } else if (strcmp(cmd, "off") == 0) {
    stopAll();
    Serial.println(F("all legs limp"));
  } else if (strcmp(cmd, "on") == 0) {
    driveAll();
    Serial.println(F("driving all legs"));
  } else if (strcmp(cmd, "table") == 0) {
    printTable();
  } else if (strcmp(cmd, "save") == 0) {
    cal.magic = MAGIC;
    EEPROM.put(0, cal);
    Serial.println(F("saved to EEPROM"));
  } else if (strcmp(cmd, "load") == 0) {
    Serial.println(load() ? F("loaded from EEPROM") : F("nothing valid in EEPROM; kept current values"));
    printStatus();
  } else if (strcmp(cmd, "forget") == 0) {
    forget();
    Serial.println(F("calibration reset to 1500 / unknown (EEPROM unchanged until `save`)"));
  } else if (strcmp(cmd, "p") == 0) {
    printStatus();
  } else if (strcmp(cmd, "h") == 0) {
    printHelp();
  } else {
    Serial.print(F("unknown: "));
    Serial.println(cmd);
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Six-servo calibration"));
  if (load()) {
    Serial.println(F("calibration loaded from EEPROM"));
  } else {
    forget();
    Serial.println(F("no saved calibration; zeros at 1500"));
  }
  for (int i = 0; i < N_LEGS; i++) pos[i] = target[i] = cal.zero[i];
  driveAll();  // each leg starts at its saved zero, one at a time
  printStatus();
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[lineLen] = '\0';
      handle(line);
      lineLen = 0;
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = ch;
    }
    lastCharMs = millis();
  }

  // Serial Monitor set to "No Line Ending": run the command once input goes quiet.
  if (lineLen > 0 && millis() - lastCharMs > 100) {
    line[lineLen] = '\0';
    handle(line);
    lineLen = 0;
  }

  if (testing && millis() - testStart > 1500) {
    testing = false;
    setTarget(testLeg, cal.zero[testLeg]);
    Serial.println(F("back at zero"));
  }

  ramp();
}
