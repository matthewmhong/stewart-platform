// Stage 1 joystick control, Arduino side: stream the stick, output six pulses.
//
// The PC runs the IK (`joystick.py` at the repo root).  This sketch knows no
// geometry: it reads the stick, sends it to the PC, and writes whatever six
// pulse widths come back - after checking each one.
//
// Wiring (STATUS.md)
//   leg 1..6 signal -> D2..D7      servo power from the 4x AA rail
//   joystick VRx -> A0, VRy -> A1, SW -> D8, +5V -> 5V, GND -> GND
//   Arduino GND -> servo - rail (common ground)
//
// Calibration: the zeros and directions saved by `firmware/servo_cal/` are
// read from EEPROM.  Without them the servos are never driven.
//
// Serial, 115200 baud, one line per message
//   to PC    C z1 .. z6 s1 .. s6     calibration (at boot, and on `?`)
//            J x y b                 stick, 0-1023 each axis, b = 1 pressed; 50 Hz
//            W home                  no pulses for TIMEOUT_MS: returning to zero
//            E <text>                a line was refused
//   from PC  P u1 u2 u3 u4 u5 u6     pulse widths, us, legs 1..6
//            ?                       resend the calibration
//
// Safety, checked here whatever the PC sends
//   - a P line is refused whole if any pulse is outside 550-2460 us or more
//     than MAX_OFF_US from that leg's zero (~30 deg of arm);
//   - every move ramps at RAMP_US per RAMP_MS;
//   - no P line for TIMEOUT_MS (PC script stopped) -> all legs ramp to zero.

#include <Servo.h>
#include <EEPROM.h>

const int N_LEGS = 6;
const int PINS[N_LEGS] = {2, 3, 4, 5, 6, 7};
const int PIN_X = A0, PIN_Y = A1, PIN_SW = 8;
const int US_MIN = 550, US_MAX = 2460;
const int MAX_OFF_US = 350;                   // ~30 deg; R1 needs ~166 us
const int RAMP_US = 20;                       // per tick: ~350 deg/s, above the servo's own speed
const unsigned long RAMP_MS = 5;
const unsigned long STICK_MS = 20;            // 50 Hz
const unsigned long TIMEOUT_MS = 500;

// Same layout as firmware/servo_cal - keep the two in step.
const uint16_t MAGIC = 0xCA11;
struct Cal {
  uint16_t magic;
  int16_t zero[N_LEGS];
  int8_t sign[N_LEGS];
};

Servo servos[N_LEGS];
Cal cal;
bool calOk = false;
int pos[N_LEGS];
int target[N_LEGS];
unsigned long rampLast = 0, stickLast = 0, lastP = 0;
bool homed = true;                            // true while holding the zeros after a timeout / at boot

char line[64];
int lineLen = 0;

bool loadCal() {
  EEPROM.get(0, cal);
  if (cal.magic != MAGIC) return false;
  for (int i = 0; i < N_LEGS; i++) {
    if (cal.zero[i] < US_MIN || cal.zero[i] > US_MAX || cal.sign[i] == 0 ||
        cal.sign[i] < -1 || cal.sign[i] > 1)
      return false;
  }
  return true;
}

void sendCal() {
  if (!calOk) {
    Serial.println(F("E no calibration in EEPROM - run firmware/servo_cal first"));
    return;
  }
  Serial.print('C');
  for (int i = 0; i < N_LEGS; i++) {
    Serial.print(' ');
    Serial.print(cal.zero[i]);
  }
  for (int i = 0; i < N_LEGS; i++) {
    Serial.print(' ');
    Serial.print(cal.sign[i]);
  }
  Serial.println();
}

void goHome() {
  for (int i = 0; i < N_LEGS; i++) target[i] = cal.zero[i];
  homed = true;
}

void handleP(char *s) {
  int u[N_LEGS];
  char *p = s + 1;
  for (int i = 0; i < N_LEGS; i++) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) {
      Serial.println(F("E P needs six pulse widths"));
      return;
    }
    if (v < US_MIN || v > US_MAX || abs(v - cal.zero[i]) > MAX_OFF_US) {
      Serial.print(F("E leg "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.print(v);
      Serial.println(F(" us out of range; line refused"));
      return;
    }
    u[i] = (int)v;
    p = end;
  }
  for (int i = 0; i < N_LEGS; i++) target[i] = u[i];
  lastP = millis();
  homed = false;
}

void handle(char *s) {
  if (s[0] == 'P') {
    if (calOk) handleP(s);
  } else if (s[0] == '?') {
    sendCal();
  } else if (s[0] != '\0') {
    Serial.print(F("E unknown: "));
    Serial.println(s);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);
  calOk = loadCal();
  if (calOk) {
    for (int i = 0; i < N_LEGS; i++) pos[i] = target[i] = cal.zero[i];
    for (int i = 0; i < N_LEGS; i++) {       // one at a time, so all six don't start together
      servos[i].attach(PINS[i], 500, 2500);
      servos[i].writeMicroseconds(pos[i]);
      delay(150);
    }
  }
  sendCal();
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
  }

  unsigned long now = millis();

  if (now - stickLast >= STICK_MS) {
    stickLast = now;
    Serial.print(F("J "));
    Serial.print(analogRead(PIN_X));
    Serial.print(' ');
    Serial.print(analogRead(PIN_Y));
    Serial.print(' ');
    Serial.println(digitalRead(PIN_SW) == LOW ? 1 : 0);
  }

  if (calOk && !homed && now - lastP > TIMEOUT_MS) {
    goHome();
    Serial.println(F("W home"));
  }

  if (calOk && now - rampLast >= RAMP_MS) {
    rampLast = now;
    for (int i = 0; i < N_LEGS; i++) {
      int e = target[i] - pos[i];
      if (e == 0) continue;
      pos[i] += constrain(e, -RAMP_US, RAMP_US);
      servos[i].writeMicroseconds(pos[i]);
    }
  }
}
