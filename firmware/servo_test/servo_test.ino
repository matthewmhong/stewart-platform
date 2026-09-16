// MG90S bench test: one servo, driven in microseconds from the Serial Monitor.
//
// Wiring
//   servo signal (orange) -> pin 9
//   servo +V (red)        -> battery + (4x AA, 4.8-6 V).  NOT the Arduino 5V / USB.
//   servo GND (brown)     -> battery -  AND  Arduino GND (common ground)
//   pin 13 LED            -> lights for 50 ms at every `jump`, for video sync
//
// Serial Monitor: 115200 baud.  Any line ending works.
//
// Commands
//   1500          go to 1500 us (any number)
//   c             centre (1500 us)
//   + / -         move by the step size (repeat by sending again)
//   s 1           set step size to 1 us
//   sweep A B N T slow sweep from A to B in N us steps, T ms per step
//                 (prints position; send anything to stop)
//   jump A B      go to A, wait 1 s, then jump to B with the LED flash
//   wide          allow 500-2500 us (default is 900-2100, safe for most servos)
//   detach        stop driving the servo (horn goes limp)
//   attach        drive it again at the last position
//   p             print position and settings
//   h             this help
//
// Tests (record results in STATUS.md, "MG90S parameters", measured column)
//   Degrees per us:  send 1000, 1500, 2000; read the protractor at each.
//                    deg/us = (angle at 2000 - angle at 1000) / 1000.
//   Usable travel:   `wide`, then `sweep 1500 2500 10 200` - stop when the
//                    horn stops moving or the servo buzzes; note the us.
//                    Repeat `sweep 1500 500 10 200`.  Back off 20 us from
//                    each end.  Don't hold a buzzing servo at its stop.
//   Deadband:        `1500`, `s 1`, then `+` repeatedly, watching the 100 mm
//                    pointer tip.  Count sends until it moves.  Always
//                    approach from the same side; repeat 5 times.
//   Backlash:        `1500` (powered, holding).  Rock the horn gently both
//                    ways by hand; read how far the pointer tip moves.
//                    Tip travel (mm) / 100 mm * 57.3 = backlash in degrees.
//   Speed:           mount a 60 degree step on video (240 fps), e.g.
//                    `jump 1200 1867` if deg/us = 0.09.  Frames from the LED
//                    lighting to the horn stopping / 240 = seconds per 60 deg.

#include <Servo.h>

const int SERVO_PIN = 9;
const int LED_PIN = 13;
const int SAFE_MIN = 900, SAFE_MAX = 2100;
const int WIDE_MIN = 500, WIDE_MAX = 2500;

Servo servo;
int us = 1500;
int stepUs = 10;
int lo = SAFE_MIN, hi = SAFE_MAX;

// sweep state
bool sweeping = false;
bool discarding = false;
int sweepTo = 0, sweepStep = 0;
unsigned long sweepPeriod = 0, sweepLast = 0;

char line[48];
int lineLen = 0;
unsigned long lastCharMs = 0;

void moveTo(int target) {
  if (target < lo || target > hi) {
    Serial.print(F("refused: "));
    Serial.print(target);
    Serial.print(F(" us is outside "));
    Serial.print(lo);
    Serial.print('-');
    Serial.print(hi);
    Serial.println(F(" (send `wide` to widen)"));
    return;
  }
  us = target;
  servo.writeMicroseconds(us);
  Serial.print(F("us = "));
  Serial.println(us);
}

void printHelp() {
  Serial.println(F("commands: <us>  c  + -  s <n>  sweep A B N T  jump A B  wide  detach  attach  p  h"));
}

void printStatus() {
  Serial.print(F("us = "));
  Serial.print(us);
  Serial.print(F("  step = "));
  Serial.print(stepUs);
  Serial.print(F("  limits = "));
  Serial.print(lo);
  Serial.print('-');
  Serial.print(hi);
  Serial.println(servo.attached() ? F("  attached") : F("  detached"));
}

void handle(char *cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') return;

  int a, b, n, t;
  if (isdigit(cmd[0])) {
    moveTo(atoi(cmd));
  } else if (strcmp(cmd, "c") == 0) {
    moveTo(1500);
  } else if (strcmp(cmd, "+") == 0) {
    moveTo(us + stepUs);
  } else if (strcmp(cmd, "-") == 0) {
    moveTo(us - stepUs);
  } else if (sscanf(cmd, "s %d", &n) == 1) {
    if (n > 0) stepUs = n;
    printStatus();
  } else if (sscanf(cmd, "sweep %d %d %d %d", &a, &b, &n, &t) == 4) {
    if (n <= 0 || t <= 0) {
      Serial.println(F("sweep: step and period must be positive"));
      return;
    }
    moveTo(a);
    if (us != a) return;  // refused
    sweepTo = b;
    sweepStep = (b >= a) ? n : -n;
    sweepPeriod = t;
    sweepLast = millis();
    sweeping = true;
    Serial.println(F("sweeping - send anything to stop"));
  } else if (sscanf(cmd, "jump %d %d", &a, &b) == 2) {
    if (a < lo || a > hi || b < lo || b > hi) {
      moveTo(a < lo || a > hi ? a : b);  // prints the refusal
      return;
    }
    moveTo(a);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    servo.writeMicroseconds(b);
    us = b;
    delay(50);
    digitalWrite(LED_PIN, LOW);
    Serial.print(F("jumped to "));
    Serial.println(us);
  } else if (strcmp(cmd, "wide") == 0) {
    lo = WIDE_MIN;
    hi = WIDE_MAX;
    printStatus();
  } else if (strcmp(cmd, "detach") == 0) {
    servo.detach();
    printStatus();
  } else if (strcmp(cmd, "attach") == 0) {
    servo.attach(SERVO_PIN, WIDE_MIN, WIDE_MAX);
    servo.writeMicroseconds(us);
    printStatus();
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
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  // Attach with 500-2500 so writeMicroseconds isn't clamped to the library's
  // default 544-2400; the safe limits above are enforced in moveTo instead.
  servo.attach(SERVO_PIN, WIDE_MIN, WIDE_MAX);
  servo.writeMicroseconds(us);
  Serial.println(F("MG90S bench test"));
  printStatus();
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (sweeping) {  // any input stops a sweep; the rest of that line is dropped
      sweeping = false;
      discarding = true;
      Serial.print(F("stopped at "));
      Serial.println(us);
    }
    if (ch == '\r') continue;
    if (discarding) {
      if (ch == '\n') discarding = false;
      continue;
    }
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

  if (sweeping && millis() - sweepLast >= sweepPeriod) {
    sweepLast += sweepPeriod;
    int next = us + sweepStep;
    bool done = (sweepStep > 0) ? next >= sweepTo : next <= sweepTo;
    if (done) next = sweepTo;
    moveTo(next);
    if (done || us != next) {
      sweeping = false;
      Serial.println(F("sweep done"));
    }
  }
}
