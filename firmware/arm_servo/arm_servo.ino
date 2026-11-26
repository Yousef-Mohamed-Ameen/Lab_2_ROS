// Arm servo firmware (Arduino Uno/Nano, Servo.h).
// Protocol (115200 baud, newline-terminated ASCII):
//     J,<a1>,<a2>,<a3>,<a4>\n      angles in degrees, 0..180
// Fewer than 4 values are accepted: only the first N servos are updated.
// ESP32: use the ESP32Servo library and change PINS.
#include <Servo.h>

const uint8_t N = 4;
const uint8_t PINS[N] = {3, 5, 6, 9};       // servo signal pins (PWM capable)
Servo servos[N];
float angle[N] = {90, 90, 90, 90};

char buf[64];
uint8_t len = 0;

void writeServo(uint8_t i, float a) {
  a = constrain(a, 0.0f, 180.0f);
  angle[i] = a;
  servos[i].writeMicroseconds((int)(500.0f + a * (2000.0f / 180.0f)));  // 500..2500 us
}

void handleLine(char *line) {
  if (line[0] != 'J' || line[1] != ',') return;
  char *tok = strtok(line + 2, ",");
  uint8_t i = 0;
  while (tok != NULL && i < N) {
    writeServo(i, atof(tok));
    tok = strtok(NULL, ",");
    i++;
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N; i++) {
    servos[i].attach(PINS[i], 500, 2500);
    writeServo(i, 90);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      handleLine(buf);
      len = 0;
    } else if (c != '\r' && len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else if (len >= sizeof(buf) - 1) {
      len = 0;  // overflow: drop line
    }
  }
}
