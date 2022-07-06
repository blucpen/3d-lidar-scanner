#include <Servo.h>

Servo servo;
char buf[80];

int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

void setup() {
  servo.attach(9,500,2500);  // (pin, min, max)
  Serial.begin(115200);
}

void loop() {
  if (readline(Serial.read(), buf, 80) > 0) {
    int angleDegrees = atoi(buf);
    int microValue = map(angleDegrees, 0,270,500,2500);
    servo.writeMicroseconds(microValue);
    delay(1000);
    Serial.write("0\n");
  }
}
