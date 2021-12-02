#include "DShot4.h"

/*

  redefine DSHOT_PORT if you want to change the default PORT

  Defaults
  UNO: PORTD, available pins 0-7 (D0-D7)
  Leonardo: PORTB, available pins 4-7 (D8-D11)

  e.g.
  #define DSHOT_PORT PORTD
*/

#define M1 8
#define M2 9
#define M3 10
#define M4 11

DShot4 esc(DShot4::Mode::DSHOT300);

uint16_t throttle = 0;
uint16_t target = 0;

void setup() {
  Serial.begin(115200);   // communicating over usb
  Serial1.begin(115200);  // communicating over UART1

  // Notice, all pins must be connected to same PORT
  esc.attach(M1);
  esc.setThrottle(M1, throttle, 0);
  esc.attach(M2);
  esc.setThrottle(M2, throttle, 0);
  esc.attach(M3);
  esc.setThrottle(M3, throttle, 0);
  esc.attach(M4);
  esc.setThrottle(M4, throttle, 0);
}

void loop() {
  if (Serial.available() > 0) {
    target = Serial.parseInt();

    if (target > 2047)  // safety measure, disarm when wrong input
      target = 0;
    Serial.print(target, DEC);  //, HEX);
    Serial.print("\t");
    Serial.print(throttle, DEC);  //, HEX);
    Serial.print("\n");
  }

  if (throttle < 48) {  // special commands disabled
    throttle = 48;
  }
  if (target <= 48) {
    esc.setThrottle(M1, target, 0);
    if (target == 0) throttle = 48;
  } else {
    if (target > throttle) {
      throttle++;
      esc.setThrottle(M1, throttle, 0);
    } else if (target < throttle) {
      throttle--;
      esc.setThrottle(M1, throttle, 0);
    }
  }

  /*
    static uint8_t bufferTlm[TLM_LENGTH];
    while (Serial1.available() < TLM_LENGTH);
    for (int i = 0; i < TLM_LENGTH; i++) {
      bufferTlm[i] = Serial1.read();
    }
    tlmData.temperature = bufferTlm[0];
    tlmData.voltage = (bufferTlm[1] << 8) | bufferTlm[2];
    tlmData.current = (bufferTlm[3] << 8) | bufferTlm[4];
    tlmData.consumption = (bufferTlm[5] << 8) | bufferTlm[6];
    tlmData.rpm = (bufferTlm[7] << 8) | bufferTlm[8];
    tlmData.crcCheck = bufferTlm[9] == calculateCrc8(bufferTlm, TLM_LENGTH-1);
    Serial.write(tlmData);
  */

  delay(2);
}
