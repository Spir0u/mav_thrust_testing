#include "DShot4.h"

//#include <ros.h>
//#include <std_msgs/UInt16.h>

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

// void esc_cb(const std_msgs::UInt16& cmd_msg);
// ros::NodeHandle  nh;

DShot4 esc(DShot4::Mode::DSHOT600);

uint16_t throttle = 48;
uint16_t target = 0;
uint16_t target_old = 0;
uint8_t b1;
uint8_t b2;
char input[5];
uint8_t charsRead;
uint16_t sendTelemetry = 0;

int analogPin = A4;

/*void esc_cb(const std_msgs::UInt16& cmd_msg){
 digitalWrite(13, HIGH);
 target = cmd_msg.data;
 if(target == 48)
   digitalWrite(13, HIGH);  //disarmed: led on
 else
   digitalWrite(13, HIGH-digitalRead(13));  //armed: toggle LED
}

ros::Subscriber<std_msgs::UInt16> sub("/arduino/esc", esc_cb);
*/

void setup() {
  Serial.begin(115200);  // communicating over usb
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(analogPin, INPUT);
  analogReference(INTERNAL);  // INTERNAL: 2.5V (DEFAULT = 5V)

  // Notice, all pins must be connected to same PORT
  esc.attach(M1);
  esc.setThrottle(M1, throttle, 0);
  esc.attach(M2);
  esc.setThrottle(M2, throttle, 0);
  esc.attach(M3);
  esc.setThrottle(M3, throttle, 0);
  esc.attach(M4);
  esc.setThrottle(M4, throttle, 0);

  //  nh.initNode();
  //  nh.subscribe(sub);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    // b1 = Serial.read();
    // b2 = Serial.read();
    // target = b1<<8 + b2;
    charsRead =
        Serial.readBytesUntil('\n', input, 4);  // fetch the two characters
    input[charsRead] = '\0';                    // Make it a string
    target = (int)strtol(input, 0, 16);         // Convert it

    if (target > 2047)  // safety measure, disarm when wrong input
      target = 0;

    // Serial.print(target, DEC);    //, HEX);
    // Serial.print("\n");

    // Current : 15.2mv/A
    // analogRead: range 0-1023 for 0V to 2.53V 
    Serial.println(analogRead(analogPin));  // returns steps
    // Serial.println(1.075268817*analogRead(analogPin));  // returns mV to pc

    /*    b1 = target >> 8;
        b2 = target & 0xff;

        Serial.write(b1);     // Send target back
        Serial.write(b2);
        b1 = throttle >> 8;
        b2 = throttle & 0xff;
        Serial.write(b1);     // Send throttle back
        Serial.write(b2);
    */
    if (target != target_old) {
      digitalWrite(LED_BUILTIN, LOW);
      target_old = target;
    }

  } else if (!Serial) {  // No serial connection --> Disarm Motors
    target = 48;
  }

  if (throttle == target) {
    // digitalWrite(LED_BUILTIN, HIGH);
  } else {
    if (throttle <= 48) {  // special commands disabled
      throttle = 48;
    } else {
      throttle = target;
    }
  }

  /*if(sendTelemetry >= 2){
    esc.setThrottle(M1, target, 1);   //  send telemetry data from M1
    sendTelemetry = 0;
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(20);
    // esc.setThrottle(M1, target, 0);   //  send no telemetry data from M1
    // digitalWrite(LED_BUILTIN, LOW);
  }
  else{
    esc.setThrottle(M1, target, 0);
    sendTelemetry++;
  }*/
  esc.setThrottle(M1, target, 1);   //  send telemetry data from M1


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
  //  nh.spinOnce();
  // delay(2);
}
