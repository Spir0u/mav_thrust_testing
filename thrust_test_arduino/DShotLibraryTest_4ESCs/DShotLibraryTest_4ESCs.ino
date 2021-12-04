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

//void esc_cb(const std_msgs::UInt16& cmd_msg);



DShot4 esc(DShot4::Mode::DSHOT150);

//ros::NodeHandle  nh;


uint16_t throttle = 0;
uint16_t target = 0;
uint16_t target_old = 0;

//void esc_cb(const std_msgs::UInt16& cmd_msg){
//  digitalWrite(13, HIGH);
//  target = cmd_msg.data;
//  if(target == 48)
//    digitalWrite(13, HIGH);  //disarmed: led on
//  else  
//    digitalWrite(13, HIGH-digitalRead(13));  //armed: toggle LED
//}

//ros::Subscriber<std_msgs::UInt16> sub("/arduino/esc", esc_cb);

void setup() {
  Serial.begin(115200);   // communicating over usb
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
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
    target = Serial.parseInt();

    if (target > 2047)  // safety measure, disarm when wrong input
      target = 0;
    Serial.print(target, DEC);  //, HEX);
    Serial.print("\t");
    Serial.print(throttle, DEC);  //, HEX);
    Serial.print("\n");

    if(target != target_old){
      digitalWrite(LED_BUILTIN, LOW);
      target_old = target;
    }
    
  }
  if(throttle == target){
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    if (throttle < 48) {  // special commands disabled
    throttle = 48;
    }
    if (target <= 48) {
      esc.setThrottle(M1, target, 0);
      if (target == 0 || target == 48) throttle = 48;
    } else {
      if (target > throttle) {
        throttle++;
        esc.setThrottle(M1, throttle, 0);
      } else if (target < throttle) {
        throttle--;
        esc.setThrottle(M1, throttle, 0);
      }
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
//  nh.spinOnce();
  delay(2);
}
