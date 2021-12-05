/*
  
  This is a simple test to see if the communication between the host (Udoo Bolt) and the built-in Arduino Leonardo works.
  On the Arduino, there is the arduino_Serialtest.ino program running.

  created 03.12.21 by Gabriel Käppeli 
*/

char b1 = 0;
char b2 = 0;
uint16_t received_int = 0;
char cstr[17];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Serial ready");

  digitalWrite(LED_BUILTIN, LOW);
  // delay(500);                    // connection errors if there are delays
  // digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if (!Serial) {
    digitalWrite(LED_BUILTIN, LOW);
    while (!Serial)
      ;
    Serial.flush();
    Serial.println("Serial ready");
  }
  if (Serial.available()) {  // If anything comes in Serial (USB),
    // uint8_t count = 0;
    // while (Serial.available()) {
    //   b1 = Serial.read();
    //   if (b1 == 0xff) count++;
    //   else count = 0;
    //   if(count >= 2) break;
    //   digitalWrite(LED_BUILTIN, LOW);
    // }

    b1 = Serial.read();
    b2 = Serial.read();
    received_int = (b1 << 8) + b2;
    if (b2 == 0xf0) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);
    if (b2 == 0xfe) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);

    itoa(received_int, cstr, 2);  // convert int to char to write to serial
    Serial.println(cstr);  // read it and send it out formatted to Serial (USB)
  }
}
