#include<Arduino.h>
#include<ModbusRTU.h>

#define TRACO_24VDC 23

#define rki_node 1
#define DE 12
#define RE 12
#define PERCENT_H2_ADDR 1

ModbusRTU RKI;
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data);

uint16_t H2_Percent = 0;

void setup() {
  pinMode(TRACO_24VDC, OUTPUT);
  digitalWrite(TRACO_24VDC, HIGH);
  // Initialize modbus RE and DE pins for RS-485
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  // Initialize in Receive mode
  digitalWrite(DE, 0);
  digitalWrite(RE, 0);

  // Initialize serial
  Serial.begin(9600);               // debug output
  Serial1.begin(9600, SERIAL_8E1); // to rki

  RKI.begin(&Serial1, DE, RE);
  RKI.setBaudrate(9600);
  RKI.client();

  Serial.println("End Setup.");

}

void loop() {

  
  Serial.println(RKI.readHreg(rki_node, 1, &H2_Percent));
  while (RKI.slave()) {
    RKI.task();
    yield();
  }


}