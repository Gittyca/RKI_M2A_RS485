//rs485 example code for rki gas sensor
//
#include <Arduino.h>
#include <ModbusRTU.h>

#define TRACO_24VDC 23

//node addr
#define rki_node 1

#define DE 12
#define RE 13

//reading addr
#define PERCENT_H2_ADDR 0x0001

//Instance
ModbusRTU RKI_M2A;

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data);

float RKI_M2A_Read_Value(void);

float Percent_H2 = 0;
bool rd_success_flg = false;

void setup() {

  // Turn on Power Supply
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

  // Start the RS-485 connection and the client
  RKI_M2A.begin(&Serial1, DE, RE);
  RKI_M2A.setBaudrate(9600);
  RKI_M2A.client();

  Serial.println("End Setup.");

}

void loop() {

  static uint16_t count = 0;

  Serial.print(count++);
  Serial.println(": ");

  Percent_H2 = RKI_M2A_Read_Value();
  Serial.print("Percent H2: ");
  Serial.println(Percent_H2);

}


bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
  static uint8_t cb_cnt = 0;
  const uint8_t max_cnt = 5;

  if ((event != Modbus::EX_SUCCESS)  && (event != Modbus::EX_DATA_MISMACH) && cb_cnt <= max_cnt)
  {
    cb_cnt++;
    Serial.print("Request result: 0x");
    Serial.println(event, HEX);
  }
  else
  {
    cb_cnt = 0;
    rd_success_flg = true;
  }

  return true;
}

float RKI_M2A_Read_Value(void)
{ 
  float rtn = 0;
  uint16_t raw_buff[2] = {0};
  uint16_t addr = PERCENT_H2_ADDR;
  union flreg_t
  {
    float asFloat;
    uint16_t asInt[2];
  };
  flreg_t flreg = {0};

  while (rd_success_flg == false)
  {
    if (!RKI_M2A.slave())
    {
      RKI_M2A.readHreg(rki_node, addr, &raw_buff[0], 2, cbWrite);
      while (RKI_M2A.slave())
      {
        RKI_M2A.task();
        yield();
      }
    }
  }
  flreg.asInt[0] = raw_buff[1];
  flreg.asInt[1] = raw_buff[0];
  rtn = flreg.asFloat;

  rd_success_flg = false;

  return rtn;
}
