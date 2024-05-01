#include <ArduinoRS485.h>
#include <ArduinoModbus.h>


#define MOD_BAUD 115200

//NOTE: Use Coolterm to log serial to txt for testing

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial7.begin(MOD_BAUD);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!ModBusRTUClient.begin)
  //send reading request 
  if (!ModbusRTUClient.requestFrom(0x65, 0x03, 0x32, 0x4)){
    Serial.println("Failed to read flow");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    //response handler
    uint16_t word1 = ModbusRTUClient.read();
    uint16_t word2 = ModbusRTUClient.read();
    uint32_t flowrate = word1 << 16 | word2;
    Serial.print("Flow Rate: ");
    Serial.println(flowrate);
  }


}
