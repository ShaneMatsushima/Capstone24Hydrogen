#include <FlexCAN_T4.h>
#include <kinetis_flexcan.h>
#include <ModbusMaster.h>
#include "TeensyThreads.h"

//CAN stuff
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
static CAN_message_t msg1;
static CAN_message_t msg2;

//Pin definitions:
#define MAX485_RE 7  //Transmitter enable pin
int PowerON = 32;
int SigLOW = 33;
int SigHIGH = 9;

//Used to create an array with data type for registers
struct RegisterPair {
  uint16_t address;
  int dataType;
  float multiplier;
};

//Modbus Settings:
uint32_t modbusBaudRate = 38400;
uint8_t slaveID = 1;  //Modbus slave ID

//Registers and data type. {Register address, datatype, multiplier} where the address is in hex and the datatype is 1 for floating point or 0 for integer and the multiplier is default 1.0 or whatever you need it to be
RegisterPair Registers[] = {
  { 0x001A, 1, 1.0 }, //Flowrate  l/min 15.29 to 4247.5
  { 0x001B, 1, 1.0 }, //Flowrate 0.54 to 150 SCFM
  { 0x0028, 1, 1.0 }, //Pressure 0 to 16 bar
  { 0x0029, 1, 1.0 }, //Pressure 0 to 250 psi
  { 0x0048, 1, 1.0 }, //Temp 0 to 60 C
  { 0x0049, 1, 1.0 }  //Temp 32 to 140 F
};
//Tells us how many total registers there are in array
const int num_registers = sizeof(Registers) / sizeof(Registers[0]);
ModbusMaster node;  //ModbusMaster object initialization

typedef union {
  float floatValue;
  uint16_t intValue;
  uint16_t regValue[2];
} RegisterData;

//Holds the response data for each register
float RegisterValues[num_registers];


long LastGPSchars = 10000;
long CurrGPSchars = 10000;
int CharsCycle = 0;
long Sats_Number = 0;
long LoopTimingLast = 0;

void setup() {
  pinMode(MAX485_RE, OUTPUT); 
  pinMode(PowerON, OUTPUT); 
  pinMode(SigLOW, OUTPUT); 
  pinMode(SigHIGH, OUTPUT); 
  Serial.begin(112500);

  can1.begin();
  can1.setBaudRate(500000);

  Serial7.begin(modbusBaudRate); // SERIAL_8N1
  Serial7.transmitterEnable(MAX485_RE);
  node.begin(slaveID, Serial7);
  
  threads.addThread(modthread); 
  threads.addThread(flashLEDs);
}

void loop() {
  float TempF = (RegisterValues[5]);
  float PressPSI = (RegisterValues[3]);
  Serial.print("Temp DegF: ");
  Serial.print(TempF);
  Serial.print(" Press PSI: ");
  Serial.print(PressPSI);
  Serial.print(" Loop Time: ");
  Serial.println(millis() - LoopTimingLast); 
  LoopTimingLast = millis();
   
   // Latitude conversion to hex for CAN-BUS message 
  float Lat = 45.4003;
  long latitude = ((Lat+150)*10000000);
  long a = latitude & 0xFF;
  long b = latitude >> 8 & 0xFF;
  long c = latitude >> 16;
  long d = latitude >> 24;
  
  // Longitude conversion to hex for CAN-BUS message
  float Lon = -122.6944;
  long longitude = ((Lon+150)*10000000);
  long e = longitude & 0xFF;
  long f = longitude >> 8 & 0xFF;
  long g = longitude >> 16;
  long h = longitude >> 24; 
  
  // Altitude conversion to hex for CAN-BUS message
  float Alt = 130;
  long Altitude = ((130+1000)*10);
  long a2 = Altitude & 0xFF;
  long b2 = Altitude >> 8 & 0xFF;
   
  // Speed conversion to hex for CAN-BUS message  
  float Speed = 23.55;
  long Speed_mph = (Speed*100);
  long c2 = Speed_mph & 0xFF;
  long d2 = Speed_mph >> 8 & 0xFF;
  // number of satellites conversion to hex for CAN-BUS message


    //building the CAN message #1
    msg1.id = 0x09A; 
    msg1.len = 8; 
    msg1.buf[0] = a;
    msg1.buf[1] = b;
    msg1.buf[2] = c;
    msg1.buf[3] = d;
    msg1.buf[4] = e;
    msg1.buf[5] = f;
    msg1.buf[6] = g;
    msg1.buf[7] = h;
    can1.write(msg1); //sending the message

    //building the CAN message #2
    msg2.id = 0x09B; 
    msg2.len = 8; 
    msg2.buf[0] = a2;
    msg2.buf[1] = b2;
    msg2.buf[2] = c2;
    msg2.buf[3] = d2;
    msg2.buf[4] = 0xFF;
    msg2.buf[5] = 0xFF;
    msg2.buf[6] = 0xFF;
    msg2.buf[7] = 0xFF;
    can1.write(msg2); //sending the message

  threads.delay(500);
}

void modthread() {
while(1) {
  threads.delay(200);
    uint8_t result;
  for (int i = 0; i < num_registers; i++) {
    result = node.readHoldingRegisters(Registers[i].address, 2);
    if (result == node.ku8MBSuccess) {
      RegisterData data;

      if (Registers[i].dataType == 1) {
        data.regValue[0] = node.getResponseBuffer(0x00);
        data.regValue[1] = node.getResponseBuffer(0x01);

        //Convert little-endian 16 bit values to float values
        float value = data.floatValue;

        //Adjust value with multiplier if there is one
        if (Registers[i].multiplier != 1.0) {
          value *= Registers[i].multiplier;
        }
        RegisterValues[i] = value;
      } else {
        data.intValue = node.getResponseBuffer(0);

        if (Registers[i].multiplier != 1.0) {
          data.intValue *= Registers[i].multiplier;
        }
        RegisterValues[i] = data.intValue;
      }
    } else {
      Serial.print("Failed, Response Code: ");
      Serial.print(result, HEX);
      Serial.println("");
      for (int i = 0; i < num_registers; i++) {
         RegisterValues[i] = 0;
      }
    }
  }
} 
}

void flashLEDs() {
 while(1) {
   threads.delay(400);
   if (Sats_Number  <= 3)
   {
      digitalWriteFast(PowerON, HIGH);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, LOW);
   } 
   if (Sats_Number  > 3 && Sats_Number < 7)
   {
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, HIGH);
      digitalWriteFast(SigHIGH, LOW);
   } 
   if (Sats_Number  >= 7)
   {
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, HIGH);
   } 
    
   CurrGPSchars = LastGPSchars;
   if (LastGPSchars == CurrGPSchars) 
    {
      CharsCycle = CharsCycle + 1;
    }
    else
    {
      CharsCycle = 0;
    }
    
    if (LastGPSchars == CurrGPSchars && CharsCycle > 3) 
    {
      CharsCycle = 0;
      digitalWriteFast(PowerON, HIGH);
      digitalWriteFast(SigLOW, HIGH);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(50);
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(50);
      digitalWriteFast(PowerON, HIGH);
      digitalWriteFast(SigLOW, HIGH);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(50);
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(50);
      digitalWriteFast(PowerON, HIGH);
      digitalWriteFast(SigLOW, HIGH);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(50);
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(50);
      digitalWriteFast(PowerON, HIGH);
      digitalWriteFast(SigLOW, HIGH);
      digitalWriteFast(SigHIGH, LOW);
    }
    else if  (Sats_Number  == 0)   {
      digitalWriteFast(PowerON, HIGH);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(35);
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, HIGH);
      digitalWriteFast(SigHIGH, LOW);
      threads.delay(35);
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, HIGH);
      threads.delay(15);
      digitalWriteFast(PowerON, LOW);
      digitalWriteFast(SigLOW, LOW);
      digitalWriteFast(SigHIGH, LOW);
    } 
}
}
