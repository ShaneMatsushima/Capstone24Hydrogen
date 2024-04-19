#include <FlexCAN_T4.h>
#include <kinetis_flexcan.h>
#include <ModbusMaster.h>
#include "TeensyThreads.h"

//TODO: create reference to make sure low and high are uploaded correctly (make another sketch for low / high based on this sketch)

//CAN stuff
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
static CAN_message_t msg1;

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
uint32_t modbusBaudRate = 19200; 
uint8_t slaveID = 101;  //Modbus slave ID

//Registers and data type. {Register address, datatype, multiplier} where the address is in hex and the datatype is 1 for floating point or 0 for integer and the multiplier is default 1.0 or whatever you need it to be
RegisterPair Registers[] = {
  {0x32, 1, 1.0} //Flow Rate
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

int CharsCycle = 0;
long Sats_Number = 0;
long LoopTimingLast = 0;

float Flow = 0;
float Temp = 0;
float Totalizer = 0;

const float lowFlowMin = 0;
const float lowFlowMax = 9;
const float highFlowMin = 0;
const float highFlowMax = 24;

enum use_status {FLOW_START, FLOW_STOP};
enum use_status use_flag = FLOW_START;

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
  threads.addThread(switchFlow);
}

void loop() {
  Flow = (RegisterValues[0]);
  Serial.print("Flow SCFM: ");
  Serial.print(Flow);

  // Temp = (RegisterValues[2]);
  // Serial.print(" Temp: ");
  // Serial.print(Temp);

  // Totalizer = (RegisterValues[1]);
  // Serial.print(" Totalizer: ");
  // Serial.print(Totalizer);

  Serial.print(" Loop Time: ");
  Serial.println(millis() - LoopTimingLast); 
  LoopTimingLast = millis();

  if(use_flag == FLOW_START) //if flagged to use
  {

    // Flow converstion to hex for CAN-BUS message
    long  FlowHex = ((Flow+150)*10000000);
    long a = FlowHex & 0xFF;
    long b = FlowHex >> 8 & 0xFF;
    long c = FlowHex >> 16;
    long d = FlowHex >> 24;

    // long TotalizerHex = ((Totalizer+1000)*10);
    // long e = TotalizerHex & 0xFF;
    // long f = TotalizerHex >> 8 & 0xFF;

    // long TemperatureHex = (Temp*100);
    // long g = TemperatureHex & 0xFF;
    // long h = TemperatureHex >> 8 & 0xFF;

    /*  
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

    */  

      //building the CAN message #1
      msg1.id = 0x09A; 
      msg1.len = 8; 
      msg1.buf[0] = a; //  start of flow
      msg1.buf[1] = b;
      msg1.buf[2] = c;
      msg1.buf[3] = d; //  end of flow
      msg1.buf[4] = 0xFF; //  start of totalizer
      msg1.buf[5] = 0xFF; //  end of totalizer
      msg1.buf[6] = 0xFF; //  start of temp
      msg1.buf[7] = 0xFF; //  end of temp
      can1.write(msg1); //sending the message
  }

  threads.delay(500);
}

void modthread() {
while(1) {
  threads.delay(200);
    uint16_t result;
  for (int i = 0; i < num_registers; i++) {
    result = node.readHoldingRegisters(Registers[i].address, 2*2);
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

void switchFlow(){
  while(1){
    threads.delay(800);
    float lowFlow;
    float highFlow;

    /**
     * cutoff @ 7.5kg/h
     * between should average (maybe average overall and find error based on flow meters and select based on lower error)
    */
    if (Serial.available()){
      // read data of other flow meter
      // using serial coms to transmit data between arduinos
      lowFlow = Flow;
      highFlow = (float)Serial.read();
    }
    float avg_flow = lowFlow + highFlow / 2;

    float low_diff = abs(avg_flow - lowFlow);
    float high_diff = abs(avg_flow - highFlow);

    if(lowFlow <= 7.5 && highFlow <= 7.5)
    {
      //use low flow meter
      Serial.println("Flow Meter Low Selected");
    } else if(lowFlow > 7.5 && highFlow > 7.5)
    {
      if(low_diff < high_diff)
        // use low flow meter
        Serial.print("Flow Meter Low Selected");
      else
        //use high flow meter
        Serial.println("Flow Meter High Selected");
    } else if(highFlow >= 9)
    {
      //use high flow meter
      Serial.println("Flow Meter High Selected");
    }

    else
      Serial.println("Both flow meters out of range...");

  }
}
