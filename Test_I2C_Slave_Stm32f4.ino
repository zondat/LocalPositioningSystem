#include <Wire.h>
#include <SoftWire.h>  //Library for I2C Communication functions

// I2C message structure
struct Position {
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

struct Message {
  uint8_t objectId;
  Position position;
};

// I2C config
#define SLAVE_ADDR 8

// free ram
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/******************/
void setup() 
{
  Serial.begin(9600);                         //Sets pin 7 as output
  Wire.begin(SLAVE_ADDR);                             // join i2c bus with its slave Address as 8 at pin (A4,A5)
  // Wire.onReceive(receiveEvent);              //Function call when Slave Arduino receives value from master STM32
  Wire.onRequest(requestEvent);              //Function call when Master STM32 request value from Slave Arduino
}

void loop() 
{
  delay(100);
}

// void receiveEvent (int howMany)              //This Function is called when Slave Arduino receives value from master STM32
// {
//   byte a = Wire.read();                      //Used to read value received from master STM32 and store in variable a
//   if (a == 1)                                //Logic to turn Slave LED ON (if received value is 1) or OFF (if received value is 0)
//   {
//    digitalWrite(LED,HIGH);
//    Serial.println("Slave LED ON");
//   }
//   else
//   {
//     digitalWrite(LED,LOW);
//     Serial.println("Slave LED OFF");
//   }

//   delay(500);
// }


void requestEvent()                            //This Function is called when Master STM32 wants value from slave Arduino
{
  Message* mess;
  mess = (Message*) malloc(sizeof(Message));
  Position position;
  position.x = 750;
  position.y = 750;
  position.z = 0;

  mess->position = position;
  mess->objectId = 0;
  Wire.beginTransmission(1);                                                
  Wire.write((byte *)mess, sizeof(Message));                                                    
  Wire.endTransmission();    
  free(mess);                        
}