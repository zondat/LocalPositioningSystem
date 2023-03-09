#include <Wire.h>
// #include <SoftWire.h>  //Library for I2C Communication functions

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
  Serial.begin(9600);  
  Wire.begin();  
}

void loop()
{
  Message* mess = malloc(sizeof(Message));
  Wire.requestFrom(SLAVE_ADDR, sizeof(Message)); 
  Wire.readBytes((byte*)mess, sizeof(Message)); 
  Serial.println("Size of message: " + String(sizeof(Message)));
  Serial.println("Position of Object " + String(mess->objectId));
  Serial.println("\tx= " + mess->position.x);
  Serial.println("\ty= " + mess->position.y);
  Serial.println("\tz= " + mess->position.z);
  free(mess);
  Serial.println("Ram: " + String(freeRam()));
  delay(1000);
}