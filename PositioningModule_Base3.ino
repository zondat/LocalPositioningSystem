#include <SPI.h>
#include <LoRa.h>

// #define DEBUG_MODE
#define TX_POWER 17

int counter = 0;
bool currentLEDState = HIGH;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  #ifdef DEBUG_MODE
    Serial.begin(9600);
    Serial.println("LoRa Sender");
  #endif

  // LoRa.setTxPower(TX_POWER);
  if (!LoRa.begin(433E6)) {
    #ifdef DEBUG_MODE
      Serial.println("Starting LoRa failed!");
    #endif
    while (1);
  }
  LoRa.setSpreadingFactor(7);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
}

void loop() {
  #ifdef DEBUG_MODE
    Serial.print("Sending packet: ");
    Serial.println(counter);
  #endif
  
  // send packet
  LoRa.beginPacket();
  LoRa.print("Base2");
  LoRa.endPacket();

  currentLEDState = !currentLEDState;
  digitalWrite(LED_BUILTIN, currentLEDState);
  counter++;

  delay(250);
}
