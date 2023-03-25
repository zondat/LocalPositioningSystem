// Debug
#define DEBUG_MODE
#ifdef DEBUG_MODE
  // #define DEBUG_BASE_1
  #define DEBUG_BASE_2
  // #define DEBUG_BASE_3
  #define DEBUG_COMPUTING
#endif

#include <SPI.h>              // include libraries
#include <LoRa.h>

// Lora Config
#define nss 8
#define rst 9
#define dio0 7

// Define constants for calculating distance
#define FREQ 433E6  // Frequency
#define RX_GAIN 2

#define BASE_1_TX_POWER 17.0        // LoRa radio transmit power in dBm
#define BASE_1_TX_GAIN 2.0
#define BASE_1_REF_RSSI -118     // Reference RSSI value at 1m distance
#define BASE_1_RSSI_CAL_FACTOR 4.9583  // Calibration factor for RSSI

#define BASE_2_TX_POWER 17.0        // LoRa radio transmit power in dBm
#define BASE_2_TX_GAIN 2.0
#define BASE_2_REF_RSSI -103     // Reference RSSI value at 1m distance
#define BASE_2_RSSI_CAL_FACTOR 1.1475  // Calibration factor for RSSI

#define BASE_3_TX_POWER 17.0        // LoRa radio transmit power in dBm
#define BASE_3_TX_GAIN 2.0
#define BASE_3_REF_RSSI -68     // Reference RSSI value at 1m distance
#define BASE_3_RSSI_CAL_FACTOR 4.9583  // Calibration factor for RSSI

// Indicator
#define INDICATOR_LED 17

void blink() {
  digitalWrite(INDICATOR_LED, HIGH);
  delay(250);
  digitalWrite(INDICATOR_LED, LOW);
  delay(250);
}

void stand() {
  digitalWrite(INDICATOR_LED, HIGH);
}


/*********************************/
#include <ArduinoEigenDense.h>
using namespace Eigen;

class Point {
  public:
    double x;
    double y;
    double z;
  public:
    Point(double x, double y, double z) {
      this->x = x;
      this->y = y;
      this->z = z;
    }

    double distanceTo(Point p2) {
    return std::sqrt(std::pow(p2.x - x, 2) +
                     std::pow(p2.y - y, 2) +
                     std::pow(p2.z - z, 2));
    }

    void trilaterate(Point base1, double dist2Base1,
                      Point base2, double dist2Base2,
                      Point base3, double dist2Base3) {
      // Calculate the distance between each pair of points
      double d12 = base1.distanceTo(base2);
      double d23 = base2.distanceTo(base3);
      double d13 = base3.distanceTo(base1);

      // Calculate the position of the point
      double dx = (std::pow(dist2Base1, 2) - std::pow(dist2Base2, 2) + std::pow(d12, 2)) / (2 * d12);
      double dy = ((std::pow(dist2Base1, 2) - std::pow(dist2Base3, 2) + std::pow(d13, 2)) / (2 * d13)
                  - dx * (base2.y - base1.y) / (base3.y - base1.y));
      double dz = std::sqrt(std::pow(dist2Base1, 2) - std::pow(dx, 2) - std::pow(dy, 2));

      // Return the position as a Point
      this->x = base1.x + dx * (base2.x - base1.x) / d12;
      this->y = base1.y + dx * (base2.y - base1.y) / d12 + dy * (base3.y - base1.y) / d13;
      this->z = base1.z + dx * (base2.z - base1.z) / d12 + dy * (base3.z - base1.z) / d13;     

    // #ifdef DEBUG_COMPUTING
    //   // received a packet
    //   // Serial.println("  x= " + String(this->x));
    //   // Serial.println("  y= " + String(this->y));
    //   // Serial.println("  z= " + String(this->z));
    //   Serial.println(String((int)this->x) + " " + String((int)this->y));
    // #endif

    }
};

double calculateDistance(double p_tx, double rssi, double rssi_1m, double n, double g_tx, double g_rx) {
    double d = std::pow(10, (p_tx - rssi + rssi_1m - g_tx - g_rx)/(10*n));
    return d;
}

/**************************************************/
Point base1(0, 0, 0), base2(1500, 0, 0), base3(0, 1500, 0), movingObject(0, 0, 0);
float dist2Base1, dist2Base2, dist2Base3;

/**************************************************/

void setup() {

  #ifdef DEBUG_MODE
    Serial.begin(9600);
    Serial.println("Initialize lora...");
  #endif
  
  pinMode(INDICATOR_LED, OUTPUT);

  bool initSucceeded = false;
  LoRa.setPins(nss, rst, dio0);
  while (!initSucceeded) {
    initSucceeded = LoRa.begin(433E6);
    blink();
  }
  stand();
 
  #ifdef DEBUG_MODE
    Serial.println("LoRa init succeeded.");
  #endif
}

void loop() {
  // Serial.println("Waiting for signal...");
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Serial.println("Received a signal");
    int rssi = LoRa.packetRssi();
    bool isUpdated = false;

    // Get Tx mess
    String baseName = "";
    while (LoRa.available()) {
      baseName += (char)LoRa.read();
    }

    if (baseName.compareTo("Base1")==0) {
      dist2Base1 = pow(10, (-rssi + BASE_1_REF_RSSI) / (double)(10.0 * BASE_1_RSSI_CAL_FACTOR)) * 1000;
      isUpdated = true;
    }
    else if (baseName.compareTo("Base2")==0) {
      dist2Base2 = pow(10, (-rssi + BASE_2_REF_RSSI) / (double)(10.0 * BASE_2_RSSI_CAL_FACTOR)) * 1000;
      isUpdated = true;
    }
    else if (baseName.compareTo("Base3")==0) {
      dist2Base3 = pow(10, (-rssi + BASE_3_REF_RSSI) / (double)(10.0 * BASE_3_RSSI_CAL_FACTOR)) * 1000;
      isUpdated = true;
    }    

    if (isUpdated) {
      movingObject.trilaterate(base1, dist2Base1,
                      base2, dist2Base2,
                      base3, dist2Base3);
    }

    #ifdef DEBUG_BASE_1
      if (baseName.compareTo("Base1")==0) {
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("Distance to ");
        Serial.print(dist2Base1);
        Serial.print(": ");
        Serial.println(distance);
        Serial.println(" mm");
      }
    #endif
    #ifdef DEBUG_BASE_2  
      if (baseName.compareTo("Base2")==0) {
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("Distance to ");
        Serial.print(baseName);
        Serial.print(": ");
        Serial.print(dist2Base2);
        Serial.println(" mm");
      }
    #endif
    #ifdef DEBUG_BASE_3 
      if (baseName.compareTo("Base3")==0) {
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("Distance to ");
        Serial.print(baseName);
        Serial.print(": ");
        Serial.print(dist2Base3);
        Serial.println(" mm");
      }
    #endif
    #ifdef DEBUG_COMPUTING
      Serial.println(String((int)movingObject.x) + " " + String((int)movingObject.y));
    #endif
    // #else
    //   lcd.clear();
    //   lcd.setCursor(0, 0);
    //   lcd.print("Str=" + String(abs(rssiFilter.updateEstimate(rssi))));
    //   lcd.setCursor(0, 1);
    //   lcd.print("Dis=" + String(distance) +"m");      
    // #endif
  }

  // delay(2000);
}

