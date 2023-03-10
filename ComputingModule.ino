#include <ArduinoEigenDense.h>

using namespace Eigen;

struct Point {
    double x;
    double y;
    double z;
};

// Function to calculate the distance between two points
double distance(Point p1, Point p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) +
                     std::pow(p2.y - p1.y, 2) +
                     std::pow(p2.z - p1.z, 2));
}

// Function to calculate the position of a point given distances to three other points
Point trilaterate(Point p1, double d1,
                  Point p2, double d2,
                  Point p3, double d3) {
    // Calculate the distance between each pair of points
    double d12 = distance(p1, p2);
    double d23 = distance(p2, p3);
    double d13 = distance(p1, p3);

    // Calculate the position of the point
    double x = (std::pow(d1, 2) - std::pow(d2, 2) + std::pow(d12, 2)) / (2 * d12);
    double y = ((std::pow(d1, 2) - std::pow(d3, 2) + std::pow(d13, 2)) / (2 * d13)
                - x * (p2.y - p1.y) / (p3.y - p1.y));
    double z = std::sqrt(std::pow(d1, 2) - std::pow(x, 2) - std::pow(y, 2));

    // Return the position as a Point
    return { p1.x + x * (p2.x - p1.x) / d12,
             p1.y + x * (p2.y - p1.y) / d12 + y * (p3.y - p1.y) / d13,
             p1.z + x * (p2.z - p1.z) / d12 + y * (p3.z - p1.z) / d13 };
}

void setup() {

    Serial.begin(9600);


}

void loop() {
  Point p1 = { 0, 0, 0 };
  Point p2 = { 1500, 0, 0 };
  Point p3 = { 0, 1500, 0 };
  
  double d1 = 750*sqrt(2)-5;
  double d2 = 750*sqrt(2)+1;
  double d3 = 750*sqrt(2)-4;

  Point result = trilaterate(p1, d1, p2, d2, p3, d3);
  Serial.println("Position:");
  Serial.println("  x=" + String(result.x));
  Serial.println("  y=" + String(result.y));
  Serial.println("  z=" + String(result.z));
  delay(1500);
}