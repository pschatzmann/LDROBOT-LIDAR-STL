/**
 * @brief Example for reading STL-19P LiDAR data from `Serial1`.
 *
 * The sketch configures the parser, reads frames from the hardware serial
 * port connected to the LiDAR, and prints decoded point data to `Serial`.
 */
#include <LidarParserSTL.h>

LidarParserSTL lidar;

void onPoint(const LidarResultData& data, void* ref) {
  Serial.print("angle=");
  Serial.print(data.angle, 3);
  Serial.print(", distance=");
  Serial.print(data.distance, 3);
  Serial.print(", intensity=");
  Serial.print(data.intensity);
  Serial.print(", obstacle=");
  Serial.println(data.is_obstacle ? "true" : "false");
}

void setup() {
  //  setup logging
  Serial.begin(115200);
  // setup Serial1 for LIDAR input (RX=16, TX=17)
  Serial1.begin(230400, SERIAL_8N1, 16, 17);

  lidar.setResultCallback(onPoint);
  lidar.setAngleUnit(LidarAngleUnit::DEG_ROS);
  lidar.setDistanceUnit(LidarDistanceUnit::M);
  lidar.begin();
}

void loop() { lidar.readData(Serial1); }
