#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "LidarLogger.h"

#ifdef ARDUINO
#include "Arduino.h"
#include "Stream.h"
#endif

/**
 * @enum LidarAngleUnit
 * @brief Angle output unit for decoded LiDAR points.
 */
enum class LidarAngleUnit {
  DEG = 0,                //!< 0..360 degree clockwise
  RAD = 1,                //!< 0..2*PI radian clockwise
  DEG_ROS = 2,            //!< +left, -right in degree (default)
  RAD_ROS = 3,            //!< +left, -right in radian
  RADIIAN_ROS = RAD_ROS,  //!< backward compatible typo alias
};

/**
 * @enum LidarDistanceUnit
 * @brief Distance output unit for decoded LiDAR points.
 */
enum class LidarDistanceUnit {
  MM,  //!< distance in millimeters
  CM,  //!< distance in centimeters
  M,   //!< distance in meters (default)
  IN,  //!< distance in inches
};

/**
 * @brief One decoded LiDAR sample point.
 *
 * Represents a single point extracted from a packet and delivered to the
 * result callback.
 */
struct LidarResultData {
  float angle;        //!< angle in the unit selected via `setAngleUnit()`
  float distance;     //!< distance in the unit selected via `setDistanceUnit()`
  uint8_t intensity;  //!< intensity in range 0..255
  uint64_t stamp;     //!< packet timestamp
  bool is_obstacle = true;  //!< true if the point is considered an obstacle based
                            //!< on distance threshold */

  /**
   * @brief Construct a result point.
   * @param angle Point angle in current output unit.
   * @param distance Point distance in current output unit.
   * @param intensity Point intensity in range 0..255.
   * @param stamp Packet timestamp.
   */
  LidarResultData(float angle, float distance, uint8_t intensity,
                  uint64_t stamp = 0, bool obstacle = true) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->stamp = stamp;
    this->is_obstacle = obstacle;
  }
  /**
   * @brief Default constructor.
   */
  LidarResultData() {}
};

/**
 * @brief LDRobot LiDAR STL-19P packet parser for Arduino streams.
 *
 * Parses LDRobot LiDAR STL-19P frames from a `Stream`, verifies CRC,
 * interpolates angles for each sample in the packet, and reports result through
 * a callback.
 *
 * Use `setResultCallback()` to receive the `LidarResultData` ,
 * `setAngleUnit()` to select angle representation, and `setDistanceUnit()`
 * to select distance representation.
 *
 */
class LidarParserSTL {
 public:
  using ReadByteCallback = bool (*)(uint8_t& data, void* ref);

  /**
   * @brief Construct a parser that reads bytes from an Arduino stream.
   */
  LidarParserSTL() = default;

  /**
   * @brief Construct a parser with a pre-registered result callback.
   * @param callback Function called with one parsed point.
   * @param ref User context pointer passed back to the callback.
   */
  LidarParserSTL(void (*callback)(const LidarResultData&, void*),
                 void* ref = nullptr) {
    setResultCallback(callback, ref);
  }

  /**
   * @brief Register a callback that is invoked for each decoded point.
   * @param callback Function called with one parsed point.
   * @param ref User context pointer passed back to the callback. If nothing
   * is passed, the parser instance pointer will be used as context.
   */
  void setResultCallback(void (*callback)(const LidarResultData&, void*),
                         void* ref = nullptr) {
    on_point_ = callback;
    this->ref = ref == nullptr ? this : ref;
  }

  /**
   * @brief Set output angle unit used for callback data and getter values.
   * @param unit Requested angle unit.
   */
  void setAngleUnit(LidarAngleUnit unit) { angle_unit = unit; }

  /**
   * @brief Set output distance unit used for callback data and getter values.
   * @param unit Requested distance unit.
   */
  void setDistanceUnit(LidarDistanceUnit unit) { distance_unit = unit; }

  /**
   * @brief Set the maximum range reported when no obstacle is detected.
   *
   * When the sensor returns a zero distance (no valid return), `is_obstacle`
   * will be set to `false` and the reported distance will be this value
   * converted to the current distance unit.
   *
   * @param meters Maximum range in metres. Default is 12.0 m.
   */
  void setMaxRange(float meters) { max_range_m_ = meters; }

  /**
   * @brief Get the speed from the most recently parsed packet.
   * @return Raw speed value from protocol.
   */
  uint16_t getSpeed() const { return out_speed_; }
  /**
   * @brief Get the converted start angle of the last parsed packet.
   * @return Start angle in the currently selected `LidarAngleUnit`.
   */
  float getStartAngle() const { return out_start_angle_; }
  /**
   * @brief Get the converted end angle of the last parsed packet.
   * @return End angle in the currently selected `LidarAngleUnit`.
   */
  float getEndAngle() const { return out_end_angle_; }
  /**
   * @brief Get packet timestamp from the most recently parsed packet.
   * @return Timestamp from protocol payload.
   */
  uint16_t getTimestamp() const { return out_timestamp_; }

  /**
   * @brief Start the parser. Must be called before `readData()`.
   * @return true if the parser is ready to read data; false otherwise.
   */
  bool begin() {
    is_active = true;
    return true;
  }

  /**
   * @brief Stop the parser. After calling this, `readData()` will return false
   * until `begin()` is called again. This can be used to temporarily disable
   * parsing without losing the registered callback or configuration.
   * @return void
   */
  void end() { is_active = false; }

  /**
   * @brief Set a custom byte-read callback used by `readData()`.
   *
   * This enables integrating non-Arduino transports by providing a function
   * that returns one byte at a time.
   *
   * @param callback Function that reads one byte into `data` and returns true
   * on success.
   * @param ref User context passed back to `callback`.
   */
  void setReadByteCallback(ReadByteCallback callback, void* ref = nullptr) {
    read_byte_func = callback;
    read_byte_ref = ref;
  }

  /**
   * @brief Set parser logger level.
   * @param level Minimum level that will be emitted.
   */
  void setLogLevel(LidarLogLevel level) { logger_.setLevel(level); }

#ifdef ARDUINO
  /**
   * @brief Set Arduino print sink for parser log messages.
   * @param output Print target.
   */
  void setLogOutput(Print& output) { logger_.setOutput(output); }
#else
  /**
   * @brief Set FILE sink for parser log messages.
   * @param output File target.
   */
  void setLogOutput(FILE* output) { logger_.setOutput(output); }
#endif

#ifdef ARDUINO
  /**
   * @brief Read and parse one packet from the input stream.
   *
   * The method synchronizes on packet header bytes (`0x54`, `0x2C`), reads
   * the full frame, validates CRC, converts angles to the requested unit, and
   * emits points via the registered callback.
   *
   * @return true if one valid packet was parsed; false otherwise.
   */
  bool readData(Stream& input_stream) {
    if (input_stream.available() == 0) {
      logger_.info("readData(Stream): input stream has no available bytes");
      return false;
    }
    setReadByteCallback(&LidarParserSTL::read_byte_from_stream_cb,
                        &input_stream);
    return readData();
  }
#endif

  /**
   * @brief Read and parse one packet using the read byte callback.
   *
   * The method synchronizes on packet header bytes (`0x54`, `0x2C`), reads
   * the full frame, validates CRC, converts angles to the requested unit, and
   * emits points via the registered callback.
   *
   * @return true if one valid packet was parsed; false otherwise.
   */
  bool readData() {
    if (read_byte_func == nullptr) {
      logger_.error("readData(): read byte callback is not set");
      return false;
    }

    if (!is_active) {
      logger_.warn("readData(): parser is not active, call begin() first");
      return false;
    }

    uint8_t frame[kFrameLength] = {0};

    uint8_t b = 0;
    bool header_seen = false;
    while (true) {
      if (!read_byte_func(b, read_byte_ref)) {
        logger_.warn("readData(): failed while waiting for frame header");
        return false;
      }

      if (!header_seen) {
        if (b == kHeader) {
          header_seen = true;
        }
        continue;
      }

      if (b == kVerLen) {
        frame[0] = kHeader;
        frame[1] = kVerLen;
        break;
      }

      // Keep sync if we received a repeated header byte, otherwise restart.
      header_seen = (b == kHeader);
      if (!header_seen) {
        logger_.trace(
            "readData(): header candidate rejected, second byte=0x%02X", b);
      }
    }

    for (size_t i = 2; i < kFrameLength; ++i) {
      if (!read_byte_func(frame[i], read_byte_ref)) {
        logger_.warn("readData(): failed to read frame payload byte %u/%u",
                     static_cast<unsigned>(i + 1),
                     static_cast<unsigned>(kFrameLength));
        return false;
      }
    }

    return parsePacket(frame, kFrameLength);
  }

 protected:
  static constexpr uint8_t kHeader = 0x54;
  static constexpr uint8_t kVerLen = 0x2C;
  static constexpr size_t kPointsPerPacket = 12;
  static constexpr size_t kFrameLength = 47;
  bool is_active = false;
  void (*on_point_)(const LidarResultData& data, void* ref) = nullptr;
  void* ref = nullptr;
  LidarAngleUnit angle_unit = LidarAngleUnit::DEG_ROS;
  LidarDistanceUnit distance_unit = LidarDistanceUnit::M;
  uint16_t out_speed_ = 0;
  float out_start_angle_ = 0.0f;
  float out_end_angle_ = 0.0f;
  float max_range_m_ = 12.0f;  // default max range for LD LiDAR in meters
  uint16_t out_timestamp_ = 0;
  ReadByteCallback read_byte_func = nullptr;
  void* read_byte_ref = nullptr;
  LidarLoggerClass logger_;

  /// @brief Parse one full packet frame and emit points via callback.
  bool parsePacket(const uint8_t* packet, size_t len) {
    logger_.debug("parsePacket(): parsing packet with length=%u", static_cast<unsigned>(len));
    if (packet == nullptr) {
      logger_.error("parsePacket(): packet is null");
      return false;
    }

    if (len != kFrameLength) {
      logger_.error("parsePacket(): invalid frame length=%u expected=%u",
                    static_cast<unsigned>(len),
                    static_cast<unsigned>(kFrameLength));
      return false;
    }

    if (packet[0] != kHeader || packet[1] != kVerLen) {
      logger_.error(
          "parsePacket(): invalid header bytes [0]=0x%02X [1]=0x%02X expected "
          "0x%02X 0x%02X",
          packet[0], packet[1], kHeader, kVerLen);
      return false;
    }

    constexpr size_t crcIndex = 46;  // CRC byte index in the native frame
    const uint8_t received_crc = packet[crcIndex];
    const uint8_t calculated_crc = calculateCRC8(packet, crcIndex);
    if (received_crc != calculated_crc) {
      logger_.error(
          "parsePacket(): crc mismatch received=0x%02X calculated=0x%02X",
          received_crc, calculated_crc);
      return false;
    }

    const uint16_t speed = readU16LE(packet + 2);
    const uint16_t start_angle_raw = readU16LE(packet + 4);
    const uint16_t end_angle_raw = readU16LE(packet + 42);
    const uint16_t timestamp = readU16LE(packet + 44);

    const float start_angle =
        static_cast<float>(start_angle_raw % 36000U) / 100.0f;
    const float end_angle = static_cast<float>(end_angle_raw % 36000U) / 100.0f;

    float angle_diff = end_angle - start_angle;
    while (angle_diff < 0.0f) {
      angle_diff += 360.0f;
    }
    while (angle_diff >= 360.0f) {
      angle_diff -= 360.0f;
    }

    const float angle_increment =
        angle_diff / static_cast<float>(kPointsPerPacket - 1);

    size_t offset = 6;
    for (size_t i = 0; i < kPointsPerPacket; ++i) {
      const uint16_t distance_mm = readU16LE(packet + offset);
      const uint8_t intensity = packet[offset + 2];

      float angle = start_angle + static_cast<float>(i) * angle_increment;
      while (angle >= 360.0f) {
        angle -= 360.0f;
      }

      const bool is_obstacle = (distance_mm > 0);
      const float raw_distance_mm = is_obstacle
                                        ? static_cast<float>(distance_mm)
                                        : (max_range_m_ * 1000.0f);
      const float converted_angle = convertAngle(angle);
      const float converted_distance = convertDistance(raw_distance_mm);
      if (on_point_ != nullptr) {
        on_point_(
            LidarResultData(converted_angle, converted_distance, intensity,
                            static_cast<uint64_t>(timestamp), is_obstacle),
            ref);
      } else {
        // if no callback is registered, print the point data to Serial for
        // debugging
        char buffer[100];
        snprintf(buffer, sizeof(buffer),
                 "Angle: %.2f, Distance: %.2f, Intensity: %u, Timestamp: %u",
                 converted_angle, converted_distance, intensity, timestamp);
        logger_.info(buffer);
      }
      offset += 3;
    }

    out_speed_ = speed;
    out_start_angle_ = convertAngle(start_angle);
    out_end_angle_ = convertAngle(end_angle);
    out_timestamp_ = timestamp;
    return true;
  }

  float convertAngle(float angle_deg) const {
    constexpr float kPi = 3.14159265358979323846f;
    switch (angle_unit) {
      case LidarAngleUnit::DEG:
        return angle_deg;
      case LidarAngleUnit::RAD:
        return angle_deg * (kPi / 180.0f);
      case LidarAngleUnit::DEG_ROS: {
        return (angle_deg <= 180.0f) ? angle_deg : (angle_deg - 360.0f);
      }
      case LidarAngleUnit::RAD_ROS: {
        const float angle_deg_ros =
            (angle_deg <= 180.0f) ? angle_deg : (angle_deg - 360.0f);
        return angle_deg_ros * (kPi / 180.0f);
      }
      default:
        return angle_deg;
    }
  }

  float convertDistance(float distance_mm) const {
    switch (distance_unit) {
      case LidarDistanceUnit::MM:
        return distance_mm;
      case LidarDistanceUnit::CM:
        return distance_mm / 10.0f;
      case LidarDistanceUnit::M:
        return distance_mm / 1000.0f;
      case LidarDistanceUnit::IN:
        return distance_mm / 25.4f;
      default:
        return distance_mm;
    }
  }

#ifdef ARDUINO
  static bool read_byte(Stream& input_stream, uint8_t& b) {
    int result = input_stream.read();
    if (result >= 0) {
      b = static_cast<uint8_t>(result);
      return true;
    }
    return false;
  }

  static bool read_byte_from_stream_cb(uint8_t& b, void* ref) {
    if (ref == nullptr) return false;
    Stream* stream = static_cast<Stream*>(ref);
    return read_byte(*stream, b);
  }

#endif

  uint8_t calculateCRC8(const uint8_t* data, size_t len) const {
    static const uint8_t crc_table[256] = {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
        0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
        0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
        0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
        0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
        0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
        0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
        0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
        0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
        0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
        0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
        0x7f, 0x32, 0xe5, 0xa8};
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
      crc = crc_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
  }

  uint16_t readU16LE(const uint8_t* p) const {
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                                 (static_cast<uint16_t>(p[1]) << 8));
  }
};
