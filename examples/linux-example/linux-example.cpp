/**
 * @brief Example for reading STL-19P LiDAR data without Arduino.
 *
 * The program opens a serial device on Linux, feeds incoming bytes to the
 * parser through a custom read callback, and prints decoded point data to
 * standard output.
 */
#include <LidarParserSTL.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

LidarParserSTL lidar;

// print one point received from the parser callback
void onPoint(const LidarResultData& data, void* ref) {
  (void)ref;
  printf("angle=%.3f, distance=%.3f, intensity=%u, obstacle=%s\n", data.angle,
         data.distance, data.intensity, data.is_obstacle ? "true" : "false");
}

// read byte callback that reads from a file descriptor passed via ref
bool readByte(uint8_t& data, void* ref) {
  if (ref == nullptr) return false;
  // Pass a pointer to an int file descriptor via ref .e.g /dev/ttyUSB0
  const int fd = *static_cast<int*>(ref);
  //  Read one byte from the file descriptor
  return ::read(fd, &data, 1) == 1;
}

bool configureSerialPort(int fd) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    perror("tcgetattr failed");
    return false;
  }

  cfmakeraw(&tty);

#ifdef B230400
  const speed_t baud = B230400;
#else
  const speed_t baud = B115200;
  fprintf(stderr,
          "Warning: B230400 not available on this platform, using 115200\n");
#endif

  if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
    perror("cfsetispeed/cfsetospeed failed");
    return false;
  }

  tty.c_cflag &= ~PARENB;           // no parity
  tty.c_cflag &= ~CSTOPB;           // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;               // 8 data bits
  tty.c_cflag |= CREAD | CLOCAL;    // enable receiver, ignore modem ctrl

  tty.c_cc[VMIN] = 1;               // block for at least 1 byte
  tty.c_cc[VTIME] = 0;              // no inter-byte timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr failed");
    return false;
  }

  if (tcflush(fd, TCIFLUSH) != 0) {
    perror("tcflush failed");
    return false;
  }

  return true;
}

// helper function to open a serial device with retry if it doesn't exist yet
int openDeviceWithWait(const char* device) {
  int fd = -1;
  bool waiting_for_device = false;
  while (fd < 0) {
    fd = ::open(device, O_RDONLY | O_NOCTTY);
    if (fd >= 0) break;

    if (errno == ENOENT) {
      if (!waiting_for_device) {
        fprintf(stderr, "Device '%s' not found. Waiting for it...\n", device);
        waiting_for_device = true;
      }
      ::sleep(1);
      continue;
    }

    perror(device);
    return -1;
  }

  if (waiting_for_device) {
    fprintf(stderr, "Device '%s' found. Starting read loop.\n", device);
  }

  return fd;
}

int main(int argc, char** argv) {
  const char* device = (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0')
                           ? argv[1]
                           : "/dev/ttyUSB0";

  int usbFd = openDeviceWithWait(device);
  if (usbFd < 0) return 1;

  if (!configureSerialPort(usbFd)) {
    ::close(usbFd);
    return 1;
  }

  lidar.setResultCallback(onPoint);
  lidar.setAngleUnit(LidarAngleUnit::DEG_ROS);
  lidar.setDistanceUnit(LidarDistanceUnit::M);
  lidar.setReadByteCallback(readByte, &usbFd);
  lidar.begin();

  while (true) {
    lidar.readData();
  }

  ::close(usbFd);
  return 0;
}
