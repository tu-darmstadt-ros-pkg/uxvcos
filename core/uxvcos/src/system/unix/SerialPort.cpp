#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <string.h>
#include <errno.h>

#include "system/SerialPort.h"

namespace System {
namespace Unix {

bool SerialPort::open()
{
  struct termios tio;
  //struct termios new_termios;
  speed_t speed = B115200;
  int ret;

  if (fd != -1) return false;

  if ((fd = ::open(device.c_str(), O_RDWR)) < 0)
  {
    fd = -1;
    return false;
    // throw SerialPort::Exception("Error opening device " + device);
  }

  memset(&tio, 0, sizeof(tio));
  tio.c_cflag |= CLOCAL | CREAD;
  ret = -1;
  switch(baudrate) {
    case 50:      speed = B50; break;
    case 75:      speed = B75; break;
    case 110:     speed = B110; break;
    case 134:     speed = B134; break;
    case 150:     speed = B150; break;
    case 200:     speed = B200; break;
    case 300:     speed = B300; break;
    case 600:     speed = B600; break;
    case 1200:    speed = B1200; break;
    case 1800:    speed = B1800; break;
    case 2400:    speed = B2400; break;
    case 4800:    speed = B4800; break;
    case 9600:    speed = B9600; break;
    case 19200:   speed = B19200; break;
    case 38400:   speed = B38400; break;
    case 57600:   speed = B57600; break;
    case 115200:  speed = B115200; break;
    case 230400:  speed = B230400; break;
    case 460000:  speed = B460800; break;
    case 500000:  speed = B500000; break;
    case 576000:  speed = B576000; break;
    case 921600:  speed = B921600; break;
    case 1000000: speed = B1000000; break;
    case 1152000: speed = B1152000; break;
    case 1500000: speed = B1500000; break;
    case 2000000: speed = B2000000; break;
    case 2500000: speed = B2500000; break;
    case 3000000: speed = B3000000; break;
    case 3500000: speed = B3500000; break;
    case 4000000: speed = B4000000; break;
  }
  ret = cfsetospeed(&tio, speed);
  if (ret != 0) {
    close();
    throw SerialPort::Exception("Invalid baud rate");
  }
  cfsetispeed(&tio, speed);  // same as output baud rate

  switch(cs) {
    case 5: tio.c_cflag |= CS5; break;
    case 6: tio.c_cflag |= CS6; break;
    case 7: tio.c_cflag |= CS7; break;
    case 8: tio.c_cflag |= CS8; break;
    default:
      close();
      throw SerialPort::Exception("Invalid character size");
  }

  switch(stopb) {
    case 1: tio.c_cflag &= ~CSTOPB; break;
    case 2: tio.c_cflag |= CSTOPB;  break;
    default:
      close();
      throw SerialPort::Exception("Invalid number of stop bits");
  }

  switch(parenb) {
    case 'n': case 'N':
      tio.c_cflag &= ~PARENB;
      break;
    case 'e': case 'E':
      tio.c_cflag |= PARENB;
      tio.c_cflag &= ~PARODD;
      break;
    case 'o': case 'O':
      tio.c_cflag |= PARENB;
      tio.c_cflag |= PARODD;
      break;
    default:
      close();
      throw SerialPort::Exception("Invalid parity");
  }

  if (rtscts) tio.c_cflag |= CRTSCTS; else tio.c_cflag &= ~CRTSCTS;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &tio );

  setRequestToSend(false);
  setDataTerminalReady(false);

  /* compare */
  /*
  tcgetattr( fd, &new_termios );
  if ( memcmp( &tio, &new_termios, sizeof( tio )) != 0 ) {
    fprintf(stderr, "Could not set baud rate and control mode for the serial device\n");
    close(fd);
    return -1;
  } */
  return true;
}

/* Set a file descriptor to non-blocking */
bool SerialPort::blocking(bool on)
{
  int flags = fcntl(fd, F_GETFL);
  if (flags < 0) return false;

  if (on)
    flags &= ~O_NONBLOCK;
  else
    flags |= O_NONBLOCK;

  return (fcntl(fd, F_SETFL, flags) != -1);
}

int SerialPort::bytesAvailable() const {
  int n = 0;

  if (ioctl(fd, FIONREAD, &n) != -1) return n;
  return 0;
}

void SerialPort::close() {
  if (fd < 0) return;
  ::close(fd);
  fd = -1;
}

int SerialPort::send(const void *source, size_t size) {
  return ::write(fd, source, size);
}

int SerialPort::receive(void *destination, size_t size) {
  return ::read(fd, destination, size);
}

bool SerialPort::setRequestToSend(bool enable) const
{
  int status;
  ::ioctl(fd, TIOCMGET, &status);
  if (enable) {
    status |= TIOCM_RTS;
  } else {
    status &= ~TIOCM_RTS;
  }
  ::ioctl(fd, TIOCMSET, &status);

  return true;
}

bool SerialPort::getClearToSend() const
{
  int status;
  ioctl(fd, TIOCMGET, &status);
  return (status & TIOCM_CTS) == TIOCM_CTS;
}

bool SerialPort::setDataTerminalReady(bool enable) const
{
  int status;
  ::ioctl(fd, TIOCMGET, &status);
  if (enable) {
    status |= TIOCM_DTR;
  } else {
    status &= ~TIOCM_DTR;
  }
  ::ioctl(fd, TIOCMSET, &status);

  return true;
}

bool SerialPort::getDataSetReady() const
{
  int status;
  ioctl(fd, TIOCMGET, &status);
  return (status & TIOCM_DSR) == TIOCM_DSR;
}

bool SerialPort::getCarrierDetect() const
{
  int status;
  ioctl(fd, TIOCMGET, &status);
  return (status & TIOCM_CD) == TIOCM_CD;
}

bool SerialPort::getRingIndicator() const
{
  int status;
  ioctl(fd, TIOCMGET, &status);
  return (status & TIOCM_RNG) == TIOCM_RNG;
}

void SerialPort::flush()
{
  BaseSerialPort::flush();

  int ret;

  ret = ioctl(fd, TCFLSH, TCIOFLUSH);

  // read until buffer is empty in case that ioctl did not work (e.g. real-time pipe)
  if (ret == -1 && errno == EINVAL) {
    char buffer[256];
    long flags;

    flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    while(::read(fd, buffer, sizeof(buffer)) > 0) {} ;
    fcntl(fd, F_SETFL, flags);
  }
}

bool SerialPort::wait(unsigned long msecs) {
  fd_set rfds;
  struct timeval tv;
  int ret;

  if (fd < 0) return false;

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  tv.tv_sec = msecs / 1000;
  tv.tv_usec = msecs * 1000;

  ret = ::select(fd + 1, &rfds, NULL, NULL, &tv);
  /* Don't rely on the value of tv now! */

  return (ret > 0);
}

} // namespace Unix
} // namespace System
