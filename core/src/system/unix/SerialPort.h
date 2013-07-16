/***************************************************************************
 *   Copyright (C) 2008 by Johannes Meyer   *
 *   meyer@fsr.tu-darmstadt.de   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef SYSTEM_UNIX_SERIALPORT_H
#define SYSTEM_UNIX_SERIALPORT_H

#include <system/BaseSerialPort.h>

namespace System {
namespace Unix {

class UXVCOS_API SerialPort : public BaseSerialPort {
public:
  SerialPort(std::string device = std::string(), unsigned long baudrate = 115200) :
    device(device),
    baudrate(baudrate),
    cs(8), stopb(1), parenb('n'),
    rtscts(false),
    fd(-1)
  { }

  virtual ~SerialPort() {
    close();
  }

  virtual bool setDevice(std::string device) {
    this->device = device;
    return true;
  }

  virtual bool setBaudrate(unsigned long b) {
    baudrate = b;
    return true;
  }

  virtual bool open();
  virtual void close();
  virtual bool blocking(bool);

  virtual bool isOpen() const {
    return (fd >= 0);
  }

  virtual int send(const void *source, size_t size);
  virtual int receive(void *source, size_t size);
  virtual int bytesAvailable() const;

  void flush();
  virtual bool wait(unsigned long msecs);

  virtual bool setRequestToSend(bool enable) const;
  virtual bool getClearToSend() const;
  virtual bool setDataTerminalReady(bool enable) const;
  virtual bool getDataSetReady() const;
  virtual bool getCarrierDetect() const;
  virtual bool getRingIndicator() const;

private:
  std::string device;
  unsigned long baudrate;
  unsigned char cs;
  unsigned char stopb;
  unsigned char parenb;
  bool rtscts;
  int fd;
};

} // namespace Unix
} // namespace System

#endif // SYSTEM_UNIX_SERIALPORT_H
