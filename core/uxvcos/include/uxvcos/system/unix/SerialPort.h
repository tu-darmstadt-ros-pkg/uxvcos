//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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

#include <uxvcos/system/BaseSerialPort.h>

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
