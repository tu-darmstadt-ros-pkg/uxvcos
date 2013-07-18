#include "SerialPort.h"

#include <atlbase.h>
#include <atlconv.h>

namespace System {
namespace Win32 {

bool SerialPort::open() {
  if (isOpen()) return false;
  if (device.empty()) return false;

  std::string deviceName = "\\\\.\\" + device;
  USES_CONVERSION;
  LPCTSTR name = A2CT((LPCSTR) deviceName.c_str());
  handle = CreateFile(name, GENERIC_READ | GENERIC_WRITE, 0, NULL,
      OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

  if (handle == INVALID_HANDLE_VALUE) {
    // LOGPATH_ERROR("platform.SerialPort", "could not open serial port '%s'", deviceName.c_str());
    return false;
  }

  SetupComm(handle, 128, 128);

  DCB dcb;
  dcb.DCBlength = sizeof(dcb);
  GetCommState(handle, &dcb);
  dcb.BaudRate = baudrate;
  dcb.ByteSize = 8;
  dcb.StopBits = ONESTOPBIT;
  dcb.Parity = NOPARITY;
  dcb.fParity = FALSE;
  dcb.fDsrSensitivity = FALSE;
  dcb.fInX = FALSE;
  dcb.fOutX = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE; // DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE; // RTS_CONTROL_ENABLE;

  SetCommState(handle, &dcb);

  blocking(false);

  return true;
}

/* Set a file descriptor to non-blocking */
bool SerialPort::blocking(bool on) {
  if (!isOpen()) {
    return false;
  }

  COMMTIMEOUTS cto;
  GetCommTimeouts(handle, &cto);
  cto.ReadIntervalTimeout = (on ? 2000 : 1);
  cto.ReadTotalTimeoutConstant = (on ? 2000 : 0);
  cto.ReadTotalTimeoutMultiplier = (on ? 2000 : 0);
  cto.WriteTotalTimeoutConstant = 30;
  cto.WriteTotalTimeoutMultiplier = 2;
  SetCommTimeouts(handle, &cto);

  return true;
}

int SerialPort::bytesAvailable() const {
  return 0;
}

void SerialPort::close() {
  if (isOpen()) {
    CloseHandle(handle);
    handle = INVALID_HANDLE_VALUE;
  }
}

int SerialPort::send(const void *source, size_t size) {
  if (!isOpen()) {
    return -1;
  }

  // event for overlapping sending
  OVERLAPPED osWrite = { 0 };
  osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  if (osWrite.hEvent == NULL) {
    // LOGPATH_ERROR("platform.SerialPort", "error creating overlapped event handle for sender");
    return -1;
  }

  DWORD written = 0;
  BOOL res = WriteFile(handle, source, size, &written, &osWrite);
  // Check result, if true sending ist completed immediately, false = pending or error
  if (res == FALSE) {
    if (GetLastError() != ERROR_IO_PENDING) {
      // LOGPATH_ERROR("platform.SerialPort", "error could not send packet");
      CloseHandle(osWrite.hEvent);
      return -1;
    }
    // wait for command to be completed
    DWORD dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
    if (dwRes == WAIT_OBJECT_0) {
      if (!GetOverlappedResult(handle, &osWrite, &written, FALSE)) {
        // Error in communications;
        // LOGPATH_ERROR("platform.SerialPort", "error could not send packet (Error in communications)");
      }
    } else {
      // LOGPATH_ERROR("platform.SerialPort", "error could not send packet (timeout)");
      CloseHandle(osWrite.hEvent);
      return -1;
    }
  }

  CloseHandle(osWrite.hEvent);
  return written;
}

int SerialPort::receive(void *destination, size_t size) {
  if (!isOpen()) {
    return -1;
  }

  // event for overlapping reading
  OVERLAPPED osReader = { 0 };
  osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  if (osReader.hEvent == NULL) {
    // LOGPATH_ERROR("platform.SerialPort", "error creating overlapped event handle for receiveer");
    return -1;
  }

  DWORD receivedBytes = 0;
  BOOL res = ReadFile(handle, (char*) destination, size, &receivedBytes,
      &osReader);
  // Check result, if true reading ist completed immediately, false = pending or error
  if (res == FALSE) {
    if (GetLastError() != ERROR_IO_PENDING) {
      // LOGPATH_ERROR("platform.SerialPort", "error could not receive answer");
      CloseHandle(osReader.hEvent);
      return -1;
    }
    // wait for command to be completed
    DWORD dwRes = WaitForSingleObject(osReader.hEvent, INFINITE);
    if (dwRes == WAIT_OBJECT_0) {
      dwRes = GetOverlappedResult(handle, &osReader, &receivedBytes, FALSE);
      if (!dwRes) {
        // Error in communications;
        // LOGPATH_ERROR("platform.SerialPort", "error could not receive answer (Error in communications)");
      }
    } else {
      // LOGPATH_ERROR("platform.SerialPort", "error could not receive answer (timeout)");
      CloseHandle(osReader.hEvent);
      return -1;
    }
  }

  CloseHandle(osReader.hEvent);
  return receivedBytes;
}

bool SerialPort::setRequestToSend(bool enable) const {
  return false;
}

bool SerialPort::getClearToSend() const
{
  return (getCommModemStatus() & MS_CTS_ON) != 0;
}

bool SerialPort::setDataTerminalReady(bool enable) const {
  return false;
}

bool SerialPort::getDataSetReady() const
{
  return (getCommModemStatus() & MS_DSR_ON) != 0;
}

bool SerialPort::getCarrierDetect() const
{
  return (getCommModemStatus() & MS_RLSD_ON) != 0;
}

bool SerialPort::getRingIndicator() const
{
  return (getCommModemStatus() & MS_RING_ON) != 0;
}

DWORD SerialPort::getCommModemStatus() const
{
  if (!isOpen()) {
    return 0;
  }

  DWORD status;
  if (!GetCommModemStatus(handle, &status)) {
    return 0;
  }

  return status;
}

void SerialPort::flush() {
  BaseSerialPort::flush();
  purgeInput();
  purgeOutput();
}

bool SerialPort::wait(unsigned long msecs) {
  if (!isOpen()) {
    return false;
  }

  /*
  DWORD      dwRes, dwOvRes;
  DWORD      dwCommEvent;
  OVERLAPPED osStatus = {0};

  if (!SetCommMask(handle, EV_RXCHAR))
    // error setting communications mask; abort
    return false;

  osStatus.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  if (osStatus.hEvent == NULL)
     // error creating event; abort
     return false;

  if (WaitCommEvent(handle, &dwCommEvent, &osStatus)) {
     // WaitCommEvent returned immediately.
     // Deal with status event as appropriate.
     CloseHandle(osStatus.hEvent);
     return true;
  }

  if (GetLastError() != ERROR_IO_PENDING) {
     // error in WaitCommEvent; abort
     CloseHandle(osStatus.hEvent);
     return false;
  }

  // Wait a little while for an event to occur.
  dwRes = WaitForSingleObject(osStatus.hEvent, msecs);
  if (dwRes == WAIT_OBJECT_0) {
     if (GetOverlappedResult(handle, &osStatus, &dwOvRes, FALSE)) {
        // Status event is stored in the event flag
        // specified in the original WaitCommEvent call.
        // Deal with the status event as appropriate.
        CloseHandle(osStatus.hEvent);
        return true;
     }

     // An error occurred in the overlapped operation;
     // call GetLastError to find out what it was
     // and abort if it is fatal.
  }

  if (dwRes == WAIT_TIMEOUT) {
      // Operation isn't complete yet
  }

  CloseHandle(osStatus.hEvent);
  return false;
  */

  COMMTIMEOUTS cto;
  GetCommTimeouts(handle, &cto);
  cto.ReadTotalTimeoutConstant = msecs;
  cto.ReadTotalTimeoutMultiplier = 0;
  SetCommTimeouts(handle, &cto);

  return true;
}

/**
 * Purges the input buffer.
 */
void SerialPort::purgeInput() {
  PurgeComm(handle, PURGE_RXCLEAR);
  needPurgeInput = false;
}

/**
 * Purges the output buffer.
 */
void SerialPort::purgeOutput() {
  PurgeComm(handle, PURGE_TXCLEAR);
  needPurgeOutput = false;
}

} // namespace Win32
} // namespace System
