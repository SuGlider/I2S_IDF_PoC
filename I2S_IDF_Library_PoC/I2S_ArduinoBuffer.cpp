/*
   I2S_ArduinoBuffer can hold data for reading or writing exclusively, never both at the same time.
   This Class implements basic Arduino functionalities over a plain byte buffer.
*/

#include <Arduino.h>
#include "I2S_ArduinoBuffer.h"

I2S_ArduinoBuffer::I2S_ArduinoBuffer(size_t bufSize) :
  _buffer_size(0),
  _buffer(NULL),
  _buffer_pos(0)
{
  setSize(bufSize);
}

I2S_ArduinoBuffer::~I2S_ArduinoBuffer()
{
  // nothing here so far...
}

void I2S_ArduinoBuffer::release()
{
  // LOCK HERE
  if (_buffer) {
    free(_buffer);
    _buffer = NULL;
    _buffer_pos = 0;
    //_buffer_size = 0;  // should not be reset for use in I2S 
  }
  // UNLOCK HERE
}

void I2S_ArduinoBuffer::setSize(size_t size)
{
  // LOCK HERE
  if (_buffer_size != size || !_buffer) {
    _buffer = (uint8_t*)realloc(_buffer, size);
    _buffer_size = size;
    if (!_buffer) {
      log_e(" -- Failure: lack of memory");
      _buffer_size = 0;
    }
  }
  // UNLOCK HERE
}

size_t I2S_ArduinoBuffer::dataLeft()
{
  // LOCK HERE
  size_t ret = _buffer_size - _buffer_pos;
  // UNLOCK HERE
  return ret;
}

size_t I2S_ArduinoBuffer::write(const void *buffer, size_t size)
{
  if (!_buffer || !buffer || !size) return 0;
  
  // LOCK HERE
  size_t space = availableForWrite();

  if (size > space) {
    size = space;
  }

  if (size > 0) {
    memcpy(&_buffer[_buffer_pos], buffer, size);
    _buffer_pos += size;
  }

  // UNLOCK HERE
  return size;
}

size_t I2S_ArduinoBuffer::readBuffer(void *buffer, size_t size, bool movePointer)
{
  if (!_buffer || !buffer || !size) return 0;

  // LOCK HERE
  size_t space = available();

  if (size > space) {
    size = space;
  }

  if (size > 0) {
    memcpy(buffer, &_buffer[_buffer_pos], size);
    if (movePointer) {
      _buffer_pos += size;
    }
  }

  // UNLOCK HERE
  return size;
}
