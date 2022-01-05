/*
   I2S_ArduinoBuffer can hold data for reading or writing exclusively, never both at the same time.
   This Class implements basic Arduino functionalities over a plain byte buffer.
*/

#pragma once
#include <stddef.h>
#include <stdint.h>

#define DEFAULT_I2S_BUFFER_SIZE 512

class I2S_ArduinoBuffer
{
  public:
    I2S_ArduinoBuffer(size_t bufSize = DEFAULT_I2S_BUFFER_SIZE);
    virtual ~I2S_ArduinoBuffer();
    void release();

    void setSize(size_t size);
    size_t write(const void *buffer, size_t size);

    
    inline size_t getSize() {
      return _buffer_size;
    }

    inline void flush() {
      _buffer_pos = 0;
    }

    // for reading -- how many bytes are left from _buffer_pos up to the end of the buffer
    //             - start reading from prositon zero after the whole buffer was previously filled with data from DMA
    inline size_t available() {
      return dataLeft();
    }

    // for writing -- how many bytes are left from _buffer_pos up to the end of the buffer
    //             - star writing from position zero up to the end of the buffer - the whole buffer will be sent by DMA
    inline size_t availableForWrite() {
      return dataLeft();
    }

    inline size_t peek(void *buffer, size_t size) {
      return readBuffer(buffer, size, false);
    }

    inline size_t read(void *buffer, size_t size) {
      return readBuffer(buffer, size, true);
    }

    inline void* buffer() {
      return _buffer;
    }

  private:
    size_t _buffer_size;
    uint8_t* _buffer;
    volatile size_t _buffer_pos;
    size_t readBuffer(void *buffer, size_t size, bool movePointer);
    size_t dataLeft();
};
