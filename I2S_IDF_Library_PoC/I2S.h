/*
   Very Basic and not flexible yet Arduino I2S Class implementation
   Just a PoC for ESP IDF 4.4 I2S Driver with Events and Callbacks
   Based on:
   https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/I2S
   https://github.com/PilnyTomas/arduino-esp32/tree/arduino_i2s
*/

#pragma once

#include <Arduino.h>
#include "I2S_ArduinoBuffer.h"

namespace esp_i2s {
#include "driver/i2s.h" // ESP specific i2s driver
}

//Espressif ESP32-LyraT V4.3 board
//https://docs.espressif.com/projects/esp-adf/en/latest/design-guide/dev-boards/get-started-esp32-lyrat.html
#define USE_LYRA_T              (1)

// Default Pin Setup - based on Espressif ESP32-LyraT V4.3 board
#define I2S_BCK_IO                  (GPIO_NUM_5)
#define I2S_WS_IO                   (GPIO_NUM_25)
#define I2S_DO_IO                   (GPIO_NUM_26)
#define I2S_DI_IO                   (GPIO_NUM_35)
#define IS2_MCLK_PIN                (GPIO_NUM_0)

#define I2S_NUM                 (0)     // Default i2s port number 

// Tells Arduino Sound that it has SetBufferSize() functionality
#define I2S_HAS_SET_BUFFER_SIZE 1

typedef enum {
  I2S_PHILIPS_MODE,
  I2S_RIGHT_JUSTIFIED_MODE,
  I2S_LEFT_JUSTIFIED_MODE
} i2s_mode_t;

/*
// Little Endian --- do we really need that?
union i2s_sample_t {
  uint8_t b8;
  int16_t b16;
  int32_t b32;
};
*/

class I2SClass : public Stream
{
  public:
    I2SClass(uint8_t deviceIndex, uint8_t sdinPin, uint8_t sdoutPin, uint8_t sckPin, uint8_t fsPin);

    // I2S Master mode - Full Duplex
    int begin(uint8_t mode = I2S_PHILIPS_MODE, uint32_t sampleRate = 44100, uint8_t bitsPerSample = 16);
    void end();

    // from Stream
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    // from Print
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);

    virtual int availableForWrite();

    int read(void* buffer, size_t size);

    size_t write(int32_t);
    size_t write(const void *buffer, size_t size);

    void onTransmit(void(*)(void));
    void onReceive(void(*)(void));

    void setBufferSize(size_t bufferSize);
    size_t getBufferSize();
    
    void setPins(uint8_t BCK = I2S_BCK_IO, uint8_t WS = I2S_WS_IO, uint8_t DOUT = I2S_DO_IO, int8_t DIN = I2S_DI_IO);
    void setSckPin(int sckPin);
    void setWsPin(int WsPin);
    void setDataInPin(int inSdPin);
    void setDataOutPin(int outSdPin);


  private:
    uint8_t _deviceIndex;
    uint8_t _sdOutPin;
    uint8_t _sdInPin;
    uint8_t _sckPin;
    uint8_t _wsPin;
    uint16_t _i2s_dma_buffer_size;
    uint8_t _mode;
    uint32_t _sampleRate;
    uint8_t _bitsPerSample;

    void (*_onTransmit)(void);
    void (*_onReceive)(void);

    QueueHandle_t _i2sEventQueue = NULL;
    TaskHandle_t _i2sCbTaskHandle = NULL;
    I2S_ArduinoBuffer _i2sTxBuffer, _i2sRxBuffer; // default constructor

    static void i2s_task(void *args);
    void i2s_event_cb();
};

extern I2SClass I2S;
