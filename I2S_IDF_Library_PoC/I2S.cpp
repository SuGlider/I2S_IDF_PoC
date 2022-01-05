/*
   Very Basic Arduino I2S Class implementation
   Just a PoC for ESP IDF 4.4 I2S Driver with Events and Callbacks
   Based on:
   https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/I2S
   https://github.com/PilnyTomas/arduino-esp32/tree/arduino_i2s

   Current Status:
   ===============
   TBD: Multi-Task Locking
   TBD: Check and test if both, I2S_0 and I2S_1, can work in together
   TBD: Extend ES8388 clsss to work with input/reading, and with other I2S parameters htan Stereo/16bits
   TBD: Test I2S.read(), I2S.onReceive(), I2S.peek(), I2S.setPins() [+individual Pins], I2S.available() and I2S.flush()

   Done: check is driver is already installed, test pointer for NULL or other bad states
   Done: Testing: I2S.setBufferSize(), I2S.end(), I2S.begin(), I2S.write(), I2S.onTransmit(), I2S.availableForWrite() and I2S.getBufferSize()
   Done: check if the I2S Event task was created correctly
*/

#include "I2S.h"

#define I2S_EVENT_QUEUE_LENGTH  (16)    // i2s event queue used to drive I2S_ArduinoBuffer data into IDF I2S data queue


#if USE_LYRA_T
/*

   ES8388 I2C setup and control - Lyra-T Board

*/

#include "ES8388.h"

#define IIC_CLK                     23
#define IIC_DATA                    18
#define GPIO_PA_EN                  GPIO_NUM_21
#define GPIO_SEL_PA_EN              GPIO_SEL_21

static ES8388 es;
static uint8_t volume = 90;

static void es8388_setup_stereo_16bits()
{
  while (not es.begin(IIC_DATA, IIC_CLK))
  {
    log_e("ES8388 initialization Failed!");
    return;
  }

  es.volume(ES8388::ES_MAIN, volume);
  es.volume(ES8388::ES_OUT1, volume);
  es.volume(ES8388::ES_OUT2, volume);
  es.mute(ES8388::ES_OUT1, false);
  es.mute(ES8388::ES_OUT2, false);
  es.mute(ES8388::ES_MAIN, false);
  // es.SetVolumeHeadphone(volume);
  //  ac.DumpRegisters();

  // Enable amplifier
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  log_d("ES8388 codec should be working fine now!\n");
}
#endif


I2SClass::I2SClass(uint8_t deviceIndex, uint8_t sdinPin, uint8_t sdoutPin, uint8_t sckPin, uint8_t wsPin) :
  _deviceIndex(deviceIndex),
  _sdInPin(sdinPin),
  _sdOutPin(sdoutPin),
  _sckPin(sckPin),
  _wsPin(wsPin),

  _onTransmit(NULL),
  _onReceive(NULL),
  _i2sCbTaskHandle(NULL),
  _i2sEventQueue(NULL),

  _mode(0),
  _bitsPerSample(16),
  _sampleRate(44100)
{
}


int I2SClass::begin(uint8_t mode, uint32_t sampleRate, uint8_t bitsPerSample) {

  if (_i2sCbTaskHandle) {  // means that the driver is installed
    // end it before restarting the driver
    end();
  }

  esp_i2s::i2s_pin_config_t pin_config = {
    .bck_io_num = _sckPin,
    .ws_io_num = _wsPin,
    .data_out_num = _sdOutPin,
    .data_in_num = _sdInPin
  };

  uint16_t num_channels = 1; //  mono - for R/L Justtified Modes
  esp_i2s::i2s_channel_fmt_t ch_format;
  switch (mode) {
    case I2S_PHILIPS_MODE:
      ch_format = esp_i2s::I2S_CHANNEL_FMT_RIGHT_LEFT;
      num_channels = 2;
      break;
    case I2S_RIGHT_JUSTIFIED_MODE:
      ch_format = esp_i2s::I2S_CHANNEL_FMT_ONLY_RIGHT;
      break;
    case I2S_LEFT_JUSTIFIED_MODE:
      ch_format = esp_i2s::I2S_CHANNEL_FMT_ONLY_LEFT;
      break;
    default:
      log_e("Invalid I2S format.");
      return 0;
  }
  _mode = mode;

  esp_i2s::i2s_bits_per_sample_t bps;
  uint16_t bytes_allocated_per_sample = 2;    // for 8 and 16 bits as per IDF driver code
  switch (bitsPerSample) {
    case 8:
      bps = esp_i2s::I2S_BITS_PER_SAMPLE_8BIT;
      break;
    case 16:
      bps = esp_i2s::I2S_BITS_PER_SAMPLE_16BIT;
      break;
    case 32:
      bps = esp_i2s::I2S_BITS_PER_SAMPLE_32BIT;
      bytes_allocated_per_sample = 4;
      break;

    default:
      log_e("Invalid I2S bits per sample.");
      return 0;
  }
  _bitsPerSample = bitsPerSample;
  _sampleRate = sampleRate;

  uint16_t dma_bytes_per_sample = bytes_allocated_per_sample * num_channels;
  _i2s_dma_buffer_size = getBufferSize() / dma_bytes_per_sample;  // shall be a perfect division!

  // IDF I2S driver only runs if dma_buffer_len is between 8 and 1024
  if (_i2s_dma_buffer_size < 8) {
    _i2s_dma_buffer_size = 8;
    // fix buffers
    setBufferSize(_i2s_dma_buffer_size * dma_bytes_per_sample);
    log_d(" -- Using minimum Buffer Size of %d bytes", getBufferSize());
  }

  if (_i2s_dma_buffer_size > 1024) {
    _i2s_dma_buffer_size = 1024;
    // fix buffers
    setBufferSize(_i2s_dma_buffer_size * dma_bytes_per_sample);
    log_d(" -- Using maximum Buffer Size of %d bytes", getBufferSize());
  }

  // check if buffer has a valid pointer (may be released after an end())
  if (!_i2sTxBuffer.buffer() || !_i2sRxBuffer.buffer()) {
    setBufferSize(getBufferSize());  // _buffer_size is never reset, thus can be used as reference of last allocation for a I2S restart
  }

  if (_i2s_dma_buffer_size * dma_bytes_per_sample != _i2sTxBuffer.getSize()) {
    log_w("For a better performance, Buffer Size shall be disivible by %d", dma_bytes_per_sample);
  }

  esp_i2s::i2s_config_t i2s_config = {
    .mode = (esp_i2s::i2s_mode_t)(esp_i2s::I2S_MODE_MASTER | esp_i2s::I2S_MODE_TX | esp_i2s::I2S_MODE_RX),
    .sample_rate = sampleRate,
    .bits_per_sample = bps,
    .channel_format = ch_format,
    .communication_format = esp_i2s::I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority

    .dma_buf_count = 2,
    .dma_buf_len = _i2s_dma_buffer_size,

    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0,
  };

  esp_i2s::i2s_driver_install((esp_i2s::i2s_port_t)_deviceIndex, &i2s_config, I2S_EVENT_QUEUE_LENGTH, &_i2sEventQueue);
  esp_i2s::i2s_set_pin((esp_i2s::i2s_port_t)_deviceIndex, &pin_config);

  xTaskCreate(i2s_task, "i2s_event_CB", 4096, this, 20, &_i2sCbTaskHandle);
  if (!_i2sCbTaskHandle) {
    log_e(" -- I2S Event Task not Created!");
    return 0;
  }

  log_d("I2S Begin SampleRate = %d BPS = %d DMA SIZE = %d BYTE_BUFFER_LEN = %d", sampleRate, bps, _i2s_dma_buffer_size, _i2sTxBuffer.getSize());

#if USE_LYRA_T
  // in case LyraT board with ES8388 Codec chip is used, just get it ready
  log_d("ES8388 - Setup : output only :: stereo and 16 bits sample.");
  es8388_setup_stereo_16bits();
#endif

  return 1;
}

void  I2SClass::end()
{
  // LOCK
  if (_i2sCbTaskHandle) {  // means that the driver is installed and running
    vTaskDelete(_i2sCbTaskHandle);
    _i2sCbTaskHandle = NULL;
    esp_i2s::i2s_driver_uninstall((esp_i2s::i2s_port_t)_deviceIndex);
    _i2sTxBuffer.release();
    _i2sRxBuffer.release();
  }
  // UNLOCK
  return;
}


// from Stream
int  I2SClass::available()
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sRxBuffer.buffer() || !_i2sCbTaskHandle) return 0;
  
  return _i2sRxBuffer.available();
}

int  I2SClass::read()
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sRxBuffer.buffer() || !_i2sCbTaskHandle) return -1;  // Arduino way to say "no data available"
  
  uint32_t sample32;
  if (_i2sRxBuffer.read(&sample32, _bitsPerSample / 8) == 0) {
    return -1;
  }
  return sample32;

  /*
    // Arduino way of doing it...
    i2s_sample_t sample;

    sample.b32 = 0;

    read(&sample, _bitsPerSample / 8);

    if (_bitsPerSample == 32) {
      return sample.b32;
    } else if (_bitsPerSample == 16) {
      return sample.b16;
    } else if (_bitsPerSample == 8) {
      return sample.b8;
    } else {
      return 0;
    }
  */
}

int  I2SClass::peek()
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sRxBuffer.buffer() || !_i2sCbTaskHandle) return -1;  // Arduino way to say "no data available"
  
  uint32_t sample32;
  if (_i2sRxBuffer.peek(&sample32, _bitsPerSample / 8) == 0){
    return -1;
  }
  return sample32;

  /*
    // Arduino way of doing it...
    i2s_sample_t sample;

    sample.b32 = 0;

    _i2sRxBuffer.peek(&sample, _bitsPerSample / 8);

    if (_bitsPerSample == 32) {
      return sample.b32;
    } else if (_bitsPerSample == 16) {
      return sample.b16;
    } else if (_bitsPerSample == 8) {
      return sample.b8;
    } else {
      return 0;
    }
  */
}

void  I2SClass::flush()
{
   // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sRxBuffer.buffer() || !_i2sTxBuffer.buffer() || !_i2sCbTaskHandle) return;
  
 // TBD
  _i2sTxBuffer.flush();
  _i2sRxBuffer.flush();
}

// from Print
size_t  I2SClass::write(uint8_t sample)
{
  return write((int32_t) sample);;
}

size_t  I2SClass::write(const uint8_t *buffer, size_t size)
{
  return write((const void *) buffer, size);
}

int  I2SClass::availableForWrite()
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sTxBuffer.buffer() || !_i2sCbTaskHandle) return 0;
  
  return _i2sTxBuffer.availableForWrite();
}

int  I2SClass::read(void* buffer, size_t size)
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sRxBuffer.buffer() || !_i2sCbTaskHandle) return -1;  // Arduino way to say "no data available"
  
  return _i2sRxBuffer.read(buffer, size);
}

size_t  I2SClass::write(int32_t sample32)
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sTxBuffer.buffer() || !_i2sCbTaskHandle) return 0;
  
  // this function must block until it can write the sample.  
  size_t written_bytes;
  do {
    written_bytes = write((const void *)&sample32, _bitsPerSample / 8);
  } while (written_bytes == 0);
  return written_bytes;

  // another way to do that ...
  //  size_t to_be_written = _bitsPerSample / 8;
  //  do {
  //    written_bytes = write((const void *)&sample32, to_be_written);
  //    to_be_written -= written_bytes;
  //    if (to_be_written) sample32 = sample32 >> (8 * written_bytes);  // adjust sample if not all bits sent
  //  } while (to_be_written);
  //  return written_bytes;


  //////////////////////// a complicated way to do that...
  /*
    do {
      switch (_bitsPerSample) {
        case 32:
          {
            written_bytes = write((const void *)&sample32, sizeof(uint32_t));
            break;
          }
        case 16:
          {
            uint16_t s16 = (uint16_t) sample32;
            written_bytes = write((const void *)&s16, sizeof(uint16_t));
            break;
          }
        case 8:
          {
            uint8_t s8 = (uint8_t) sample32;
            written_bytes = write((const void *)&s8, sizeof(uint8_t));
            break;
          }
        default:
          return 0;
      }
    } while (written_bytes == 0);
    return written_bytes;
  */
}

size_t  I2SClass::write(const void *buffer, size_t size)
{
  // safe check: buffer pointer not NULL and I2S driver running
  if (!_i2sTxBuffer.buffer() || !_i2sCbTaskHandle) return 0;
  
  return _i2sTxBuffer.write(buffer, size);
}

void  I2SClass::setBufferSize(size_t bufferSize)
{
  if (_i2sCbTaskHandle) {  // means that the driver is installed and running
    log_w(" -- Resizing I2S buffers after I2S.begin() will reset the driver.");
    end();
    _i2sTxBuffer.setSize(bufferSize);
    _i2sRxBuffer.setSize(bufferSize);
    begin(_mode, _sampleRate, _bitsPerSample);
  } else {                // called before starting the driver, just resize the buffers
    _i2sTxBuffer.setSize(bufferSize);
    _i2sRxBuffer.setSize(bufferSize);
  }
  return;
}

size_t I2SClass::getBufferSize()
{
  return _i2sTxBuffer.getSize();
}


void I2SClass::onTransmit(void(*function)(void))
{
  _onTransmit = function;
}

void I2SClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void I2SClass::i2s_task(void *args)
{
  I2SClass *i2s = (I2SClass *) args;
  i2s->i2s_event_cb();
}

void I2SClass::i2s_event_cb()
{
  esp_i2s::i2s_event_t i2s_event;

  if (_i2sEventQueue == NULL) {
    log_e("I2S Event Queue is NULL.");
    goto err;
  }
  log_d("I2S Event Callback Started and Running.");
  for (;;) {
    if (xQueueReceive(_i2sEventQueue, &i2s_event, portMAX_DELAY) == pdTRUE) // portMAX_DELAY);
      if (i2s_event.type == esp_i2s::I2S_EVENT_TX_DONE) {
        if (!_i2sTxBuffer.availableForWrite()) {  // wait until the buffer is full
          size_t written = 0;
          // copy buffer to IDF I2S ring buffer
          esp_i2s::i2s_write((esp_i2s::i2s_port_t)_deviceIndex, _i2sTxBuffer.buffer(), _i2sTxBuffer.getSize(), &written, 0);   // no timeout
          // safety test:
          if (written != _i2sTxBuffer.getSize()) log_w("----- ooopsss ---- didn't send all I2S DMA Buffer data!");
          // flush buffer for a next turn
          _i2sTxBuffer.flush();
          // callback function emulation
          if (_onTransmit) {
            _onTransmit();
          }
        }
      } else if (i2s_event.type == esp_i2s::I2S_EVENT_RX_DONE) {
        if (!_i2sRxBuffer.available()) {  // wait until the buffer is full
          size_t read = 0;
          // copy buffer to IDF I2S ring buffer
          esp_i2s::i2s_read((esp_i2s::i2s_port_t)_deviceIndex, _i2sRxBuffer.buffer(), _i2sRxBuffer.getSize(), &read, 0);   // no timeout
          // safety test:
          if (read != _i2sRxBuffer.getSize()) log_w("----- ooopsss ---- didn't read all I2S DMA Buffer data!");
          // flush buffer for a next turn
          _i2sRxBuffer.flush();
          // callback function emulation
          if (_onReceive) {
            _onReceive();
          }
        }
      }
  }

err:
  vTaskDelete(NULL);
}

void I2SClass::setSckPin(int sckPin)
{
  _sckPin = sckPin;
  setPins(_sckPin, I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE);
}

void I2SClass::setWsPin(int wsPin)
{
  _wsPin = wsPin;
  setPins(I2S_PIN_NO_CHANGE, _wsPin, I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE);
}

void I2SClass::setDataOutPin(int outSdPin)
{
  _sdOutPin = outSdPin;
  setPins(I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE, _sdOutPin, I2S_PIN_NO_CHANGE);
}

void I2SClass::setDataInPin(int inSdPin)
{
  _sdInPin = inSdPin;
  setPins(I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE, _sdInPin);
}

void I2SClass::setPins(uint8_t SCK, uint8_t WS, uint8_t DOUT, int8_t DIN) {
  _sckPin = SCK;          // Serial Clock
  _wsPin = WS;            // Word Select
  _sdOutPin = DOUT;       // Serial Data Out
  _sdInPin = DIN;         // Serial Data In

  esp_i2s::i2s_pin_config_t i2sPins = {
    .bck_io_num = _sckPin,
    .ws_io_num =  _wsPin,
    .data_out_num = _sdOutPin,
    .data_in_num = _sdInPin
  };

  esp_i2s::i2s_set_pin((esp_i2s::i2s_port_t)_deviceIndex, &i2sPins);
}

I2SClass I2S(I2S_NUM, I2S_DI_IO, I2S_DO_IO, I2S_BCK_IO, I2S_WS_IO);
