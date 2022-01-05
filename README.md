# I2S_IDF_PoC

This is a set of ESP32 Arduino sketches that test I2S interface using IDF.
This has been created in order to help me understand how IDF works with I2S and how Arduino could use it.

Most of them are designed for using ES8388 Codec, but it can be easily changed to work with any I2S amplifier chip.

## Current Status:

   - TBD: To Add Multi-Task Locking
   - TBD: To Check and test if both, I2S_0 and I2S_1, can work in together
   - TBD: To Extend ES8388 clsss to work with input/reading, and with other I2S parameters than Stereo/16bits
   - TBD: To Test I2S.read(), I2S.onReceive(), I2S.peek(), I2S.setPins() [+individual Pins], I2S.available() and I2S.flush()
#
   - Done: checking is driver is already installed, testing pointer for NULL or other bad states
   - Done: Testing: I2S.setBufferSize(), I2S.end(), I2S.begin(), I2S.write(), I2S.onTransmit(), I2S.availableForWrite() and I2S.getBufferSize()
   - Done: checking if the I2S Event task was created correctly
