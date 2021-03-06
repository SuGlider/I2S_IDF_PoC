// WIP/PoC for a simple and thin I2S Arduino Style Library and Sketch
// Local I2S, es8388 and I2S_ArduinoBuffer Libraries! 
// Also Designed to be able to work with LyraT Board
#include "I2S.h"

/*
 *  This I2S Driver has ES8388 Mode Turned ON! ---> please take a look on Line 33 of I2S.h to deactivate it, if necessary
 */

// Flash stored Music -- mono channel, int8_t sample, at 20,000 samples per second
// https://github.com/bitluni/DawnOfAV/blob/master/DawnOfAV/sfx/music.h
#include "music.h"

// PLAY_MUSIC Application is an example of onTransmit() usage
// SIMPLE_TONE Application is an example of a sware wave tone generated in Arduino loop()

// Pick an application:  Play a Music Sample or a Simple Tone
#define PLAY_MUSIC    0   // 1 means "Play Flash Stored Music Sample" -- 0 means "Play Simple Tone Frequency"
#if PLAY_MUSIC
#define SIMPLE_TONE   0
#else
#define SIMPLE_TONE   1
#define ON_TRASNMIT   1   // 1 for sending sample in OnTransmit() or 0 for sending samples in loop()
#define USE_SET_BUFFER_SIZE 1   // ONLY with ON_TRASNMIT=1 ==> 1 means to adjust via I2S.setBufferSize() -- 0 means adjust via SampleRate
#endif

#if ON_TRASNMIT
size_t sampleInBytes = 0;
int16_t *sampleBuffer = NULL;

void sendTone() {
    I2S.write(sampleBuffer, sampleInBytes);
}
#endif

#if PLAY_MUSIC
void sendMusicSamples() {
  // Sound is sent sample by sample in loop() or from here - check #define PLAY_MUSIC
  static uint32_t position = 0;  // tracks sample position in the music flash array 
  static size_t BUF_LEN = I2S.getBufferSize();   // number of bytes of I2S DMA buffer 

  // writes samples from the music translating mono 8bits samples to stereo 16bits samples
  // +=4 means: 2 bytes per sample (16 bits) in 2 channels (stereo mode) 
  // = 4 (2x2) bytes per sample from a single byte of musicSamples[] - which is 8bits/Mono description
  for ( int i = 0; i < BUF_LEN; i+=4 ){   

    // volume (amplitude) compensation for taking 8bits sample to 16bits
    uint16_t sample = ((musicSamples[position++] + 128) << 6) + 0x1F;
 
    if (position == sizeof(musicSamples)) position = 0; // loop - play for ever

    // if not using the LyraT Board, it shall send only one sample!
    I2S.write(sample);   // 16 bits on Right channel
#if USE_LYRA_T            // See Line 33 in I2S.h to change it    
    I2S.write(sample);   // another 16 bits on both channels of the ES8388 (setup up is stereo)
#endif
  }
}
#endif

#if SIMPLE_TONE
// So far, the current LyraT configuration is Stereo with 16 bits - any Sample Rate
#define BITS_PER_SAMPLE   (16)       // amplitud level must be adjusted depending on #bits
#define AMPLITUDE         (500)               // works on 16 bits
#define I2S_MODE          I2S_PHILIPS_MODE    // I2S_PHILIPS_MODE, I2S_RIGHT_JUSTIFIED_MODE, I2S_LEFT_JUSTIFIED_MODE


#if ON_TRASNMIT
#if !USE_SET_BUFFER_SIZE
// Simple Tone parameters based on 512 bytes (4 bytes per streo sample) = 128 samples per Hz:
#define WAVE_FREQ_HZ      (440)      // any value 
#define SAMPLE_PER_CYCLE  (DEFAULT_I2S_BUFFER_SIZE / sizeof(int16_t) / 2)     // number of samples per Hz - considering 2 bytes per sample and 2 samples for stereo
#define SAMPLE_RATE       (WAVE_FREQ_HZ * SAMPLE_PER_CYCLE)    // exactly proportional to default 512 bytes in I2S buffer
#else
// ANOTHER way to solve it is to use I2S.SetBufferSize(SAMPLE_PER_CYCLE * 4); and keep SAMPLE_RATE whatever wanted 
#define SAMPLE_RATE       (44100)    // tested with 8000, 16000, 22050 and 44100
#define WAVE_FREQ_HZ      (440)      // any value 
#define SAMPLE_PER_CYCLE  (SAMPLE_RATE/WAVE_FREQ_HZ)     // number of samples per Hz
#endif
#else
// Simple Tone parameters:
#define SAMPLE_RATE       (44100)    // tested with 8000, 16000, 22050 and 44100
#define WAVE_FREQ_HZ      (440)      // any value 
#define SAMPLE_PER_CYCLE  (SAMPLE_RATE/WAVE_FREQ_HZ)     // number of samples per Hz
#endif
#else
// So far, the current LyraT configuration is Stereo with 16 bits - any Sample Rate

// Flash Stored parameters:
// ========================
// music sample is actually 20000 samples/sec in MONO channel
// But we use ES8388 in Stereo mode, thus we use double sample rate and send 2 copies of the same sample 
// Just a trick to make it work as stereo mode...
#if USE_LYRA_T            // See Line 33 in I2S.h to change it    
#define SAMPLE_RATE       (40000)    
#else
#define SAMPLE_RATE       (20000)    
#endif
// music will be sent as 16 bits - adjusted from 8bits/sample by software
#define BITS_PER_SAMPLE   (16)       

// music is Mono and we shall keep it this way in the I2S driver
#define I2S_MODE          I2S_RIGHT_JUSTIFIED_MODE    

#endif


//void app_main_arduino(void);

void setup() {
  Serial.begin(115200);

  //app_main_arduino();

#if SIMPLE_TONE
  I2S.begin(I2S_MODE, SAMPLE_RATE, BITS_PER_SAMPLE); // stereo, 44100 samples/sec, 16bits per sample

  Serial.printf("\nI2S Application: Square Wave at %dHz.\n", WAVE_FREQ_HZ);
  Serial.println("Using loop() to fill up the samples, sending one by one.\n-----------");
  Serial.printf("Square Wave - Writing Sample by Sample in loop() :: %d samples/sec | %d bits sample size | Stereo Mode\n", SAMPLE_RATE, BITS_PER_SAMPLE);
  Serial.printf("%d Samples per Hz\n-----------\n", SAMPLE_PER_CYCLE);
  
#else  // Play Flash Stored Music Sample
  I2S.begin(I2S_MODE, SAMPLE_RATE, BITS_PER_SAMPLE); // stereo, 44000 samples/sec, 16bits per sample

  // Begin :: Testing possible combinations.... 1 by 1, all combination of 2, all the 3
  //I2S.end();
  //I2S.setBufferSize(5000);
  //I2S.begin(I2S_MODE, SAMPLE_RATE, BITS_PER_SAMPLE); // stereo, 44000 samples/sec, 16bits per sample
  // End:: Testing possible combinations.... 
  
  I2S.onTransmit(sendMusicSamples);   // setting up the callback function that will actually play the music
  sendMusicSamples();                 // kick off! Send the first buffer to I2S and start it up!

  Serial.println("\n-----------\nI2S Application: Music Sample");
  Serial.println("Using onTransmit() to fill up the samples, sending block by block.\n-----------");

#endif
#if ON_TRASNMIT
  uint16_t sample = AMPLITUDE;
#if USE_SET_BUFFER_SIZE
  I2S.setBufferSize(SAMPLE_PER_CYCLE * 4);  
#endif
  sampleInBytes = I2S.availableForWrite();
  sampleBuffer = (int16_t *)malloc(sampleInBytes);
  size_t numOfSamples = sampleInBytes / sizeof(sample);
  
  Serial.printf("\n -- Total Bytes = %d, Sample# = %d, HalfWave = %d \n", sampleInBytes, numOfSamples, SAMPLE_PER_CYCLE / 2);
  for (int i = 0, c = 0; i < numOfSamples / 2; i++) { // 2 for stereo 
    if (c++ == SAMPLE_PER_CYCLE / 2) {
      sample = -sample;
      c = 0;
    }
    sampleBuffer[2*i] = sample;
    sampleBuffer[2*i + 1] = sample;
    //Serial.printf("Sample [%d] = %x\n", i, sample);
  }

  Serial.printf(" INIT Available For Write: %d\n", I2S.availableForWrite());
  Serial.printf(" INIT Available For Read: %d\n", I2S.available());



  I2S.onTransmit(sendTone);   // setting up the callback function that will actually play the music
  sendTone();                 // kick off! Send the first buffer to I2S and start it up!
#endif
}

void loop() {
#if SIMPLE_TONE
#if ON_TRASNMIT
  return;            // using onTransmit() to send the samples instead of here in loop()
#endif
  static int count = 0;
  static int sample = AMPLITUDE;

  if (count++ ==  SAMPLE_PER_CYCLE / 2) {  // reached half cycle (1/2 Hz)
    count = 0;
    sample = -sample;
  }

  // Writing a single sample is a blocking operation
  I2S.write(sample);   // Left Channel
  I2S.write(sample);   // Right Channel
#endif
}
