// Slightly modified Adalight protocol implementation that uses FastLED
// library (http://fastled.io) for driving WS2811/WS2812 led stripe
// Was tested only with Prismatik software from Lightpack project
#include "FastLED.h"

#define NUM_LEDS 114 // Max LED count
#define LED_PIN 6 // arduino output pin
#define GROUND_PIN 10
#define BRIGHTNESS 255 // maximum brightness
#define SERIALSPEED 115200 // virtual serial port speed, must be the same in boblight_config

CRGB leds[NUM_LEDS];
uint8_t * ledsRaw = (uint8_t *)leds;

/* demoreel start */
#define FRAMES_PER_SECOND  120
// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

/* demoreel end */

// A 'magic word' (along with LED count & checksum) precedes each block
// of LED data; this assists the microcontroller in syncing up with the
// host-side software and properly issuing the latch (host I/O is
// likely buffered, making usleep() unreliable for latch).  You may see
// an initial glitchy frame or two until the two come into alignment.
// The magic word can be whatever sequence you like, but each character
// should be unique, and frequent pixel values like 0 and 255 are
// avoided -- fewer false positives.  The host software will need to
// generate a compatible header: immediately following the magic word
// are three bytes: a 16-bit count of the number of LEDs (high byte
// first) followed by a simple checksum value (high byte XOR low byte
// XOR 0x55).  LED data follows, 3 bytes per LED, in order R, G, B,
// where 0 = off and 255 = max brightness.

static const uint8_t magic[] = {
  'A','d','a'};
#define MAGICSIZE  sizeof(magic)
#define HEADERSIZE (MAGICSIZE + 3)

#define MODE_HEADER 0
#define MODE_DATA   2

// If no serial data is received for a while, the LEDs are shut off
// automatically.  This avoids the annoying "stuck pixel" look when
// quitting LED display programs on the host computer.
static const unsigned long serialTimeout = 15000; // 15 seconds
static const unsigned long serialLongTimeout = 5 * 60000; // 15 seconds

void setup()
{
  //pinMode(GROUND_PIN, OUTPUT); 
  //digitalWrite(GROUND_PIN, LOW);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  // set master brightness control
  FastLED.setBrightness(96);
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();
  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  
  // Dirty trick: the circular buffer for serial data is 256 bytes,
  // and the "in" and "out" indices are unsigned 8-bit types -- this
  // much simplifies the cases where in/out need to "wrap around" the
  // beginning/end of the buffer.  Otherwise there'd be a ton of bit-
  // masking and/or conditional code every time one of these indices
  // needs to change, slowing things down tremendously.
  uint8_t
    buffer[256],
  indexIn       = 0,
  indexOut      = 0,
  mode          = MODE_HEADER,
  hi, lo, chk, i, spiFlag, showDemo = 1;
  int16_t
    bytesBuffered = 0,
  hold          = 0,
  c;
  int32_t
    bytesRemaining;
  unsigned long
    startTime,
  lastByteTime,
  lastAckTime,
  t;
  int32_t outPos = 0;

  Serial.begin(SERIALSPEED); // Teensy/32u4 disregards baud rate; is OK!

  Serial.print("Ada\n"); // Send ACK string to host

    startTime    = micros();
  lastByteTime = lastAckTime = millis();

  // loop() is avoided as even that small bit of function overhead
  // has a measurable impact on this code's overall throughput.

  for(;;) {

    // Implementation is a simple finite-state machine.
    // Regardless of mode, check for serial input each time:
    t = millis();
    if((bytesBuffered < 256) && ((c = Serial.read()) >= 0)) {
      buffer[indexIn++] = c;
      bytesBuffered++;
      lastByteTime = lastAckTime = t; // Reset timeout counters
      showDemo = 0;
    } 
    else {
      // No data received.  If this persists, send an ACK packet
      // to host once every second to alert it to our presence.
      if((t - lastAckTime) > 1000) {
        Serial.print("Ada\n"); // Send ACK string to host
        lastAckTime = t; // Reset counter
      }
      // If no data received for an extended time, turn off all LEDs.
      if(showDemo < 2 && (t - lastByteTime) > serialLongTimeout) {
        memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB)); //filling Led array by zeroes
        FastLED.show();
        //lastByteTime = t; // Reset counter
        showDemo = 2;
      }
      else if(showDemo < 2 && (t - lastByteTime) > serialTimeout) {
        showDemo = 1;
      }
      
      if (showDemo == 1){
      
        // Call the current pattern function once, updating the 'leds' array
        gPatterns[gCurrentPatternNumber]();

        // send the 'leds' array out to the actual LED strip
        FastLED.show();  
        // insert a delay to keep the framerate modest
        FastLED.delay(1000/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
        
      }
    }

    switch(mode) {

    case MODE_HEADER:

      // In header-seeking mode.  Is there enough data to check?
      if(bytesBuffered >= HEADERSIZE) {
        // Indeed.  Check for a 'magic word' match.
        for(i=0; (i<MAGICSIZE) && (buffer[indexOut++] == magic[i++]););
        if(i == MAGICSIZE) {
          // Magic word matches.  Now how about the checksum?
          hi  = buffer[indexOut++];
          lo  = buffer[indexOut++];
          chk = buffer[indexOut++];
          if(chk == (hi ^ lo ^ 0x55)) {
            // Checksum looks valid.  Get 16-bit LED count, add 1
            // (# LEDs is always > 0) and multiply by 3 for R,G,B.
            bytesRemaining = 3L * (256L * (long)hi + (long)lo + 1L);
            bytesBuffered -= 3;
            outPos = 0;
            memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));
            mode           = MODE_DATA; // Proceed to latch wait mode
          } 
          else {
            // Checksum didn't match; search resumes after magic word.
            indexOut  -= 3; // Rewind
          }
        } // else no header match.  Resume at first mismatched byte.
        bytesBuffered -= i;
      }
      break;

    case MODE_DATA:

      if(bytesRemaining > 0) {
        if(bytesBuffered > 0) {
          if (outPos < sizeof(leds))
            ledsRaw[outPos++] = buffer[indexOut++];   // Issue next byte
          bytesBuffered--;
          bytesRemaining--;
        }
        // If serial buffer is threatening to underrun, start
        // introducing progressively longer pauses to allow more
        // data to arrive (up to a point).
      } 
      else {
        // End of data -- issue latch:
        startTime  = micros();
        mode       = MODE_HEADER; // Begin next header search
        FastLED.show();
      }
    } // end switch
  } // end for(;;)
}

void loop()
{
  // Not used.  See note in setup() function.
}
