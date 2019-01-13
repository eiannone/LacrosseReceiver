#ifndef _Timings2Measure_h
#define _Timings2Measure_h
/*
  LacrosseReceiver - Arduino library for decoding RF 433Mhz signals of Lacrosse
  wheater stations sensors.
  Tested with the following transmitter models: TX3-TH, TX4 and TX7U

  OVERVIEW OF MAIN LOGIC
  (by @Joetgithub https://github.com/Joetgithub/TX7U/blob/master/TX7UReceiver.ino)

  Continuously loads pulses into a rolling buffer that is sized to hold one temp
  or humidity reading. About every 57 seconds the TX4 and TX7 sensors send a
  data transmission consisting of a 44 bit temperature sequence followed by a
  repeat of that same 44 bit temperature sequence followed by a 44 bit humidity
  sequence.  A relatively long delay occurs after each temp/humidity 44 bit sequence
  and that delay is used as the trigger to evaluate the contents of the buffer and
  obtain the data values from the respective sequence.
  A pulse is the time in microseconds between changes in the data pin. The pulses
  have to follow a rule of being a LONG or SHORT followed by FIXED.
  A TOLERANCE value specifies the allowable range from the hard coded LONG, SHORT
  and FIXED pulse times. A 1 bit is a SHORT followed by a FIXED.  A 0 bit is a LONG
  followed by a SHORT.
  Here is example of a 10101. Note the variations in the SHORT, LONG and FIXED times.
            SHORT  LONG SHORT  LONG  LONG
             505   1300  595   1275  1395
             ┌-┐  ┌---┐  ┌-┐  ┌---┐  ┌-┐
             |1|  | 0 |  |1|  | 0 |  |1|
           --┘ └--┘   └--┘ └--┘   └--┘ └-
  FIXED        980    1030 1105   950
  Example showing two timings needed for each pulse
          t2          t4
           \           \              t2-t1=pulse1  FIXED
            ┌----┐     ┌----┐         t3-t2=pulse2  LONG or SHORT
            |    |     |    |     |   t4-t3=pulse3  FIXED
        ----┘    └-----┘    └-----┘   t5-t4=pulse4  LONG or SHORT
       /         /          /
      t1        t3         t5
  Because two timings are needed for each bit a total of 88 pulses are needed
  to decode the 44 bits.
  The pulses are converted into bits and stored in a six byte array as follows:
  [0]00001010 [1]11101110 [2]11110100 [3]10010000 [4]01001001 [5]1111
                   |   \      / |  \      |   |       |   |       |
     00001010    1110  1110111  1  0100  1001 0000   0100 1001   1111
  bits: 8         4     4+3=7   1    4    4    4       4    4      4
  key: (1)       (2)     (3)   (4)  (5)  (6)  (7)     (8)  (9)   (10)
      header    sensor sensor parity 10s  1s  10ths   10s  10th  check
                 type    ID    bit                                sum
  key: 1) Start Sequence is always 0x0A
       2) sensor 0000 = temp; 1110 = humidity
       3) sensor id
       4) parity bit
       5) tens
       6) ones
       7) tenths (for temp)
       8) repeat tens
       9) repeat ones
       10) checksum
 http://www.f6fbb.org/domo/sensors/tx3_th.php
*/


#ifdef ARDUINO
    #include <Arduino.h>
#else
    // These are used for unit testing in Desktop environment
    #include <cstdint>
    #include <cstdlib>
    #include <stdint-gcc.h>
    #include <cstddef>

    typedef uint8_t byte;
#endif

#define PW_FIXED 975  // Pulse width for the "fixed" part of signal
#define PW_SHORT 550  // Pulse width for the "short" part of signal
#define PW_LONG 1400  // Pulse width for the "long" part of signal
#define PW_LAST 5000  // Minimum pulse width for "sync" signal (last one)
#define PW_TOL 210    // Tolerance for pulse width detection (range is PW ± PW_TOL)
#define PW_TOL_F 500  // Fuzzy tolerance (moore loose)

enum measureType : uint8_t {TEMPERATURE, HUMIDITY, UNKNOWN};

struct timings_packet {
    uint32_t msec = 0;
    uint32_t size = 0;
    uint32_t getTiming(size_t pos) {
        return (pos >= size - 1)? PW_LAST : peekTiming(pos);
    }
    virtual uint32_t peekTiming(size_t pos) = 0;
};

struct measure {
    uint32_t msec;
    uint8_t sensorAddr;
    measureType type;
    uint8_t units;
    uint8_t decimals;
    int8_t sign;
};

class Timings2Measure {
public:
    Timings2Measure() : _ignoreChecksum(false) {};
    Timings2Measure(bool ignoreChecksum) : _ignoreChecksum(ignoreChecksum) {};
    measure getMeasure(timings_packet* pk);
    
    inline static bool isLongShort(uint32_t t) {
        return (t > (PW_SHORT - PW_TOL) && t < (PW_SHORT + PW_TOL))
            || (t > (PW_LONG  - PW_TOL) && t < (PW_LONG  + PW_TOL));
    }
    inline static bool isValidTiming(uint32_t t) {        
        return (t > (PW_FIXED - PW_TOL) && t < (PW_FIXED + PW_TOL)) 
		    || isLongShort(t); // Fixed or long/short
    }

private:
    timings_packet* _timings;
    measure _measure;
    bool _ignoreChecksum;
    bool _fuzzy; // true if pulse detection needs to be in "fuzzy" mode

    size_t _tHeader;

    // Number of ones in a digit (decimal 0 - 9)
    static const uint8_t ONES_COUNT[10];

    struct bits_pos { byte bits; size_t timings; };
    struct measure_pos { uint8_t units; uint8_t decimals; size_t timings; };

    uint32_t longShortTiming(uint32_t);
    bool isFixed(uint32_t);

    size_t getFixedTiming(size_t, bool ungreedy = false);
    bits_pos getBit(size_t, bool ungreedy = false);
    bits_pos fetchBits(size_t, size_t, bool ungreedy = false, bool fuzzy = false);
    bool fetchHeader(bool fuzzy = false);
    bool fetchHeaderFuzzy();
    measure_pos fetchMeasure(size_t timingPos, uint8_t parity, bool ungreedy = false);
    measure_pos fetchMeasureRep(size_t timingPos, bool ungreedy = false);
    uint8_t measureChecksum(uint8_t sensorId, measureType mType, int8_t units, uint8_t decimals);
    bool readForward();

    size_t getFixedTimingBk(size_t, bool ungreedy = false);
    bits_pos getBitBk(size_t, bool ungreedy = false);
    bits_pos fetchBitsBk(size_t, size_t, bool ungreedy = false, bool fuzzy = false);
    measure_pos fetchMeasureBk(size_t, bool ungreedy = false);
    measure_pos fetchMeasureRepBk(size_t timingPos, bool ungreedy = false);
    bool readBackward();

    /**
     * Checks if the last 'numBits' bits of 'bits' match with a part of header
     */
    inline bool isPartOfHeader(const byte bits, const size_t numBits) {
        byte mask = 0xFF >> (8 - numBits);
        for (size_t i = 0; i <= 8 - numBits; i++) {
            if (((0x0A >> i) & mask) == bits) return true;
        }
        return false;
    }
};

#endif // _Timings2Measure_h
