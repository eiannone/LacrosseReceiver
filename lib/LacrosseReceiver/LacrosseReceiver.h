#ifndef _LacrosseReceiver_h
#define _LacrosseReceiver_h

#include "Timings2Measure.h"

#define TIMINGS_BUFFER_SIZE 120  // Max number of bits in a packet = 60
#define PACKET_BUFFER_SIZE 1024  // Packets buffer contains variable sized packets
#define PACKET_POS_BUFFER_SIZE 128 // This buffer contains last packet start positions inside packet buffer

#ifdef ESP8266
    // Interrupt handler and related code must be in RAM on ESP8266
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

// This struct represents a packet of timings inside packets buffer
struct packet : timings_packet {
public:
    packet(size_t startPos) : _startPos(startPos) {};
    packet() : _startPos(0) {};
    uint32_t peekTiming(size_t pos);

private: int _startPos;
};

class LacrosseReceiver {
public:
    // Buffer containing the detected timings packets (each with a different size)
    static volatile uint32_t packetsBuf[PACKET_BUFFER_SIZE];

    LacrosseReceiver(const int pin, const bool ignoreChecksum = true);
    void enableReceive();
    void disableReceive();
    measure getNextMeasure();

private:
    int _interrupt;
    Timings2Measure* _t2m;

    // Buffer containing the received pulses
    static volatile uint32_t timingsBuf[TIMINGS_BUFFER_SIZE];
    // Buffer containing the start position of each packet in the packets buffer
    static volatile size_t packetPosBuf[PACKET_POS_BUFFER_SIZE];
    static volatile size_t
            packetsBufSize, // Size of used packet buffer
            firstPacketPos, // Tail position of stored packets
            lastPacketPos;  // Head position of stored packets

    static void handleInterrupt();
    inline static bool isLongShort(uint32_t timing) {
        return (timing > (PW_SHORT - PW_TOL) && timing < (PW_SHORT + PW_TOL))
          || (timing > (PW_LONG  - PW_TOL) && timing < (PW_LONG  + PW_TOL));
    }
    inline static bool isFixed(uint32_t timing) {
        return timing > (PW_FIXED - PW_TOL) && timing < (PW_FIXED + PW_TOL);
    }
};

#endif
