#include "LacrosseReceiver.h"

volatile uint32_t LacrosseReceiver::timingsBuf[TIMINGS_BUFFER_SIZE];
volatile uint32_t LacrosseReceiver::packetsBuf[PACKET_BUFFER_SIZE];
volatile size_t LacrosseReceiver::packetPosBuf[PACKET_POS_BUFFER_SIZE];
volatile size_t LacrosseReceiver::packetsBufSize = 0;
volatile size_t LacrosseReceiver::firstPacketPos = 0;
volatile size_t LacrosseReceiver::lastPacketPos = 0;

uint32_t packet::peekTiming(size_t pos)
{
    size_t i = _startPos + 2 + pos;
    return LacrosseReceiver::packetsBuf[(i >= PACKET_BUFFER_SIZE)? i - PACKET_BUFFER_SIZE : i];
}

// Board                               Digital Pins Usable For Interrupts
// Uno, Nano, Mini, other 328-based    2, 3
// Mega, Mega2560, MegaADK             2, 3, 18, 19, 20, 21
// Micro, Leonardo, other 32u4-based   0, 1, 2, 3, 7
// Zero                                all digital pins, except 4
// MKR1000 Rev.1                       0, 1, 4, 5, 6, 7, 8, 9, A1, A2
// Due                                 all digital pins
LacrosseReceiver::LacrosseReceiver(const int pin, const bool ignoreChecksum)
{
#ifdef ESP8266
    _interrupt = pin;
#else
    _interrupt = digitalPinToInterrupt(pin);
#endif
    _t2m = new Timings2Measure(ignoreChecksum);
}

void RECEIVE_ATTR LacrosseReceiver::handleInterrupt()
{
    static size_t timingPos = 0;
    static size_t packetPos = 0;
    static bool receiving = false;
    static bool bufferFull = false;
    static uint32_t lastTime = 0;

    const uint32_t time = micros();
    const uint32_t duration = time - lastTime;
    lastTime = time;

    if (!receiving) {
        // Ignores pulses, until we receive a short or long pulse
        if (!Timings2Measure::isLongShort(duration)) return;

        // First pulse detected (short or long)
        receiving = true;
        timingPos = 0;
        bufferFull = false;
    }
    // If we are here, we are receiving pulses

    // Stores pulse duration in a circular buffer 'timingsBuf'
    timingsBuf[timingPos] = duration;
    if (++timingPos == TIMINGS_BUFFER_SIZE) {
        bufferFull = true;
        timingPos = 0;
    }

    if (duration < PW_LAST) return;
    // Possible synchronization signal detected (long duration >= PW_LAST) - End of packet
    receiving = false;

    // PRELIMINARY VALIDITY CHECK
    // Verifies if there are enough legitimate timings

    // Scans the timings backward and accept no more than 10 errors
    size_t collected = bufferFull? TIMINGS_BUFFER_SIZE : timingPos;
    size_t packetSize = 0;
    int errors = -1; // Last timing is always considered an error because duration is different
    while(collected > 0 && errors < 10) {
        packetSize++;
        collected--;
        if (timingPos-- == 0) timingPos = TIMINGS_BUFFER_SIZE - 1;
        if (!Timings2Measure::isValidTiming(timingsBuf[timingPos])) errors++;
    }
    if (packetSize < 32) return; // Excludes packets with less than 16 bits

    // OK. TIMINGS PACKET MAY BE VALID. SO WE STORE IT IN THE PACKET BUFFER

    // Checks if packets buffer is full
    packetsBufSize += (packetSize + 2);
    while (packetsBufSize > PACKET_BUFFER_SIZE) {
        // Packets buffer full. Removes the oldest packet (from the tail)
        packetsBufSize -= (packetsBuf[packetPosBuf[firstPacketPos]] + 2);
        if (++firstPacketPos == PACKET_POS_BUFFER_SIZE) firstPacketPos = 0;
    }

    // Stores the packet starting position (= packetPos) in the packets starting positions buffer
    packetPosBuf[lastPacketPos] = packetPos;
    // Advances the head of stored packets positions in the buffer
    if (++lastPacketPos == PACKET_POS_BUFFER_SIZE) lastPacketPos = 0;

    // Stores packet size in the buffer (as first element)
    packetsBuf[packetPos] = packetSize;

    // Stores current milliseconds as second element
    if (++packetPos == PACKET_BUFFER_SIZE) packetPos = 0;
    packetsBuf[packetPos] = millis();

    // Stores all timings in the remaining positions
    for(size_t t = 0; t < packetSize; t++) {
        if (++packetPos == PACKET_BUFFER_SIZE) packetPos = 0;
        packetsBuf[packetPos] = timingsBuf[timingPos];
        if (++timingPos == TIMINGS_BUFFER_SIZE) timingPos = 0;
    }
    // Normalizes last timing duration (for Lacrosse sensors it can have an arbitrary duration)
    if (packetsBuf[packetPos] > PW_LAST + 1000) packetsBuf[packetPos] = PW_LAST;

    if (++packetPos == PACKET_BUFFER_SIZE) packetPos = 0;
}

/**
 * Enable receiving data
 */
void LacrosseReceiver::enableReceive()
{
    attachInterrupt(_interrupt, handleInterrupt, CHANGE);
}

/**
 * Disable receiving data
 */
void LacrosseReceiver::disableReceive()
{
    detachInterrupt(_interrupt);
}

measure LacrosseReceiver::getNextMeasure()
{
    if (firstPacketPos == lastPacketPos) // Return empty measure
        return {0, 0, UNKNOWN, 0, 0};

    // Extracts packet from buffer
    size_t pStart = packetPosBuf[firstPacketPos];
    packet pk(pStart);
    pk.size = packetsBuf[pStart];
    pk.msec = packetsBuf[(pStart < PACKET_BUFFER_SIZE - 1)? pStart + 1 : 0];

    // Advances the head of packets in the buffer
    if (++firstPacketPos == PACKET_POS_BUFFER_SIZE) firstPacketPos = 0;
    // Updates the size of packets buffer
    packetsBufSize -= (pk.size + 2);

    // Converts packet to measure
    measure m = _t2m->getMeasure(&pk);

    // If measure is invalid, tries the next one
    return (m.type != UNKNOWN)? m : getNextMeasure();
}
