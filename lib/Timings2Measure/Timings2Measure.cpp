#include "Timings2Measure.h"

const uint8_t Timings2Measure::ONES_COUNT[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2};

uint32_t Timings2Measure::longShortTiming(uint32_t t)
{
    if (t > (PW_LONG - PW_TOL) && t < (PW_LONG + PW_TOL)) return PW_LONG;
    if (t > (PW_SHORT - PW_TOL) && t < (PW_SHORT + PW_TOL)) return PW_SHORT;
    if (_fuzzy) {
        if (t > (PW_LONG - PW_TOL_F) && t < (PW_LONG + PW_TOL_F)) return PW_LONG;
        if (t > (PW_SHORT - PW_TOL_F) && t < (PW_SHORT + PW_TOL_F)) return PW_SHORT;
    }
    return 0;
}

bool Timings2Measure::isFixed(uint32_t t)
{
    // Last timing (plus some short timings added) is considered fixed
    if (t >= PW_LAST && t <= (PW_LAST + 1000)) return true;
    return t > (PW_FIXED - (_fuzzy? PW_TOL_F : PW_TOL))
        && t < (PW_FIXED + (_fuzzy? PW_TOL_F : PW_TOL));
}

size_t Timings2Measure::getFixedTiming(size_t timingPos, bool ungreedy)
{
    uint32_t tSum = 0;
    for(size_t t = timingPos; t < _timings->size; t++) {
        tSum += _timings->getTiming(t);
        if (t < (_timings->size - 1) && tSum > PW_LONG) break;
        if (isFixed(tSum)) {
            if (ungreedy) return t - timingPos + 1;
            while(++t < _timings->size) {
                tSum += _timings->getTiming(t);
                if (!isFixed(tSum)) return t - timingPos;
            }
            return _timings->size - timingPos;
        }
    }
    return 0;
}

Timings2Measure::bits_pos Timings2Measure::getBit(size_t timingPos, bool ungreedy)
{
    if (timingPos >= _timings->size) return {0, 0};
    uint32_t tt = longShortTiming(_timings->getTiming(timingPos));
    if (tt != 0) {
        size_t nTimings = getFixedTiming(timingPos + 1, ungreedy);
        if (nTimings > 0) return {(byte)((tt == PW_LONG)? 0 : 1), 1 + nTimings};
    }
    // Tries to look ahead, merging multiple timings
    uint32_t tSum = 0;
    for(size_t t = timingPos; t < _timings->size; t++) {
        tSum += _timings->getTiming(t);
        tt = longShortTiming(tSum);
        if (tt != 0) {
            // Search for next long/short timing and verifies that the timing in between equals to fixed
            size_t nTimings = getFixedTiming(t + 1, ungreedy);
            if (nTimings == 0) return {0, 0};
            size_t tNext = t + nTimings + 1;
            if (tNext == _timings->size || longShortTiming(_timings->getTiming(tNext)) > 0)
                return {(byte)((tt == PW_LONG)? 0 : 1), tNext - timingPos};
        }
        if (tSum > PW_LONG) break;
    }
    return {0, 0};
}

Timings2Measure::bits_pos Timings2Measure::fetchBits(size_t timingPos, size_t nBits, bool ungreedy, bool fuzzy)
{
    byte bits = 0;
    size_t fetched = 0;
    size_t t = 0;
    _fuzzy = fuzzy;
    while(fetched < nBits) {
        bits_pos bp = getBit(timingPos + t, ungreedy);
        if (bp.timings == 0)
            return _fuzzy? (bits_pos){0, 0} : fetchBits(timingPos, nBits, ungreedy, true);
        bits = (bits << 1) + bp.bits;
        fetched++;
        t += bp.timings;
    }
    return {bits, t};
}

bool Timings2Measure::fetchHeader(bool fuzzy)
{
    // There should be at least room for 2 (remaining) bit of header (4 timings), measure type (8 timings),
    // sensor id (14 timings), parity (2 timings), measure (24 timings) -> total 52 timings
    if (_timings->size < 52) return false;

    _fuzzy = fuzzy;
    bits_pos bp{};

    // Find a sequence of two bits, of which the first is 0 (starting of header)
    size_t t = 0;
    byte header = 0;
    bool found = false;
    while (t < _timings->size - 40) {
        bp = getBit(t);
        t += (bp.timings == 0)? 1 : bp.timings;
        if (bp.timings == 0 || bp.bits != 0) continue;
        bp = getBit(t);
        t += (bp.timings == 0)? 1 : bp.timings;
        if (bp.timings != 0) {
            header = bp.bits;
            found = true;
            break;
        }
    }
    // "Cannot find header start sequence (0X)";
    if (!found) return _fuzzy? false : fetchHeader(true);

    // Fetch the rest of header
    size_t hBits = 2;
    do {
        // "Reached last reasonable timing without finding header";
        if (t > _timings->size - 40) return _fuzzy? false : fetchHeader(true);
        bp = getBit(t);
        // "Cannot decode a bit inside header";
        if (bp.timings == 0) return _fuzzy? false : fetchHeader(true);
        header = (header << 1) + bp.bits;
        t += bp.timings;
        if (hBits < 8) hBits++;
    } while (isPartOfHeader(header, hBits));

    // Discard last bit because is not part of header
    hBits--;
    header >>= 1;

    // Checks if the bits match with last part of header
    if (header != (0x0A & (0xFF >> (8 - hBits)))) return _fuzzy? false : fetchHeader(true);
    _tHeader = t - bp.timings;
    return true;
}

bool Timings2Measure::fetchHeaderFuzzy()
{
    if (_timings->size < 54) return false;
    size_t t = 0;
    while (t < _timings->size - 54) {
        for(size_t len = 0; len < 6; len++) {
            bits_pos bp = fetchBits(t, 8 - len);
            byte headerPart = 0x0A & (0xFF >> len);
            if (bp.timings == 0 || bp.bits != headerPart) bp = fetchBits(t, 8 - len, true);
            if (bp.timings != 0 && bp.bits == headerPart) {
                _tHeader = t + bp.timings;
                return true;
            }
        }
        t++;
    }
    return false; //"Unable to detect header";
}

Timings2Measure::measure_pos Timings2Measure::fetchMeasure(size_t timingPos, uint8_t parity, bool ungreedy)
{
    measure_pos m = {0};
    size_t t = timingPos;
    // Fetch measure digits
    for (uint8_t digit = 0; digit < 3; digit++) {
        bits_pos bp = fetchBits(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return m;

        if (digit == 0) m.units = bp.bits * 10;
        else if (digit == 1) m.units += bp.bits;
        else m.decimals = bp.bits;

        t += bp.timings;
        parity += ONES_COUNT[bp.bits];
    }
    // Check parity
    if (parity % 2 == 0) m.timings = t - timingPos;
    return m;
}

Timings2Measure::measure_pos Timings2Measure::fetchMeasureRep(size_t timingPos, bool ungreedy)
{
    measure_pos m = {0};
    size_t t = timingPos;
    // Fetch measure digits
    for (uint8_t digit = 0; digit < 2; digit++) {
        bits_pos bp = fetchBits(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return m;

        if (digit == 0) m.units = bp.bits * 10;
        else m.units += bp.bits;

        t += bp.timings;
    }
    m.timings = t - timingPos;
    return m;
}

uint8_t Timings2Measure::measureChecksum(uint8_t sensorId, measureType mType, int8_t units, uint8_t decimals)
{
    uint8_t ones = ONES_COUNT[decimals] + ONES_COUNT[(units / 10)] + ONES_COUNT[(units % 10)];

    // Calculate checksum as sum of nibbles
    auto sum = static_cast<uint8_t>(10 + // 10 = Header checksum (0000 + 1010)
        ((mType == TEMPERATURE)? 0x0 : 0xE) +
        (sensorId >> 3) +
        ((sensorId << 1) & 0x0F) + (ones % 2) +
        ((units / 10) * 2) +
        ((units % 10) * 2) +
        decimals);
    return static_cast<uint8_t>(sum & 0x0F);
}

/*
 MEASURE STRUCTURE (44 bit)

 0-7:   8 bits header 00001010
 8-11:  4 bits measure type: 0000 = temp, 1110 = humidity
 12-18: 7 bits sensor id
 19:    1 bits parity. Makes even the number of bits "1" from 19 to 31
 20-23: 4 bits digit for tens
 24-27: 4 bits digit for ones
 28-31: 4 bits digit for decimals
 32-35: 4 bits digit for tens again
 36-39: 4 bits digit for ones again
 40-43: 4 bits CRC (sum of nibbles from bit 0 to 39)

*/
bool Timings2Measure::readForward()
{
    _measure = {0,0,UNKNOWN,0,0,1};

    if (!fetchHeader() && !fetchHeaderFuzzy()) return false; // Unable to decode header

    // There should be at least 24 more bits (48 timings)
    // 4 bits for measure type (0000 or 1110)
    // 7 bits fot sensor id
    // 1 bit for parity (makes measure digits even)
    // 12 bits for measure (4 bits for each digit)
    if (_timings->size - _tHeader < 48) return false; // "Not enough timings to decode measure";

    // Fetch measure type
    bits_pos bp = fetchBits(_tHeader, 4);
    if (bp.timings == 0 || (bp.bits != 0x0 && bp.bits != 0xE)) // Measure type should be 0000 or 1110
        bp = fetchBits(_tHeader, 4, true);

    if (bp.timings == 0) return false; // "Cannot decode a bit inside measure type";
    if (bp.bits == 0x0) _measure.type = TEMPERATURE;
    else if (bp.bits == 0xE) _measure.type = HUMIDITY;
    else return false; //"Wrong measure type";

    size_t t = _tHeader + bp.timings;

    // Fetch sensor id
    bp = fetchBits(t, 7);
    if (bp.timings == 0) return false; // "Cannot decode a bit inside sensor addr";
    _measure.sensorAddr = (uint8_t)bp.bits;
    t += bp.timings;

    // Fetch parity
    _fuzzy = false;
    bp = getBit(t);
    if (bp.timings == 0) {
        _fuzzy = true;
        bp = getBit(t);
        if (bp.timings == 0) return false; // "Cannot decode parity bit";
    }
    uint8_t parity = bp.bits;
    t += bp.timings;

    measure_pos mp = fetchMeasure(t, parity);
    if (mp.timings == 0) {
        mp = fetchMeasure(t, parity, true);
        if (mp.timings == 0) return false;
    }
    _measure.units = mp.units;
    _measure.decimals = mp.decimals;
    t += mp.timings;

    // Check if there are enough timings for repeated measure (8 bit)
    if (_timings->size - t < 16) return _ignoreChecksum; // Ignores error

    // Fetch repeated measure
    measure_pos mp2 = fetchMeasureRep(t);
    if (mp2.timings == 0) {
        mp2 = fetchMeasureRep(t, true);
        if (mp2.timings == 0) return _ignoreChecksum;
    }
    t += mp2.timings;

    if (mp2.units != mp.units) return false; // Measures don't match!

    if (_ignoreChecksum) return true;

    // Check if there are enough timings for checksum (4 bit)
    if (_timings->size - t < 8) return false;

    // Fetch checksum
    bp = fetchBits(t, 4);
    return bp.timings != 0 && bp.bits == measureChecksum(_measure.sensorAddr, _measure.type, mp.units, mp.decimals);
}

size_t Timings2Measure::getFixedTimingBk(size_t timingPos, bool ungreedy)
{
    uint32_t tSum = 0;
    for(auto t = timingPos; t >= 0; t--) {
        tSum += _timings->getTiming(t);
        if (t < (_timings->size - 1) && tSum > PW_LONG) break;
        if (isFixed(tSum)) {
            if (ungreedy) return timingPos - t + 1;
            while(--t >= 0) {
                tSum += _timings->getTiming(t);
                if (!isFixed(tSum)) return timingPos - t;
            }
            return timingPos + 1;
        }
    }
    return 0;
}

Timings2Measure::bits_pos Timings2Measure::getBitBk(size_t timingPos, bool ungreedy)
{
    size_t nTimings = getFixedTimingBk(timingPos, ungreedy);
    if (nTimings > timingPos) return {0, 0};
    uint32_t tt = (nTimings == 0)? 0 : longShortTiming(_timings->getTiming(timingPos - nTimings));
    if (nTimings == 0 || tt == 0) return {0, 0};

    return {(byte)((tt == PW_LONG)? 0 : 1), nTimings + 1};
}

Timings2Measure::bits_pos Timings2Measure::fetchBitsBk(size_t timingPos, size_t nBits, bool ungreedy, bool fuzzy)
{
    byte bits = 0;
    size_t fetched = 0;
    auto t = timingPos;
    _fuzzy = fuzzy;
    while(fetched < nBits) {
        bits_pos bp = getBitBk(t, ungreedy);
        if (bp.timings == 0)
            return _fuzzy? (bits_pos){0, 0} : fetchBitsBk(timingPos, nBits, ungreedy, true);
        if (bp.bits == 1) bits |= (1 << fetched);
        fetched++;
        if (bp.timings > t && fetched < nBits) return {0, 0};
        t -= bp.timings;
    }
    return {bits, timingPos - t};
}

Timings2Measure::measure_pos Timings2Measure::fetchMeasureBk(size_t timingPos, bool ungreedy)
{
    measure_pos m = {0};
    size_t t = timingPos;
    // Fetch measure digits
    uint8_t ones = 0;
    for (uint8_t digit = 3; digit > 0; digit--) {
        bits_pos bp = fetchBitsBk(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return m;

        if (digit == 3) m.decimals = bp.bits;
        else if (digit == 2) m.units = bp.bits;
        else m.units += bp.bits * 10;

        t -= bp.timings;
        ones += ONES_COUNT[bp.bits];
    }

    // Fetch parity
    _fuzzy = false;
    bits_pos bp = getBitBk(t);
    if (bp.timings == 0) {
        _fuzzy = true;
        bp = getBitBk(t);
        if (bp.timings == 0) return m; // Unable to decode parity bit
    }
    t -= bp.timings;
    ones += bp.bits;
    if (ones % 2 == 0) m.timings = timingPos - t;
    return m;
}

Timings2Measure::measure_pos Timings2Measure::fetchMeasureRepBk(size_t timingPos, bool ungreedy)
{
    measure_pos m = {0};
    size_t t = timingPos;
    // Fetch measure digits
    for (uint8_t digit = 0; digit < 2; digit++) {
        bits_pos bp = fetchBitsBk(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return m;

        if (digit == 0) m.units = bp.bits;
        else m.units += bp.bits * 10;

        t -= bp.timings;
    }
    m.timings = timingPos - t;
    return m;
}

bool Timings2Measure::readBackward()
{
    _measure = {0,0,UNKNOWN,0,0,1};

    size_t t = _timings->size - 2;
    uint32_t tt = longShortTiming(_timings->getTiming(t));
    if (tt == 0) return false; // "Unable to decode last bit";
    uint8_t checksum = (tt == PW_LONG)? 0 : 1;

    bits_pos bp = fetchBitsBk(t - 1, 3);
    if (bp.timings == 0) return false; // "Unable to decode a bit inside checksum"
    checksum |= (bp.bits << 1);
    t -= (bp.timings + 1);

    // Fetch repeated measure
    measure_pos mp2 = fetchMeasureRepBk(t);
    if (mp2.timings == 0) {
        mp2 = fetchMeasureRepBk(t, true);
        if (mp2.timings == 0) return false;
    }
    t -= mp2.timings;

    // Fetch measure and parity
    measure_pos mp = fetchMeasureBk(t);
    if (mp.timings == 0) {
        mp = fetchMeasureBk(t, true);
        if (mp.timings == 0) return false;
    }
    t -= mp.timings;
	_measure.units = mp.units;
    _measure.decimals = mp.decimals;

    if (mp.units != mp2.units) return false; // Measures don't match!

    // Check if there are enough timings for sensor id (7 bit) and measure type (4 bit)
    if (t < 22) return false; // "Not enough timings to decode sensor address and measure type";

    // Fetch sensor id
    bp = fetchBitsBk(t, 7);
    if (bp.timings == 0) return false; // "Cannot decode a bit inside sensor addr";
    _measure.sensorAddr = (uint8_t) bp.bits;
    t -= bp.timings;

//    // Check if there are enough timings for measure type (4 bit)
//    if (t < 8) return false; // "Not enough timings to decode measure type";

    // Fetch measure type
    bp = fetchBitsBk(t, 4);
    if (bp.timings == 0 || (bp.bits != 0x0 && bp.bits != 0xE)) // Measure type should be 0000 or 1110
        bp = fetchBitsBk(t, 4, true);

    if (bp.timings == 0) return false ;// "Cannot decode a bit inside measure type";
    if (bp.bits == 0x0) _measure.type = TEMPERATURE;
    else if (bp.bits == 0xE) _measure.type = HUMIDITY;
    else return false; //"Wrong measure type";
    _tHeader = t - bp.timings + 1;

    // Check checksum
    return _ignoreChecksum || (checksum == measureChecksum(_measure.sensorAddr, _measure.type, mp.units, mp.decimals));
}

measure Timings2Measure::getMeasure(timings_packet* pk)
{
    _timings = pk;

    // Excludes packets with more than 12 initial bits missing (header and sensor type)
    // If there are more than 12 bits missing, we cannot identify sensor address because
    // t start at the 13th bit.
    // So the minimum number of bits is 44 - 12 = 32, that is 64 timings
    if (_timings->size < 64 || (!readForward() && !readBackward()))
        return { pk->msec, 0, UNKNOWN, 0, 0, 1 };

    _measure.msec = pk->msec;
    // For temperature decrease the value by 50 (beware of negative values!)
    if (_measure.type == TEMPERATURE) {
        if (_measure.units >= 50) _measure.units -= 50;
        else { // Negative values
            _measure.sign = -1;
            _measure.units = static_cast<uint8_t>(50 - _measure.units);
            if (_measure.decimals > 0) {
                _measure.units--;
                _measure.decimals = static_cast<uint8_t>(10 - _measure.decimals);
            }
        }
    }
    return _measure;
}
