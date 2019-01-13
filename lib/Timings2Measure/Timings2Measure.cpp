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
    _fuzzy = fuzzy;
    bits_pos bp{};

    // Find a sequence of two bits, of which the first is 0 (starting of header)
    size_t t = 0;
    byte header = 0;
    bool found = false;
    // There should be at least room for 2 bit of header (4 timings), measure type (8 timings),
    // sensor id (14 timings), parity (2 timings), measure (24 timings) -> total 52 timings
    while (t < _timings->size - 52) {
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
        if (t > _timings->size - 48) return _fuzzy? false : fetchHeader(true);
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
    size_t t = 0;
    while (t < _timings->size - 54) {
        for(size_t len = 0; len < 6; len++) {
            bits_pos bp = fetchBits(t, 8 - len);
            byte headerPart = 0x0A & (0xFF >> len);
            if (bp.bits != headerPart) bp = fetchBits(t, 8 - len, true);
            if (bp.bits == headerPart) {
                _tHeader = t + bp.timings;
                return true;
            }
        }
        t++;
    }
    return false; //"Unable to detect header";
}

size_t Timings2Measure::fetchMeasure(size_t timingPos, bool ungreedy)
{
    size_t t = timingPos;
    // Fetch measure digits
    uint8_t ones = _parity;
    for (uint8_t digit = 0; digit < 3; digit++) {
        bits_pos bp = fetchBits(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return 0;

        if (digit == 0) _mUnits = bp.bits * 10;
        else if (digit == 1) _mUnits += bp.bits;
        else _mDecimals = bp.bits;

        t += bp.timings;
        ones += ONES_COUNT[bp.bits];
    }
    // Check parity
    if (ones % 2 == 1) return 0; // "Parity mismatch"
    return t - timingPos;
}

size_t Timings2Measure::fetchMeasureRep(size_t timingPos, bool ungreedy)
{
    size_t t = timingPos;
    // Fetch measure digits
    for (uint8_t digit = 0; digit < 2; digit++) {
        bits_pos bp = fetchBits(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return 0;

        if (digit == 0) _m2Units = bp.bits * 10;
        else _m2Units += bp.bits;

        t += bp.timings;
    }
    return t - timingPos;
}

uint8_t Timings2Measure::getChecksum()
{
    // Calculate checksum as sum of nibbles
    uint8_t sum = 10 + // 10 = Header checksum (0000 + 1010)
          ((_mType == TEMPERATURE)? 0x0 : 0xE) +
          (_sensorId >> 3) +
          ((_sensorId << 1) & 0x0F) + _parity +
          ((_mUnits / 10) * 2) +
          ((_mUnits % 10) * 2) +
          _mDecimals;
    return sum & 0x0F;
}

bool Timings2Measure::readForward()
{
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

    if (bp.timings == 0) return false ;// "Cannot decode a bit inside measure type";
    if (bp.bits == 0x0) _mType = TEMPERATURE;
    else if (bp.bits == 0xE) _mType = HUMIDITY;
    else return false; //"Wrong measure type";

    size_t t = _tHeader + bp.timings;

    // Fetch sensor id
    bp = fetchBits(t, 7);
    if (bp.timings == 0) return false; // "Cannot decode a bit inside sensor addr";
    auto sensorId = (uint8_t)bp.bits;
    t += bp.timings;

    // Fetch parity
    _fuzzy = false;
    bp = getBit(t);
    if (bp.timings == 0) {
        _fuzzy = true;
        bp = getBit(t);
        if (bp.timings == 0) return false; // "Cannot decode parity bit";
    }
    _parity = bp.bits;
    t += bp.timings;

    size_t nTimings = fetchMeasure(t);
    if (nTimings == 0) {
        nTimings = fetchMeasure(t, true);
        if (nTimings == 0) return false;
    }
    t += nTimings;

    // Assumes also sensor id was read correctly
    _sensorId = sensorId;

    // Check if there are enough timings for repeated measure (8 bit)
    if (_timings->size - t < 16) return _ignoreChecksum; // Ignores error

    // Fetch repeated measure
    nTimings = fetchMeasureRep(t);
    if (nTimings == 0) {
        nTimings = fetchMeasureRep(t, true);
        if (nTimings == 0) return _ignoreChecksum;
    }
    t += nTimings;

    if (_m2Units != _mUnits) return false; // Measures don't match!

    if (_ignoreChecksum) return true;

    // Check if there are enough timings for checksum (4 bit)
    if (_timings->size - t < 8) return false;

    // Fetch checksum
    bp = fetchBits(t, 4);
    return bp.timings != 0 && bp.bits == getChecksum();
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

size_t Timings2Measure::fetchMeasureBk(size_t timingPos, bool ungreedy)
{
    size_t t = timingPos;
    // Fetch measure digits
    uint8_t ones = 0;
    for (uint8_t digit = 3; digit > 0; digit--) {
        bits_pos bp = fetchBitsBk(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return 0;

        if (digit == 3) _mDecimals = bp.bits;
        else if (digit == 2) _mUnits = bp.bits;
        else _mUnits += bp.bits * 10;

        t -= bp.timings;
        ones += ONES_COUNT[bp.bits];
    }

    // Fetch parity
    _fuzzy = false;
    bits_pos bp = getBitBk(t);
    if (bp.timings == 0) {
        _fuzzy = true;
        bp = getBitBk(t);
        if (bp.timings == 0) return 0; // Unable to decode parity bit
    }
    t -= bp.timings;
    _parity = bp.bits;
    ones += _parity;
    if (ones % 2 == 1) return 0; // Parity mismatch

    return timingPos - t;
}

size_t Timings2Measure::fetchMeasureRepBk(size_t timingPos, bool ungreedy)
{
    size_t t = timingPos;
    // Fetch measure digits
    for (uint8_t digit = 0; digit < 2; digit++) {
        bits_pos bp = fetchBitsBk(t, 4, ungreedy);
        // "Unable to decode a bit inside the measure" or "Wrong measure digit"
        if (bp.timings == 0 || bp.bits > 9) return 0;

        if (digit == 0) _m2Units = bp.bits;
        else _m2Units += bp.bits * 10;

        t -= bp.timings;
    }
    return timingPos - t;
}

bool Timings2Measure::readBackward()
{
    size_t t = _timings->size - 2;
    uint32_t tt = longShortTiming(_timings->getTiming(t));
    if (tt == 0) return false; // "Unable to decode last bit";
    uint8_t checksum = (tt == PW_LONG)? 0 : 1;

    bits_pos bp = fetchBitsBk(t - 1, 3);
    if (bp.timings == 0) return false; // "Unable to decode a bit inside checksum"
    checksum |= (bp.bits << 1);
    t -= (bp.timings + 1);

    // Fetch repeated measure
    size_t nTimings = fetchMeasureRepBk(t);
    if (nTimings == 0) {
        nTimings = fetchMeasureRepBk(t, true);
        if (nTimings == 0) {
            _m2Units = 0xFF;
            return false;
        }
    }
    t -= nTimings;

    // Fetch measure and parity
    nTimings = fetchMeasureBk(t);
    if (nTimings == 0) {
        nTimings = fetchMeasureBk(t, true);
        if (nTimings == 0) {
            _mUnits = 0xFF;
            return false;
        }
    }
    t -= nTimings;

    if (_m2Units != _mUnits) return false; // Measures don't match!

    // Check if there are enough timings for sensor id (7 bit) and measure type (4 bit)
    if (t < 22) return false; // "Not enough timings to decode sensor address and measure type";

    // Fetch sensor id
    bp = fetchBitsBk(t, 7);
    if (bp.timings == 0) return false; // "Cannot decode a bit inside sensor addr";
    _sensorId = (uint8_t) bp.bits;
    t -= bp.timings;

//    // Check if there are enough timings for measure type (4 bit)
//    if (t < 8) return false; // "Not enough timings to decode measure type";

    // Fetch measure type
    bp = fetchBitsBk(t, 4);
    if (bp.timings == 0 || (bp.bits != 0x0 && bp.bits != 0xE)) // Measure type should be 0000 or 1110
        bp = fetchBitsBk(t, 4, true);

    if (bp.timings == 0) return false ;// "Cannot decode a bit inside measure type";
    if (bp.bits == 0x0) _mType = TEMPERATURE;
    else if (bp.bits == 0xE) _mType = HUMIDITY;
    else return false; //"Wrong measure type";
    _tHeader = t - bp.timings + 1;

    // Check checksum
    return _ignoreChecksum || (checksum == getChecksum());
}

measure Timings2Measure::getMeasure(timings_packet* pk)
{
    _mType = UNKNOWN;
    _sensorId = 0xFF;
    _mUnits = 0xFF;
    _m2Units = 0xFF;

    _timings = pk;
    measure m = { pk->msec, 0, UNKNOWN, 0, 0 };

    // Excludes packets with more than 12 initial bits missing (header and sensor type)
    // If there are more than 12 bits missing, we cannot identify sensor address because
    // t start at the 13th bit.
    // So the minimum number of bits is 44 - 12 = 32, that is 64 timings
    if (_timings->size < 64) return m;

    if (readForward() || readBackward() ||
        (_ignoreChecksum && _sensorId != 0xFF && _mUnits != 0xFF && _mType != UNKNOWN)) {
        // I've found that less than 10% of measures with wrong checksum are
        // effectively wrong. So it can be better to accept them and make a
        // secondary double check after.
        m.sensorAddr = _sensorId;
        m.type = _mType;
        m.units = _mUnits;
        m.decimals = _mDecimals;

        // For temperature decrease the value by 50 (beware of negative values!)
        if (_mType == TEMPERATURE) {
            m.units -= 50;
            if (m.units < 0 && m.decimals > 0) {
                m.units++;
                m.decimals = 10 - m.decimals;
            }
        }
    }
    return m;
}
