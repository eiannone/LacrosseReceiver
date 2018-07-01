#include <unity.h>
#include <iostream>
#include <cstring>
#include "Timings2Measure.h"

struct packet : timings_packet {
    uint32_t timings[200];
    uint32_t peekTiming(size_t pos) {
        return (pos >= 0 && pos < size)? timings[pos] : 0xFFFFFFFF;
    }
};

inline const char* mTypeToStr(measureType mType)
{
    switch (mType) {
        case TEMPERATURE: return "TMP";
        case HUMIDITY:    return "HUM";
        default:          return "???";
    }
}

void test_timings2measure(void) {
    int nTests, nTimings, units, sensorAddr, decimals;
    char mType[4], msgBuf[100];
    unsigned long msec;
    Timings2Measure t2m;

    freopen("test/desktop/test_Timings2Measure.dat", "r", stdin);
    std::cin >> nTests;
    printf("N. misure di test: %d\r\n", nTests);
    for(int t = 0; t < nTests; t++) {
        scanf("%lu %d %d.%d %d %s", &msec, &nTimings, &units, &decimals, &sensorAddr, mType);
        packet pk;
        pk.msec = (uint32_t) msec;
        pk.size = (uint32_t) nTimings;
        for(uint32_t tm = 0; tm < pk.size; tm++) scanf("%u", &pk.timings[tm]);

        measure m = t2m.getMeasure(&pk);
        sprintf(msgBuf, "Msec %u: unita'.", m.msec);
        TEST_ASSERT_EQUAL_INT_MESSAGE(units, m.units, msgBuf);//,
        sprintf(msgBuf, "Msec %u: decimali'.", m.msec);
        TEST_ASSERT_EQUAL_INT_MESSAGE(decimals, m.decimals, "Decimali");

        // printf("%d (%d): %d %s %d.%d\n", m.msec, pk.size, m.sensorAddr,
        //     mTypeToStr(m.type), m.units, m.decimals);
    }
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_timings2measure);
    UNITY_END();
}
