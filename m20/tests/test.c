#include <stdio.h>
#include <stdint.h>
#include "gps.h"
#include "xm_gps.h"
#include "utils.h"

int tests_passed = 0;
int tests_failed = 0;

#define TEST_ASSERT(condition, message) \
    if (condition) { \
        printf("\033[32m✓ PASS\033[0m: %s\n", message); \
        tests_passed++; \
    } else { \
        printf("\033[31m✗ FAIL\033[0m: %s\n", message); \
        tests_failed++; \
    }

void test_convert_buffer_to_uint32_basic() {
    uint8_t buffer[] = {0x12, 0x34, 0x56, 0x78};
    uint32_t result = convert_buffer_to_uint32(buffer, 4);
    TEST_ASSERT(result == 0x12345678, "convert_buffer_to_uint32 basic test: {0x12, 0x34, 0x56, 0x78} -> 0x12345678");
}

void test_convert_buffer_to_uint32_single_byte() {
    uint8_t buffer[] = {0xAB};
    uint32_t result = convert_buffer_to_uint32(buffer, 1);
    TEST_ASSERT(result == 0xAB, "convert_buffer_to_uint32 single byte test");
}

void test_convert_buffer_to_uint32_two_bytes() {
    uint8_t buffer[] = {0x12, 0x34};
    uint32_t result = convert_buffer_to_uint32(buffer, 2);
    TEST_ASSERT(result == 0x1234, "convert_buffer_to_uint32 two bytes test");
}

void test_calculateAscentRate_basic() {
    uint32_t t1, t2;
    t1 = 12 * 3600 + 30 * 60 + 30;
    t2 = 12 * 3600 + 50 * 60 + 30; // t1 + 20 min
    TEST_ASSERT(calculateAscentRate(1000, 1100, t1, t2) == 8, "calculateAscentRate basic");
    t1 = 23 * 3600 + 50 * 60 + 30;
    t2 = 00 * 3600 + 10 * 60 + 30; // t1 + 20 min
    TEST_ASSERT(calculateAscentRate(1000, 1100, t1, t2) == 8, "calculateAscentRate basic cross midnight");
    TEST_ASSERT(calculateAscentRate(1100, 1000, t1, t2) == -8, "calculateAscentRate basic descent");
}

void test_parseXMframe() {
    uint8_t buffer[] = {
        0xAA, 0xAA, 0xAA, 0x03, // preambule
        0x03,                   // fix
        0x03, 0x40, 0x58, 0x28, // latitude => 54.548519
        0x01, 0x1A, 0xFA, 0x13, // longitude => 18.545172
        0x00, 0x1A, 0x31,       // alt => 67
        0xFF, 0xE6,             // lat dir
        0xFF, 0xBC,             // lon dir
        0x00, 0x01,             // alt dir
        0x07, 0xAE, 0xD0,       // gps time => 19:51:44
        0x01,                   // ?
        0x57,                   // ?
        0x12,                   // 0x12
        0x11, 0x16, 0x22, 0x28, 0x1C, 0x24, 0x20, 0x20, 0x14, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sats
        0x04, 0x05, 0x10, 0x12, 0x15, 0x19, 0x1A, 0x1C, 0x1D, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sats
        0x24, 0x1C              // checksum
    };
    GPS gps;
    parseXM(&gps, buffer);
    TEST_ASSERT(gps.Fix == 3, "ParseXM Fix");
    TEST_ASSERT(gps.Lat > 54.54 && gps.Lat < 54.55, "ParseXM Lat");
    TEST_ASSERT(gps.Lon > 18.54 && gps.Lon < 18.55, "ParseXM Lon");
    TEST_ASSERT(gps.Alt == 67, "ParseXM Alt");
    TEST_ASSERT(gps.Hours == 19, "ParseXM Hours");
    TEST_ASSERT(gps.Minutes == 51, "ParseXM Minutes");
    TEST_ASSERT(gps.Seconds == 44, "ParseXM Seconds");
    TEST_ASSERT(gps.AscentRate == 0, "ParseXM AscentRate"); // ???
    TEST_ASSERT(gps.Sats == 10, "ParseXM Sats");
}

int main() {
    printf("Running tests...\n\n");

    test_convert_buffer_to_uint32_basic();
    test_convert_buffer_to_uint32_single_byte();
    test_convert_buffer_to_uint32_two_bytes();
    test_calculateAscentRate_basic();
    test_parseXMframe();

    printf("\nTests passed: %d, failed %d\n", tests_passed, tests_failed);

    return tests_failed;
}
