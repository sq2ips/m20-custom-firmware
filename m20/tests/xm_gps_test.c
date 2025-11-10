#include <stdio.h>
#include <stdint.h>

extern uint32_t changeBytesOrder(const uint8_t *buffer, const uint8_t size);
extern int16_t calculateAscentRate(uint16_t alt1, uint16_t alt2, uint32_t time1, uint32_t time2);

int tests_passed = 0;
int tests_failed = 0;

#define TEST_ASSERT(condition, message) \
    if (condition) { \
        printf("✓ PASS: %s\n", message); \
        tests_passed++; \
    } else { \
        printf("✗ FAIL: %s\n", message); \
        tests_failed++; \
    }

void test_changeBytesOrder_basic() {
    uint8_t buffer[] = {0x12, 0x34, 0x56, 0x78};
    uint32_t result = changeBytesOrder(buffer, 4);
    TEST_ASSERT(result == 0x78563412, "changeBytesOrder basic test: {0x12, 0x34, 0x56, 0x78} -> 0x78563412");
}

void test_changeBytesOrder_single_byte() {
    uint8_t buffer[] = {0xAB};
    uint32_t result = changeBytesOrder(buffer, 1);
    TEST_ASSERT(result == 0xAB, "changeBytesOrder single byte test");
}

void test_changeBytesOrder_two_bytes() {
    uint8_t buffer[] = {0x12, 0x34};
    uint32_t result = changeBytesOrder(buffer, 2);
    TEST_ASSERT(result == 0x3412, "changeBytesOrder two bytes test");
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

int main() {
    printf("Running tests for changeBytesOrder...\n\n");

    test_changeBytesOrder_basic();
    test_changeBytesOrder_single_byte();
    test_changeBytesOrder_two_bytes();
    test_calculateAscentRate_basic();

    printf("\n===================\n");
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);
    printf("===================\n");

    return tests_failed;
}
