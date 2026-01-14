#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include <stddef.h>

#define EULER_CONST 2.718281828459045235
#define TAYLOR_ITERATIONS 20

#define PI 3.14159265358979323846f

float Log(float x);
int32_t Round(float number);
uint32_t convert_buffer_to_uint32(const uint8_t* buffer, uint8_t size);
int16_t timeDifference(uint32_t previousTime, uint32_t currentTime);
int16_t calculateAscentRate(uint16_t previousAlt, uint16_t currentAlt, uint32_t previousTime, uint32_t currentTime);
long long Floor(double num);
uint16_t crc16(char* string, uint8_t len);
void * Memset (void *dest, int val, size_t len);
void * Memcpy (void *dest, const void *src, size_t len);
int Memcmp (const void *str1, const void *str2, size_t count);
size_t Strlen(const char *str);
char * Strchr (register const char *s, int c);
uint16_t a_strtof(char* buffer);
void Strncpy( char* _dst, const char* _src, size_t _n );
char *Strstr(const char *s1, const char *s2);
size_t Strspn(const char *str, const char *chars);
char *Strtok(char *s, const char *delim);
char * Strpbrk(const char *s, const char *accept);

#endif
