#include "utils.h"

float Log(float x) {
	// Trap illegal values
	if (x <= 0) {
		return 0.0;
	}

	// Confine x to a sensible range
	int power_adjust = 0;
	while (x > 1.0) {
		x /= EULER_CONST;
		power_adjust++;
	}
	while (x < .25) {
		x *= EULER_CONST;
		power_adjust--;
	}

	// Now use the Taylor series to calculate the logarithm
	x -= 1.0;
	float t = 0.0, s = 1.0, z = x;
	for (int k = 1; k <= TAYLOR_ITERATIONS; k++) {
		t += z * s / k;
		z *= x;
		s = -s;
	}

	// Combine the result with the power_adjust value and return
	return t + power_adjust;
}

int32_t Round(float number) {
	// Get the integer part by truncating the float
	int int_part = (int)number;
	float fractional_part = number - int_part;
	if (fractional_part >= 0.5) {
		return int_part + 1;
	} else {
		return int_part;
	}
}

uint32_t convert_buffer_to_uint32(const uint8_t* buffer, const uint8_t size) {
	uint32_t result = 0;
	for (int i = 0; i < size; i++) {
		result <<= 8;
		result |= buffer[i];
	}
	return result;
}

// time difference in seconds, handles midnight crossing
int16_t timeDifference(const uint32_t previousTime, const uint32_t currentTime) {
	if (previousTime < currentTime) return (int16_t)(currentTime - previousTime);

	return (int16_t)(24 * 60 * 60 - (previousTime - currentTime));
}

// ascent rate in cm/s
int16_t calculateAscentRate(const uint16_t previousAlt, const uint16_t currentAlt, const uint32_t previousTime, const uint32_t currentTime) {
	const int16_t altDiff = (int16_t)(currentAlt - previousAlt);
	const int16_t timeDiff = timeDifference(previousTime, currentTime);

	return (int16_t)Round((float)altDiff / timeDiff);
}

long long Floor(double num) { // Used only for positive values
    if (num >= 0) {
        return (long long)num; // Truncates positive numbers down
    } else {
        // Handle negative numbers separately if needed, or rely on the logic below
        return (long long)num; // For negative, this truncates towards zero (e.g., -4.7 becomes -4)
    }
}

uint16_t crc16(char* string, uint8_t len) {
	uint16_t crc = 0xffff;
	char i;
	uint8_t ptr = 0;
	while (ptr < len) {
		ptr++;
		crc = crc ^ (*(string++) << 8);
		for (i = 0; i < 8; i++) {
			if (crc & 0x8000)
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			else
				crc <<= 1;
		}
	}
	return crc;
}
uint16_t a_strtof(char* buffer) {
	uint8_t d_pos = 0;
	uint16_t value = 0;

	while (buffer[d_pos] != '.') {
		if (buffer[d_pos] == '\0') return 0;
		d_pos++;
	}
	uint16_t e = 1;
	for (int8_t pos = d_pos - 1; pos >= 0; pos--) {
		value += (buffer[pos] - '0') * e;
		e *= 10;
	}
	if ((buffer[d_pos + 1] - '0') >= 5) value++; // rounding first decimal place

	return value;
}

// From GNU GlibC
void * Memcpy (void *dest, const void *src, size_t len)
{
  char *d = dest;
  const char *s = src;
  while (len--)
    *d++ = *s++;
  return dest;
}

void * Memset (void *dest, int val, size_t len)
{
  unsigned char *ptr = dest;
  while (len-- > 0)
    *ptr++ = val;
  return dest;
}

int Memcmp (const void *str1, const void *str2, size_t count)
{
  const unsigned char *s1 = str1;
  const unsigned char *s2 = str2;

  while (count-- > 0)
    {
      if (*s1++ != *s2++)
	  return s1[-1] < s2[-1] ? -1 : 1;
    }
  return 0;
}

size_t Strlen(const char *str)
{
        const char *s;

        for (s = str; *s; ++s)
                ;
        return (s - str);
}

char * Strchr (register const char *s, int c)
{
  do {
    if (*s == c)
      {
	return (char*)s;
      }
  } while (*s++);
  return (0);
}
void Strncpy( char* _dst, const char* _src, size_t _n )
{
   size_t i = 0;
   while(i++ != _n && (*_dst++ = *_src++));
}
char *Strstr(const char *s1, const char *s2)
{
    size_t n = Strlen(s2);
    while(*s1)
        if(!Memcmp(s1++,s2,n))
            return (char *) (s1-1);
    return 0;
}
size_t Strspn(const char *str, const char *chars){
    unsigned char ta[32]={0};
    size_t i;
    for(i=0;chars[i];++i)
        ta[chars[i]>>3]|=0x1<<(chars[i]%8);
    for(i=0;((ta[str[i]>>3]>>(str[i]%8))&0x1);++i);
    return i;
}
char * Strpbrk (const char *s, const char *accept)
{
  while (*s != '\0')
    {
      const char *a = accept;
      while (*a != '\0')
	if (*a++ == *s)
	 return (char *) s;
      ++s;
    }

  return NULL;
}
char *Strtok(char *s, const char *delim)
{
    static char *olds;
    char *token;

    if (s == NULL)
        s = olds;

    /* Scan leading delimiters.  */
    s += Strspn(s, delim);

    /* if *s points to the null byte \0, that means
        we have reached the end of the string and
        we return NULL
    */
    if (*s == '\0')
    {
        olds = s;
        return (NULL);
    }

    /* Find the end of the token.  */
    token = s;
    s = Strpbrk(token, delim);
    if (s == NULL)
        /* This token finishes the string.  */
        olds = Strchr(token, '\0');
    else
    {
        /* Terminate the token and make OLDS point past it.  */
        *s = '\0';
        olds = s + 1;
    }
    return (token);
}