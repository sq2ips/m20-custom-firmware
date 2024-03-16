
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/


void ADF_WriteReg(uint32_t val);
void ADF_Reset(void);
void myspi(uint32_t data);
uint32_t ADF_setfreq(float freq, float fPFD, uint8_t prescaler);

