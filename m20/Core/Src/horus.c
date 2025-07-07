/*
 *  horus.c
 *  Based on https://github.com/whallmann/RS41HUP_V2/blob/master/horus_l2.c
 *  Adapted by SQ2IPS
 */

#include "horus.h"
#include "config.h"
#include <assert.h>
#include <string.h>

const static char uw[] = {'$','$'};

#define X22             0x00400000   /* vector representation of X^{22} */
#define X11             0x00000800   /* vector representation of X^{11} */
#define MASK12          0xfffff800   /* auxiliary vector for testing */
#define GENPOL          0x00000c75   /* generator polinomial, g(x) */

uint16_t crc16(char *string, uint8_t len) {
  uint16_t crc = 0xffff;
  char i;
  uint8_t ptr = 0;
  while (ptr < len) {
    ptr++;
    crc = crc ^ (*(string++) << 8);
    for (i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (uint16_t) ((crc << 1) ^ 0x1021);
      else
        crc <<= 1;
    }
  }
  return crc;
}

#ifdef INTERLEAVER

const static uint16_t primes[] = {
    2,      3,      5,      7,      11,     13,     17,     19,     23,     29,
    31,     37,     41,     43,     47,     53,     59,     61,     67,     71,
    73,     79,     83,     89,     97,     101,    103,    107,    109,    113,
    127,    131,    137,    139,    149,    151,    157,    163,    167,    173,
    179,    181,    191,    193,    197,    199,    211,    223,    227,    229,
    233,    239,    241,    251,    257,    263,    269,    271,    277,    281,
    283,    293,    307,    311,    313,    317,    331,    337,    347,    349,
    379,    383,    389,    757,    761,    769,    773
};

void interleave(unsigned char *inout, int nbytes, int dir)
{
    uint16_t nbits = (uint16_t)nbytes*8;
    uint32_t i, j, n, ibit, ibyte, ishift, jbyte, jshift;
    uint32_t b;
    unsigned char out[nbytes];


    memset(out, 0, nbytes);

    /* b chosen to be co-prime with nbits, I'm cheating by just finding the
       nearest prime to nbits.  It also uses storage, is run on every call,
       and has an upper limit.  Oh Well, still seems to interleave OK. */
    i = 1;
    while ((primes[i] < nbits) && (i < 77))
        i++;
    b = primes[i-1];

    for(n=0; n<nbits; n++) {

        /*
          "On the Analysis and Design of Good Algebraic Interleavers", Xie et al,eq (5)
        */

        i = n;
        j = (b*i) % nbits;

        if (dir) {
            uint16_t tmp = j;
            j = i;
            i = tmp;
        }

        /* read bit i and write to bit j postion */

        ibyte = i/8;
        ishift = i%8;
        ibit = (inout[ibyte] >> ishift) & 0x1;

        jbyte = j/8;
        jshift = j%8;

        /* write jbit to ibit position */

        out[jbyte] |= ibit << jshift; // replace with i-th bit
        //out[ibyte] |= ibit << ishift; // replace with i-th bit
    }

    memcpy(inout, out, nbytes);

}
#endif

#ifdef SCRAMBLER

/* 16 bit DVB additive scrambler as per Wikpedia example */

void scramble(unsigned char *inout, int nbytes)
{
    int nbits = nbytes*8;
    int i, ibit, ibits, ibyte, ishift, mask;
    uint16_t scrambler = 0x4a80;  /* init additive scrambler at start of every frame */
    uint16_t scrambler_out;

    /* in place modification of each bit */

    for(i=0; i<nbits; i++) {

        scrambler_out = ((scrambler & 0x2) >> 1) ^ (scrambler & 0x1);

        /* modify i-th bit by xor-ing with scrambler output sequence */

        ibyte = i/8;
        ishift = i%8;
        ibit = (inout[ibyte] >> ishift) & 0x1;
        ibits = ibit ^ scrambler_out;                  // xor ibit with scrambler output

        mask = 1 << ishift;
        inout[ibyte] &= ~mask;                  // clear i-th bit
        inout[ibyte] |= ibits << ishift;         // set to scrambled value

        /* update scrambler */

        scrambler >>= 1;
        scrambler |= scrambler_out << 14;

    }
}
#endif

int32_t get_syndrome(int32_t pattern)
/*
 * Compute the syndrome corresponding to the given pattern, i.e., the
 * remainder after dividing the pattern (when considering it as the vector
 * representation of a polynomial) by the generator polynomial, GENPOL.
 * In the program this pattern has several meanings: (1) pattern = infomation
 * bits, when constructing the encoding table; (2) pattern = error pattern,
 * when constructing the decoding table; and (3) pattern = received vector, to
 * obtain its syndrome in decoding.
 */
{
    int32_t aux = X22;

    if (pattern >= X11)
       while (pattern & MASK12) {
           while (!(aux & pattern))
              aux = aux >> 1;
           pattern ^= (aux/X11) * GENPOL;
           }
    return(pattern);
}

int horus_l2_get_num_tx_data_bytes(int num_payload_data_bytes) {
    int num_payload_data_bits, num_golay_codewords;
    int num_tx_data_bits, num_tx_data_bytes;

    num_payload_data_bits = num_payload_data_bytes*8;
    num_golay_codewords = num_payload_data_bits/12;
    if (num_payload_data_bits % 12) /* round up to 12 bits, may mean some unused bits */
        num_golay_codewords++;

    num_tx_data_bits = sizeof(uw)*8 + num_payload_data_bits + num_golay_codewords*11;
    num_tx_data_bytes = num_tx_data_bits/8;
    if (num_tx_data_bits % 8) /* round up to nearest byte, may mean some unused bits */
        num_tx_data_bytes++;

    return num_tx_data_bytes;
}

int horus_l2_encode_tx_packet(unsigned char *output_tx_data,
                              unsigned char *input_payload_data,
                              int            num_payload_data_bytes)
{
    int            num_tx_data_bytes, num_payload_data_bits;
    unsigned char *pout = output_tx_data;
    int            ninbit, ningolay, nparitybits;
    int32_t        ingolay, paritybyte, inbit, golayparity;
    int            ninbyte, shift, golayparitybit, i;

    num_tx_data_bytes = horus_l2_get_num_tx_data_bytes(num_payload_data_bytes);
    memcpy(pout, uw, sizeof(uw)); pout += sizeof(uw);
    memcpy(pout, input_payload_data, num_payload_data_bytes); pout += num_payload_data_bytes;

    /* Read input bits one at a time.  Fill input Golay codeword.  Find output Golay codeword.
       Write this to parity bits.  Write parity bytes when we have 8 parity bits.  Bits are
       written MSB first. */

    num_payload_data_bits = num_payload_data_bytes*8;
    ninbit = 0;
    ingolay = 0;
    ningolay = 0;
    paritybyte = 0;
    nparitybits = 0;

    while (ninbit < num_payload_data_bits) {

        /* extract input data bit */

        ninbyte = ninbit/8;
        shift = 7 - (ninbit % 8);
        inbit = (input_payload_data[ninbyte] >> shift) & 0x1;
        ninbit++;

        /* build up input golay codeword */

        ingolay = ingolay | inbit;
        ningolay++;

        /* when we get 12 bits do a Golay encode */

        if (ningolay % 12) {
            ingolay <<= 1;
        }
        else {
            golayparity = get_syndrome(ingolay<<11);
            ingolay = 0;

            /* write parity bits to output data */

            for (i=0; i<11; i++) {
                golayparitybit = (golayparity >> (10-i)) & 0x1;
                paritybyte = paritybyte | golayparitybit;
                nparitybits++;
                if (nparitybits % 8) {
                   paritybyte <<= 1;
                }
                else {
                    /* OK we have a full byte ready */
                    *pout = paritybyte;
                    pout++;
                    paritybyte = 0;
                }
            }
        }
    } /* while(.... */


    /* Complete final Golay encode, we may have partially finished ingolay, paritybyte */

    if (ningolay % 12) {
        ingolay >>= 1;
        golayparity = get_syndrome(ingolay<<12);

        /* write parity bits to output data */

        for (i=0; i<11; i++) {
            golayparitybit = (golayparity >> (10 - i)) & 0x1;
            paritybyte = paritybyte | golayparitybit;
            nparitybits++;
            if (nparitybits % 8) {
                paritybyte <<= 1;
            }
            else {
                /* OK we have a full byte ready */
                *pout++ = (unsigned char)paritybyte;
                paritybyte = 0;
            }
        }
    }

    /* and final, partially complete, parity byte */

    if (nparitybits % 8) {
        paritybyte <<= 7 - (nparitybits % 8);  // use MS bits first
        *pout++ = (unsigned char)paritybyte;
    }

    assert(pout == (output_tx_data + num_tx_data_bytes));

    /* optional interleaver - we dont interleave UW */

    #ifdef INTERLEAVER
    interleave(&output_tx_data[sizeof(uw)], num_tx_data_bytes-2, 0);
    #endif

    /* optional scrambler to prevent long strings of the same symbol
       which upsets the modem - we dont scramble UW */

    #ifdef SCRAMBLER
    scramble(&output_tx_data[sizeof(uw)], num_tx_data_bytes-2);
    #endif

    return num_tx_data_bytes;
}

void print_hex(char *data, uint8_t length, char *tmp) // prints 8-bit data in hex
{
 uint8_t first;
 int j=0;
 for (uint8_t i=0; i<length; i++)
 {
   first = ((uint8_t)data[i] >> 4) | 48;
   if (first > 57) tmp[j] = first + (uint8_t)39;
   else tmp[j] = first ;
   j++;

   first = ((uint8_t)data[i] & 0x0F) | 48;
   if (first > 57) tmp[j] = first + (uint8_t)39;
   else tmp[j] = first;
   j++;
 }
 tmp[length*2] = '\n';
 tmp[length*2+1] = 0;
}
