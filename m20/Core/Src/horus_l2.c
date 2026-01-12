/*
 *  horus.c
 *  Based on https://github.com/whallmann/RS41HUP_V2/blob/master/horus_l2.c
 *  Adapted by SQ2IPS
 */

#include "horus_l2.h"
#include "utils.h"

#define X22             0x00400000   /* vector representation of X^{22} */
#define X11             0x00000800   /* vector representation of X^{11} */
#define MASK12          0xfffff800   /* auxiliary vector for testing */
#define GENPOL          0x00000c75   /* generator polinomial, g(x) */

#define INTERLEAVER
#define SCRAMBLER

static char uw[] = {'$','$'};

/* Function Prototypes ------------------------------------------------*/

int32_t get_syndrome(int32_t pattern);
void golay23_init(void);
int golay23_decode(int received_codeword);
unsigned short gen_crc16(unsigned char* data_p, unsigned char length);
void interleave(unsigned char *inout, int nbytes, int dir);
void scramble(unsigned char *inout, int nbytes);

/* Functions ----------------------------------------------------------*/

/*
   We are using a Golay (23,12) code which has a codeword 23 bits
   long.  The tx packet format is:

      | Unique Word | payload data bits | parity bits |

   This function works out how much storage the caller of
   horus_l2_encode_tx_packet() will need to store the tx packet
 */

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


/*
  Takes an array of payload data bytes, prepends a unique word and appends
  parity bits.

  The encoder will run on the payload on a small 8-bit uC.  As we are
  memory constrained so we do a lot of burrowing for bits out of
  packed arrays, and don't use a LUT for Golay encoding.  Hopefully it
  will run fast enough.  This was quite difficult to get going,
  suspect there is a better way to write this.  Oh well, have to start
  somewhere.
 */

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
    Memcpy(pout, uw, sizeof(uw)); pout += sizeof(uw);
    Memcpy(pout, input_payload_data, num_payload_data_bytes); pout += num_payload_data_bytes;

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

#ifdef INTERLEAVER

// from https://github.com/drowe67/codec2/blob/96e8a19c2487fd83bd981ce570f257aef42618f9/src/gp_interleaver.c#L39-L40
int is_prime(int x) {
  for (int i = 2; i < x; i++) {
    if ((x % i) == 0) return 0;
  }
  return 1;
}

int next_prime(int x) {
  x++;
  while (is_prime(x) == 0) x++;
  return x;
}

int choose_interleaver_b(int Nbits) {
  int b = Floor(Nbits / 1.62);
  b = next_prime(b);
  return b;
}

void interleave(unsigned char *inout, int nbytes, int dir)
{   
    /* note: to work on small uCs (e.g. AVR) needed to declare specific words sizes */
    uint16_t nbits = (uint16_t)nbytes*8;
    uint32_t i, j, n, ibit, ibyte, ishift, jbyte, jshift;
    uint32_t b;
    unsigned char out[nbytes];

    Memset(out, 0, nbytes);

    switch(nbytes)
    {
        case 43: // horus v1
            b = 337;
            break;
        case 63: // horus v2
            b = 389;
            break;
        default: // everything else (including horus v3)
            b = choose_interleaver_b(nbits);
    }

    // fprintf(stderr,"n: %d b: %d bits: %d\n",nbytes, b, nbits);


    for(n=0; n<nbits; n++) {

        /*
          "On the Analysis and Design of Good Algebraic Interleavers", Xie et al,eq (5)
        */

        i = n;
        j = (b*i) % nbits; /* note these all need to be 32-bit ints to make multiply work without overflow */
        
        if (dir) {
            uint16_t tmp = j;
            j = i;
            i = tmp;
        }
        
        #ifdef DEBUG0
        printf("i: %d j: %d\n",i, j);
        #endif

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
 
    Memcpy(inout, out, nbytes);

    #ifdef DEBUG0
    printf("\nInterleaver Out:\n");
    for (i=0; i<nbytes; i++)
        printf("%02d 0x%02x\n", i, inout[i]);
    #endif
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

