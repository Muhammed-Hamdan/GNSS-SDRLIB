/*------------------------------------------------------------------------------
* sdrnav_gal.c : Galileo navigation data
*
* Copyright (C) 2014 Taro Suzuki <gnsssdrlib@gmail.com>
*-----------------------------------------------------------------------------*/
#include "sdr.h"

#define P2P11       2048.0                /* 2^11 */
#define P2P12       4096.0                /* 2^12 */
#define P2P14       16384.0               /* 2^14 */
#define P2P15       32768.0               /* 2^15 */
#define P2P16       65536.0               /* 2^16 */
#define P2_51       4.440892098500626E-16 /* 2^-51 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606881E-20 /* 2^-66 */
#define P2_68       3.388131789017201E-21 /* 2^-68 */
#define P2_28       3.725290298461914E-09 /* 2^-28 */
#define P2_41       4.547473508864641E-13 /* 2^-41 */

/* decode IRNSS navigation data (subframe 1) -------------------------------
*
* args   : uint8_t  *buff   I   navigation data bits
*          sdreph_t *eph    I/O sdr ephemeris structure
* return : none
*-----------------------------------------------------------------------------*/
void decode_subframe1(const uint8_t *buff, sdreph_t *eph)
{
    return;
}

/* decode IRNSS navigation data (subframe 2) -------------------------------
*
* args   : uint8_t  *buff   I   navigation data bits
*          sdreph_t *eph    I/O sdr ephemeris structure
* return : none
*-----------------------------------------------------------------------------*/
void decode_subframe2(const uint8_t *buff, sdreph_t *eph)
{
    return;
}

/* decode IRNSS navigation data (subframe 3) -------------------------------
*
* args   : uint8_t  *buff   I   navigation data bits
*          sdreph_t *eph    I/O sdr ephemeris structure
* return : none
*-----------------------------------------------------------------------------*/
void decode_subframe3(const uint8_t *buff, sdreph_t *eph)
{
    return;
}

/* decode IRNSS navigation data (subframe 4) -------------------------------
*
* args   : uint8_t  *buff   I   navigation data bits
*          sdreph_t *eph    I/O sdr ephemeris structure
* return : none
*-----------------------------------------------------------------------------*/
void decode_subframe4(const uint8_t *buff, sdreph_t *eph)
{
    return;
}

/* check IRNSS L5/S CRC -------------------------------------------------------
* compute and check CRC of IRNSS L5/S subframe data
* args   : uint8_t  *data  I   IRNSS subframe (35 bytes + 6 bits (286 bits))
* return : int                 1:okay 0: wrong parity
*-----------------------------------------------------------------------------*/
extern int checkcrc_iss(uint8_t *data)
{
    uint8_t crcbins[36]={0};
    int i,j,crcbits[286],crc,crcmsg;

    /* Can optimize right alignment procedure */
    for (i=0;i<36;i++) {
        for (j=0;j<8;j++) {
            if (8*i+j==286) break;
            crcbits[8*i+j]=-2*(((data[i]<<j)&0x80)>>7)+1;
        }
    }
    bits2byte(crcbits,286,36,1,crcbins); /* right alignment for crc */
    crc=rtk_crc24q(crcbins,33); /* compute crc24 */
    crcmsg=getbitu(data,262,24); /* crc in message */
    
    /* crc matching */
    if (crc==crcmsg)  return 0;
    else return -1;
}
/* decode navigation data (IRNSS subframe) -----------------------------------
*
* args   : uint8_t *buffsub   I   navigation data bits (subframe)
*          sdreph_t *eph      I/O sdr ephemeris structure
* return : int                  word type
*-----------------------------------------------------------------------------*/
extern int decode_subframe_iss(const uint8_t *buffsub, sdreph_t *eph)
{
    int id;
    /* buff is 286 bits (35 bytes + 6 bits) corresponding to one subframe */
    /* see IRNSS SPSICD Figure 12, pp. 16 */
    uint8_t buff[36];
    memcpy(buff,buffsub,36);
    
    id=getbitu(buff,27,2); /* word type */
    switch (id) {
    case 0: decode_subframe1(buff,eph); break;
    case 1: decode_subframe2(buff,eph); break;
    case 2: decode_subframe3(buff,eph); break;
    case 3: decode_subframe4(buff,eph); break;
    }
    return id;
}
/* decode IRNSS navigation data ----------------------------------------------
* decode IRNSS (I/NAV) navigation data and extract ephemeris
* args   : sdrnav_t *nav    I/O sdr navigation struct
* return : int                  word type
*-----------------------------------------------------------------------------*/
extern int decode_iss(sdrnav_t *nav)
{
    int i,id=0,bits[600],bits_iss[584];
    uint8_t enc_iss[584],dec_iss[36];

    /* copy navigation bits (600 symbols in 1 subframe) */
    for (i=0;i<nav->flen;i++) bits[i]=nav->polarity*nav->fbits[i];

    /* initialize viterbi decoder */
    init_viterbi27_port(nav->fec,0);
        
    /* deinterleave (73 rows x 8 columns) see IRNSS SPSICD Table 9, pp. 14 */
    interleave(&bits[NAVPRELEN_ISS],73,8,bits_iss);

    /* copy subframe (exclude preamble) */
    for (i=0;i<584;i++) {
        enc_iss[i] = (bits_iss[i]==1)?0:255;
    }

    /* decode subframe symbols to bits */
    update_viterbi27_blk_port(nav->fec,enc_iss,292);
    chainback_viterbi27_port(nav->fec,dec_iss,292-6,0);

    /* CRC sheck */
    if (checkcrc_iss(dec_iss)<0) {
        SDRPRINTF("error: ISS CRC mismatch\n");
        return -1;
    } else {
        /* decode navigation data */
        id=decode_subframe_iss(dec_iss,&nav->sdreph);
    }
    return id;
}
