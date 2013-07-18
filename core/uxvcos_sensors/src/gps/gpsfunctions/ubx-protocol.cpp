//
// -*- c++ -*-
//
// $Id: ubx-protocol.cpp, v 0.2 2007/03/27 10:00:00 Schmidt-Bruecken Exp $
//

#include "ubx-protocol.h"

TUBX_PROTOCOL::TUBX_PROTOCOL(void)
{
}

TUBX_PROTOCOL::~TUBX_PROTOCOL(void)
{
}

void TUBX_PROTOCOL::ResetChk(void)
{
    Chk_A = 0;
    Chk_B = 0;
}


void TUBX_PROTOCOL::Add2Chk(unsigned char pData)
{
    Chk_A += pData;
    Chk_B += Chk_A;
}


signed char TUBX_PROTOCOL::CheckForMsg(unsigned char  *pData,  unsigned short pMaxSize,
                                       unsigned char   pSync1, unsigned char  pSync2,
                                       unsigned char  *pClass, unsigned char *pId,
                                       unsigned short *pLength)
{
    unsigned short i;                                 // Counter

    *pClass = 0;
    *pId    = 0;
    *pLength= 0;
    // check for uBlox header
    if ((pMaxSize >= 6 + 2)
            && (*(pData) == pSync1)
            && (*(pData+1) == pSync2))
    {
        // length of msg is little endian
        *pLength = *(pData+4);
        if (*(pData+5) > 0)
            *pLength += (*(pData+5) << 8);

        if (*pLength+8 <= pMaxSize)
        {
            // calculate the checksum of this msg
            Chk_A = 0;
            Chk_B = 0;
            for (i=2; i < *pLength+6; i++)
            {
                Chk_A += *(pData+i);
                Chk_B += Chk_A;
            }
//        my_printf("CheckForMsg: ChkA:%x  Chk_B:%x\n", Chk_A, Chk_B);
            // compare the calculated checksum and the one in the msg
            if ((*(pData+7-1+*pLength) == Chk_A)
                    && (*(pData+8-1+*pLength) == Chk_B))
            {
                *pClass = *(pData+2);
                *pId    = *(pData+3);
//          my_printf("CheckForMsg: uBlox Class:%x  ID:%x  total length:%d bytes\n", *pClass, *pId, *pLength);
                return TRUE;
            }
            else
                // checksum error
                return -1;
        }
    }
    // no Ubx msg or error
    return FALSE;
}


signed char TUBX_PROTOCOL::CheckForMsg(unsigned char  *pData,  unsigned int   pMaxSize,
                                       unsigned char   pSync1, unsigned char  pSync2,
                                       unsigned char  *pClass, unsigned char *pId,
                                       unsigned int   *pLength)
{
    unsigned int i;                                 // Counter

    *pClass = 0;
    *pId    = 0;
    *pLength= 0;
    // check for uBlox header
    if ((pMaxSize >= 6 + 2)
            && (*(pData) == pSync1)
            && (*(pData+1) == pSync2))
    {
        // length of msg is little endian
        *pLength = *(pData+4);
        if (*(pData+5) > 0)
            *pLength += (*(pData+5) << 8);

        if (*pLength+8 <= pMaxSize)
        {
            // calculate the checksum of this msg
            Chk_A = 0;
            Chk_B = 0;
            for (i=2; i < *pLength+6; i++)
            {
                Chk_A += *(pData+i);
                Chk_B += Chk_A;
            }
//        my_printf("CheckForMsg: ChkA:%x  Chk_B:%x\n", Chk_A, Chk_B);
            // compare the calculated checksum and the one in the msg
            if ((*(pData+7-1+*pLength) == Chk_A)
                    && (*(pData+8-1+*pLength) == Chk_B))
            {
                *pClass = *(pData+2);
                *pId    = *(pData+3);
//          my_printf("CheckForMsg: uBlox Class:%x  ID:%x  total length:%d bytes\n", *pClass, *pId, *pLength);
                return TRUE;
            }
            else
                // checksum error
                return -1;
        }
    }
    // no Ubx msg or error
    return FALSE;
}


unsigned short TUBX_PROTOCOL::EncodeMsg(unsigned char *pDestData, unsigned short pDestMaxSize,
                                        unsigned char  pSync1,    unsigned char  pSync2,
                                        unsigned char *pPayload,  unsigned short pPayloadSize,
                                        unsigned char  pClass,    unsigned char  pId)
{
    unsigned short i,ii;                            // Counter

    if (pDestMaxSize < 2+2+2+pPayloadSize+2)
        return 0;

    i = 0;
    // copy header
    *(pDestData + i++) = pSync1;
    *(pDestData + i++) = pSync2;
    // reset checksum
    Chk_A = 0;
    Chk_B = 0;
    // copy Class / Id
    *(pDestData + i++) = pClass;
    Add2Chk(pClass);
    *(pDestData + i++) = pId;
    Add2Chk(pId);
    // copy payloadlength
    ii = pPayloadSize & 0xFF;             // low byte
    *(pDestData + i++) = (unsigned char)ii;
    Add2Chk((unsigned char)ii);
    ii = pPayloadSize >> 8;               // high byte
    *(pDestData + i++) = (unsigned char)ii;
    Add2Chk((unsigned char)ii);
    // copy payload
    for (ii=0; ii < pPayloadSize; ii++)
    {
        *(pDestData + i++) = *(pPayload+ii);
        Add2Chk(*(pPayload+ii));
    }

    // copy checksum
    *(pDestData + i++) = Chk_A;
    *(pDestData + i++) = Chk_B;

    // return length
    return i;
}
