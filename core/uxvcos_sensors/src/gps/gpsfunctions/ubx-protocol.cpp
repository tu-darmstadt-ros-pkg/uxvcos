//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
