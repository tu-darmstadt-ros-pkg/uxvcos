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
// $Id: ubx-protocol.h,v 0.2 2007/03/27 10:00:00 Schmidt-Bruecken Exp $
//

//!
//! @file ubx-protocol.h
//!
//! @author Frank Schmidt-Bruecken
//!
//! @brief Some functions to handle the binary ubx protocol
//!
//! @section  OVERVIEW Overview
//!   These routines are for handling the ubx binary protocol
//!
//! @section  HISTORY  Document Revision History
//! @li @c  07/03/27 - Initial draft (FSB), TU Darmstadt FG Flugsysteme und Regelungstechnik
//! @li @c  08/04/21 - Enhancement for packets > 255 Bytes (FSB), TU Darmstadt FG Flugsysteme und Regelungstechnik
//!



#ifndef __UBX_PROTOCOL_H__
#define __UBX_PROTOCOL_H__


#define TRUE 1
#define true 1
#define FALSE 0
#define false 0

class TUBX_PROTOCOL
{
public:

    TUBX_PROTOCOL(void);
    virtual ~TUBX_PROTOCOL(void);

    //! @brief Check for ubx message. Just insert some data and this function
    //!	gives you an error or interesting information about the msg
    //!
    //! @param pData    Starting point of Data
    //! @param pMaxSize Maximum size a message could be
    //! @param pSync1   first sync byte
    //! @param pSync2   second sync byte
    //! @param pClass   Return value: Message class
    //! @param pId      Return value: Message id
    //! @param pLength  Return value: Message payload length
    //!
    //! @return Errormessage
    //!   @li TRUE      decoding successfull, return values valid
    //!   @li FALSE     no msg available
    //!   @li -1        msg chk error
    signed char CheckForMsg(unsigned char  *pData,  unsigned int    pMaxSize,
                            unsigned char   pSync1, unsigned char   pSync2,
                            unsigned char  *pClass, unsigned char  *pId,
                            unsigned int   *pLength);
    signed char CheckForMsg(unsigned char  *pData,  unsigned short  pMaxSize,
                            unsigned char   pSync1, unsigned char   pSync2,
                            unsigned char  *pClass, unsigned char  *pId,
                            unsigned short *pLength);

    //! @brief Encode an ubx message.
    //!
    //! @param pDestData    Place to store uBlox message to
    //! @param pDestMaxSize Maximum size the message could be
    //! @param pSync1       first sync byte
    //! @param pSync2       second sync byte
    //! @param pPayload     Starting point of Payload data
    //! @param pPayloadSize Length of payload
    //! @param pClass       Use this Message class
    //! @param pId          Use this Message id
    //!
    //! @return Overall length of the ubx msg
    unsigned short EncodeMsg(unsigned char *pDestData, unsigned short pDestMaxSize,
                             unsigned char  Sync1,     unsigned char  pSync2,
                             unsigned char *pPayload,  unsigned short pPayloadSize,
                             unsigned char  pClass,    unsigned char  pId);

private:
    //! @brief Reset the ubx protocol checksum
    void ResetChk(void);

    //! @brief Add one byte to the checksum calculation
    void Add2Chk(unsigned char pData);

    unsigned char Chk_A;              //!< first byte of ubx checksum
    unsigned char Chk_B;              //!< second byte of ubx checksum

};


#endif
