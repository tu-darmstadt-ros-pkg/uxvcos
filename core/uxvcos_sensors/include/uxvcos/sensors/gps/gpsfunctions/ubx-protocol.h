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
