/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author Ulrich Herrmann
 *
 *  \brief Implementation of Topaz aka NFC type 1 tag
 *
 */
/*!
 * 
 */

#ifndef TOPAZ_H
#define TOPAZ_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define TOPAZ_UID_LENGTH 4
#define TOPAZ_HR_LENGTH 2

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/
/*!< 
 * struct representing an TOPAZ PICC as returned by
 * #topazReqaWupa().
 */
typedef struct
{
    u8 hr[TOPAZ_HR_LENGTH]; /*<! UID of the PICC */
    u8 uid[TOPAZ_UID_LENGTH]; /*<! UID of the PICC */
    u16 actlength; /*!< actual UID length */
    u8 atqa[2]; /*!< content of answer to request byte */
    bool_t collision; /*!< TRUE, if there was a collision which has been resolved,
                        otherwise no collision occured */
}topazProximityCard_t;

/*! 
 * PCD command set.
 */
typedef enum
{
    TOPAZ_CMD_REQA    = 0x26, /*!< command REQA */
    TOPAZ_CMD_WUPA    = 0x52, /*!< command WUPA */
    TOPAZ_CMD_RID     = 0x78, /*!< command Read UID */
    TOPAZ_CMD_RALL    = 0x00, /*!< command Read All */
    TOPAZ_CMD_READ    = 0x01, /*!< command Read */
    TOPAZ_CMD_WRITE_E = 0x53, /*!< command Write with erase */
    TOPAZ_CMD_WRITE_NE= 0x1a, /*!< command Write, no erase */
}topazCommand_t;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Initialize Topaz mode.
 *  \note This function needs to be called every time after switching
 *  from a different mode.
 *
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 topazInitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Deinitialize Topaz mode.
 *  \note This function should be called every time Topaz is not needed
 *  any more.
 *  \param keep_on: if true the RF field will not be switched off
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 topazDeinitialize(u8 keep_on);

/*! 
 *****************************************************************************
 *  \brief  Select a PICC and put it into READY state
 *
 *  This function sends to all PICCs in field either the REQA or the
 *  WUPA (depending on \a cmd parameter). This command puts the
 *  PICCs into READY state.
 *
 *  \param[in] cmd : Used command to put the PICCs in READY state. This
 *                   could either be #TOPAZ_CMD_REQA or #TOPAZ_CMD_WUPA
 *  \param[out] card : Parameter of type #topazProximityCard_t which holds
 *                the found card then.
 *
 *  \return ERR_TIMEOUT : Timeout waiting for interrupts or no reply from cards.
 *  \return ERR_PARAM : Parameter \a cmd not available/unvalid.
 *  \return ERR_NOTFOUND : No PICC could be selected..
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 topazReqaWupa(topazCommand_t cmd, topazProximityCard_t* card);

/*! 
 *****************************************************************************
 *  \brief  Read the UID from a tag in READY state
 *
 *
 *  \param[out] card : Parameter of type #topazProximityCard_t which will hold
 *                the UID of the read card.
 *
 *  \return ERR_TIMEOUT : Timeout waiting for interrupts or no reply from cards.
 *  \return ERR_NOTFOUND : No PICC could be selected..
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 topazReadUID(topazProximityCard_t* card);

/*! 
 *****************************************************************************
 *  \brief  Read the memory from a tag in READY state
 *
 *
 *  \param[in] card : Parameter of type #topazProximityCard_t which holds
 *                the UID of the card to read.
 *  \param[out] buf : buffer where the complete memory should be put
 *  \param[in] buf_size : Size of the given buffer
 *  \param[out] act_size : Size actually received
 *
 *  \return ERR_TIMEOUT : Timeout waiting for interrupts or no reply from cards.
 *  \return ERR_NOTFOUND : No PICC could be selected..
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 topazReadAll(const topazProximityCard_t* card, u8 *buf, u16 buf_size, u16* act_size);

/*! 
 *****************************************************************************
 *  \brief  Write one byte of memory to a tag in READY state
 *
 *
 *  \param[in] card : Parameter of type #topazProximityCard_t which holds
 *                the UID of the card to read.
 *  \param[in] addr : Address of the byte to be written.
 *  \param[in] data : Byte to be written to the given address
 *
 *  \return ERR_TIMEOUT : Timeout waiting for interrupts or no reply from cards.
 *  \return ERR_NOTFOUND : No PICC could be selected..
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 topazWriteByte(topazProximityCard_t* card, u8 addr, u8 data);

#endif /* TOPAZ_H */

