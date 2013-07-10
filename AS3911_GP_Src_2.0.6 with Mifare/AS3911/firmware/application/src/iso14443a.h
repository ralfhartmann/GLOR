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
 *  \author Christian Eisendle
 *
 *  \brief Implementation of ISO-14443A
 *
 */
/*!
 * 
 */

#ifndef ISO_14443_A_H
#define ISO_14443_A_H

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
#define ISO14443A_MAX_UID_LENGTH 10
#define ISO14443A_MAX_CASCADE_LEVELS 3
#define ISO14443A_CASCADE_LENGTH 7
#define ISO14443A_RESPONSE_CT  0x88

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/
/*!< 
 * struct representing an ISO14443A PICC as returned by
 * #iso14443ASelect.
 */
typedef struct
{
    u8 uid[ISO14443A_MAX_UID_LENGTH]; /*<! UID of the PICC */
    u8 actlength; /*!< actual UID length */
    u8 atqa[2]; /*!< content of answer to request byte */
    u8 sak[ISO14443A_MAX_CASCADE_LEVELS]; /*!< SAK bytes */
    u8 cascadeLevels; /*!< number of cascading levels */
    bool_t collision; /*!< TRUE, if there was a collision which has been resolved,
                        otherwise no collision occured */
}iso14443AProximityCard_t;

/*! 
 * PCD command set.
 */
typedef enum
{
    ISO14443A_CMD_REQA = 0x26,  /*!< command REQA */
    ISO14443A_CMD_WUPA = 0x52, /*!< command WUPA */
    ISO14443A_CMD_SELECT_CL1 = 0x93, /*!< command SELECT cascade level 1 */
    ISO14443A_CMD_SELECT_CL2 = 0x95, /*!< command SELECT cascade level 2 */
    ISO14443A_CMD_SELECT_CL3 = 0x97, /*!< command SELECT cascade level 3 */
    ISO14443A_CMD_HLTA = 0x50, /*!< command HLTA */
    ISO14443A_CMD_PPSS = 0xd0, /*!< command PPSS */
    ISO14443A_CMD_RATS = 0xe0, /*!< command RATS */
}iso14443ACommand_t;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Initialize ISO14443-A mode.
 *  \note This function needs to be called every time after switching
 *  from a different mode.
 *
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 iso14443AInitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Deinitialize ISO14443-A mode.
 *  \note This function should be called every time iso 14443 a is not needed
 *  any more.
 *  \param keep_on: if true the RF field will not be switched off
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 iso14443ADeinitialize(u8 keep_on);

/*! 
 *****************************************************************************
 *  \brief  Select a PICC and put it into ACTIVE state
 *
 *  This function sends to all PICCs in field either the REQA or the
 *  WUPA (depending on \a cmd parameter). This command puts the
 *  PICCs into READY state. After that anticollision loop is performed to
 *  select a unique PICC. In the end this PICC should be in ACTIVE
 *  state and its UID is returned.
 *
 *  \param[in] cmd : Used command to put the PICCs in READY state. This
 *                   could either be #ISO14443A_CMD_REQA or #ISO14443A_CMD_WUPA
 *  \param[out] card : Parameter of type #iso14443AProximityCard_t which holds
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
extern s8 iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card);

/*! 
 *****************************************************************************
 *  \brief  Send the HLTA command
 *
 *  This function is used to send the HLTA command to put a PICCs to
 *  state HALT so that they do not reply to REQA commands (only to WUPA).
 *
 *  \return ERR_NOMSG : PICC send not acknowledge
 *  \return ERR_TIMEOUT : Timeout waiting for interrupts.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 iso14443ASendHlta(void);

/*! 
 *****************************************************************************
 *  \brief  Enter protocol mode
 *
 *  This function is used to send the RATS command which puts the PICC
 *  into protocol mode. The PICC must be in ACTIVE mode using #iso14443ASelect
 *  command. If the PICC receives the RATS command it backscatters the ATS
 *  which will be stored to \a answer.
 *
 *  \param[in] fscid : the parameter byte send with RATS containing 
 *           frame size and card id. Refer to ISO14443-4 for exact coding.
 *  \param[out] answer : buffer where ATS will be stored
 *  \param[in] maxlength : max. length of the answer (= size of \a answer)
 *  \param[out] length : actual length of ATS
 *
 *  \return ERR_NOTFOUND : PICC not in field or in right state.
 *  \return ERR_TIMEOUT : Timeout during transmit.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error, ATS received and written to \a answer.
 *
 *****************************************************************************
 */
extern s8 iso14443AEnterProtocolMode(u8 fscid, u8* answer, u16 maxlength, u16* length);

/*! 
 *****************************************************************************
 *  \brief  Send protocol and parameter selection request
 *
 *  After a PICC has been put to protocol mode using #iso14443AEnterProtocolMode
 *  some parameter can be set. This function is used to set these parameters.
 *  For more information on codeing of the \a pps1 byte refer to ISO14443-4.
 *  PPS0 byte is set to 0x11 implicitely.
 *  In case of success the PICC sends back the PPS start byte.
 *
 *  \param[in] cid : logical number of the addressed PICC (see ISO14443-4)
 *  \param[in] pps1 : pps1 byte to be written (see ISO14443-4)
 *
 *  \return ERR_NOTFOUND : PICC not in field or in right state.
 *  \return ERR_TIMEOUT : Timeout during transmit.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 iso14443ASendProtocolAndParameterSelection(u8 cid, u8 pps1);

/*! 
 *****************************************************************************
 *  \brief  Deselect a PICC
 *
 *  This function deselects a PICC which is in PROTOCOL state. The card
 *  is identified using \a cid.
 *
 *  \param[in] cid : logical number of the addressed PICC (see ISO14443-4)
 *
 *  \return ERR_NOTFOUND : PICC not in field or in right state.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 iso14443ADeselect(u8 cid);

#endif /* ISO_14443_A_H */

