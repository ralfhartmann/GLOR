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

#ifndef FELICA_H
#define FELICA_H

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
#define FELICA_MAX_ID_LENGTH 8

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/
/*! 
  struct representing an FELICA PICC as returned by
  #felicaPoll.
  Layout is:
  <ol>
    <li> length[1]
    <li> response_code[1];
    <li> IDm[8];
    <li> PMm[8];
    <li> request_data[2];
  </ol>
 */
struct felicaProximityCard
{
    u8 length[1];
    u8 response_code[1];
    u8 IDm[FELICA_MAX_ID_LENGTH]; /*<! ID of the PICC */
    u8 PMm[8];
    u8 request_data[2];
};

/*! 
 * PCD command set.
 */
enum felicaCommand
{
    FELICA_CMD_POLLING                  = 0x00, /*!< acquire and identify a card. */
    FELICA_CMD_REQUEST_SERVICE          = 0x02, /*!< verify the existence of Area and Service. */
    FELICA_CMD_REQUEST_RESPONSE         = 0x04, /*!< verify the existence of a card. */
    FELICA_CMD_READ_WITHOUT_ENCRYPTION  = 0x06, /*!< read Block Data from a Service that requires no authentication. */
    FELICA_CMD_WRITE_WITHOUT_ENCRYPTION = 0x08, /*!< write Block Data to a Service that requires no authentication. */
    FELICA_CMD_REQUEST_SYSTEM_CODE      = 0x0c, /*!< acquire the System Code registered to a card. */
    FELICA_CMD_AUTHENTICATION1          = 0x10, /*!< authenticate a card. */
    FELICA_CMD_AUTHENTICATION2          = 0x12, /*!< allow a card to authenticate a Reader/Writer. */
    FELICA_CMD_READ                     = 0x14, /*!< read Block Data from a Service that requires authentication. */
    FELICA_CMD_WRITE                    = 0x16, /*!< write Block Data to a Service that requires authentication. */
};

enum felicaSlots
{
    FELICA_1_SLOT = 0,
    FELICA_2_SLOTS = 1,
    FELICA_4_SLOTS = 3,
    FELICA_8_SLOTS = 7,
    FELICA_16_SLOTS = 15,

};

enum felicaComParamRequest
{
    FELICA_REQ_NO_REQUEST = 0x00,
    FELICA_REQ_SYSTEM_CODE = 0x01,
    FELICA_REQ_COM_PERFORMANCE = 0x02,
};

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Initialize FELICA mode.
 *  \note This function needs to be called every time after switching
 *  from a different mode.
 *
 *  \param[in] modulation_index : Desired modulation index value between 8-30%
 *             e.g. AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_10percent
 *
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 felicaInitialize(u8 modulation_index);

/*! 
 *****************************************************************************
 *  \brief  Deinitialize FELICA mode.
 *  \note This function should be called every time iso 14443 a is not needed
 *  any more.
 *  \param keep_on: if true the RF field will not be switched off
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 felicaDeinitialize(u8 keep_on);

/*! 
 *****************************************************************************
 *  This function sends to all PICCs in field the POLL command with the given
 *  number of slots.
 *
 *  \param[in] slots: the number of slots to be performed
 *  \param[in] sysCode1: as given in FeliCa poll command 
 *  \param[in] sysCode2: as given in FeliCa poll command 
 *  \param[in] compar: FeliCa communication paramters
 *  \param[out] card : Parameter of type #felicaProximityCard which holds
 *                the found card then.
 *  \param[in] num_cards : size of the card array
 *  \param[out] num_cards : actual number of cards found
 *  \param[out] num_cols : number of collisions encountered
 *
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 felicaPoll(enum felicaSlots slots,
                     u8 sysCode1,
                     u8 sysCode2,
                     enum felicaComParamRequest compar,
                     struct felicaProximityCard *card,
                     u8 *num_cards,
                     u8 *num_cols
                    );

/*! 
 *****************************************************************************
 *  Transfer sizeof_txbuf bytes to a FeliCa card and receive back up to 
 *  sizeof_rxbuf bytes 
 *  \param[in] txbuf: buffer to transfer
 *  \param[in] sizeof_txbuf: number of bytes to transfer
 *  \param[out] rxbuf: buffer of read data
 *  \param[out] sizeof_rxbuf: maximum number of bytes to receive
 *  \param[out] actrxlength : the number of actually received bytes
 *****************************************************************************
 */
extern s8 felicaTxRxNBytes(const u8 *txbuf, u16 sizeof_txbuf, u8 *rxbuf, u16 sizeof_rxbuf, u16 *actrxlength);
#endif /* FELICA_H */
