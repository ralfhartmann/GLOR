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
 *  \brief Common functions for 14443A and 14443B
 *
 */
/*!
 * 
 */

#ifndef ISO_14443_COMMON_H
#define ISO_14443_COMMON_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Transmit an ISO14443 frame and get response
 *
 *  This function is used to transfer an ISO14443 compatible frame.
 *  It takes the data given by \a txbuf with \a txlen and calls
 *  #as3911TxNBytes.
 *  Normally, a PICC should answer. This answer is then written to \a rxbuf
 *  and should have a max. length of \a rxlen (length of \a rxbuf).
 *  In case more data is received this data is discarded then.
 *
 *  \param[in] txbuf: data to be transmitted.
 *  \param[in] txlen: Number of bytes to transmit.
 *  \param[in] rxbuf: Buffer where the result will be written to.
 *  \param[in] rxlen: Max. number of bytes to receive (= length of \a rxbuf)
 *  \param[out] actrxlength: actual receive length.
 *
 *  \return ERR_NOTFOUND : No answer from PICC
 *  \return ERR_TIMEOUT : Timeout waiting for interrupts
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 iso14443TransmitAndReceive(const u8* txbuf,
                                    u16 txlen,
                                    u8* rxbuf,
                                    u16 rxlen,
                                    u16* actrxlength);

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
extern s8 iso14443Deselect(u8 cid);

#endif /* ISO_14443_COMMON_H */

