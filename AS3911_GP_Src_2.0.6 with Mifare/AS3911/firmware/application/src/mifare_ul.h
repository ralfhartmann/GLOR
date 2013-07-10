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
 *  \brief Mifare UL read and write command implementation
 *
 */
/*!
 * 
 */

#ifndef MIFARE_UL_H
#define MIFARE_UL_H

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
 *  \brief  Read out a given number of bytes from a MIFARE UL PICC.
 *
 *  This function reads out a given number of bytes from a MIFARE UL
 *  compatible PICC in field.
 *  \note Due to the nature of the MIFARE UL read-command normally 16bytes are
 *  read out. This function however only returns the number of bytes given.
 *  \note Mifare UL only has 16 pages. If a roll over occurs function
 *  continues at address 0x0 as MIFARE UL specifies.
 *  \note PICC must be in ACTIVE state using #iso14443ASelect
 *
 *  \param[in] startAddr: Address of the first page to read out.
 *  \param[out] readbuf: Buffer with size \a length where the result is stored.
 *  \param[in] length: Number of bytes to read out (size of \a readbuf)
 *  \param[in] actLength: Number of bytes actually read.
 *
 *  \return ERR_NOTFOUND : No answer from PICC.
 *  \return ERR_NOMSG : NAK received.
 *  \return ERR_PARAM : \a startAddr behind 0xf.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 mifareUlReadNBytes(u8 startAddr, u8* readbuf, u8 length, u8* actLength);

/*! 
 *****************************************************************************
 *  \brief  Write a page of a MIFARE UL PICC.
 *
 *  This function writes one page data to a MIFARE UL compatible PICC in field.
 *  \note PICC must be in ACTIVE state using #iso14443ASelect
 *
 *  \param[in] pageAddr: Address of page write.
 *  \param[in] writebuf: one page (4 bytes!) longe buffer to write to the
 *                      given page.
 *
 *  \return ERR_NOTFOUND : No answer from PICC.
 *  \return ERR_NOMSG : NAK received.
 *  \return ERR_PARAM : \a startAddr behind 0xf.
 *  \return ERR_WRITE : Write failed.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 mifareUlWritePage(u8 pageAddr, const u8* writebuf);

#endif /* MIFARE_UL_H */

