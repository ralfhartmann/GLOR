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

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "iso14443_common.h"
#include "mifare_ul.h"
#include "as3911_com.h"
#include "as3911.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

//#define MIFARE_UL_DEBUG DEBUG
#define MIFARE_UL_DEBUG(...)

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define MIFARE_UL_CMD_READ 0x30
#define MIFARE_UL_CMD_WRITE 0xA2

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 mifareUlReadNBytes(u8 startAddr, u8* readbuf, u8 length, u8* actLength)
{
    s8 err;
    u8 txbuf[2];
    u16 actrxlength;
    u8 rxbuf[16];

    *actLength = 0;

    if (startAddr > 0xf)
    {
        return ERR_PARAM;
    }

    txbuf[0] = MIFARE_UL_CMD_READ;

    do
    {
        MIFARE_UL_DEBUG("send startaddr 0x%x\n", startAddr);
        txbuf[1] = startAddr;
        err = iso14443TransmitAndReceive(txbuf,
                sizeof(txbuf),
                rxbuf,
                sizeof(rxbuf),
                &actrxlength);
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

        if (actrxlength != sizeof(rxbuf))
        {
            /* only NAK received or less than 16 bytes */
            err = ERR_NOMSG;
            goto out;
        }
        if (actrxlength > length)
        {
            length = 0;
        }
        else
        {
            length -= actrxlength;
        }
        /* copy received bytes to output buffer */
        AMS_MEMCPY(readbuf, rxbuf, actrxlength);
        readbuf += actrxlength;
        *actLength += actrxlength;

        startAddr += 4;
        /* roll back in case we go behind 0xf */
        if (startAddr > 0xf)
        {
            startAddr -= 0x10;
        }
    } while (length > 0);

out:
    return err;
}


s8 mifareUlWritePage(u8 pageAddr, const u8* writebuf)
{
    s8 err;
    u8 txbuf[6];
    u16 actrxlength;
    u8 rxbuf;

    if (pageAddr > 0xf)
    {
        return ERR_PARAM;
    }

    /* first disable CRC while receiving since answer contains no CRC */
    err = as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_no_crc_rx, AS3911_REG_AUX_no_crc_rx);
    /* disable parity checking */
    err = as3911ModifyRegister(AS3911_REG_ISO14443A_NFC,
                               AS3911_REG_ISO14443A_NFC_no_rx_par,
                               AS3911_REG_ISO14443A_NFC_no_rx_par);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    txbuf[0] = MIFARE_UL_CMD_WRITE;
    txbuf[1] = pageAddr;
    AMS_MEMCPY(&txbuf[2], (u8*)writebuf, 4);

    /* note: iso14443TransmitAndReceive can't be used in this case
       as it doesn't support 4 bit response frames */
    /* prepare receive which enables interrupts */
    err = as3911PrepareReceive(TRUE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    err = as3911TxNBytes(txbuf, sizeof(txbuf), 0, AS3911_TX_FLAG_CRC);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    err = as3911RxNBytes(&rxbuf, 1, &actrxlength, 0);

    /* map parity error or receive timeout to not found error */
    if ((ERR_TIMEOUT == err) ||
            (ERR_NOMSG == err))
    {
        err = ERR_NOTFOUND;
    }
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    if (0x0a != rxbuf)
    {
        err = ERR_WRITE;
    }

out:
    /* enable CRC again */
    as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_no_crc_rx, 0x0);
    /* enable parity checking */
    as3911ModifyRegister(AS3911_REG_ISO14443A_NFC,
                         AS3911_REG_ISO14443A_NFC_no_rx_par, 0x0);

    return err;
}

