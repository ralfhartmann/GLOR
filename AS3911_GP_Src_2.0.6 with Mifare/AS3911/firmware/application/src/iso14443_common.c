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
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "iso14443_common.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "utils.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define ISO14443_CMD_DESELECT  0xca /*!< command DESELECT */

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 iso14443TransmitAndReceive(const u8* txbuf,
                                    u16 txlen,
                                    u8* rxbuf,
                                    u16 rxlen,
                                    u16* actrxlength)
{
    s8 err;

    /* prepare receive which enables interrupts */
    err = as3911PrepareReceive(TRUE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    err = as3911TxNBytes(txbuf, txlen, 0, AS3911_TX_FLAG_CRC);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    err = as3911RxNBytes(rxbuf, rxlen, actrxlength, 0);

    /* map parity error or receive timeout to not found error */
    if ((ERR_TIMEOUT == err) ||
            (ERR_NOMSG == err))
    {
        err = ERR_NOTFOUND;
    }

out:
    return err;
}


s8 iso14443Deselect(u8 cid)
{
    s8 err;
    u8 buf[2];
    u16 tmp;

    /* send DESELECT command */
    buf[0] = ISO14443_CMD_DESELECT;
    buf[1] = cid;
    err = iso14443TransmitAndReceive(buf, 2, buf, 1, &tmp);

    return err;
}
