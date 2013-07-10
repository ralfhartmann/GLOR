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
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "iso14443a.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "iso14443_common.h"
#include "logger.h"
#include "config.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

//#define ISO_14443A_DEBUG dbgLog
#define ISO_14443A_DEBUG(...)
/* ISO14443A_MASK_RECEIVE_TIME Spec: FDT = (n * 128 + 84) / fc  with n_min = 9 
   set it lower: 2*5=10 */
#define ISO14443A_MASK_RECEIVE_TIME 10 

/*  REQA, etc. have much shorter time of 1172/fc ~= 19*64/fc */
#define ISO14443A_INVENTORY_WAITING_TIME 35

/* Activation frame waiting time 65536/fc (~4833us) = 1024*64/fc */
#define ISO14443A_ACTIVATION_WAITING_TIME 1300

/* iso14443 max FWT is ~5secs. However this is much too large for our USB 
   communication. Limit to 100ms = 21186 * 64/fc. */
#define ISO14443A_FRAME_DELAY_TIME  21186

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static s8 iso14443ADoAntiCollisionLoop(iso14443AProximityCard_t* card);
static s8 iso14443AVerifyBcc(const u8* uid, u8 length, u8 bcc);
static u8 iso14443aSavedOpReg;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 iso14443AInitialize()
{
    s8 err;
    u8 buf[3];

    ISO_14443A_DEBUG(__func__);

    as3911ReadRegister(AS3911_REG_OP_CONTROL,&iso14443aSavedOpReg);

    buf[0] = (iso14443aSavedOpReg
           | AS3911_REG_OP_CONTROL_en
           | AS3911_REG_OP_CONTROL_tx_en
           | AS3911_REG_OP_CONTROL_rx_en
           )
           & ~AS3911_REG_OP_CONTROL_wu;

    buf[1] = AS3911_REG_MODE_om_iso14443a;
    buf[2] = 0x00; /* AS3911_REG_BIT_RATE : 106 kBit/s both direction */

    /* set iso14443 mode A, lowest Rx and TX rate. Enable Rx and Tx */
    err = as3911WriteMultipleRegisters(AS3911_REG_OP_CONTROL, buf, 3);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, ISO14443A_MASK_RECEIVE_TIME);

    as3911SetNoResponseTime_64fcs(ISO14443A_FRAME_DELAY_TIME);

    err = as3911ExecuteCommand(AS3911_CMD_ANALOG_PRESET);

out:
    return err;
}

s8 iso14443ADeinitialize(u8 keep_on)
{
    s8 err;

    ISO_14443A_DEBUG(__func__);
    /* disable rx and tx */
    iso14443aSavedOpReg &= ~ (AS3911_REG_OP_CONTROL_tx_en | AS3911_REG_OP_CONTROL_rx_en);
    if (keep_on)
    {
        iso14443aSavedOpReg |=
              AS3911_REG_OP_CONTROL_en
            | AS3911_REG_OP_CONTROL_tx_en;
    }

    err = as3911WriteRegister(AS3911_REG_OP_CONTROL, iso14443aSavedOpReg);

    /* enable CRC because it is default */
    err = as3911ModifyRegister(AS3911_REG_AUX,
                               AS3911_REG_AUX_no_crc_rx,
                               0);

    return err;
}

s8 iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card)
{
    s8 err = ERR_NONE;
    u8 directcmd;
    u8 mask;
    u16 actlength;

    ISO_14443A_DEBUG(__func__);
    as3911SetNoResponseTime_64fcs(ISO14443A_INVENTORY_WAITING_TIME);
    /* first disable CRC while receiving since ATQA has no CRC included */
    err = as3911ModifyRegister(AS3911_REG_AUX,
                               AS3911_REG_AUX_no_crc_rx,
                               AS3911_REG_AUX_no_crc_rx);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* Enable antcl to recognize collision in first byte of ATQA */
    err = as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, AS3911_REG_ISO14443A_NFC_antcl, AS3911_REG_ISO14443A_NFC_antcl);
    /* enable required interrupts: bit collision, recv error, end of tx and end of rx */
    err = as3911EnableInterrupts(AS3911_IRQ_MASK_COL |
                                 AS3911_IRQ_MASK_TXE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    switch (cmd)
    {
        case ISO14443A_CMD_REQA:
            directcmd = AS3911_CMD_TRANSMIT_REQA;
            break;
        case ISO14443A_CMD_WUPA:
            directcmd = AS3911_CMD_TRANSMIT_WUPA;
            break;
        default:
            err = ERR_PARAM;
            goto out_disable_irq;
    }

#if AS3911_TXRX_ON_CSX
    as3911WriteTestRegister(0x1,0x0a); /* digital modulation on pin CSI */
#endif
    ISO_14443A_DEBUG("1");
    /* prepare receive which enables interrupts */
    err = as3911PrepareReceive(TRUE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out_disable_irq);

    /* now send either WUPA or REQA. All affected tags will backscatter ATQA and
       change to READY state */
    err = as3911ExecuteCommand(directcmd);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out_disable_irq);

    ISO_14443A_DEBUG("2");
    /* wait for transmit finshed interrupt. */
    mask = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_TXE, 10);
    if (0 == mask)
    {
        ISO_14443A_DEBUG("no txe in %s\n",__func__);
        err = ERR_TIMEOUT;
        goto out_disable_irq;
    }
    ISO_14443A_DEBUG("Sent WUPA/REQA\n");

    /* request sent - wait for an answer */
    err = as3911RxNBytes((u8*)&card->atqa, sizeof(u16), &actlength, 0);
    if (ERR_TIMEOUT == err)
    {
        /* no tag reply at all*/
        goto out_disable_irq;
    }
    ISO_14443A_DEBUG("Got ATQA\n");

    if (0xc == (card->atqa[1] & 0xf)) /* Topaz aka type 1 id */
    {
        err = ERR_NOTSUPP; /* Select/anticollision not supported */
        goto out_disable_irq;
    }

    /* at least one tag responded - start anticollision loop regardless
       on if there was a collision within ATQA or not */
    err = iso14443ADoAntiCollisionLoop(card);

    if (ERR_TIMEOUT == err)
    {
        err = ERR_NOTSUPP; /* Select/anticollision not supported */
    }

out_disable_irq:
    as3911DisableInterrupts(AS3911_IRQ_MASK_COL |
                            AS3911_IRQ_MASK_TXE);
out:
    /* Disable antcl again */
    as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, AS3911_REG_ISO14443A_NFC_antcl, 0);
    as3911SetNoResponseTime_64fcs(ISO14443A_FRAME_DELAY_TIME);
    return err;
}


s8 iso14443ASendHlta()
{
    s8 err;
    u8 buf[2];
    u8 mask;

    /* enable RX interrupt first */
    err = as3911EnableInterrupts(AS3911_IRQ_MASK_RXS);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* send HLTA command */
    buf[0] = ISO14443A_CMD_HLTA;
    buf[1] = 0;
    err = as3911TxNBytes(buf, 2, 0, AS3911_TX_FLAG_CRC);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* according to ISO14443-3 we should wait here for 1.1ms.
       If any PICC responds within this time, HLTA command shall
       be interpreted as not acknowledged */
     mask = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_RXS, 2);

     if (0 == mask)
     {
         err = ERR_NONE;
     }
     else
     {
         err = ERR_NOMSG;
     }
out:
     as3911DisableInterrupts(AS3911_IRQ_MASK_RXS);
     return err;
}

s8 iso14443AEnterProtocolMode(u8 fscid, u8* answer, u16 maxlength, u16* length)
{
    s8 err;
    u8 buf[2];

    as3911SetNoResponseTime_64fcs(ISO14443A_ACTIVATION_WAITING_TIME);

    /* send RATS command */
    buf[0] = ISO14443A_CMD_RATS;
    buf[1] = fscid;
    err = iso14443TransmitAndReceive(buf, 2, answer, maxlength, length);

    /* map timeout error to PICC not found */
    if (ERR_TIMEOUT == err)
    {
        err = ERR_NOTFOUND;
    }
    as3911SetNoResponseTime_64fcs(ISO14443A_FRAME_DELAY_TIME);
    return err;
}

s8 iso14443ASendProtocolAndParameterSelection(u8 cid, u8 pps1)
{
    s8 err;
    u8 buf[3];
    u16 tmp;

    as3911SetNoResponseTime_64fcs(ISO14443A_ACTIVATION_WAITING_TIME);
    /* send PPS command */
    buf[0] = ISO14443A_CMD_PPSS | cid;
    buf[1] = 0x11;
    buf[2] = pps1;
    err = iso14443TransmitAndReceive(buf, 3, buf, 1, &tmp);

    /* map timeout error to PICC not found */
    if ((ERR_TIMEOUT == err) || ((ISO14443A_CMD_PPSS | cid) != buf[0]))
    {
        err = ERR_NOTFOUND;
    }

    as3911SetNoResponseTime_64fcs(ISO14443A_FRAME_DELAY_TIME);
    return err;
}

s8 iso14443ADeselect(u8 cid)
{
    return iso14443Deselect(cid);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
static s8 iso14443ADoAntiCollisionLoop(iso14443AProximityCard_t* card)
{
    u8 cscs[ISO14443A_MAX_CASCADE_LEVELS][ISO14443A_CASCADE_LENGTH];
    u8 cl = ISO14443A_CMD_SELECT_CL1;
    u8 colreg;
    u8 bytesBeforeCol = 2;
    u8 bitsBeforeCol = 0;
    s8 err;
    u8 colbyte = 0;
    u8 i;
    u16 actlength;
    u16 actsaklength;
    u8* buf;

    card->cascadeLevels = 0;
    card->collision = FALSE;
    card->actlength = 0;
    buf = cscs[card->cascadeLevels];
    /* start anticollosion loop by sending SELECT command and NVB 0x20 */
    buf[1] = 0x20;
    ISO_14443A_DEBUG("Start Anticollision loop\n");

    do {
        buf[0] = cl;
        /* prepare receive which enables interrupts */
        err = as3911PrepareReceive(TRUE);
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

        err = as3911TxNBytes(buf, bytesBeforeCol, bitsBeforeCol, AS3911_TX_FLAG_ANTCL);
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
        ISO_14443A_DEBUG("Sent 0x%x bytes\n", bytesBeforeCol);

        err = as3911RxNBytes(buf + bytesBeforeCol, ISO14443A_CASCADE_LENGTH - bytesBeforeCol, &actlength, 0);
        ISO_14443A_DEBUG("after rx\n");
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
        ISO_14443A_DEBUG("Received 0x%x bytes\n", actlength);

        if (bitsBeforeCol > 0)
        {
            buf[bytesBeforeCol] >>= bitsBeforeCol;
            buf[bytesBeforeCol] <<= bitsBeforeCol;
            buf[bytesBeforeCol] |= colbyte;
        }

        /* now check for collision */
        if (as3911GetInterrupt(AS3911_IRQ_MASK_COL))
        {
            ISO_14443A_DEBUG("Collision!\n");
            card->collision = TRUE;
            /* read out collision register */
            err = as3911ReadRegister(AS3911_REG_COLLISION_STATUS, &colreg);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

            bytesBeforeCol = (colreg >> AS3911_REG_COLLISION_STATUS_shift_c_byte) & 0xf;
            bitsBeforeCol = (colreg >> AS3911_REG_COLLISION_STATUS_shift_c_bit) & 0x7;
            bitsBeforeCol++;
            if (bitsBeforeCol == 8)
            {
                bitsBeforeCol = 0;
                bytesBeforeCol++;
            }
            ISO_14443A_DEBUG("Bytes before Col 0x%x ", bytesBeforeCol);
            ISO_14443A_DEBUG("Bits before Col 0x%x\n", bitsBeforeCol);
            /* FIXME handle c_pb collision in parity bit */
            /* update NVB. Add 2 bytes for SELECT and NVB itself */
            buf[1] = bytesBeforeCol << 4;
            buf[1] |= bitsBeforeCol;
            /* save the colision byte */
            buf[bytesBeforeCol] <<= (8 - bitsBeforeCol);
            buf[bytesBeforeCol] >>= (8 - bitsBeforeCol);
            colbyte = buf[bytesBeforeCol];
        }
        else
        {
            ISO_14443A_DEBUG("Got a frame\n");
            /* got a frame w/o collision - store the uid and check for CT */

            /* enable CRC while receiving SAK */
            err = as3911ModifyRegister(AS3911_REG_AUX,
                                       AS3911_REG_AUX_no_crc_rx, 0x0);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

            err = as3911EnableInterrupts(AS3911_IRQ_MASK_CRC);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

            /* answer with complete uid and check for SAK. */
            buf[1] = (actlength + bytesBeforeCol) << 4;
            buf[0] = cl;
            ISO_14443A_DEBUG("Request SAK\n");
            err = iso14443TransmitAndReceive(buf,
                                            actlength + bytesBeforeCol,
                                            &card->sak[card->cascadeLevels],
                                            1,
                                            &actsaklength);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
            ISO_14443A_DEBUG("Got SAK\n");

            if (as3911GetInterrupt(AS3911_IRQ_MASK_CRC))
            {
                /* CRC error... */
                err = ERR_CRC;
                goto out;
            }

            /* disable CRC again */
            err = as3911ModifyRegister(AS3911_REG_AUX,
                                       AS3911_REG_AUX_no_crc_rx,
                                       AS3911_REG_AUX_no_crc_rx);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

            err = as3911DisableInterrupts(AS3911_IRQ_MASK_CRC);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

            if (card->sak[card->cascadeLevels] & 0x4)
            {
                ISO_14443A_DEBUG("Next cascading level\n");
                /* reset variables for next cascading level */
                bytesBeforeCol = 2;
                bitsBeforeCol = 0;

                if (ISO14443A_CMD_SELECT_CL1 == cl)
                {
                    cl = ISO14443A_CMD_SELECT_CL2;
                }
                else if (ISO14443A_CMD_SELECT_CL2 == cl)
                {
                    cl = ISO14443A_CMD_SELECT_CL3;
                }
                else
                {
                    /* more than 3 cascading levels are not possible ! */
                    err = ERR_NOMSG;
                    goto out;
                }
            }
            else
            {
                ISO_14443A_DEBUG("UID done\n");
                card->cascadeLevels++;
                break;
            }
            card->cascadeLevels++;
            buf = cscs[card->cascadeLevels];
            buf[0] = cl;
            buf[1] = 0x20;
        } /* no collision detected */

    } while (card->cascadeLevels <= ISO14443A_MAX_CASCADE_LEVELS);

    /* do final checks... */
    for (i = 0; i< card->cascadeLevels; i++)
    {
            err = iso14443AVerifyBcc(&cscs[i][2], 4, cscs[i][6]);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
    }

    /* extract pure uid */
    switch (card->cascadeLevels)
    {
        case 3:
            AMS_MEMMOVE(card->uid+6, cscs[2]+2, 4);
            AMS_MEMMOVE(card->uid+3, cscs[1]+3, 3);
            AMS_MEMMOVE(card->uid+0, cscs[0]+3, 3);
            card->actlength = 10;
            break;
        case 2:
            AMS_MEMMOVE(card->uid+3, cscs[1]+2, 4);
            AMS_MEMMOVE(card->uid+0, cscs[0]+3, 3);
            card->actlength = 7;
            break;
        case 1:
            AMS_MEMMOVE(card->uid+0, cscs[0]+2, 4);
            card->actlength = 4;
            break;
        default:
            err = ERR_NOTSUPP;
            goto out;
    }

out:
    /* clean up a bit */
    as3911DisableInterrupts(AS3911_IRQ_MASK_CRC);
    /* enable CRC checking for upcoming commands */
    as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_no_crc_rx, 0x0);
    return err;
}

static s8 iso14443AVerifyBcc(const u8* uid, u8 length, u8 bcc)
{
    u8 actbcc = 0;

    do
    {
        length--;
        actbcc ^= uid[length];
    } while (length);

    if (actbcc != bcc)
    {
        return ERR_CRC;
    }
    else
    {
        return ERR_NONE;
    }
}

