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
 *  \brief Implementation of ISO-14443B
 *
 */
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "iso14443b.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "iso14443_common.h"
#include "as3911errno.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define ISO14443B_PARAM_APF 0x5
#define ISO14443B_MAX_AC_LOOP_COUNT 5
/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

//#define ISO_14443B_DEBUG dbgLog
#define ISO_14443B_DEBUG(...)

/* Calculating with TR0min = TR1min = 16*16/fc => 8*64/fc */
#define ISO14443B_MASK_RECEIVE_TIME 4 

/* iso14443 TR0 + TR1 is 456/fs = (456*16)/fc = 114*64/fc */
#define ISO14443B_ACTIVATION_WAITING_TIME 150

/* iso14443 max FDT is ~5secs. However this is much too large for our USB 
   communication. Limit to 100ms = 21186 * 64/fc. */
#define ISO14443B_FRAME_DELAY_TIME  21186

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static s8 iso14443BDoAntiCollisionLoop(iso14443BCommand_t cmd,
                                    iso14443BProximityCard_t* card,
                                    u8 afi,
                                    iso14443BSlotCount_t slotCount);

static s8 iso14443BSendApfAndGetResult(iso14443BCommand_t cmd,
                                    iso14443BProximityCard_t* card,
                                    u8 afi,
                                    iso14443BSlotCount_t slotCount);


static u8 iso14443bSavedOpReg;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 iso14443BInitialize(u8 mi, u8* result)
{
    u8 buf[3];
    s8 err;

    ISO_14443B_DEBUG(__func__);

    as3911ReadRegister(AS3911_REG_OP_CONTROL,&iso14443bSavedOpReg);

    buf[0] = (iso14443bSavedOpReg
           | AS3911_REG_OP_CONTROL_en
           | AS3911_REG_OP_CONTROL_tx_en
           | AS3911_REG_OP_CONTROL_rx_en
           )
           & ~AS3911_REG_OP_CONTROL_wu;
    buf[1] = AS3911_REG_MODE_om_iso14443b;
    buf[2] = 0x00; /* AS3911_REG_BIT_RATE : 106 kBit/s both direction */

    /* set iso14443 mode B, lowest Rx and TX rate. Enable Rx and Tx */
    err = as3911WriteMultipleRegisters(AS3911_REG_OP_CONTROL, buf, 3);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* Set modulation depth to 10%. */
    err = as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONTROL, mi);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
    /* calibrate modulation depth */
    err = as3911CalibrateModulationDepth(result);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, ISO14443B_MASK_RECEIVE_TIME);

    as3911SetNoResponseTime_64fcs(ISO14443B_FRAME_DELAY_TIME);

    err = as3911ExecuteCommand(AS3911_CMD_ANALOG_PRESET);
out:
    return err;
}

s8 iso14443BDeinitialize(u8 keep_on)
{
    s8 err;

    ISO_14443B_DEBUG(__func__);
    iso14443bSavedOpReg &= ~(AS3911_REG_OP_CONTROL_tx_en | AS3911_REG_OP_CONTROL_rx_en);
    if (keep_on) iso14443bSavedOpReg |=
                  AS3911_REG_OP_CONTROL_en 
                | AS3911_REG_OP_CONTROL_tx_en;

    /* op control to proper state */
    err = as3911WriteRegister(AS3911_REG_OP_CONTROL, iso14443bSavedOpReg);

    return err;
}

s8 iso14443BSelect(iso14443BCommand_t cmd,
                iso14443BProximityCard_t* card,
                u8 afi,
                iso14443BSlotCount_t slotCount)
{
    s8 err = ERR_NONE;

    as3911SetNoResponseTime_64fcs(ISO14443B_ACTIVATION_WAITING_TIME);

    /* make sure that CRC is enabled */
    err = as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_no_crc_rx, 0x0);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    err = iso14443BSendApfAndGetResult(cmd, card, afi, ISO14443B_SLOT_COUNT_1);
    if (ERR_NOTUNIQUE == err)
    {
        ISO_14443B_DEBUG("Got ATQB from more PICCs\n");
        card->collision = TRUE;
        err = iso14443BDoAntiCollisionLoop(cmd, card, afi, slotCount);
    }
    else if (ERR_NONE == err)
    {
        ISO_14443B_DEBUG("Got ATQB\n");
        /* no crc error - only one PICC replied */
        card->collision = FALSE;
    }
out:
    as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);
    return err;
}

s8 iso14443BSendHltb(iso14443BProximityCard_t* card)
{
    s8 err;
    u16 actlength;
    u8 buf[5];

    buf[0] = ISO14443B_CMD_HLTB;
    AMS_MEMCPY(&buf[1], card->pupi, 4);

    err = iso14443TransmitAndReceive(buf, 5, buf, 1, &actlength);

    if ((ERR_TIMEOUT == err) || (buf[0] != 0x0))
    {
        /* in case we ran into timeout or answer is not 0x0
           then we got no valid ACK */
        err = ERR_NOMSG;
    }

    return err;
}

s8 iso14443BEnterProtocolMode(iso14443BProximityCard_t* card,
                                iso14443BAttribParameter_t* param,
                                iso14443BAttribAnswer_t* answer)
{
    s8 err;
    u16 actlength;
    u8 buf[9];

    as3911SetNoResponseTime_64fcs(ISO14443B_ACTIVATION_WAITING_TIME);

    buf[0] = ISO14443B_CMD_ATTRIB;
    AMS_MEMCPY(&buf[1], card->pupi, 4);
    AMS_MEMCPY(&buf[5], (u8*)param, sizeof(iso14443BAttribParameter_t));

    err = iso14443TransmitAndReceive(buf,
                                    9,
                                    buf,
                                    1,
                                    &actlength);

    if (actlength < 1)
    {
        err = ERR_NOMSG;
    }
    else
    {
        answer->mbli = (buf[0] >> 4) & 0xf;
        answer->cid = buf[0] & 0xf;
    }

    as3911SetNoResponseTime_64fcs(ISO14443B_FRAME_DELAY_TIME);

    return err;
}

s8 iso14443BDeselect(u8 cid)
{
    return iso14443Deselect(cid);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
static s8 iso14443BDoAntiCollisionLoop(iso14443BCommand_t cmd,
                                    iso14443BProximityCard_t* card,
                                    u8 afi,
                                    iso14443BSlotCount_t slotCount)
{
    s8 err = ERR_NOTFOUND;
    u8 apn;
    u8 i;
    u8 j;
    u16 actlength;

    ISO_14443B_DEBUG("Start AC loop\n");
    for (i = 0; i < ISO14443B_MAX_AC_LOOP_COUNT; i++)
    {
        ISO_14443B_DEBUG("loop iteration 0x%x\n", i);
        /* first send Apf (wupb or reqb) with given slot count.
           if only one PICC respond return immediately. Otherwise
           send slot marker */
        err = iso14443BSendApfAndGetResult(cmd, card, afi, slotCount);
        EVAL_ERR_EQ_GOTO(ERR_NONE, err, out);
        for (j = 1; j < (1 << slotCount); j++)
        {
            apn = (j << 4) | 0x5;
            ISO_14443B_DEBUG("Send slot marker 0x%x\n", apn);
            /* send slot marker */
            err = iso14443TransmitAndReceive(&apn,
                    sizeof(u8),
                    (u8*)card,
                    0xc,
                    &actlength);
            if (actlength != 0xc)
            {
                ISO_14443B_DEBUG("Received 0x%x bytes only\n", actlength);
                /* map this error to not unique error cause this normally
                   happens if more PICCs in field */
                err = ERR_NOTUNIQUE;
            }
            else if (ERR_NONE == err)
            {
                /* found unique PICC */
                goto out;
            }
        }
        if (ERR_NOTFOUND == err)
        {
            ISO_14443B_DEBUG("No card in field\n");
            break;
        }
    }
out:
    return err;
}

static s8 iso14443BSendApfAndGetResult(iso14443BCommand_t cmd,
                                    iso14443BProximityCard_t* card,
                                    u8 afi,
                                    iso14443BSlotCount_t slotCount)
{
    s8 err = ERR_NONE;
    u16 actlength;
    u8 buf[3];

    buf[0] = ISO14443B_PARAM_APF;
    buf[1] = afi;
    buf[2] = slotCount;
    buf[2] |= cmd;

    err = iso14443TransmitAndReceive(buf,
                                    3,
                                    (u8*)card,
                                    0xc,
                                    &actlength);

    if (ERR_NONE == err && actlength != 0xc)
    {
        ISO_14443B_DEBUG("Received 0x%x bytes only\n", actlength);
        dbgHexDump((u8*)card,actlength);
        /* map this error to not unique error cause this normally
           happens if more PICCs in field */
        err = ERR_NOTUNIQUE;
    }
    if (ERR_CRC == err || ERR_PAR == err || ERR_FRAMING == err)
    {
        err = ERR_NOTUNIQUE;
    }
    return err;
}

