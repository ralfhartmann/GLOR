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
 *  \brief Implementation of Topaz aka NFF type 1 tag
 *
 */
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "topaz.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "iso14443_common.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

//#define TOPAZ_DEBUG dbgLog
#define TOPAZ_DEBUG(...)
/* TOPAZ_MASK_RECEIVE_TIME Spec: FDT = (n * 128 + 84) / fc  with n_min = 9 
   set it lower: 2*5=10 */
#define TOPAZ_MASK_RECEIVE_TIME 10 

/*  REQA, etc. have much shorter time of 1172/fc ~= 19*64/fc */
#define TOPAZ_INVENTORY_WAITING_TIME 35

/* DRD for WRITE_E is n=554 => 1109*64/fc */
#define TOPAZ_WRITE_E_WAITING_TIME 1200
/* DRD for WRITE_E is n=281 => 563*64/fc */
#define TOPAZ_WRITE_NE_WAITING_TIME 600

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static u8 topazSavedOpReg;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 topazInitialize()
{
    s8 err;
    u8 buf[3];

    TOPAZ_DEBUG(__func__);

    as3911ReadRegister(AS3911_REG_OP_CONTROL,&topazSavedOpReg);

    buf[0] = (topazSavedOpReg
           | AS3911_REG_OP_CONTROL_en
           | AS3911_REG_OP_CONTROL_tx_en
           | AS3911_REG_OP_CONTROL_rx_en
           )
           & ~AS3911_REG_OP_CONTROL_wu;

    buf[1] = AS3911_REG_MODE_om_topaz;
    buf[2] = 0x00; /* AS3911_REG_BIT_RATE : 106 kBit/s both direction */

    /* set topaz mode, lowest Rx and TX rate. Enable Rx and Tx */
    err = as3911WriteMultipleRegisters(AS3911_REG_OP_CONTROL, buf, 3);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, TOPAZ_MASK_RECEIVE_TIME);

    as3911SetNoResponseTime_64fcs(TOPAZ_INVENTORY_WAITING_TIME);

    err = as3911ExecuteCommand(AS3911_CMD_ANALOG_PRESET);

out:
    return err;
}

s8 topazDeinitialize(u8 keep_on)
{
    s8 err;

    TOPAZ_DEBUG(__func__);
    /* disable rx and tx */
    topazSavedOpReg &= ~ (AS3911_REG_OP_CONTROL_tx_en | AS3911_REG_OP_CONTROL_rx_en);
    if (keep_on)
    {
        topazSavedOpReg |=
              AS3911_REG_OP_CONTROL_en
            | AS3911_REG_OP_CONTROL_tx_en;
    }

    err = as3911WriteRegister(AS3911_REG_OP_CONTROL, topazSavedOpReg);

    /* enable CRC because it is default */
    err = as3911ModifyRegister(AS3911_REG_AUX,
                               AS3911_REG_AUX_no_crc_rx,
                               0);

    return err;
}

s8 topazReqaWupa(topazCommand_t cmd, topazProximityCard_t* card)
{
    s8 err = ERR_NONE;
    u8 directcmd;
    u8 mask;
    u16 actlength;

    TOPAZ_DEBUG(__func__);
    as3911SetNoResponseTime_64fcs(TOPAZ_INVENTORY_WAITING_TIME);
    /* first disable CRC while receiving since ATQA has no CRC included */
    err = as3911ModifyRegister(AS3911_REG_AUX,
                               AS3911_REG_AUX_no_crc_rx,
                               AS3911_REG_AUX_no_crc_rx);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* enable required interrupts: bit collision, recv error, end of tx and end of rx */
    err = as3911EnableInterrupts(AS3911_IRQ_MASK_COL |
                                 AS3911_IRQ_MASK_TXE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    switch (cmd)
    {
        case TOPAZ_CMD_REQA:
            directcmd = AS3911_CMD_TRANSMIT_REQA;
            break;
        case TOPAZ_CMD_WUPA:
            directcmd = AS3911_CMD_TRANSMIT_WUPA;
            break;
        default:
            err = ERR_PARAM;
            goto out_disable_irq;
    }

    TOPAZ_DEBUG("1");
    /* prepare receive which enables interrupts */
    err = as3911PrepareReceive(TRUE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out_disable_irq);

    /* now send either WUPA or REQA. All affected tags will backscatter ATQA and
       change to READY state */
    err = as3911ExecuteCommand(directcmd);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out_disable_irq);

    TOPAZ_DEBUG("2");
    /* wait for transmit finshed interrupt. */
    mask = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_TXE, 10);
    if (0 == mask)
    {
        TOPAZ_DEBUG("no txe in %s\n",__func__);
        err = ERR_TIMEOUT;
        goto out_disable_irq;
    }
    TOPAZ_DEBUG("Sent WUPA/REQA\n");

    /* request sent - wait for an answer */
    err = as3911RxNBytes((u8*)&card->atqa, sizeof(u16), &actlength, 0);
    if (ERR_TIMEOUT == err)
    {
        /* no tag reply at all*/
        goto out_disable_irq;
    }
    TOPAZ_DEBUG("Got ATQA\n");

out_disable_irq:
    as3911DisableInterrupts(AS3911_IRQ_MASK_COL |
                            AS3911_IRQ_MASK_TXE);
out:
    return err;
}


s8 topazReadUID(topazProximityCard_t* card)
{
    const u8 buf[7] = {TOPAZ_CMD_RID, 0, 0, 0, 0, 0, 0}; 

    return iso14443TransmitAndReceive(buf, sizeof(buf), card->hr,
            sizeof(card->hr) + sizeof(card->uid), &card->actlength);
}

s8 topazReadAll(const topazProximityCard_t* card, u8 *buf, u16 buf_size, u16* act_size)
{
    const u8 cmd[7] = {TOPAZ_CMD_RALL, 0, 0, card->uid[0], card->uid[1], card->uid[2], card->uid[3]};

    return iso14443TransmitAndReceive(cmd, sizeof(cmd), buf, buf_size, act_size);
}

s8 topazWriteByte(topazProximityCard_t* card, u8 addr, u8 data)
{
    const u8 buf[7] = {TOPAZ_CMD_WRITE_E, addr, data, card->uid[0], card->uid[1], card->uid[2], card->uid[3]};
    u8 response[2];
    u16 len;

    as3911SetNoResponseTime_64fcs(TOPAZ_WRITE_E_WAITING_TIME);
    return iso14443TransmitAndReceive(buf, sizeof(buf), response, sizeof(response), &len);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
