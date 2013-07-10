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
 *  \brief Low level implementation of FeliCa
 *
 */
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "felica.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "logger.h"
#include "iso14443_common.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

#define FELICA_DEBUG dbgLog
//#define FELICA_DEBUG(...)

/* FELICA_MASK_RECEIVE_TIME spec: ~42*64/fc = 197us, set it to half: */
#define FELICA_MASK_RECEIVE_TIME 21

/*  FeliCa Response Time A is 2.417 ms ~512*64/fc, higher than can be set in AS3911 -> max */
#define FELICA_POLL_WAIT_TIME 255

/*  FeliCa Response Time A is 1.208 ms ~256*64/fc */
#define FELICA_POLL_SLOT_TIME 256

/* FeliCa max FWT is almost unlimited. However this is much too large for our USB 
   communication. Limit to 10ms = 21186 * 64/fc. */
#define FELICA_FRAME_WAIT_TIME  21186

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static u8 felicaSavedOpReg;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 felicaInitialize(u8 modulation_index)
{
    s8 err;
    u8 buf[3];
    u8 result;

    as3911ReadRegister(AS3911_REG_OP_CONTROL,&felicaSavedOpReg);

    buf[0] = (felicaSavedOpReg
           | AS3911_REG_OP_CONTROL_en
           | AS3911_REG_OP_CONTROL_tx_en
           | AS3911_REG_OP_CONTROL_rx_en
           )
           & ~AS3911_REG_OP_CONTROL_wu;

    buf[1] = AS3911_REG_MODE_om_felica;
    buf[2] = 0x11; /* AS3911_REG_BIT_RATE : 212 kBit/s both direction */

    /* set felica mode, lowest Rx and TX rate. Enable Rx and Tx */
    err = as3911WriteMultipleRegisters(AS3911_REG_OP_CONTROL, buf, 3);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_tr_am, AS3911_REG_AUX_tr_am);
    /* Set modulation depth to given value. */
    as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONTROL, modulation_index);
    /* calibrate modulation depth */

    as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_tr_am, 0);

    as3911CalibrateModulationDepth(&result);

    err = as3911ExecuteCommand(AS3911_CMD_ANALOG_PRESET);

out:
    return err;
}

s8 felicaDeinitialize(u8 keep_on)
{
    s8 err;

    /* disable rx and tx */
    felicaSavedOpReg &= ~ (AS3911_REG_OP_CONTROL_tx_en | AS3911_REG_OP_CONTROL_rx_en);
    if (keep_on)
    {
        felicaSavedOpReg |=
              AS3911_REG_OP_CONTROL_en
            | AS3911_REG_OP_CONTROL_tx_en;
    }

    err = as3911WriteRegister(AS3911_REG_OP_CONTROL, felicaSavedOpReg);

    return err;
}

s8 felicaTxRxNBytes(const u8 *txbuf, u16 sizeof_txbuf, u8 *rxbuf, u16 sizeof_rxbuf, u16 *actrxlength)
{
    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, FELICA_MASK_RECEIVE_TIME);

    as3911SetNoResponseTime_64fcs(FELICA_FRAME_WAIT_TIME);

    return iso14443TransmitAndReceive(txbuf, sizeof_txbuf,
                                      rxbuf, sizeof_rxbuf,
                                      actrxlength);
}

s8 felicaPoll(enum felicaSlots slots,
        u8 sysCode1,
        u8 sysCode2,
        enum felicaComParamRequest compar,
        struct felicaProximityCard * card,
        u8 *num_cards,
        u8 *num_cols)
{
    u8 polling[5];
    u16 actLen;
    s8 err;
    u8 crds = *num_cards;
    *num_cols = 0;
    *num_cards = 0;

    polling[0] = FELICA_CMD_POLLING;
    polling[1] = sysCode1; /*  System Code */
    polling[2] = sysCode2; /*  System Code */
    polling[3] = compar; /* Communication Parameter Request */
    polling[4] = slots; /* TimeSlot */

    as3911PrepareReceive(TRUE);

    /* NRT should not stop on reception */
    as3911ModifyRegister(AS3911_REG_GPT_CONTROL,
                         AS3911_REG_GPT_CONTROL_nrt_emv,
                         AS3911_REG_GPT_CONTROL_nrt_emv);

    as3911SetNoResponseTime_64fcs(FELICA_POLL_SLOT_TIME * (slots + 1 + 2));
    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, FELICA_POLL_WAIT_TIME);

    as3911TxNBytes(polling, sizeof(polling), 0, AS3911_TX_FLAG_CRC);

    do { 
        as3911EnableInterrupts(AS3911_IRQ_MASK_RXS |
                        AS3911_IRQ_MASK_RXE |
                        AS3911_IRQ_MASK_FWL |
                        AS3911_IRQ_MASK_PAR |
                        AS3911_IRQ_MASK_CRC |
                        AS3911_IRQ_MASK_NRE |
                        AS3911_IRQ_MASK_ERR1);
        err = as3911RxNBytes((u8*)card, sizeof(*card), &actLen, 0);
        if (ERR_NONE == err)
        {
            as3911ExecuteCommand(AS3911_CMD_UNMASK_RECEIVE_DATA);
            card++;
            crds--;
            (*num_cards)++;
        }
        else if (ERR_TIMEOUT == err)
        {
            break;
        }
        else
        {
            (*num_cols)++;
        }
    }while (slots-- && crds);

    /* restore NRT to normal mode */
    as3911ModifyRegister(AS3911_REG_GPT_CONTROL,
                         AS3911_REG_GPT_CONTROL_nrt_emv,
                         0);
    return ERR_NONE;
}
