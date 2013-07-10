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
 *  \author Oliver Regenfelder
 *
 *  \brief Implementation of NFCIP-1
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <string.h>
#include "platform.h"
#include "nfc.h"
#include "as3911.h"
#include "as3911_interrupt.h"
#include "as3911_com.h"
#include "logger.h"
#include "led.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

// #define NFC_DEBUG dbgLog
#define NFC_DEBUG(...)
// #define NFC_DEBUG_HEX_DUMP dbgHexDump
#define NFC_DEBUG_HEX_DUMP(...)


/*! Sanity timeout for initial and response RF collision avoidance completed interrupt (milliseconds). */
#define NFC_RFCA_IRQ_TIMEOUT    10

/*! Sanity timeout for oscillator activation after a bitrate detection interrupt (milliseconds). */
#define NFC_OSC_IRQ_TIMEOUT     10

/*! Size of the nfc module receive buffer (bytes).
 *   1 byte:  SOD (0xF0) marker for 106 kBit/s frames this is stored in the FIFO by v1 silicon.
 * 255 bytes: maximum size für the length byte + payload
 *   2 bytes: CRC, this has to be stored in the FIFO for v2 silicon bug workaround.
 */
#define NFC_RX_BUFFER_SIZE  258

/*! Number of bytes read from the AS3911 when handling a fifo water level irq.
 * \impl This is lower than the FIFO water level to make sure that a FIFO read out
 * does not delay a response collision avoidance.
 */
#define NFC_FIFO_READ_SIZE  32

/*! Response RF collision avoidance timeout is 8192 carrier cycles. */
#define RESPONSE_RFCA_NRT1  0x00
#define RESPONSE_RFCA_NRT2  0x02

/*! Timeout is ~2400 milliseconds. */
#define TIMEOUT_NRT1        0x20
#define TIMEOUT_NRT2        0x00

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

static
struct
{
	u8 regFieldThreshold;
	u8 regRxConf3;
}
 /*! Stores analog settings which are changed by nfcInitialize to restore them in nfcDeinitialize. */
nfcSavedSettings;

/*! Inidcates that target mode bitrate detection irq was fired during reception. */
static bool_t nfcBitrateDetected = FALSE;
/*! Indicates whether the AS3911 is in the bitrate detection low power mode or not. */
static bool_t nfcLowPowerMode = FALSE;
/*! Indicates whether RF response collision avoidance has to be performed after end of reception or not. */
static bool_t nfcPerformResponseCa = FALSE;
/*! Stores the response from the P2P partner. */
static u8 nfcRxBuffer[NFC_RX_BUFFER_SIZE];
/*! Points to one byte after the last valid byte in the receive buffer. */
static u8 *nfcRxBufferInPtr = nfcRxBuffer;
/*! Error status of the last data reception. */
static s8 nfcRxError = ERR_NONE;
static
enum
{
    /*! The nfc module is not initialized. */
    NFC_TASK_INACTIVE,
    /*! The nfc module is waiting for data from the GUI to be transmitted. */
    NFC_TASK_TRANSMITTING,
    /*! Data has been transmitted and the nfc module waits for an EON interrupt
     * to signal that the P2P partner device has performed response RF collision avoidance.
     */
    NFC_TASK_WAIT_FOR_EON,
    /*! The P2P partner device has performed response collision avoidance (EON interrupt was received) and
     * the NFC module is waiting for an RXS interrupt.
     */
    NFC_TASK_WAIT_FOR_RXS,
    /*! An RxS interrupt has been received and reception of data is still in progress. */
    NFC_TASK_RX_IN_PROGRESS,
    /*! An RxE interrupt has been received and the nfc module waits for the P2P partner device to turn off its field. */
    NFC_TASK_WAIT_FOR_EOF,
    /*! Reception is completed. \a nfcRxBuffer contains any received data, and \a nfcRxError the error status of the data
     * reception. If nfcPerformResponseCa was \c TRUE then a response collision avoidance has been performed.
     */
    NFC_TASK_RX_DONE,
} nfcTaskStatus;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

s8 nfcInitialize(u8 is_active, u8 is_initiator, u8 bitrate)
{
    s8 err = ERR_NONE;
    u8 regOpControl = 0;
    
    NFC_DEBUG("nfcInitialize(.is_active=%hhx,.is_initiator=%hhx,.bitrate=%hhx)\n",
            is_active, is_initiator, bitrate);
            
    /* Passive mode is currently not supported by this NFC module. */
    if (!is_active)
        return ERR_PARAM;
            
    err |= as3911ReadRegister(AS3911_REG_OP_CONTROL, &regOpControl);
    if (!(AS3911_REG_OP_CONTROL_en & regOpControl))
    {
        u32 oscIrqs = as3911GetInterrupt(AS3911_IRQ_MASK_OSC);
        as3911EnableInterrupts(AS3911_IRQ_MASK_OSC);
        
        err |= as3911ModifyRegister(AS3911_REG_OP_CONTROL,
            AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en | AS3911_REG_OP_CONTROL_tx_en,
            AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en);
                
        /* Wait for oscillators to start. */
        oscIrqs = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_OSC, NFC_OSC_IRQ_TIMEOUT);
        as3911DisableInterrupts(AS3911_IRQ_MASK_OSC);
                
        if (0 == oscIrqs)
        {
            NFC_DEBUG("NFC_OSC_IRQ_TIMEOUT expired\n");
            return ERR_INTERNAL;
        }
    }

    /* Stop ongoing operations. */
    err |= as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);

    /* Enable receiver, turn off RF field. */
    err |= as3911ModifyRegister(
            AS3911_REG_OP_CONTROL,
            AS3911_REG_OP_CONTROL_rx_en | AS3911_REG_OP_CONTROL_tx_en,
            AS3911_REG_OP_CONTROL_rx_en);
    
    if (is_initiator)
        /* NFCIP1 active communication initiator mode. */
        err |= as3911WriteRegister(AS3911_REG_MODE, 0x00);
    else
        /* NFCIP1 active communication fixed bitrate target mode. */
        err |= as3911WriteRegister(AS3911_REG_MODE, 0x88);
        
    /* Peer detection threshold: 105mV
     * Collision Avoidance Threshold: 105mV
     */
    err |= as3911ReadRegister(AS3911_REG_FIELD_THRESHOLD, &nfcSavedSettings.regFieldThreshold);
    err |= as3911WriteRegister(AS3911_REG_FIELD_THRESHOLD, 0x11);
    /* Silicon v2 bug: we have to always receive crc bytes */
    err |= as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_crc_2_fifo, AS3911_REG_AUX_crc_2_fifo);
        
    /* Start the GPT after end of TX, EMV no response timer mode, no response timer set in 4096 fc intervalls. */
    err |= as3911WriteRegister(AS3911_REG_GPT_CONTROL, 0x63);
    /* The field is turned off 37.76µs after the end of the transmission. */
    err |= as3911WriteRegister(AS3911_REG_GPT1, 0x00);
    err |= as3911WriteRegister(AS3911_REG_GPT2, 0x40);

    /* Mask receive timer. */
    err |= as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, 0x01);
        
    /* Reduce first stage gain for 106 kBit/s bitrate.
     */
    err |= as3911ReadRegister(AS3911_REG_RX_CONF3, &nfcSavedSettings.regRxConf3);
    if (0x00 == bitrate)
        err |= as3911ModifyRegister(AS3911_REG_RX_CONF3, 0xE0, 0xC0);
    else
        err |= as3911ModifyRegister(AS3911_REG_RX_CONF3, 0xE0, 0x00);
        
    if (bitrate > 0x02)
        return ERR_PARAM;
    err |= as3911SetBitrate(bitrate,bitrate);
    
    if (bitrate > 0x00)
    {
        /* Set to 25% AM modulation.
		 * modulation index: 25%
		 * a/b = 1.66666666
		 * a/b = b1.101010
		 */
        err |= as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONTROL, 0x54);
		err |= as3911CalibrateModulationDepth(NULL);
    }
    
    err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER1, TIMEOUT_NRT1);
    err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER2, TIMEOUT_NRT2);

    err |= as3911ClearInterrupts();
    err |= as3911EnableInterrupts(AS3911_IRQ_MASK_RXS | AS3911_IRQ_MASK_RXE | AS3911_IRQ_MASK_FWL | AS3911_IRQ_MASK_NRE | AS3911_IRQ_MASK_NFCT | AS3911_IRQ_MASK_EOF | AS3911_IRQ_MASK_EON | AS3911_IRQ_MASK_CRC | AS3911_IRQ_MASK_PAR | AS3911_IRQ_MASK_ERR2 | AS3911_IRQ_MASK_ERR1);

    if (is_initiator)
    {
        nfcTaskStatus = NFC_TASK_TRANSMITTING;
        nfcBitrateDetected = FALSE;
        nfcLowPowerMode = FALSE;
        nfcPerformResponseCa = FALSE;
        nfcRxBufferInPtr = nfcRxBuffer;
        nfcRxError = ERR_NONE;
    }
    else
    {
        nfcTaskStatus = NFC_TASK_INACTIVE;
        nfcBitrateDetected = FALSE;
        nfcLowPowerMode = FALSE;
        nfcPerformResponseCa = FALSE;
        nfcRxBufferInPtr = nfcRxBuffer;
        nfcRxError = ERR_NONE;
    }

    if (ERR_NONE != err)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 nfcDeinitialize()
{
    s8 err = ERR_NONE;
    u8 regOpControl = 0;

    NFC_DEBUG("nfcDeinitialize()\n");
                
    nfcTaskStatus = NFC_TASK_INACTIVE;

    err |= as3911ReadRegister(AS3911_REG_OP_CONTROL, &regOpControl);
    if (!(AS3911_REG_OP_CONTROL_en & regOpControl))
    {
        u32 oscIrqs = as3911GetInterrupt(AS3911_IRQ_MASK_OSC);
        as3911EnableInterrupts(AS3911_IRQ_MASK_OSC);
        
        err |= as3911ModifyRegister(AS3911_REG_OP_CONTROL,
            AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en | AS3911_REG_OP_CONTROL_tx_en,
            AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en);
                
        /* Wait for oscillators to start. */
        oscIrqs = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_OSC, NFC_OSC_IRQ_TIMEOUT);
        as3911DisableInterrupts(AS3911_IRQ_MASK_OSC);
                
        if (0 == oscIrqs)
        {
            NFC_DEBUG("NFC_OSC_IRQ_TIMEOUT expired\n");
            return ERR_INTERNAL;
        }
    }

    err |=  as3911SetBitrate(0,0);
	
    /* Restore old settings. */
    err |= as3911WriteRegister(AS3911_REG_FIELD_THRESHOLD, nfcSavedSettings.regFieldThreshold);
    err |= as3911WriteRegister(AS3911_REG_RX_CONF3, nfcSavedSettings.regRxConf3);
    err |= as3911WriteRegister(AS3911_REG_GPT_CONTROL, 0);

    /* Turn off reader field. */
    err |= as3911ModifyRegister(AS3911_REG_OP_CONTROL, AS3911_REG_OP_CONTROL_tx_en, 0);

    if (ERR_NONE != err)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 nfcTxNBytes(const u8 *buf, u8 bufSize, u8 perform_collision_avoidance)
{
    s8 err = ERR_NONE;
    u8 regValue = 0;
    
    NFC_DEBUG("nfcTxNBytes(.buf=");
    NFC_DEBUG_HEX_DUMP(buf,bufSize);
    NFC_DEBUG(".bufSize=%hhx,.perform_collision_avoidance=%hhx)\n",
            bufSize, perform_collision_avoidance);

    err |= as3911ReadRegister(AS3911_REG_OP_CONTROL, &regValue);
    
    if (!(regValue & AS3911_REG_OP_CONTROL_tx_en))
    {
        /* RF field is off, perform initial RF collision avoidance. */
        u32 irqs = 0;
        
        err |= as3911EnableInterrupts(AS3911_IRQ_MASK_CAC | AS3911_IRQ_MASK_CAT);
        err |= as3911ExecuteCommand(AS3911_CMD_INITIAL_RF_COLLISION);
        irqs = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_CAC | AS3911_IRQ_MASK_CAT, NFC_RFCA_IRQ_TIMEOUT);
        err |= as3911DisableInterrupts(AS3911_IRQ_MASK_CAC | AS3911_IRQ_MASK_CAT);
        
        if (0 == irqs)
        {
            NFC_DEBUG("NFC_INITIAL_RFCA_IRQ_TIMEOUT expired\n");
            return ERR_INTERNAL;
        }
        else if (AS3911_IRQ_MASK_CAC & irqs)
        {
            return ERR_RF_COLLISION;
        }
    }
    
    /* Setup NRT timer for rf response RF collision timeout. */
    err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER1, RESPONSE_RFCA_NRT1);
    err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER2, RESPONSE_RFCA_NRT2);
    
    nfcPerformResponseCa = perform_collision_avoidance;
    as3911PrepareReceive(TRUE);
    err |= as3911TxNBytes(buf, bufSize, 0, AS3911_TX_FLAG_CRC);
    
    nfcRxBufferInPtr = nfcRxBuffer;
    nfcTaskStatus = NFC_TASK_WAIT_FOR_EON;

    if (ERR_NONE != err)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 nfcRxNBytes(u8 *buf, u8 bufsize, u8 *actlen)
{
    NFC_DEBUG("nfcRxNBytes( .bufSize=%hhx)\n");
    *actlen = 0;
    
    if (NFC_TASK_RX_DONE == nfcTaskStatus)
    {
        *actlen = nfcRxBufferInPtr - nfcRxBuffer - 2;
        memcpy(buf, nfcRxBuffer, *actlen);
        return nfcRxError;
    }

    return ERR_BUSY;
}

s8 nfcSwitchToRx(u8 perform_collision_avoidance)
{
    u8 err = ERR_NONE;
    NFC_DEBUG("nfcSwitchToRx()");
    
    if (!(NFC_TASK_RX_DONE == nfcTaskStatus))
        return ERR_IO;
    
    /* Setup NRT timer for rf response RF collision timeout. */
    err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER1, RESPONSE_RFCA_NRT1);
    err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER2, RESPONSE_RFCA_NRT2);

    /* Prepare data reception. */
    nfcPerformResponseCa = perform_collision_avoidance;
    as3911PrepareReceive(TRUE);
    
    /* Enter low power bitrate detection mode. */
    err |= as3911WriteRegister(AS3911_REG_MODE, 0x80);

    /* Start the NRE timer. */
    err |= as3911ExecuteCommand(AS3911_CMD_START_NO_RESPONSE_TIMER);

    /* Disable the RF field manually. */
    err |= as3911ModifyRegister(AS3911_REG_OP_CONTROL, AS3911_REG_OP_CONTROL_tx_en, 0);
    
    nfcRxBufferInPtr = nfcRxBuffer;
    nfcTaskStatus = NFC_TASK_WAIT_FOR_EON;
    
    while (nfcTaskStatus != NFC_TASK_RX_DONE)
        err |= nfcReceptionTask();
    
    if (ERR_NONE != err)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 nfcStartInitialTargetRx()
{
    s8 err = ERR_NONE;
    
    NFC_DEBUG("nfcStartInitialTargetRx()\n");

    if (  (NFC_TASK_TRANSMITTING == nfcTaskStatus)
       || (NFC_TASK_RX_IN_PROGRESS == nfcTaskStatus))
       return ERR_BUSY;
       
    err |= as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);
        
    /* Enter low power bitrate detection mode. */
    err |= as3911WriteRegister(AS3911_REG_MODE, 0x80);
        
    /* Turn off the chip. */
    err |= as3911ModifyRegister(AS3911_REG_OP_CONTROL,
        AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en | AS3911_REG_OP_CONTROL_tx_en,
        0x00);
        
    if (ERR_NONE != err)
        return ERR_IO;
    
    nfcLowPowerMode = TRUE;
    nfcPerformResponseCa = TRUE;
    nfcRxBufferInPtr = nfcRxBuffer;
    nfcRxError = ERR_NONE;
    nfcTaskStatus = NFC_TASK_WAIT_FOR_EON;
    
    return ERR_NONE;
}

s8 nfcReceptionTask()
{
    s8 err = ERR_NONE;
    u32 irqs = 0;
    
    if (NFC_TASK_INACTIVE == nfcTaskStatus)
        return ERR_NONE;
        
    irqs = as3911GetInterrupt(AS3911_IRQ_MASK_RXS | AS3911_IRQ_MASK_RXE | AS3911_IRQ_MASK_FWL | AS3911_IRQ_MASK_NRE | AS3911_IRQ_MASK_NFCT | AS3911_IRQ_MASK_EON | AS3911_IRQ_MASK_EOF);
    
    if (AS3911_IRQ_MASK_EON & irqs)
    {
        if (NFC_TASK_WAIT_FOR_EON == nfcTaskStatus)
        {
            if (nfcLowPowerMode)
            {
                nfcLowPowerMode = FALSE;
                /* Clear oscillator on interrupt flag. */
                u32 oscIrqs = as3911GetInterrupt(AS3911_IRQ_MASK_OSC);
                as3911EnableInterrupts(AS3911_IRQ_MASK_OSC);
                
                err |= as3911ModifyRegister(AS3911_REG_OP_CONTROL,
                    AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en | AS3911_REG_OP_CONTROL_tx_en,
                    AS3911_REG_OP_CONTROL_en | AS3911_REG_OP_CONTROL_rx_en);
                
                /* Wait for oscillators to start. */
                oscIrqs = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_OSC, NFC_OSC_IRQ_TIMEOUT);
                as3911DisableInterrupts(AS3911_IRQ_MASK_OSC);
                
                if (0 == oscIrqs)
                {
                    NFC_DEBUG("NFC_OSC_IRQ_TIMEOUT expired\n");
                    return ERR_INTERNAL;
                }
                
                /* Switch to normal target mode. */
                // err |= as3911WriteRegister(AS3911_REG_MODE, 0x88);
            }
            
            err |= as3911ExecuteCommand(AS3911_CMD_UNMASK_RECEIVE_DATA);
            
            /* The no response timer is stopped on an EON event. We therefore
            * restart the timer to be able to use it as a message timeout timer.
            * In addition to its NFC-P2P mode default usage as an external field on
            * timeout timer.
            */
            err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER1, TIMEOUT_NRT1);
            err |= as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER2, TIMEOUT_NRT2);
            err |= as3911ExecuteCommand(AS3911_CMD_START_NO_RESPONSE_TIMER);
            
            nfcTaskStatus = NFC_TASK_WAIT_FOR_RXS;
        }
    }
    if (AS3911_IRQ_MASK_RXS & irqs)
    {
        nfcRxBufferInPtr = nfcRxBuffer;
        nfcTaskStatus = NFC_TASK_RX_IN_PROGRESS;
    }
    if (AS3911_IRQ_MASK_FWL & irqs)
    {
        err |= as3911ReadFifo(nfcRxBufferInPtr, NFC_FIFO_READ_SIZE);
        nfcRxBufferInPtr += NFC_FIFO_READ_SIZE;
    }
    if (AS3911_IRQ_MASK_RXE & irqs)
    {
        /* Reading the FIFO is postponed until the response collision avoidance has been performed. */
        nfcTaskStatus = NFC_TASK_WAIT_FOR_EOF;
    }
    if (AS3911_IRQ_MASK_NFCT & irqs)
    {
        nfcBitrateDetected = TRUE;
    }
    if (AS3911_IRQ_MASK_NRE & irqs)
    {
        if (  (NFC_TASK_WAIT_FOR_EON == nfcTaskStatus)
           || (NFC_TASK_WAIT_FOR_RXS == nfcTaskStatus))
        {
            nfcTaskStatus = NFC_TASK_RX_DONE;
            nfcRxError = ERR_TIMEOUT;
        }
    }
    if (AS3911_IRQ_MASK_EOF & irqs)
    {
        /* A bitrate detection event can only happen when we are in the bitrate
         * detection target mode. Thus, if such an event has happened during the
         * reception (the irq happens shortly after the RxS irq) we need to switch
         * back to normal nfc mode.
         */
        if (  (NFC_TASK_WAIT_FOR_EOF == nfcTaskStatus)
           && nfcBitrateDetected)
        {
            err |= as3911ExecuteCommand(AS3911_CMD_NORMAL_NFC_MODE);
        }
        nfcBitrateDetected = FALSE;
        
        if (  (NFC_TASK_WAIT_FOR_EOF == nfcTaskStatus)
           && nfcPerformResponseCa)
        {
            nfcPerformResponseCa = FALSE;
        
            err |= as3911EnableInterrupts(AS3911_IRQ_MASK_CAC | AS3911_IRQ_MASK_CAT);
            err |= as3911ExecuteCommand(AS3911_CMD_RESPONSE_RF_COLLISION_0);
            irqs = as3911WaitForInterruptsTimed(AS3911_IRQ_MASK_CAC | AS3911_IRQ_MASK_CAT, NFC_RFCA_IRQ_TIMEOUT);
            err |= as3911DisableInterrupts(AS3911_IRQ_MASK_CAC | AS3911_IRQ_MASK_CAT);
        
            if (0 == irqs)
            {
                NFC_DEBUG("NFC_RFCA_IRQ_TIMEOUT expired\n");
                nfcTaskStatus = NFC_TASK_RX_DONE;
                nfcRxError = ERR_INTERNAL;
            }
            else if (AS3911_IRQ_MASK_CAC & irqs)
            {
                nfcTaskStatus = NFC_TASK_RX_DONE;
                nfcRxError = ERR_RF_COLLISION;
            }
        }
        
        if (  (NFC_TASK_WAIT_FOR_RXS == nfcTaskStatus)
           || (NFC_TASK_RX_IN_PROGRESS == nfcTaskStatus))
        {
            nfcTaskStatus = NFC_TASK_RX_DONE;
            nfcRxError = ERR_TIMEOUT;
        }
        
        if (NFC_TASK_WAIT_FOR_EOF == nfcTaskStatus)
        {
            u8 fifoReadSize;
             
            err |= as3911ReadRegister(AS3911_REG_FIFO_RX_STATUS1, &fifoReadSize);
            as3911ReadFifo(nfcRxBufferInPtr, fifoReadSize);
            nfcRxBufferInPtr += fifoReadSize;
            nfcTaskStatus = NFC_TASK_RX_DONE;
            
            irqs = as3911GetInterrupt(AS3911_IRQ_MASK_CRC | AS3911_IRQ_MASK_PAR | AS3911_IRQ_MASK_ERR2 | AS3911_IRQ_MASK_ERR1);
            if (irqs & AS3911_IRQ_MASK_CRC)
                nfcRxError = ERR_CRC;
            else if (irqs & AS3911_IRQ_MASK_PAR)
                nfcRxError = ERR_PAR;
            else if ((irqs & AS3911_IRQ_MASK_ERR1) || (irqs & AS3911_IRQ_MASK_ERR2))
                nfcRxError = ERR_FRAMING;
            else if (0 != irqs)
                nfcRxError = ERR_FRAMING;
            else
            nfcRxError = ERR_NONE;

            
            /* Stop NRE timer via a clear fifo cmd. */
            as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);
        
            /* Clear pending NRE timer events. */
            as3911GetInterrupt(AS3911_IRQ_MASK_NRE);
            irqs &= ~AS3911_IRQ_MASK_NRE;
        }
    }
    
    if (ERR_NONE != err)
        return ERR_IO;
    else
        return ERR_NONE;
}
