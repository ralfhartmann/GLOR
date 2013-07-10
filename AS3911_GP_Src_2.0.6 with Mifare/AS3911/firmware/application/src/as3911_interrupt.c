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
 *  \brief AS3911 Interrupt handling and ISR
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "as3911_interrupt.h"
#include "as3911_com.h"
#include "ic.h"
#include "led.h"
#include "delay.h"
#include "as3911.h"
#include "board.h"

#define AS3911_IRQ_MASK_TIM (0x02) /*< additional interrupts in AS3911_REG_IRQ_TIMER_NFC */
#define AS3911_IRQ_MASK_ERR (0x01) /*< additional interrupts in AS3911_REG_IRQ_ERROR_WUP */

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/
volatile u32 as3911InterruptStatus = 0;
volatile u32 as3911InterruptMask = 0; /* negative mask = AS3911 mask regs */

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void INTERRUPT as3911Isr(void)
{
    u8 iregs[3] = {0,0,0};
    u32 irqStatus;

    /* clear the interrupt flag */
    icClearInterrupt(IC_SOURCE_AS3911);
    do
    {
        /* just read out the status register to keep the isr short and simple */
        as3911ReadRegister(AS3911_REG_IRQ_MAIN,iregs);
        if (iregs[0] & AS3911_IRQ_MASK_TIM)
        {
            as3911ReadRegister(AS3911_REG_IRQ_TIMER_NFC,iregs + 1);
        }
        if (iregs[0] & AS3911_IRQ_MASK_ERR)
        {
            as3911ReadRegister(AS3911_REG_IRQ_ERROR_WUP,iregs + 2);
        }
        irqStatus  = (u32)iregs[0];
        irqStatus |= (u32)iregs[1]<<8;
        irqStatus |= (u32)iregs[2]<<16; 
        /* forward all interrupts, even masked ones to application. */
        as3911InterruptStatus |= irqStatus;
    } while(AS3911_PORT_INTR); /* PIC24 irq is edge triggered, sometimes there is no edge but still an interrupt */
}

s8 as3911ModifyInterrupts(u32 clr_mask, u32 set_mask)
{
    s8 err = 0;
    int i;
    u32 new_mask;

    icDisableInterrupt(IC_SOURCE_AS3911);
    new_mask = (~as3911InterruptMask & set_mask) |
                (as3911InterruptMask & clr_mask);
    as3911InterruptMask &= ~clr_mask;
    as3911InterruptMask |= set_mask;
    for (i=0; i<3 ; i++)
    { 
        if (! ((new_mask >> (8*i)) & 0xff)) continue;
        err = as3911WriteRegister(AS3911_REG_IRQ_MASK_MAIN + i,
                (as3911InterruptMask>>(8*i))&0xff);
        //dbgLog("INT %x -> %x\n", AS3911_REG_IRQ_MASK_MAIN + i, (as3911InterruptMask>>(8*i))&0xff);
        /* Writes also AS3911_REG_IRQ_MASK_TIMER_NFC and AS3911_REG_IRQ_MASK_ERROR_WUP */
    }
    icEnableInterrupt(IC_SOURCE_AS3911);
    return err;
}

s8 as3911EnableInterrupts(u32 mask)
{
    return as3911ModifyInterrupts(mask,0);
}

s8 as3911DisableInterrupts(u32 mask)
{
    return as3911ModifyInterrupts(0,mask);
}

s8 as3911ClearInterrupts()
{
    s8 err;
    u8 iregs[3];

    err = as3911ReadMultipleRegisters(AS3911_REG_IRQ_MAIN, iregs, 3);

    if (ERR_NONE == err)
    {
        icDisableInterrupt(IC_SOURCE_AS3911);
        as3911InterruptStatus = 0;
        icEnableInterrupt(IC_SOURCE_AS3911);
    }

    return err;
}

u32 as3911WaitForInterruptsTimed(u32 mask, u16 tmo)
{
    u32 status = as3911InterruptStatus & mask;
    bool_t timer_done = FALSE;


    if (tmo) delayNMilliSecondsStart(tmo);

    while (!status && !timer_done)
    {
        if (tmo)
        { /* We could sleep here but this will open a race condition if the 
             AS3911 interrupt occurs before entering sleep which will lead to 
             extra 1ms delay which will cause data loss on FIFO reading. */
            timer_done = delayNMilliSecondsIsDone(FALSE);
        }
        status = as3911InterruptStatus & mask;
    }
    if (tmo) delayNMilliSecondsStop();

    icDisableInterrupt(IC_SOURCE_AS3911);
    as3911InterruptStatus &= ~status;
    icEnableInterrupt(IC_SOURCE_AS3911);

    return status;
}

u32 as3911GetInterrupt(u32 mask)
{
    mask &= as3911InterruptStatus;

    if (mask)
    {
        /* clear interrupt */
        icDisableInterrupt(IC_SOURCE_AS3911);
        as3911InterruptStatus &= ~mask;
        icEnableInterrupt(IC_SOURCE_AS3911);
    }

    return mask;
}

