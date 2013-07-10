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
 *  \brief Interrupt controller driver for PIC24f series
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "ic.h"

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 icInitialize()
{
    return ERR_NONE;
}

s8 icDeinitialize()
{
    /* disable all interrupts */
    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;
    IEC3 = 0;
    IEC4 = 0;
    IPC0 = 0;
    IPC1 = 0;
    IPC2 = 0;
    IPC3 = 0;
    IPC4 = 0;
    return ERR_NONE;
}

s8 icEnableInterrupt(icSource_t src)
{
    switch (src)
    {
        case IC_SOURCE_UART:
            IRQ_INC_DISABLE();
            /* enable interrupt request */
            IEC0bits.U1RXIE = 1;
            IPC2bits.U1RXIP = 2;
            U1STAbits.URXISEL = 1;
            IRQ_DEC_ENABLE();
        break;
        case IC_SOURCE_AS3911:
            IRQ_INC_DISABLE();
            /* AS3911 is connected to capture module 1 */
            /* enable interrupt request */
            IEC0bits.IC1IE = 1;
            /* set priority which enables the interrupt */
            IPC0bits.IC1IP = 1;
            /* configure capture module - interrupt on rising edge */
            IC1CON1bits.ICM1 = 1;
            IC1CON1bits.ICM0 = 1;
            IRQ_DEC_ENABLE();
        break;
        case IC_SOURCE_TRANSPARENT_MODE:
            IRQ_INC_DISABLE();
            /* AS3911 transparent mode input is connected to
               external interrupt 1 */
            IPC5bits.INT1IP = 7;
            /* capture on falling edge */
            INTCON2bits.INT1EP = 1;
            IFS1bits.INT1IF = 0;
            /* enable interrupt request */
            IEC1bits.INT1IE = 1;
            IRQ_DEC_ENABLE();
            break;
        case IC_SOURCE_DELAY:
            IRQ_INC_DISABLE();
            IFS1bits.T5IF = 0;
            IEC1bits.T5IE = 1;
            IPC7bits.T5IP = 4;
            IRQ_DEC_ENABLE();
            break;
        default:
            return ERR_PARAM;
    }

    return ERR_NONE;
}

s8 icDisableInterrupt(icSource_t src)
{
    switch (src)
    {
        case IC_SOURCE_AS3911:
            IRQ_INC_DISABLE();
            /* AS3911 is connected to capture module 1 */
            /* set priority to 0 which disables the interrupt */
            IPC0bits.IC1IP = 0;
            /* disable interrupt request */
            IEC0bits.IC1IE = 0;
            IRQ_DEC_ENABLE();
        break;
        case IC_SOURCE_TRANSPARENT_MODE:
            /* disable interrupt request */
            IEC1bits.INT1IE = 0;
            break;
        case IC_SOURCE_UART:
            IEC0bits.U1RXIE = 0;
            IPC2bits.U1RXIP = 0;
            U1STAbits.URXISEL = 0;
            break;
        case IC_SOURCE_DELAY:
            IEC1bits.T5IE = 0;
            IPC7bits.T5IP = 0;
            break;
        default:
            return ERR_PARAM;
    }

    return ERR_NONE;
}

s8 icClearInterrupt(icSource_t src)
{
    switch (src)
    {
        case IC_SOURCE_AS3911:
            /* AS3911 is connected to capture module 1 */
            /* clear interrupts */
            IFS0bits.IC1IF = 0;
        break;
        case IC_SOURCE_UART:
            /* clear interrupts */
            IFS0bits.U1RXIF = 0;
            break;
        case IC_SOURCE_DELAY:
            IFS1bits.T5IF = 0;
            break;
        default:
            return ERR_PARAM;
    }

    return ERR_NONE;
}

