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
 *  \brief Clock driver for PIC24F.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "clock.h"

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 clkInitialize()
{
    return 0;
    CLKDIV = 0;

    /* switch to default clock source (internal) */
    return clkSetClockSource(CLK_SOURCE_INTERNAL);
}

s8 clkSetClockSource(clkSource_t source)
{
    u8 src = 0x0;
    return 0;

    IRQ_INC_DISABLE();

    /* first switch to frc (w/o pll) since switching between two pll sources is
       not allowed. */
    /* Unlock the OSCCON high byte and set new oscillator bits. */
    __builtin_write_OSCCONH(0x0);

    /* unlock the OSCCON low byte and set oscillator switch enable to 1. */
    __builtin_write_OSCCONL(0x01);

    /* Wait until oscillator switch is finished. */
    while((OSCCON & 0x1) != 0x0);

    switch(source)
    {
        case CLK_SOURCE_EXTERNAL:
            src = 0x3;
            break;
        default:
            src = 0x1;

    }

    /* Unlock the OSCCON high byte and set new oscillator bits. */
    __builtin_write_OSCCONH(src);

    /* unlock the OSCCON low byte and set oscillator switch enable to 1. */
    __builtin_write_OSCCONL(0x01);

    /* Wait until oscillator switch is finished. */
    while((OSCCON & 0x1) != 0x0)
        ;
    /* Wait until PLL is locked */
    while((OSCCON & 0x20) != 0x20)
        ;

    IRQ_DEC_ENABLE();

    return ERR_NONE;
}

