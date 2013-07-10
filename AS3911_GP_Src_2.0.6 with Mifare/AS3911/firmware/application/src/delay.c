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
 *  \brief Delay module.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "delay.h"
#include "ic.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static volatile bool_t delayDone;

/*
******************************************************************************
* LOCAL CONSTANTS
******************************************************************************
*/
static const u16 TIMER_MS_TICK = (SYSCLK / 1000);
static const u16 TIMER_US_TICK = ((SYSCLK / 1000) / 1000);

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void INTERRUPT delayIsr(void)
{
    /* clear the interrupt flag */
    icClearInterrupt(IC_SOURCE_DELAY);
    icDisableInterrupt(IC_SOURCE_DELAY);
    T4CONbits.TON = 0;
    delayDone = TRUE;
}

s8 delayInitialize()
{
    /* following timers are used:
       - timer 4+5 as 32 bit timer for milli seconds delay
       - timer 1 as 16 bit timer for micro seconds delay
       */

    /* configure timer 4 to work as 32bit timer together with timer 5 */
    T4CON = (1 << 3);

    T1CON = 0;

    /* clear interrupt flags */
    IFS1bits.T5IF = 0;
    IFS0bits.T1IF = 0;

    return ERR_NONE;
}

s8 delayDeinitialize()
{
    T4CON = 0;
    T1CON = 0;

    return ERR_NONE;
}

s8 delayNMilliSeconds(u16 ms)
{
    u32 period;

    delayDone = FALSE;
    period = (u32)TIMER_MS_TICK * ms;
    PR4 = period & 0xffff;
    PR5 = (period >> 16) & 0xffff;
    /* clear timer */
    TMR4 = 0;
    TMR5 = 0;
    icEnableInterrupt(IC_SOURCE_DELAY);
    /* enable timer */
    T4CONbits.TON = 1;
    SLEEP_CPU();
    while (!delayDone)
        ;
#if 0
    while (!IFS1bits.T5IF)
        ;
#endif

    return ERR_NONE;
}

void delayNMilliSecondsStop()
{
    T4CONbits.TON = 0;
}

void delayNMilliSecondsStart(u16 ms)
{
    u32 period;

    delayDone = FALSE;
    period = (u32)TIMER_MS_TICK * ms;
    PR4 = period & 0xffff;
    PR5 = (period >> 16) & 0xffff;
    /* clear timer */
    TMR4 = 0;
    TMR5 = 0;
    /* enable timer */
    T4CONbits.TON = 1;

    icEnableInterrupt(IC_SOURCE_DELAY);
}

bool_t delayNMilliSecondsIsDone(bool_t do_sleep)
{
    if (do_sleep)
    {
        asm volatile("disi #0x3fff");
        if(!delayDone)
        {
            SLEEP_CPU();
            asm volatile("disi #0x0000");
        }
    }
    return delayDone;
}

s8 delayNMicroSeconds(u16 us)
{
    delayNMicroSecondsPrepare(us);
    delayNMicroSecondsRun();
    return ERR_NONE;
}

s8 delayNMicroSecondsPrepare(u16 us)
{
    PR1 = (u16)(((u32)TIMER_MS_TICK * us) / 1000);

    return ERR_NONE;
}

s8 delayNMicroSecondsRun()
{
    IFS0bits.T1IF = 0;
    TMR1 = 0;
    T1CONbits.TON = 1;

    while (!IFS0bits.T1IF)
        ;
    T1CONbits.TON = 0;
    return ERR_NONE;
}

void stopWatchInitialize()
{
    /* following timers are used:
       - timer 2+3 as 32 bit timer for milli seconds measurment
       */

    /* disable timer */
    T2CONbits.TON = 0;

    /* configure timer 2 to work as 32bit timer together with timer 3 */
    T2CON = (1 << 3);
    
    PR2 = 0xffff;
    PR3 = 0xffff;
}

void stopWatchStart()
{
    /* disable timer */
    T2CONbits.TON = 0;

    /* clear timer */
    TMR2 = 0;
    TMR3 = 0;

    /* enable timer */
    T2CONbits.TON = 1;
}

u32 stopWatchMeasure()
{
    u32 val;
    u16 t2, t3;

    t2 = TMR2;
    t3 = TMR3HLD;

    val = ((u32)t2) | (((u32)t3)<<16UL);
    val += TIMER_US_TICK / 2;
    val /= TIMER_US_TICK;

    return val;
}
