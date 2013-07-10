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
 *      PROJECT:   AS3911
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author Christian Eisendle
 *
 *  \brief Platform specific header file
 *
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
//#include <p24FJ64GB002.h>
#include <p24Fxxxx.h>
#include "ams_types.h"
#include "as3911errno.h"
#include "utils.h"

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/
extern umword IRQ_COUNT;

//#define SYSCLK             13560000UL    /*!< SYSCLK frequency in Hz */
#define SYSCLK             16000000UL    /*!< SYSCLK frequency in Hz */
/* for __delay_us() and __delay_ms() */
#define FCY 16000000ULL

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#define SLEEP_CPU() __asm__ volatile("PWRSAV #1")

#define RESET_CPU() __asm__ volatile("reset")

#define NOP() Nop() /*!< map NOP to something which executes a
                        single cycle instruction - use ful for delays */

/*! macro which globally disables interrupts and increments interrupt count */
#define IRQ_INC_DISABLE() do {                                              \
    __asm__ volatile("disi #0x3FFF");                                       \
    IRQ_COUNT++;                                                            \
} while(0)


/*! macro to globally enable interrupts again if interrupt count is 0 */
#define IRQ_DEC_ENABLE() do {                                               \
   if (IRQ_COUNT != 0) IRQ_COUNT--;                                         \
   if (IRQ_COUNT == 0)  __asm__ volatile("disi #0x0");                      \
} while(0)

/*! on pic platform, signature of interrupt service routing must include
 * __attribute__((interrupt, no_auto_psv))
 */
#define INTERRUPT __attribute__((interrupt, no_auto_psv))

/*! map as3911Isr to _IC1Interrupt */
#define as3911Isr _IC1Interrupt

/*! map delayIsr to _T5Interrupt */
#define delayIsr _T5Interrupt

#include <libpic30.h>

#endif /* PLATFORM_H */

