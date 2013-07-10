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
 *  \brief delay module declaration file
 *
 */
/*!
 * 
 * The delay module is used to delay the current program for given number
 * of milli or micro seconds. It makes use of the system timers to allow
 * quite exact delays.
 *
 * API:
 * - Initialize delay module: #delayInitialize
 * - Deinitialize delay module: #delayDeinitialize
 * - Delay for N milli seconds: #delayNMilliSeconds
 * - Delay for N micro seconds: #delayNMicroSeconds
 * - Prepare for later micro seconds delay: #delayNMicroSecondsPrepare
 * - Run previously prepared delay: #delayNMicroSecondsRun
 */

#ifndef DELAY_H
#define DELAY_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Timer Interrupt Service Routine
 *
 *  \note Not to be called directly.
 *
 *****************************************************************************
 */
extern void INTERRUPT delayIsr(void);

/*! 
 *****************************************************************************
 *  \brief  Initialize delay modules
 *
 *  This function initializes the delay module. It sets up the corresponding
 *  timers and local variables.
 *  \note System frequency must not be changed since timer calculation
 *  is done on base of a fixed frequency (#SYSCLK define)
 *
 *  \return ERR_NONE : No error, delay module initialized.
 *
 *****************************************************************************
 */
extern s8 delayInitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Deinitialize delay module
 *
 *  Calling this function deinitializes the delay module.
 *  API calls to delay module (except #delayInitialize) are then not valid
 *  any more.
 *
 *  \return ERR_NONE : No error, delay module deinitialized.
 *
 *****************************************************************************
 */
extern s8 delayDeinitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Delay the program for a given number of milli seconds
 *
 *  This function doesn't return before \a ms milli seconds passed by.
 *
 *  \param[in] ms: number of milli seconds to delay
 *
 *  \return ERR_NONE : No error, program delayed for \a ms milliseconds.
 *
 *****************************************************************************
 */
extern s8 delayNMilliSeconds(u16 ms);

/*! 
 *****************************************************************************
 *  \brief  Delay the program for a given number of micro seconds
 *
 *  This function doesn't return before \a us micro seconds passed by.
 *  \note In case of very small delays (< 5us) to overhead for setting up
 *  the timer might lead to an inaccuracy. In this case the setup should be
 *  done outside of the time critical part by calling #delayNMicroSecondsPrepare.
 *  Then the actual delay loop is started by calling #delayNMicroSecondsRun
 *
 *  \param[in] us: number of micro seconds to delay
 *
 *  \return ERR_NONE : No error, program delayed for \a us micro seconds.
 *
 *****************************************************************************
 */
extern s8 delayNMicroSeconds(u16 us);

/*! 
 *****************************************************************************
 *  \brief  Prepare for a later delay
 *
 *  In case of very small delays (< 5us) to overhead for setting up
 *  the timer might lead to an inaccuracy. In this case the setup should be
 *  done outside of the time critical part by calling this function.
 *  The actual delay is then applied by calling #delayNMicroSecondsRun
 *
 *  \param[in] us: number of micro seconds to delay
 *
 *  \return ERR_NONE : No error, timer prepared for later use.
 *
 *****************************************************************************
 */
extern s8 delayNMicroSecondsPrepare(u16 us);

/*! 
 *****************************************************************************
 *  \brief  Perfrom a previously prepared delay
 *
 *  This function starts the timer (which is used for the delay) configured
 *  before using #delayNMicroSecondsPrepare.
 *  This allows very small delays w/o having a big overhead introduced
 *  by the calculation of the delay.
 *
 *  \return ERR_NONE : No error, program delayed for the previously configured
 *                     amount of time.
 *
 *****************************************************************************
 */
extern s8 delayNMicroSecondsRun();

extern void delayNMilliSecondsStop();
extern void delayNMilliSecondsStart(u16 ms);
extern bool_t delayNMilliSecondsIsDone(bool_t do_sleep);

void stopWatchInitialize();
void stopWatchStart();
u32 stopWatchMeasure();
#endif /* DELAY_H */

