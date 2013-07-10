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
 *  \brief clock driver declaration file
 *
 */
/*!
 * 
 * The clock driver provides simple functionality for changing the clock source.
 * This is needed since the controller should use the AS3911 oscillator output.
 * This output is not enabled per default but needs to be configured first.
 * Therefore the controller comes up with a different clock source (e.g. internal
 * oscillator), configures the AS3911 oscillator and switches then to this clock source.
 *
 * API:
 * - Initialize clock driver #clkInitialize
 * - Switch to new clock source #clkSetClockSource
 */

#ifndef CLOCK_H
#define CLOCK_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/
typedef enum
{
    CLK_SOURCE_INTERNAL,
    CLK_SOURCE_EXTERNAL
}clkSource_t;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Initialize clock driver
 *
 *  \return ERR_NONE : No error, clock driver initialized.
 *
 *****************************************************************************
 */
extern s8 clkInitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Switch to a given clock source
 *
 *  Using this function the controllers clock source can be changed.
 *  There are two supported clock sources: Internal and external (AS3911
 *  oscillator) clock source.
 *
 *  \param[in] source : CLK_SOURCE_INTERNAL or CLK_SOURCE_EXTERNAL
 *
 *  \return ERR_NONE : No error, clock source switched.
 *
 *****************************************************************************
 */
extern s8 clkSetClockSource(clkSource_t source);

#endif /* CLOCK_H */

