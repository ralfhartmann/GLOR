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
 *  \brief Macros for controlling LEDs
 *
 */
/*!
 * 
 */

#ifndef LED_H
#define LED_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "board.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define LED_1 BOARD_LED1
#define LED_2 BOARD_LED2
#define LED_3 BOARD_LED3
#define LED_4 BOARD_LED4

#define LED_USB    LED_1
#define LED_AS3911 LED_2
#define LED_RESP   LED_3

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#if !DOOR_BOARD
/*! 
 * macro for switching on given LED
 */ 
#define LED_ON(LED) LED = 0

/*! 
 * macro for switching off given LED
 */ 
#define LED_OFF(LED) LED = 1
#else
/*! 
 * macro for switching on given LED
 */ 
#define LED_ON(LED) LED = 1

/*! 
 * macro for switching off given LED
 */ 
#define LED_OFF(LED) LED = 0
#endif

/*! 
 * inline initialization function for LED driver
 */
#define ledInitialize() \
do { } while(0)

#endif /* LED_H */

