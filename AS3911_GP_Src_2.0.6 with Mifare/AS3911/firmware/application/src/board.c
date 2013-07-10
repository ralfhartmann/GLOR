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
 *  \brief Board specific configuration
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <PPS.h>
#include "platform.h"
#include "board.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

/* TRIS bit access macros */
#define _TRIS(PIN)                      _TRISR(PIN)
/* Relay macro to have macro-expansion of the argument */
#define _TRISR(PIN)                      _TRIS##PIN

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 boardInitialize()
{
    /* Set up I/O ports to all digital I/O. */
    AD1PCFG = 0xFFFF;

    _TRIS(AS3911_PIN_TM)   = 0;
    _TRIS(AS3911_PIN_SEN)  = 0;
    _TRIS(LED1_PIN) = 0;
    _TRIS(LED2_PIN) = 0;
    _TRIS(LED3_PIN) = 0;
    _TRIS(LED4_PIN) = 0;
#if DOOR_BOARD
    _TRIS(BEEP_PIN) = 0;
    _LATB3          = 0;
#endif

    return ERR_NONE;
}

s8 boardDeinitialize()
{
    /* reset to POR values */
    TRISA = 0xFFFF;
    TRISB = 0xFFFF;

    return ERR_NONE;
}


s8 boardPeripheralPinInitialize(boardPeripheral_t peri)
{
    s8 err = ERR_NONE;

    switch (peri)
    {
        case BOARD_PERIPHERAL_SPI1:

            IN_FN_PPS_SDI1    = AS3911_PIN_MISO;
            AS3911_PIN_MOSI   = OUT_FN_PPS_SDO1;
            AS3911_PIN_SCLK   = OUT_FN_PPS_SCK1OUT;
            break;

        case BOARD_PERIPHERAL_AS3911_TRANSPARENT_MODE:

            IN_FN_PPS_SDI1    = IN_PIN_PPS_RP0;
            AS3911_PIN_MOSI = OUT_FN_PPS_SDO1;
            /* disable SCK1 */
            AS3911_PIN_SCLK   = OUT_FN_PPS_NULL;
            IN_FN_PPS_INT1   = AS3911_PIN_MISO;
            /* route output of OC1 to pin RP15=? */
            OUT_PIN_PPS_RP15 = OUT_FN_PPS_OC1;
            IN_FN_PPS_SDI2   = AS3911_PIN_MISO;
            /* route RP15=? back to SCK2 */
            IN_FN_PPS_SCK2IN = IN_PIN_PPS_RP15;
            break;

        case BOARD_PERIPHERAL_UART1:

            UART_TX_PIN = OUT_FN_PPS_U1TX;
            IN_FN_PPS_U1RX  = UART_RX_PIN;
            break;

        case BOARD_PERIPHERAL_AS3911_INT:
            /* route pin as3911.INTR to IC1 (capture unit) */
            IN_FN_PPS_IC1 = AS3911_PIN_INTR;
            break;

        default:
            err = ERR_PARAM;
    }

    return err;
}

s8 boardPeripheralPinDeinitialize(boardPeripheral_t peri)
{
    s8 err = ERR_NONE;

    switch (peri)
    {
        case BOARD_PERIPHERAL_AS3911_TRANSPARENT_MODE:
            IN_FN_PPS_SDI1   = IN_PIN_PPS_RP0;
            AS3911_PIN_MOSI = OUT_FN_PPS_NULL;
            IN_FN_PPS_INT1   = IN_PIN_PPS_RP0;
            OUT_PIN_PPS_RP15 = OUT_FN_PPS_NULL;
            IN_FN_PPS_SDI2   = IN_PIN_PPS_RP0;
            IN_FN_PPS_SCK2IN = IN_PIN_PPS_RP0;
            break;

        case BOARD_PERIPHERAL_SPI1:
            AS3911_PIN_MOSI = OUT_FN_PPS_NULL;
            OUT_PIN_PPS_RP8  = OUT_FN_PPS_NULL;
            break;

        case BOARD_PERIPHERAL_UART1:
            OUT_PIN_PPS_RP2  = OUT_FN_PPS_NULL;
            break;

        case BOARD_PERIPHERAL_AS3911_INT:
            IN_FN_PPS_IC1    = IN_PIN_PPS_RP0;
            break;

        default:
            err = ERR_PARAM;
    }

    return err;
}

