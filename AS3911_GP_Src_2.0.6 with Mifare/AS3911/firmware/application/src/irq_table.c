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
 * PROJECT: AS3911 firmware
 * $Revision$
 * LANGUAGE: ANSI C
 */

/*! \file
 *
 * \author Christian Eisendle
 *
 * \brief AS3911 IRQ table.
 *
 * IRQ table which is used for bootloaded images, i.e. when bootloader is used.
 * Interrupt vector table of bootloader jumps to the table listed below.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "as3911_interrupt.h"
#include "delay.h"

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/
void __attribute__ ((space(prog), section(".irqtable"))) IRQTable()
{
    /* asm("lnk....") is created from compiler will never be jumped, ends up at 0x1e00 */
    asm("nop");         
    /* Here we start at address 0x1e04 = ReservedTrap0, jumped from 0x0008 */
    asm("nop"); asm("nop");                 // ReservedTrap0
    asm("nop"); asm("nop");                 // OscillatorFail
    asm("nop"); asm("nop");                 // AddressError
    asm("nop"); asm("nop");                 // StackError
    asm("nop"); asm("nop");                 // MathError
    asm("nop"); asm("nop");                 // ReservedTrap5
    asm("nop"); asm("nop");                 // ReservedTrap6
    asm("nop"); asm("nop");                 // ReservedTrap7
    asm("nop"); asm("nop");                 // INT0Interrupt
    asm("goto %0"::"i"(&as3911Isr));        // IC1Interrupt
    asm("nop"); asm("nop");                 // OC1Interrupt
    asm("nop"); asm("nop");                 // T1Interrupt
    asm("nop"); asm("nop");                 // Interrupt4
    asm("nop"); asm("nop");                 // IC2Interrupt
    asm("nop"); asm("nop");                 // OC2Interrupt
    asm("nop"); asm("nop");                 // T2Interrupt
    asm("nop"); asm("nop");                 // T3Interrupt
    asm("nop"); asm("nop");                 // SPI1ErrInterrupt
    asm("nop"); asm("nop");                 // SPI1Interrupt
    asm("nop"); asm("nop");                 // U1RXInterrupt
    asm("nop"); asm("nop");                 // U1TXInterrupt
    asm("nop"); asm("nop");                 // ADC1Interrupt
    asm("nop"); asm("nop");                 // Interrupt14
    asm("nop"); asm("nop");                 // Interrupt15
    asm("nop"); asm("nop");                 // SI2C1Interrupt
    asm("nop"); asm("nop");                 // MI2C1Interrupt
    asm("nop"); asm("nop");                 // CompInterrupt
    asm("nop"); asm("nop");                 // CNInterrupt
    asm("nop"); asm("nop");                 // INT1Interrupt
    asm("nop"); asm("nop");                 // Interrupt21
    asm("nop"); asm("nop");                 // Interrupt22
    asm("nop"); asm("nop");                 // Interrupt23
    asm("nop"); asm("nop");                 // Interrupt24
    asm("nop"); asm("nop");                 // OC3Interrupt
    asm("nop"); asm("nop");                 // OC4Interrupt
    asm("nop"); asm("nop");                 // T4Interrupt
    asm("goto %0"::"i"(&delayIsr));         // T5Interrupt
    asm("nop"); asm("nop");                 // INT2Interrupt
    asm("nop"); asm("nop");                 // U2RXInterrupt
    asm("nop"); asm("nop");                 // U2TXInterrupt
    asm("nop"); asm("nop");                 // SPI2ErrInterrupt
    asm("nop"); asm("nop");                 // SPI2Interrupt
    asm("nop"); asm("nop");                 // Interrupt34
    asm("nop"); asm("nop");                 // Interrupt35
    asm("nop"); asm("nop");                 // Interrupt36
    asm("nop"); asm("nop");                 // IC3Interrupt
    asm("nop"); asm("nop");                 // IC4Interrupt
    asm("nop"); asm("nop");                 // IC5Interrupt
    asm("nop"); asm("nop");                 // Interrupt40
    asm("nop"); asm("nop");                 // OC5Interrupt
    asm("nop"); asm("nop");                 // Interrupt42
    asm("nop"); asm("nop");                 // Interrupt43
    asm("nop"); asm("nop");                 // Interrupt44
    asm("nop"); asm("nop");                 // PMPInterrupt
    asm("nop"); asm("nop");                 // Interrupt46
    asm("nop"); asm("nop");                 // Interrupt47
    asm("nop"); asm("nop");                 // Interrupt48
    asm("nop"); asm("nop");                 // SI2C2Interrupt
    asm("nop"); asm("nop");                 // MI2C2Interrupt
    asm("nop"); asm("nop");                 // Interrupt51
    asm("nop"); asm("nop");                 // Interrupt52
    asm("nop"); asm("nop");                 // Interrupt53
    asm("nop"); asm("nop");                 // Interrupt54
    asm("nop"); asm("nop");                 // Interrupt55
    asm("nop"); asm("nop");                 // Interrupt56
    asm("nop"); asm("nop");                 // Interrupt57
    asm("nop"); asm("nop");                 // Interrupt58
    asm("nop"); asm("nop");                 // Interrupt59
    asm("nop"); asm("nop");                 // Interrupt60
    asm("nop"); asm("nop");                 // Interrupt61
    asm("nop"); asm("nop");                 // RTCCInterrupt
    asm("nop"); asm("nop");                 // Interrupt63
    asm("nop"); asm("nop");                 // Interrupt64
    asm("nop"); asm("nop");                 // U1ErrInterrupt
    asm("nop"); asm("nop");                 // U2ErrInterrupt
    asm("nop"); asm("nop");                 // CRCInterrupt
    asm("nop"); asm("nop");                 // Interrupt68
    asm("nop"); asm("nop");                 // Interrupt69
    asm("nop"); asm("nop");                 // Interrupt70
    asm("nop"); asm("nop");                 // Interrupt71
    asm("nop"); asm("nop");                 // LVDInterrupt
    asm("nop"); asm("nop");                 // Interrupt73
    asm("nop"); asm("nop");                 // Interrupt74
    asm("nop"); asm("nop");                 // Interrupt75
    asm("nop"); asm("nop");                 // Interrupt76
    asm("nop"); asm("nop");                 // Interrupt77
    asm("nop"); asm("nop");                 // Interrupt78
    asm("nop"); asm("nop");                 // Interrupt79
    asm("nop"); asm("nop");                 // Interrupt80
    asm("nop"); asm("nop");                 // Interrupt81
    asm("nop"); asm("nop");                 // Interrupt82
    asm("nop"); asm("nop");                 // Interrupt83
    asm("nop"); asm("nop");                 // Interrupt84
    asm("nop"); asm("nop");                 // Interrupt85
    asm("goto __USB1Interrupt");            // USB1Interrupt
}

