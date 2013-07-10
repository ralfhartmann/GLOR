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
 *  \author Ulrich Herrmann
 *
 *  \brief Application dispatcher
 * 
 *  This file is responsible for taking commands from USB HID stream layer, 
 *  executing the proper AS3911 commands and returning the results over USB.
 *  As entry point start reading documentation of processCmd() function.
 *
 */
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "dispatcher.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "iso14443a.h"
#include "iso15693_3.h"
#include "iso15693_2.h"
#include "iso14443_common.h"
#include "iso14443b.h"
#include "clock.h"
#include "led.h"
#include "logger.h"
#include "nfc.h"
#include "spi_driver.h"
#include "utils.h"
#include "mifare_ul.h"
#include "felica.h"
#include "topaz.h"
#ifdef HAS_MIFARE
#include "mifare.h"
#include "mifare_raw_request.h"
#endif
#include "delay.h"
#include "stream_dispatcher.h"
#if DOOR_BOARD
#include "beep.h"
#endif

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! Timeout of mifare read command in milliseconds. */
#define MIFARE_READ_TIMEOUT             1

/*! Timeout of mifare write command request transmission part in milliseconds. */
#define MIFARE_WRITE_COMMAND_TIMEOUT    1

/*! Timeout of mifare write command data transmission part in milliseconds. */
#define MIFARE_WRITE_DATA_TIMEOUT       7

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static const u8 dispatcherInterruptResultRegs[24]=
{
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, /* 1st byte */
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, /* 2nd byte */
    AS3911_REG_CAPACITANCE_MEASURE_RESULT,
    AS3911_REG_PHASE_MEASURE_RESULT,
    AS3911_REG_AMPLITUDE_MEASURE_RESULT,0xff,0xff,0xff,0xff,0xff, /* 3rd byte */
};

static u16 dispatcherInterruptResults[24];

static u8 nfc_is_active;
static u8 nfc_is_initiator;
static u8 nfc_bitrate;
static iso15693PhyConfig_t iso15693_config;
static u8 iso14443bModIndex = AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_10percent;
static u8 felicaModIndex = AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_10percent;

/*
******************************************************************************
* GLOBAL CONSTANTS
******************************************************************************
*/
extern unsigned int ADCResult[3];

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static s8 processDirectCommand(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
static s8 processInterruptResult(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
static s8 processTopaz(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
static s8 processIso14443a(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
static s8 processIso14443b(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
static s8 processNfc(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
static s8 processIso15693(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
#ifdef HAS_MIFARE
static s8 processMifare(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);
#endif
static s8 processFeliCa(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize);

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 dispatcherInitialize()
{
    return ERR_NONE;
}

s8 dispatcherDeinitialize()
{
    return ERR_NONE;
}

/*!
  Process the different protocols provided via \ref usb_hid_stream_driver.h. This function 
  also cares for implicit deinitialisation/initialisation if necessary.
  Actuals commands are implemented in 
  - #processTopaz()
  - #processIso14443a()
  - #processIso14443b()
  - #processIso15693()
  - #processNfc()
  - #processFeliCa()
  - #processMifare()
  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()
  */
u8 processProtocols ( const u8 * rxData, u16 rxSize, u8 * txData, u16 *txSize)
{
    static u8 protocol = 0;
    u8 cmd = *rxData;
    u8 err = ERR_REQUEST;
    u8 subcmd = cmd & 0x0f;
    u8 newprot = cmd & 0xf0;
    
    if (newprot != protocol)
    {
        if ((protocol & 0xf0) == 0x90)
        { /* topaz commands */
            err = topazDeinitialize(0);
        }
        else if ((protocol & 0xf0) == 0xa0)
        { /* iso14443a+mifare UL commands */
            err = iso14443ADeinitialize(0);
        }
        else if ((protocol & 0xf0) == 0xb0)
        { /* iso14443a+mifare UL commands */
            err = iso14443BDeinitialize(0);
        }
        else if ((protocol & 0xf0) == 0xc0)
        { /* nfc commands */
            err = nfcDeinitialize();
        }
        else if ((protocol & 0xf0) == 0xd0)
        { /* iso15693 commands */
            err = iso15693Deinitialize(0);
        }
#ifdef HAS_MIFARE
        else if ((protocol & 0xf0) == 0xe0)
        { /* mifare commands */
            err = mifareDeinitialize(0);
        }
#endif
        else if ((protocol & 0xf0) == 0xf0)
        { /* mifare commands */
            err = felicaDeinitialize(0);
        }
        delayNMilliSeconds(10);

        if (subcmd != 0x0) 
        { /* if not an initialize reuse last config */
            if ((protocol & 0xf0) == 0x90)
            { /* topaz commands */
                err = topazInitialize();
            }
            else if ((cmd & 0xf0) == 0xa0)
            { /* iso14443a+mifare UL commands */
                err = iso14443AInitialize();
            }
            else if ((cmd & 0xf0) == 0xb0)
            { /* iso14443a+mifare UL commands */
                u8 tmp;
                err = iso14443BInitialize(iso14443bModIndex, &tmp);
            }
            else if ((cmd & 0xf0) == 0xc0)
            { /* nfc commands */
                err = nfcInitialize(nfc_is_active, nfc_is_initiator, nfc_bitrate);
            }
            else if ((cmd & 0xf0) == 0xd0)
            { /* iso15693 commands */
                err = iso15693Initialize(&iso15693_config);
            }
#ifdef HAS_MIFARE
            else if ((cmd & 0xf0) == 0xe0)
            { /* mifare commands */
                err = mifareInitialize();
            }
#endif
            else if ((cmd & 0xf0) == 0xf0)
            { /* FeliCa commands */
                err = felicaInitialize(felicaModIndex);
            }
        }
    }

    /* call more specific dispatcher */
    if ((cmd & 0xf0) == 0x90)
    { /* topaz commands */
        err = (u8) processTopaz(rxData, rxSize, txData, txSize);
    }
    else if ((cmd & 0xf0) == 0xa0)
    { /* iso14443a+mifare UL commands */
        err = (u8) processIso14443a(rxData, rxSize, txData, txSize);
    }
    else if ((cmd & 0xf0) == 0xb0)
    { /* iso14443a+mifare UL commands */
       err = (u8) processIso14443b(rxData, rxSize, txData, txSize);
    }
    else if ((cmd & 0xf0) == 0xc0)
    { /* nfc commands */
       err = (u8) processNfc(rxData, rxSize, txData, txSize);
    }
    else if ((cmd & 0xf0) == 0xd0)
    { /* iso15693 commands */
       err = (u8) processIso15693(rxData, rxSize, txData, txSize);
    }
#ifdef HAS_MIFARE
    else if ((cmd & 0xf0) == 0xe0)
    { /* Mifare commands */
       err = (u8) processMifare(rxData, rxSize, txData, txSize);
    }
#endif
    else if ((cmd & 0xf0) == 0xf0)
    { /* FeliCa commands */
       err = (u8) processFeliCa(rxData, rxSize, txData, txSize);
    }

    protocol = newprot;

    if (subcmd == 0xf)
        protocol = 0;

    return err;
}

/*!
  Process various generic commands. Forward RFID protocol commands to 
  processProtocols().
  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Exact description of chaining of HID reports and stuff are given in file ams_stream.h
  Over USB each payload will have a header. With the above rxSize and 
  assuming just one command transmitted in one stream which expects an answer the OUT report will look like this:
  <table>
    <tr><th>   Byte</th><th>  0</th><th>       1</th><th>       2<th>       3</th><th>      4     </th><th>      5     </th><th>      6     </th><th>      7     </th><th> 8..8+rxSize-1</th></tr>
    <tr><th>Content</th><td>TID</td><td>length  </td><td>reserved<td>protocol</td><td>tx-prot MSB </td><td>tx-prot LSB </td><td>rx-prot MSB </td><td>rx-prot LSB </td><td>data</td></tr>
    <tr><th>Content</th><td>TID</td><td>rxSize+4</td><td>    0x00<td>    0x44</td><td>rxSize   MSB</td><td>rxSize   LSB</td><td>txSize   MSB</td><td>txSize   LSB</td><td>rxData[0..rxSize-1]</td></tr>
  </table>
  where
  <ul>
    <li> \e TID : arbitrary unique transaction ID, can be a counter
    <li> \e rxSize: the expected size of data this command will/is allowed to return.
  </ul>
  The response to such an OUT report will be an IN report looking like this:
  <table>
    <tr><th>   Byte</th><th>  0</th><th>   1    </th><th>         2</th><th>       3</th><th>       4</th><th>      5</th><th>      6    </th><th>      7    </th><th> 8..8+txSize-1     </th></tr>
    <tr><th>Content</th><td>TID</td><td>length  </td><td>HID status</td><td>protocol</td><td>reserved</td><td>status </td><td>tx-prot MSB</td><td>tx-prot LSB</td><td>data               </td></tr>
    <tr><th>Content</th><td>TID</td><td>txSize+4</td><td>   ret_val</td><td>0x44    </td><td>       0</td><td>ret_val</td><td>txSize MSB </td><td>txSize LSB </td><td>txData[0..txSize-1]</td></tr>
  </table>
  where

  Example for an antenna calibration command:

  OUT:
  <table>
    <tr><th>   Byte</th><th>  0  </th><th>   1  </th><th>       2</th><th>      3</th><th>      4</th><th>      5</th><th>      6</th><th> 7       </th><th>        7</th></tr>
    <tr><th>Content</th><td>TID  </td><td>length</td><td>protocol</td><td>tx-prot</td><td>tx-prot</td><td>rx-prot</td><td>rx-prot</td><td>rxData[0]</td><td>rxData[1]</td></tr>
    <tr><td>Content</td><td>0x1F </td><td>0x07  </td><td>0x44    </td><td>0x00   </td><td>0x02   </td><td>0x00   </td><td>0x01   </td><td>0x15     </td><td>0xD8     </td></tr>
  </table>
  where:<ul>
  <li> 0x15: cmd id for #processDirectCommand()
  <li> 0xd8: the sub command #AS3911_CMD_CALIBRATE_ANTENNA
  </ul>
  The reader answer with this report:
  <table>
    <tr><th>   Byte</th><th>  0 </th><th>   1  </th><th>     2    </th><th>       3</th><th>     4  </th><th>     5</th><th>    6  </th><th>  7    </th><th> 8       </th></tr>
    <tr><th>Content</th><td>TID </td><td>length</td><td>HID status</td><td>protocol</td><td>reserved</td><td>status</td><td>tx-prot</td><td>tx-prot</td><td>txData[0]</td></tr>
    <tr><td>Content</td><td>0xFD</td><td>0x06  </td><td>    0x00  </td><td>0x44    </td><td>0x00    </td><td>0x00  </td><td>0x00   </td><td>  1    </td><td>0x60     </td></tr>
  </table>
  <ul>
  <li> 0x60: the data this command produced (content of AS3911_REG_ANT_CAL_RESULT)
  <li> 0x00: status 0 : ERR_NONE
  </ul>

  Implemented commands:

  - Return ADC measurement for Board identification
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0x14(ID)</td></tr>
    </table>
    txSize needs to accomodate 3 bytes of response:
    <table>
      <tr><th>   Byte</th><th>             0</th><th>             1</th><th>           2 </th></tr>
      <tr><th>Content</th><td>adc(AN2=PGED1)</td><td>adc(AN3=PGEC1)</td><td>adc(AN3=U1TX)</td></tr>
    </table>
  -  #processProtocols()
  -  #processInterruptResult()
  -  #processDirectCommand() see details in the function
  - #iso14443TransmitAndReceive()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>1..rxSize</th></tr>
      <tr><th>Content</th><td>0x18(ID)</td><td>data to be transmitted</td></tr>
    </table>
    #iso14443TransmitAndReceive() is called to sent rxSize-1 bytes and receive at most *txSize-4 bytes.
    *txSize is adjusted to the actual received data length + 4.
    <table>
      <tr><th>   Byte</th><th>0..3</th><th>4..*txSize</th></tr>
      <tr><th>Content</th><td>passed microseconds for #iso14443TransmitAndReceive()</td><td>received data buffer</td></tr>
    </table>
  - #as3911SetBitrate()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>                  1</th><th>                  2</th></tr>
      <tr><th>Content</th><td>0x19(ID)</td><td>rx bitrate r:2^r*106kBit</td><td>tx bitrate t:2^t*106kBit</td></tr>
    </table>
    no response except status.
  - Measure antenna amplitude and phase.
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0x20(ID)</td></tr>
    </table>
    #as3911MeasureRF() and as3911MeasureAntennaResonance() get called.
    *txSize must be >= 2, response is:
    <table>
      <tr><th>   Byte</th><th>0</th><th>1</th></tr>
      <tr><th>Content</th><td>result of #as3911MeasureRF()</td><td>result of #as3911MeasureAntennaResonance()</td></tr>
    </table>
  - Enable/Disable RF field
    <table>
      <tr><th>   Byte</th><th>       0</th><th>       1</th></tr>
      <tr><th>Content</th><td>0x22(ID)</td><td>on</td></tr>
    </table>
    where \e on is a boolean value denoting if the field should be turned on(1) or off(0).
    no response, only status.
  */
static u8 processCmd ( const u8 * rxData, u16 rxSize, u8 * txData, u16 *txSize)
{
    u8 cmd = *rxData;
    const u8 *buf = rxData + 1;
    u16 bufSize = rxSize - 1;
    u8 err = ERR_REQUEST;

    if (cmd == 0x14)
    {
        if (*txSize < 3)
        {
            err = ERR_PARAM;
        }
        *txSize = 3;
#if DOOR_BOARD
        txData[0] = 41;
        txData[1] = 42;
        txData[2] = 43;
#else
        /* Convert 10 to 8 bit */
        txData[0] = (ADCResult[0] >> 2); /* AN2/RP0/PGED1 */
        txData[1] = (ADCResult[1] >> 2); /* AN3/RP1/PGEC1 */
        txData[2] = (ADCResult[2] >> 2); /* AN4/RP2/U1TX  */
#endif
    }
    if (cmd == 0x15)
    {
       err = (u8) processDirectCommand(buf, bufSize, txData, txSize);
    }
    if (cmd ==  0x18)
    {
        u32 us;
        u16 actlength = 0;
        if (*txSize < 4) return ERR_PARAM;
        stopWatchStart();
        err = iso14443TransmitAndReceive(buf,
                bufSize,
                txData + 4,
                *txSize - 4,
                &actlength);
        us = stopWatchMeasure();
        txData[0] = (us >>  0) & 0xff;
        txData[1] = (us >>  8) & 0xff;
        txData[2] = (us >> 16) & 0xff;
        txData[3] = (us >> 24) & 0xff;
        if (*txSize) *txSize = actlength + 4;
    }
    if (cmd ==  0x19)
    {
        err = as3911SetBitrate(buf[0], buf[1]);
        *txSize = 0;
    }
    if (cmd == 0x20)
    {
        /* Antenna measurement. */
        err  = as3911MeasureRF(txData);
        err |= as3911MeasureAntennaResonance(txData + 1);
        if (*txSize) *txSize = 2;
    }
    if (cmd == 0x21)
    { /* Get interrupt and result */
        err = processInterruptResult( buf, bufSize, txData, txSize);
    }
    if (cmd == 0x22)
    { /* Turn on/off antenna */
        if (bufSize < 1) return ERR_PARAM;
        as3911ModifyRegister(AS3911_REG_OP_CONTROL, AS3911_REG_OP_CONTROL_tx_en,
                (buf[0])?(AS3911_REG_OP_CONTROL_tx_en | AS3911_REG_OP_CONTROL_en):0);
    }

    if ((cmd>>4) >= 0x9)
        err = processProtocols(rxData, rxSize, txData, txSize);

    return err;
}

u8 applProcessCmd( u8 protocol, u16 rxSize, const u8 * rxData, u16 * txSize, u8 * txData )
{ /* forward to different function to have place for doxygen documentation
     because applProcessCmd is already documented in usb_hid_stream_driver.h*/
    return processCmd( rxData, rxSize, txData, txSize);
}

#if 0
s8 dispatcher(u8 cmd, u8* buf, u16 length)
{
    u8 tmp;
    s8 err = ERR_NONE;

    if (0)
    {
    }
#ifdef HAS_MIFARE
    else if ((cmd & 0xf0) == 0xe0)
    {
        /* mifare classic */
        err = dispatchMifare(cmd, buf, length);
    }
#endif
    else
    {

        switch (cmd)
        {
            case 0x1:
                /* reinitialize AS3911 */
                clkSetClockSource(CLK_SOURCE_INTERNAL);
                as3911Deinitialize();
                as3911Initialize();
                clkSetClockSource(CLK_SOURCE_EXTERNAL);

                /*
                pccomSendResponseString(DEMOBOARD_VERSION, sizeof(DEMOBOARD_VERSION), PCCOM_BRACKETS_SQUARE);
                ...
                */

                /* Send back AntDrvXtalCLKuC setup. */
                as3911ReadRegister(AS3911_REG_IO_CONF1, &tmp);
                pccomSendResponseByteArray(&tmp, 1, FALSE, PCCOM_BRACKETS_SQUARE);

                /* Enable AGC (automatic gain control). */
                as3911ModifyRegister(AS3911_REG_RX_CONF1, 0x10, 0x10);
                break;
            case 0x2:
                /* reset the PIC */
                pccomSendPrompt();
                clkSetClockSource(CLK_SOURCE_INTERNAL);
                RESET_CPU();
                break;
            case 0x16:
                err = as3911WriteFifo(buf, length);
                break;
            default:
                err = ERR_PARAM;
                break;
        }
    }
    return err;
}
#endif

    
/*!
  This function processes certain interrupts and stores results retrieved for 
  later transmission over USB */
void dispatcherInterruptHandler()
{
    int i;
    u8 val;
    u16 isrs;

    for (i=0; i<24; i++)
    {
        if(dispatcherInterruptResultRegs[i] >= 0x40) continue;
        if(!as3911GetInterrupt(1UL<<i)) continue;

        isrs = dispatcherInterruptResults[i] >> 8;
        if (isrs < 255) isrs++;

        as3911ReadRegister(dispatcherInterruptResultRegs[i], &val);

        dispatcherInterruptResults[i] = (isrs<<8) | val;
    }
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

/*!
  Return the interrupt results stored from dispatcherInterruptHandler() via USB to GUI.
  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - Get interrupt result:
  <table>
    <tr><th>   Byte</th><th>       0</th><th>       1</th></tr>
    <tr><th>Content</th><td>0x21(ID)</td><td>0..23:irq in question</td></tr>
  </table>
  txSize needs to accomodate 2 bytes of response:
  <table>
    <tr><th>   Byte</th><th>       0</th><th>           1</th></tr>
    <tr><th>Content</th><td>num of isrs occurred</td><td>last result</td></tr>
  </table>
    Currently supported interrups and corresponding results are:
    - Wakeup Capacitive IRQ(16): AS3911_REG_CAPACITANCE_MEASURE_RESULT
    - Wakeup Phase IRQ(17)     : AS3911_REG_PHASE_MEASURE_RESULT
    - Wakeup Amplituted IRQ(18): AS3911_REG_AMPLITUDE_MEASURE_RESULT
  */
static s8 processInterruptResult(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    if(rxSize < 1) return ERR_REQUEST;
    if(rxData[0] > 23) return ERR_PARAM;
    if(dispatcherInterruptResultRegs[rxData[0]] > 0x3f) return ERR_PARAM;
    if(*txSize < 2) return ERR_PARAM;

    *txSize = 2;
    txData[0] =  dispatcherInterruptResults[rxData[0]] >> 8;
    txData[1] =  dispatcherInterruptResults[rxData[0]] & 0xff;

    dispatcherInterruptResults[rxData[0]] = 0; /* clear value */

    return ERR_NONE;
}

/*!
  Process ISO14443A type commands.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - #topazInitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0x90(ID)</td></tr>
    </table>
    no return value only status
  - #topazDeinitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>         1</th></tr>
      <tr><th>Content</th><td>0x9f(ID)</td><td>keep_rf_on</td></tr>
    </table>
    no return value only status
  - #topazReqaWupa() + topazReadUID()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>1</th></tr>
      <tr><th>Content</th><td>0x91(ID)</td><td>0x26(REQA) or 0x52(WUPA)</td></tr>
    </table>
    *txsize must be set to allow at least 17 return values.
    <table>
      <tr><th>   Byte</th><th> 1..2</th><th>3..6</th></tr>
      <tr><th>Content</th><td>atqa </td><td> uid</td></tr>
    </table>
*/
static s8 processTopaz(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    s8 err = ERR_REQUEST;
    u16 bufSize = rxSize - 1;
    topazProximityCard_t card;
    const u8 *buf = rxData + 1;
    u8 cmd = rxData[0];
    memset(txData, 0xff, *txSize);

    switch (cmd)
    {
        case 0x90:
            err = topazInitialize();
            *txSize = 0;
            break;
        case 0x9f:
            {
                u8 keep_on = 0;
                if (bufSize > 0) keep_on = buf[0];
                err = topazDeinitialize(keep_on);
                *txSize = 0;
            }
            break;
        case 0x91:
            if (*txSize < 8) return ERR_PARAM;
            err = topazReqaWupa((topazCommand_t)buf[0], &card);
            if (ERR_NONE == err)
                err = topazReadUID(&card);

            txData[0] = card.atqa[0];
            txData[1] = card.atqa[1];
            txData[2] = card.hr[0];
            txData[3] = card.hr[1];
            txData[4] = card.uid[0];
            txData[5] = card.uid[1];
            txData[6] = card.uid[2];
            txData[7] = card.uid[3];

            if(*txSize > 0) *txSize = 8;
            if(ERR_NOTSUPP == err)
                *txSize = 2;
            break;
        case 0x92:
            if (*txSize < 6) return ERR_PARAM;
            err = topazReadUID(&card);

            txData[0] = card.hr[0];
            txData[1] = card.hr[1];
            txData[2] = card.uid[0];
            txData[3] = card.uid[1];
            txData[4] = card.uid[2];
            txData[5] = card.uid[3];

            if(*txSize > 0) *txSize = 6;
            break;
        case 0x93:
            if (bufSize < 4) { *txSize = 0; return ERR_PARAM;}
            memcpy(card.uid, buf, 4);
            err = topazReadAll(&card, txData, *txSize, txSize);
            break;
        case 0x94: /* FIXME: should be Read Byte */
            break;
        case 0x95:
            if (bufSize < 6) return ERR_PARAM;
            memcpy(card.uid, buf, 4);
            {
                u8 addr = buf[4], data = buf[5];
                err = topazWriteByte(&card, addr, data);
            }
            break;
        default:
            err = ERR_PARAM;
            *txSize = 0;
    }
    return err;
}


/*!
  Process ISO14443A type commands.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - #iso14443AInitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0xa0(ID)</td></tr>
    </table>
    no return value only status
  - #iso14443ADeinitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>         1</th></tr>
      <tr><th>Content</th><td>0xaf(ID)</td><td>keep_rf_on</td></tr>
    </table>
    no return value only status
  - #iso14443ASelect()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>1</th></tr>
      <tr><th>Content</th><td>0xa1(ID)</td><td>0x26(REQA) or 0x52(WUPA)</td></tr>
    </table>
    *txsize must be set to allow at least 17 return values.
    <table>
      <tr><th>   Byte</th><th>  0</th><th> 1..2</th><th>                 3</th><th>4..6</th><th>7..17</th></tr>
      <tr><th>Content</th><td>col</td><td>atqa</td><td>cascadelevels(1..3)</td><td>sak</td><td> uid</td></tr>
    </table>
      There are three possible values of \e cascadelevels:
      - 1: sak is one byte long, \e uid is 4 bytes long
      - 2: sak is two bytes long, \e uid is 7 bytes long
      - 3: sak is three bytes long, \e uid is 10 bytes long
  - #iso14443ASendHlta()
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0xa3(ID)</td></tr>
    </table>
    not response data available only status
  - #iso14443AEnterProtocolMode() sends ISO14443A RATS command
    <table>
      <tr><th>   Byte</th><th>       0</th><th>   1</th><th>  2</th></tr>
      <tr><th>Content</th><td>0xa4(ID)</td><td>fsdi</td><td>cid</td></tr>
    </table>
    where:
      - \e fsdi: 0-0xf Frame size for Device Index
        <table>
          <tr><th> 0</th><th> 1</th><th> 2</th><th> 3</th><th> 4</th><th> 5</th><th> 6</th><th>  7</th><th>  8</th></tr>
          <tr><td>16</td><td>24</td><td>32</td><td>40</td><td>48</td><td>64</td><td>96</td><td>128</td><td>256</td></tr>
        </table>
      - \e cid: Card Identifier to assign to the card.
      .
    *txsize must be set to allow at least 7 return values for ATS. But card response may be even longer.
    <table>
      <tr><th>   Byte</th><th> 0</th><th> 1</th><th>    2</th><th>    3</th><th>    4</th><th> 5</th><th> 6</th></tr>
      <tr><th>Content</th><td>TL</td><td>T0</td><td>TA(1)</td><td>TB(1)</td><td>TC(1)</td><td>T1</td><td>Tk</td></tr>
    </table>
  - #iso14443ASendProtocolAndParameterSelection()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>  1</th><th>        2</th><th>   3</th></tr>
      <tr><th>Content</th><td>0xa5(ID)</td><td>cid</td><td>pss0=0x11</td><td>pss1</td></tr>
    </table>
    where:
      - \e cid: the cid set using RATS command
      - \e pss0: not used internally set to 0x11, should always be set to 0x11
      - \e pss1: PSS1 according to standard, upper 4 bits shall be 0, lower 4 bits are DSI and DRI.
    no return value only status
  - #iso14443Deselect()
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0xa6(ID)</td></tr>
    </table>
    no return value only status
  - #mifareUlReadNBytes()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>    1</th></tr>
      <tr><th>Content</th><td>0xa7(ID)</td><td>start</td></tr>
    </table>
    This function reads starting from \e start *txSize bytes. Response is:
    <table>
      <tr><th>   Byte</th><th>0..actual_received length</th></tr>
      <tr><th>Content</th><td>received data</td></tr>
    </table>
  - #mifareUlWritePage()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>    1</th><th>2..6</th></tr>
      <tr><th>Content</th><td>0xa8(ID)</td><td>start</td><td>data</td></tr>
    </table>
    no return value only status
  */
static s8 processIso14443a(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    s8 err;
    u16 bufSize = rxSize - 1;
    u16 actlength;
    iso14443AProximityCard_t card;
    const u8 *buf = rxData + 1;
    u8 cmd = rxData[0];
    memset(txData, 0xff, *txSize);

    switch (cmd)
    {
        case 0xa0:
            err = iso14443AInitialize();
            *txSize = 0;
            break;
        case 0xaf:
            {
                u8 keep_on = 0;
                if (bufSize > 0) keep_on = buf[0];
                err = iso14443ADeinitialize(keep_on);
                *txSize = 0;
            }
            break;
        case 0xa1:
            if (*txSize < 17) return ERR_PARAM;
            err = iso14443ASelect((iso14443ACommand_t)buf[0], &card);
            txData[0] = card.collision;
            txData[1] = card.atqa[0];
            txData[2] = card.atqa[1];
            txData[3] = card.cascadeLevels;
            txData[4] = card.sak[0];
            txData[5] = card.sak[1];
            txData[6] = card.sak[2];
#if DOOR_BOARD
            if (ERR_NONE == err)
            { /* Some tag was found */
                beep();
            }
#endif
            AMS_MEMCPY(txData+7, card.uid, card.actlength);
            if(*txSize > 0) *txSize = 17;
            break;
        case 0xa3:
            err = iso14443ASendHlta();
            *txSize = 0;
            break;
        case 0xa4:
            if (bufSize < 2) return ERR_PARAM;
            if (*txSize < 7) return ERR_PARAM;
            {
                u8 fsdi = buf[0];
                u8 cid = buf[1];
                u8 fscid = (cid&0xf) | ((fsdi&0xf)<<4);
                err = iso14443AEnterProtocolMode(fscid, txData, *txSize, &actlength);
                *txSize = actlength;
            }
            break;
        case 0xa5:
            {
                u8 cid = buf[0] & 0xf;
                /* buf[1] may be used for PSS0, should be 0x11 */
                u8 pss1 = buf[2];
                err = iso14443ASendProtocolAndParameterSelection(cid, pss1);
            }
            *txSize = 0;
            break;
        case 0xa6:
            err = iso14443Deselect(0);
            *txSize = 0;
            break;
        case 0xa7:
            /* mifare ul read */
            {
                u8 actlen;
                err = mifareUlReadNBytes(buf[0], txData, *txSize, &actlen);
                *txSize = actlen;
            }
            break;
        case 0xa8:
            /* mifare ul write page */
            if (bufSize != 5)
            {
                return ERR_PARAM;
            }
            err = mifareUlWritePage(buf[0], buf+1);
            *txSize = 0;
            break;
        default:
            err = ERR_PARAM;
            *txSize = 0;
    }
    return err;
}

/*!
  Process ISO14443B type commands.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - #iso14443BInitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>       1        </th></tr>
      <tr><th>Content</th><td>0xb0(ID)</td><td>modulation_index</td></tr>
    </table>
    response:
    <table>
      <tr><th>   Byte</th><th>   0              </th></tr>
      <tr><th>Content</th><td>modulation_setting</td></tr>
    </table>
  - #iso14443BDeinitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>         1</th></tr>
      <tr><th>Content</th><td>0xbf(ID)</td><td>keep_rf_on</td></tr>
    </table>
    no return value only status
  - #iso14443Deselect()
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0xb4(ID)</td></tr>
    </table>
    no return value only status
  - #iso14443BSelect() sends REQB or WUPB
    <table>
      <tr><th>   Byte</th><th>       0</th><th>  1           </th><th>  2</th><th>      3    </th></tr>
      <tr><th>Content</th><td>0xb1(ID)</td><td>wubp_else_reqb</td><td>afi</td><td>slotcnt_exp</td></tr>
    </table>
    where:
    - \e wupb_else_reqb : if true wupb is sent else reqb
    - \e afi : The AFI field as in the standard
    - \e slotcnt_exp : lower three bits determine slot count(0=1,1=2,2=4,3=8,4=16)
    response:
    <table>
      <tr><th>   Byte</th><th>   0</th><th>1..4</th><th>   5..8 </th><th>   9..11 </th><th>12 </th></tr>
      <tr><th>Content</th><td>atqb</td><td>pupi</td><td>app_data</td><td>prot_info</td><td>col</td></tr>
    </table>
    where:
    - \e col: 1=collision has happened and was resolved, 0=no collision
    .
  - #iso14443BSendHltb()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>1..4</th></tr>
      <tr><th>Content</th><td>0xb2(ID)</td><td>PUPI</td></tr>
    </table>
    no response data only status
  - #iso14443BEnterProtocolMode() sends ATTRIB
    <table>
      <tr><th>   Byte</th><th>       0</th><th>1..4</th><th> 5..8 </th></tr>
      <tr><th>Content</th><td>0xb3(ID)</td><td>pupi</td><td>params</td></tr>
    </table>
    response:
    <table>
      <tr><th>   Byte</th><th>   0    </th></tr>
      <tr><th>Content</th><td>mbli_cid</td></tr>
    </table>
    where:
    - \e mbli_cid : upper 4 bits: mbli and lower 4 bits: cid

*/
static s8 processIso14443b(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    s8 err;
    u8 cmd = rxData[0];
    const u8 *buf = rxData + 1;
    const u16 bufSize = rxSize - 1;
    iso14443BProximityCard_t card;
    iso14443BAttribParameter_t param;
    iso14443BAttribAnswer_t answer;

    switch (cmd)
    {
        case 0xb0:
            if (bufSize > 0) iso14443bModIndex = buf[0];
            if (*txSize) *txSize = 1;
            err = iso14443BInitialize(iso14443bModIndex, txData);
            if (*txSize) *txSize = 1;
            break;
        case 0xbf:
            {
                u8 keep_on = 0;
                if (bufSize > 0) keep_on = buf[0];
                err = iso14443BDeinitialize(keep_on);
            }
            *txSize = 0;
            break;
        case 0xb1:
            if(bufSize < 3) return ERR_PARAM;
            if(*txSize > 13) *txSize = 13;
            err = iso14443BSelect(buf[0] ?  ISO14443B_CMD_WUPB : ISO14443B_CMD_REQB,
                    &card,
                    buf[1],
                    buf[2] & 0x7);
            AMS_MEMCPY(txData , &card,*txSize);
            break;
        case 0xb2:
            AMS_MEMCPY(card.pupi, &buf[0], ISO14443B_PUPI_LENGTH);
            err = iso14443BSendHltb(&card);
            *txSize = 0;
            break;
        case 0xb3:
            /* map to iso14443BEnterProtocolMode */
            AMS_MEMCPY(card.pupi, &buf[0], ISO14443B_PUPI_LENGTH);
            AMS_MEMCPY(&param, &buf[4], 4);
            err = iso14443BEnterProtocolMode(&card, &param, &answer);
            if (ERR_NONE == err)
            {
                txData[0] = (answer.mbli << 4) | answer.cid;
            }
            if(*txSize > 0) *txSize = 1;
            break;
        case 0xb4:
            err = iso14443Deselect(0);
            *txSize = 0;
            break;
        default:
            err = ERR_REQUEST;
            *txSize = 0;
    }

    return err;
}

/*!
  Process NFC type commands.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - #nfcInitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>        1</th><th>        2</th><th>      3</th></tr>
      <tr><th>Content</th><td>0xc0(ID)</td><td>is_active</td><td>initiator</td><td>bitrate</td></tr>
    </table>
    no return value only status
  - #nfcDeinitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0xcf(ID)</td></tr>
    </table>
    no return value only status
  - #nfcTxNBytes()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>       1                   </th><th>2..txSize-2</th></tr>
      <tr><th>Content</th><td>0xc1(ID)</td><td>perform_collision_avoidance</td><td>       data</td></tr>
    </table>
    no return value only status
  - #nfcRxNBytes() receives at most *txSize bytes from the other NFC device
    <table>
      <tr><th>   Byte</th><th>       0</th></tr>
      <tr><th>Content</th><td>0xc2(ID)</td></tr>
    </table>
    response:
    ERR_NOMSG is returned as status in case there was no message.
    <table>
      <tr><th>   Byte</th><th>0..actlength</th></tr>
      <tr><th>Content</th><td>        data</td></tr>
    </table>
*/
static s8 processNfc(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    const u8 *buf = rxData + 1;
    u16 bufSize = rxSize - 1;
    u8 cmd = rxData[0];
    s8 err = ERR_NONE;
    u8 actlength;

    // dbgLog("nfc cmd %hhx, rxSize = %hhx txSize = %hhx\n",cmd,bufSize,*txSize);
    // dbgHexDump(rxData,rxSize);

    switch (cmd)
    {
        case 0xc0:
            if (bufSize < 3) return ERR_PARAM;
            {
                nfc_is_active = !!buf[0];
                nfc_is_initiator = !!buf[1];
                nfc_bitrate = buf[2]; /* speed is 2^bitrate * 106 kb/s */
                err = nfcInitialize(nfc_is_active, nfc_is_initiator, nfc_bitrate);
            }
            break;
        case 0xcf:
            err = nfcDeinitialize();
            break;
        case 0xc1:
            if (bufSize < 1) return ERR_PARAM;
            {
                u8 perform_collision_avoidance = !!buf[0];
                buf++; bufSize--;
                err = nfcTxNBytes(buf, bufSize, perform_collision_avoidance);
            }
            break;
        case 0xc2:
            err = nfcRxNBytes(txData, *txSize, &actlength);
            *txSize = actlength;
            break;
        case 0xc3:
            err = nfcStartInitialTargetRx();
            break;
        case 0xcc:
            if (bufSize < 1) return ERR_PARAM;
            err = nfcSwitchToRx((buf[0])?TRUE:FALSE);
            break;
        default:
            err = ERR_REQUEST;
    }

    return err;
}

/*!
  Process ISO15693 type commands.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - #iso15693Initialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>           1</th><th>        2              </th></tr>
      <tr><th>Content</th><td>0xd0(ID)</td><td>use_1_of_256</td><td>modulation_index_as3911</td></tr>
    </table>
    no return value only status
  - #iso15693Deinitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>         1</th></tr>
      <tr><th>Content</th><td>0xdf(ID)</td><td>keep_rf_on</td></tr>
    </table>
    no return value only status
  - #iso15693Inventory()
    <table>
      <tr><th>   Byte</th><th>       0        </th></tr>
      <tr><th>Content</th><td>0xd2 or 0xd3(ID)</td></tr>
    </table>
    Command \e 0xd2 uses 1 slot, \e 0xd3 uses 16 slots inventory round.
    txSize has to be set to accomodate enough cards.
    Response is:
    <table>
      <tr><th>   Byte</th><th>   0     </th><th>                1..8 </th><th>..</th><th>1+(num_cards*8)..8*(num_cards+1)</th></tr>
      <tr><th>Content</th><td>num_cards</td><td>flags_0,dsfid_0,uid_0</td><td>..</td><td>flags,dsfid,uid_num_cards</td></tr>
    </table>
  - #iso15693SendStayQuiet()
    <table>
      <tr><th>   Byte</th><th>   0    </th><th>  1  </th><th>2..9</th></tr>
      <tr><th>Content</th><td>0xd4(ID)</td><td>flags</td><td> UID</td></tr>
    </table>
    If SELECT flag(0x10) is set in \e flags, then UID can be omitted, operations works on selected flag.
    no response data only status
  - #iso15693SelectPicc()
    <table>
      <tr><th>   Byte</th><th>   0    </th><th>  1  </th><th>2..9</th></tr>
      <tr><th>Content</th><td>0xd5(ID)</td><td>flags</td><td> UID</td></tr>
    </table>
    If SELECT flag(0x10) is set in \e flags, then UID can be omitted, operations works on selected flag.
    no response data only status

  - #iso15693GetPiccSystemInformation()
    <table>
      <tr><th>   Byte</th><th>   0    </th><th>  1  </th><th>2..9</th></tr>
      <tr><th>Content</th><td>0xd6(ID)</td><td>flags</td><td> UID</td></tr>
    </table>
    If SELECT flag(0x10) is set in \e flags, then UID can be omitted, operations works on selected flag.
    response is:
    <table>
      <tr><th>   Byte</th><th> 0   </th><th>        1</th><th>2..9</th><th>10   </th><th>11 </th><th>12 </th><th>13 </th><th>14 </th></tr>
      <tr><th>Content</th><td>flags</td><td>infoFlags</td><td>uid </td><td>dsfid</td><td>afi</td><td>memNumBlocks</td><td>memBlockSize</td><td>icReference</td></tr>
    </table>
  - #iso15693ReadSingleBlock()
    <table>
      <tr><th>   Byte</th><th>   0    </th><th>  1  </th><th>2..9</th><th>10</th></tr>
      <tr><th>Content</th><td>0xd7(ID)</td><td>flags</td><td> UID</td><td>block_no</td></tr>
    </table>
    If SELECT flag(0x10) is set in \e flags, then UID can be omitted, operations works on selected flag.
    Response is:
    <table>
      <tr><th>   Byte</th><th>   0     </th><th>1..memBlock.actualSize+1</th></tr>
      <tr><th>Content</th><td>rec_flags</td><td>read_data               </td></tr>
    </table>
  - #iso15693WriteSingleBlock()
    <table>
      <tr><th>   Byte</th><th>   0    </th><th>  1  </th><th>2..9</th><th>   10   </th><th>11..11+blocksize</th></tr>
      <tr><th>Content</th><td>0xd8(ID)</td><td>flags</td><td> UID</td><td>block_no</td><td>   data         </td></tr>
    </table>
    If SELECT flag(0x10) is set in \e flags, then UID can be omitted, operations works on selected flag.
    No response data only status is returned.

  - #iso15693TxRxNBytes() generic sending of a byte stream, receives at most txSize
    <table>
      <tr><th>   Byte</th><th>   0    </th><th>1..rxSize</th></tr>
      <tr><th>Content</th><td>0xde(ID)</td><td>tx_buf   </td></tr>
    </table>
    Response is:
    <table>
      <tr><th>   Byte</th><th>0..txSize</th></tr>
      <tr><th>Content</th><td>read_data</td></tr>
    </table>
*/

static s8 processIso15693(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    const u8 *buf = rxData + 1;
    const u16 bufSize = rxSize - 1;
    u8 cmd = rxData[0];
    s8 err = ERR_NONE;
    iso15693ProximityCard_t cards[10];
    iso15693ProximityCard_t* cardptr;
    iso15693PiccSystemInformation_t sysInfo;
    iso15693PiccMemoryBlock_t memBlock;
    u8 actcnt;
    u8 i;

    cardptr = NULL;
    if (cmd>=0xd4 && cmd<0xdf)
    {
        if (bufSize == 0) return ERR_PARAM;
        if (bufSize >= 1+ISO15693_UID_LENGTH && !(buf[0]&0x10))
        {
            memcpy(cards[0].uid, buf + 1, ISO15693_UID_LENGTH);
            cardptr = cards + 0;
        }
    }
    switch (cmd)
    {
        case 0xd0:
            iso15693_config.coding = (buf[0]) ?
                ISO15693_VCD_CODING_1_256 :
                ISO15693_VCD_CODING_1_4;
            iso15693_config.mi = buf[1];

            err = iso15693Initialize(&iso15693_config);
            break;
        case 0xdf:
            {
                u8 keep_on = 0;
                if (bufSize > 0) keep_on = buf[0];
                err = iso15693Deinitialize(keep_on);
            }
            break;
        case 0xd2:
        case 0xd3:
            err = iso15693Inventory((0xd2 == cmd) ?
                    ISO15693_NUM_SLOTS_1 :
                    ISO15693_NUM_SLOTS_16,
                    0,
                    NULL,
                    cards,
                    sizeof(cards)/sizeof(iso15693ProximityCard_t),
                    &actcnt);
            if (ERR_NONE == err)
            {
                u8 *tx = txData;
                *tx = actcnt;
                tx++;
                for (i = 0; i < actcnt; i++)
                {
                    if (*txSize < (i+1) * ISO15693_UID_LENGTH + 1) 
                    {
                        i++;
                        break;
                    }
                    /* flags, dsfid, uid */
                    memcpy(tx,&cards[i].flags,ISO15693_UID_LENGTH + 2);
                    tx += ISO15693_UID_LENGTH + 2;
                }
                *txSize = 1 + i * (ISO15693_UID_LENGTH + 2);
            }
            else
            {
                *txData = 0;
                *txSize = 1;
            }
            break;
        case 0xd4:
            if (bufSize < 9) return ERR_PARAM;
            err = iso15693SendStayQuiet(cardptr);
            break;
        case 0xd5:
            if (bufSize < 9) return ERR_PARAM;
            err = iso15693SelectPicc(cardptr);
            break;
        case 0xd6:
            if (bufSize < 8) return ERR_PARAM;
            err = iso15693GetPiccSystemInformation(cardptr, &sysInfo);
            if (ERR_NONE == err)
            {
                if (*txSize > sizeof(sysInfo))
                    *txSize = sizeof(sysInfo);
                memcpy(txData, &sysInfo, *txSize);
            }
            else
                *txSize = 0;
            break;
        case 0xd7:
            if (bufSize < ISO15693_UID_LENGTH + 2) return ERR_PARAM;
            if (*txSize < 2) return ERR_PARAM;
            memBlock.blocknr = buf[9];
            err = iso15693ReadSingleBlock(cardptr, &memBlock);
            if (ERR_NONE == err)
            {
                txData[0] = memBlock.flags;
                if (memBlock.errorCode == 0)
                {
                    if (*txSize > memBlock.actualSize + 1)
                        *txSize = memBlock.actualSize + 1;
                    memcpy(txData+1,memBlock.data,*txSize - 1);
                }
                else
                {
                    txData[1] = memBlock.errorCode;
                    *txSize = 2;
                }
            }
            else
                *txSize = 0;
            break;
        case 0xd8:
            if (bufSize < ISO15693_UID_LENGTH + 3) return ERR_PARAM;
            memBlock.actualSize = bufSize - ISO15693_UID_LENGTH - 2;
            memBlock.blocknr = buf[9];
            AMS_MEMCPY((u8*)memBlock.data,
                    buf + 10,
                    memBlock.actualSize);
            err = iso15693WriteSingleBlock(cardptr,
                    buf[0],
                    &memBlock);
            break;
        case 0xdd:
            {
                u16 actlength = 0;
                u8 bits_before_col;
                u16 response_wait_time_ms = buf[0] | (buf[1] << 8);

                if (bufSize < 2) return ERR_PARAM;
                /* Generic command iso15693_2 command, i.e. no flag and crc handling */
                /* Relax settings of mask receive timer for protocols which answer earlier */
                as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, 1);
                iso15693PhySetNoResponseTime_64fcs(MS_TO_64FCS(response_wait_time_ms));
                err = iso15693PhySendFrame((u8*)buf + 2, bufSize - 2, FALSE, FALSE);
                err = iso15693PhyReceiveFrame(txData, *txSize, &actlength, &bits_before_col);
                iso15693PhySetNoResponseTime_64fcs(ISO15693_NO_RESPONSE_TIME);
                as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, ISO15693_MASK_RECEIVE_TIME);
                *txSize = actlength;
            }
            break;
        case 0xde:
            {
                u16 actlength = 0;
                if (bufSize < 2) return ERR_PARAM;
                u16 response_wait_time_ms = buf[0] | (buf[1] << 8);
                /* Generic command */
                err = iso15693TxRxNBytes((u8*)buf + 2, bufSize - 2, txData, *txSize, &actlength, response_wait_time_ms);
                *txSize = actlength;
            }
            break;
        default:
            err = ERR_REQUEST;
    }
    return err;
}

/*!
  Process direct commmands. Some direct commands produce a value which can be read back.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  All AS3911 direct commands between 0xC0 and 0xFF are executed. 
  The Host->Device packets all have the same layout:

  <table>
    <tr><th>   Byte</th><th>       0</th><th>       1             </th></tr>
    <tr><th>Content</th><td>0x15(ID)</td><td>AS3911 direct command</td></tr>
  </table>

  Some direct commands have special handling, i.e. they return values:

  - #AS3911_CMD_MEASURE_AMPLITUDE 0xD3:
    Function as3911MeasureRF() is called which returns  the measured amplitude read from register AS3911_REG_AD_RESULT.
    Response is:
    <table>
      <tr><th>   Byte</th><th>         0          </th></tr>
      <tr><th>Content</th><td>AS3911_REG_AD_RESULT</td></tr>
    </table>
  - #AS3911_CMD_ADJUST_REGULATORS 0xD6:
    Clear reg_s bit and call as3911AdjustRegulators(). The register 
    AS3911_REG_REGULATOR_RESULT is read and converted to u16 milliVolts.
    Response is:
    <table>
      <tr><th>   Byte</th><th>         0    </th><th>         0    </th></tr>
      <tr><th>Content</th><td>LSB millivolts</td><td>MSB millivolts</td></tr>
    </table>
  - #AS3911_CMD_CALIBRATE_MODULATION 0xD7:
    Function as3911CalibrateModulationDepth() is called which returns register AS3911_REG_AM_MOD_DEPTH_RESULT.
    Response is:
    <table>
      <tr><th>   Byte</th><th>                   0          </th></tr>
      <tr><th>Content</th><td>AS3911_REG_AM_MOD_DEPTH_RESULT</td></tr>
    </table>
  - #AS3911_CMD_CALIBRATE_ANTENNA 0xD8:
    First the bit trim_s gets cleared to make command do something.
    Function as3911CalibrateAntenna() gets called which returns register AS3911_REG_ANT_CAL_RESULT.
    Response is:
    <table>
      <tr><th>   Byte</th><th>                   0     </th></tr>
      <tr><th>Content</th><td>AS3911_REG_ANT_CAL_RESULT</td></tr>
    </table>
  - #AS3911_CMD_MEASURE_PHASE 0xD9:
    Function as3911MeasureAntennaResonance() is called which returns the register AS3911_REG_AD_RESULT.
    Response is:
    <table>
      <tr><th>   Byte</th><th>         0          </th></tr>
      <tr><th>Content</th><td>AS3911_REG_AD_RESULT</td></tr>
    </table>
  - #AS3911_CMD_CALIBRATE_C_SENSOR 0xDD:
    Response is:
    <table>
      <tr><th>   Byte</th><th>         0          </th></tr>
      <tr><th>Content</th><td>AS3911_REG_CAP_SENSOR_RESULT</td></tr>
    </table>
  - #AS3911_CMD_MEASURE_CAPACITANCE 0xDE:
    Response is:
    <table>
      <tr><th>   Byte</th><th>         0          </th></tr>
      <tr><th>Content</th><td>AS3911_REG_AD_RESULT</td></tr>
    </table>
  - #AS3911_CMD_MEASURE_VDD 0xDF :
    This command is special as it takes as additional parameter mpsv the id 
    of the power supply which should be measured. If set the value is written 
    into register AS3911_REG_REGULATOR_CONTROL_mpsv_vdd:
    <table>
      <tr><th>  0</th><th>    1</th><th>  2  </th><th>   3  </th></tr>
      <tr><th>VDD</th><td>VSP_A</td><td>VSP_D</td><td>VSP_RF</td></tr>
    </table>
    So the packet sent to the host looks like this:
    <table>
      <tr><th>   Byte</th><th>       0</th><th> 1  </th><th> 2  </th></tr>
      <tr><th>Content</th><td>0x15(ID)</td><td>0xDF</td><td>mpsv</td></tr>
    </table>
    Function as3911MeasureVoltage() returns the measured voltage as u16 milliVolts:
    <table>
      <tr><th>   Byte</th><th>         0    </th><th>         0    </th></tr>
      <tr><th>Content</th><td>LSB millivolts</td><td>MSB millivolts</td></tr>
    </table>
*/
static s8 processDirectCommand(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    u8 cmd = rxData[0];
    memset(txData, 0xff, *txSize);
    s8 err = ERR_NONE;

    /* direct commands */
    switch (cmd)
    {
        case AS3911_CMD_MEASURE_AMPLITUDE:
            err = as3911MeasureRF(txData);
            if (*txSize) *txSize = 1;
            break;
        case AS3911_CMD_ADJUST_REGULATORS:
            { /* make sure MSB of voltage definition register is cleared */
                u16 mV;
                err = as3911ModifyRegister(AS3911_REG_REGULATOR_CONTROL, AS3911_REG_REGULATOR_CONTROL_reg_s, 0x0);
                as3911AdjustRegulators(&mV);
                txData[0] = mV & 0xff;
                txData[1] = mV >> 8;
                if (*txSize) *txSize = 2;
            }
            break;
        case AS3911_CMD_CALIBRATE_MODULATION:
            err = as3911CalibrateModulationDepth(txData );
            if (*txSize) *txSize = 1;
            break;
        case AS3911_CMD_CALIBRATE_ANTENNA:
            /* make sure MSB of external trim register is cleared */
            err = as3911ModifyRegister(AS3911_REG_ANT_CAL_CONTROL, AS3911_REG_ANT_CAL_CONTROL_trim_s, 0x0);
            if (ERR_NONE == err)
            {
                err = as3911CalibrateAntenna(txData);
                 
            }
            if (*txSize) *txSize = 1;
            break;
        case AS3911_CMD_MEASURE_PHASE:
            err = as3911MeasureAntennaResonance(txData );
            if (*txSize) *txSize = 1;
            break;
        case AS3911_CMD_MEASURE_CAPACITANCE:
            as3911ExecuteCommandAndGetResult(cmd,
                                        AS3911_REG_AD_RESULT,
                                        100,
                                        txData);
            break;
        case AS3911_CMD_CALIBRATE_C_SENSOR:
            as3911ExecuteCommandAndGetResult(cmd,
                                        AS3911_REG_CAP_SENSOR_RESULT,
                                        255,
                                        txData);
            break;
        case AS3911_CMD_MEASURE_VDD:
            {
                u16 mV;
                u8 mpsv = 0;
                if (rxSize > 1)
                    mpsv = rxData[1];
                else
                {
                    as3911ReadRegister(AS3911_REG_REGULATOR_CONTROL_mpsv_vdd, &mpsv);
                    mpsv &= AS3911_REG_REGULATOR_CONTROL_mask_mpsv;
                }

                mV = as3911MeasureVoltage(mpsv);
                txData[0] = mV & 0xff;
                txData[1] = mV >> 8;
                if (*txSize > 0) *txSize = 2;
            }
            break;
        default:
            err = as3911ExecuteCommand(cmd);
            *txSize = 0;
    }

    return err;
}

#ifdef HAS_MIFARE
static s8 processMifare(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    u8 cmd = rxData[0];
    s8 err = ERR_NONE;
    const u8 *buf = rxData + 1;
    u16 bufSize = rxSize - 1;

    memset(txData, 0xff, *txSize);

    u8 authentication_key;
    u16 numBytesReceived;
    u8 mifare_request[2];
    u8 mifare_reply;

    switch (cmd)
    {
        case 0xE0:
            /* Configure demoboard for MiFare. */
            err = mifareInitialize();
            mifareResetCipher();
            *txSize = 0;
            break;
        case 0xE1:
        case 0xE2:
            *txSize = 0;
            if (cmd == 0xE1)
            {
                authentication_key = MIFARE_AUTH_KEY_A;
            }
            else
            {
                authentication_key = MIFARE_AUTH_KEY_B;
            }

            err = mifareAuthenticateStep1(authentication_key,
                    buf[0],
                    &buf[8],
                    buf[7],
                    &buf[1]);
            if (ERR_NONE != err)
            {
                break;
            }

            err = mifareAuthenticateStep2(0x11223344);
            break;
        case 0xE3:
            err = mifareSendRequest(buf,
                    bufSize,
                    txData,
                    *txSize,
                    &numBytesReceived,
                    0xFFFF,
                    FALSE);

            *txSize = numBytesReceived;
            break;
        case 0xE5:
            /* MiFare read block command. */
            if(*txSize < 16) return ERR_PARAM;
            if(bufSize < 1) return ERR_PARAM;

            mifare_request[0] = MIFARE_READ_BLOCK;
            mifare_request[1] = buf[0];

            err = mifareSendRequest(mifare_request,
                    sizeof(mifare_request),
                    txData,
                    *txSize,
                    &numBytesReceived,
                    MIFARE_READ_TIMEOUT,
                    FALSE);

            *txSize = numBytesReceived-2;
            break;
        case 0xE6:
            /* MiFare write block command. */
            /* Generate MiFare write block request. */
            mifare_request[0] = MIFARE_WRITE_BLOCK;
            mifare_request[1] = buf[0];

            /* Send write block request, enabling mifare 4 bit response. */
            err = mifareSendRequest(mifare_request,
                    sizeof(mifare_request),
                    &mifare_reply,
                    sizeof(mifare_reply),
                    &numBytesReceived,
                    MIFARE_WRITE_COMMAND_TIMEOUT,
                    TRUE);

            /* Stop processing write request if an error occured. */
            if (ERR_NONE == err)
            {
                /* No error occured. Send the data. */
                err = mifareSendRequest(&buf[1],
                        16,
                        txData,
                        *txSize,
                        &numBytesReceived,
                        MIFARE_WRITE_DATA_TIMEOUT,
                        FALSE);

                *txSize = numBytesReceived;

            }
            break;
        case 0xEF:
            if (bufSize < 1) return ERR_PARAM;
            *txSize = 0;
            mifareDeinitialize(buf[0]);
        default:
            err = ERR_REQUEST;
    }
    return err;
}
#endif

/*!
  Process FeliCa type commands.

  \param rxData : forward from applProcessCmd()
  \param rxSize : forward from applProcessCmd()
  \param txData : forward from applProcessCmd()
  \param txSize : forward from applProcessCmd()

  Implemented commands:

  - #felicaInitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>       1        </th></tr>
      <tr><th>Content</th><td>0xf0(ID)</td><td>modulation_index</td></tr>
    </table>
    no response except status.
  - #felicaDeinitialize()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>         1</th></tr>
      <tr><th>Content</th><td>0xff(ID)</td><td>keep_rf_on</td></tr>
    </table>
    no return value only status
  - #felicaPoll()
    <table>
      <tr><th>   Byte</th><th>       0</th><th>       1 </th><th>   2</th><th>   3</th><th>     1</th></tr>
      <tr><th>Content</th><td>0xf1(ID)</td><td>num_slots</td><td>sys1</td><td>sys2</td><td>compar</td></tr>
    </table>
    where
    <ul>
      <li> \e num_slots : (0,1,3,7,15) the number of slots(1,2,4,8,16) to be performed
      <li> \e sys1: paramter accoring standard for POLL command.
      <li> \e sys2: paramter accoring standard for POLL command.
      <li> \e compar: paramter accoring standard for POLL command:
          <ul>
              <li> 0: no additional info requested
              <li> 1: request system code
              <li> 2: request communication information
              <li> others: reserved
           </ul>
    </ul>
    Response is:
    <table>
      <tr><th>   Byte</th><th>   0     </th><th>   1    </th><th> 2    </th><th>..</th><th> 22   </th><th>..</th><th> 42   </th><th>..</th></tr>
      <tr><th>Content</th><td>num_cards</td><td>num_cols</td><td>card 0</td><td>..</td><td>card 1</td><td>..</td><td>card 2</td><td>..</td></tr>
    </table>
    <ul>
      <li> \e num_cards : the number of cards following
      <li> \e num_cols: the number of collisions detected in responses to POLL command
      <li> \e card: felica Proximity card response according to layout of struct #felicaProximityCard.
    </ul>
    */
static s8 processFeliCa(const u8 *rxData, u16 rxSize, u8 *txData, u16 *txSize)
{
    u8 cmd = rxData[0];
    s8 err = ERR_NONE;
    const u8 *buf = rxData + 1;
    u16 bufSize = rxSize - 1;
    u16 numBytesReceived;

    memset(txData, 0xff, *txSize);

    switch (cmd)
    {
        case 0xF0:
            /* Configure demoboard for FeliCa. */
            if (bufSize >= 1) felicaModIndex = buf[0];
            err = felicaInitialize(felicaModIndex);
            *txSize = 0;
            break;
        case 0xF1:
            {
            u8 slots = buf[0];
            u8 sys1 = buf[1];
            u8 sys2 = buf[2];
            u8 compar = buf[3];
            u8 *num_cards, *num_cols;
            if (bufSize < 4) return ERR_PARAM;
            if (*txSize < 2) return ERR_PARAM;
            num_cards = txData;
            num_cols = txData + 1;
            *num_cards = (*txSize - 2) / sizeof(struct felicaProximityCard);
            *num_cols = 0;

            err = felicaPoll(slots, sys1, sys2, compar,
                    (struct felicaProximityCard*) (txData + 2),
                    num_cards, num_cols); 
            break;
            }
        case 0xF2:
            err =  felicaTxRxNBytes(buf, bufSize, txData, *txSize, &numBytesReceived);
            *txSize = numBytesReceived;
            break;
        case 0xFF:
            if (bufSize < 1) return ERR_PARAM;
            *txSize = 0;
            felicaDeinitialize(buf[0]);
        default:
            err = ERR_REQUEST;
    }
    return err;
}

u8 applPeripheralReset(void)
{
    return 0;
    //as3911Reset();
}

u8 applReadReg ( u16 rxSize, const u8 * rxData, u16 *txSize, u8 * txData)
{
    u8 addr;
    
    if(rxSize < 1) return ERR_REQUEST;

    addr = rxData[0];
    if (addr & 0xc0)
    { /* virtual addresses above 0x3f provide access to test registers */
        as3911TestPrefix();
    }
    addr &= 0x3f;
    as3911ReadRegister(addr,txData);
    *txSize = 1;
    return 0;
}

u8 applWriteReg  ( u16 rxSize, const u8 * rxData, u16 *txSize, u8 * txData)
{
    u8 addr, val;

    addr = rxData[0];
    val = rxData[1];
    if (addr & 0xc0)
    { /* virtual addresses above 0x3f provide access to test registers */
        as3911TestPrefix();
    }
    addr &= 0x3f;

    if(rxSize < 2) return ERR_REQUEST;

    as3911WriteRegister(addr, val);
    return 0;
}

const u32 firmwareNumber = 0x020006;

const char * applFirmwareInformation(void)
{
    return "AS3911 General Purpose Firmware v2.0.06";
}

