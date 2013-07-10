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
 * $Revision: $
 * LANGUAGE: ANSI C
 */
/*! \file
 *
 * \author Christian Eisendle
 *
 * \brief AS3911 firmware main function.
 *
 * Initializes the evaluation board and AS3911 and then
 * starts the GUI command dispatcher.
 */
/*!
  \mainpage
  This firmware is targetted to run on a Microchip PIC24FJ64GB002. It 
  includes a driver for AS3911 and provides proper functions over USB to a GUI.
  The protocols it supports are currently:
  <ul>
    <li> ISO14443A
    <li> Mifare-Ultralight
    <li> ISO14443B
    <li> ISO15693
    <li> ISO18092 (NFC-IP1)
    <li> FeliCa
  </ul>
  Others like Topaz are supported by AS3911 but not yet by this firmware.
  Using \link as3911_stream.h stream mode \endlink it is also possible to
  support other protols not directly supported by AS3911 like ISO15693.

  In the following page the following topics will be detailed:
  <ul>
    <li> \ref Layering
    <li> \ref Startup
    <li> \ref Operation
    <li> \ref Porting
    <li> \ref Resources
  </ul>
  \section Layering
  This system is layered as follows:

  <table align="center" cellpadding=20 cellspacing=0 rules="none">
  <tr>
      <td align="center" colspan=13 bgcolor="#34AAFF">AS3911 GUI</td>
  </tr>
  <tr>
      <td align="center" colspan=13 bgcolor="#78AAFC">\link usb_hid_stream_driver.h USB\endlink</td>
  </tr>
  <tr>
      <td align="center" colspan=13 bgcolor="#FC6666">\link dispatcher.c Dispatcher\endlink</td>
  </tr>
  <tr>
      <td align="center" colspan=6 bgcolor="#FC6666">&nbsp;</td>
      <td align="center" bgcolor="#FC99BB">\link mifare_ul.h Mifare Ultralight\endlink</td>
      <td align="center" colspan=6 bgcolor="#FC6666">&nbsp;</td>
  </tr>
  <tr>
      <td align="center" colspan=6 bgcolor="#FC6666">&nbsp;</td>
      <td align="center" colspan=2 bgcolor="#EEEEEE">\link iso14443_common.h ISO14443 Common\endlink</td>
      <td align="center" colspan=5 bgcolor="#FC6666">&nbsp;</td>
  </tr>
  <tr>
      <td align="center" colspan=2 rowspan=2 bgcolor="#FC6666">&nbsp;</td>
      <td align="center" colspan=1 rowspan=2 bgcolor="#777777">\link topaz.h Topaz \endlink </td>
      <td align="center" colspan=1 rowspan=2 bgcolor="#888888">\link mifare.h MiFare \endlink <br> \link mifare.h Classic \endlink</td>
      <td align="center" colspan=1 rowspan=2 bgcolor="#CCCCCC">\link felica.h FeliCa\endlink</td>
      <td align="center" colspan=2 rowspan=2 bgcolor="#DDDDDD">\link iso14443a.h ISO14443a\endlink</td>
      <td align="center" colspan=2 rowspan=2 bgcolor="#ECEEBC">\link iso14443b.h ISO14443b\endlink</td>
      <td align="center" colspan=2 rowspan=2 bgcolor="#FCFE9C">\link nfc.h NFC\endlink</td>
      <td align="center" colspan=2 bgcolor="#EFA8A5">\link iso15693_3.h ISO15693-3\endlink</td>
  </tr>
  <tr>
      <td align="center" colspan=2 bgcolor="#8FEA8A">\link iso15693_2.h ISO15693-2\endlink</td>
  </tr>
  <tr>
      <td align="center" bgcolor="#FC6666">&nbsp;</td>
      <td align="center" colspan=10 bgcolor="#ACFC9C">\link as3911.h AS3911 Driver\endlink</td>
      <td align="center" colspan=2 rowspan=1 bgcolor="#7AFF14">\link as3911_stream.h Stream Mode\endlink</td>
  </tr>
  <tr>
      <td align="center" colspan=13 bgcolor="#7AAF14">\link as3911_com.h AS3911 Communication\endlink</td>
  </tr>
  <tr>
      <td align="center" colspan=13 bgcolor="#1AAF14">\link spi_driver.h SPI\endlink</td>
  </tr>
  <tr>
      <td align="center" colspan=13 bgcolor="#1A7F14">AS3911 RFID Reader Chip</td>
  </tr>
  <tr>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
  </tr>
  </table>

  There are some more additional modules which are used by most of the modules listed above. They
  have a supporting function and can therefore not be listed within the layering model. These modules are:
  - \link delay.h Delay module\endlink
  - \link clock.h Clock handling module \endlink
  - \link board.h Board specific configuration \endlink
  - \link crc.h CRC calculation module \endlink
  - \link ic.h Interrupt controller module\endlink

  \section Porting
  In the above diagram all layers between(including) Dispatcher and AS3911 Communication don't need to be touched.
  User who wish to port this firmware as is to another microcontroller need to modifiy the following files:
  <ul>
    <li> main.c 
    <li> Files coming from Microchip:
      <ul>
        <li>usb_device.c    
        <li>usb_device.h    
        <li>usb_function_generic.c  
        <li>usb_function_generic.h  
        <li>usb_function_hid.c  
        <li>usb_function_hid.h  
        <li>usb_hal_pic24.c 
        <li>usb_hal_pic24.h 
      </ul>
    <li>timer.c
    <li>beep.c
    <li>board.c
    <li>clock.c
    <li>delay.c
    <li>ic.c
    <li>irq_table.c
    <li>logger.c
    <li>spi_driver.c
    <li>uart.c
    <li>as3911_interrupt.c
  </ul>
  All other as3911* and RFID protocol files don't need to be touched.


  \section Startup Board startup
  On startup and after executing the internal startup code of the Microcontroller,
  function main in file main.c is called. Purpose of this function is to initialize
  the required modules so they can be used later on by the application. The initialization
  is divided into the following parts:
  - Initialization of the clock driver (#clkInitialize). At this point the controller
    runs from internal clock.
  - GPIO and Pin configuration (#boardInitialize). Since the used controller has a PPS unit
    (peripheral pin select) which allows multplexing the controller's peripherals to different IO's,
    this pps unit of course needs to be configured for proper work of the system.
  - Configuring used timers (#delayInitialize). Most of the protocols require
    specific delays which should be quite accurate. Therefore delay functions
    are implemented using timers (1 32bit timer for milliseconds delay and 1 16 bit
    timer for microseconds delay).
  - Setting up the interrupt controller (#icInitialize). The current system uses
    three interrupt sources: External AS3911 interrupt line
    (refer to \link as3911_interrupt.c\endlink), USB interrupt and timer 5 interrupt
    which is used by the #delayNMilliSeconds functions to allow
    system sleep while delaying.
  - Configuration of SPI (#spiInitialize). Since the AS3911 is connected to the
    controller via SPI the SPI module needs to be configured prior to any AS3911 calls.
  After these steps, AS3911 is powered up and configured (#as3911Initialize). 
  From this time on the controller can use the clock generated by the AS3911. 

  \section Operation
  After startup the application enters the main loop. Within the main loop the
  \link dispatcher.c module\endlink checks for any received \link usb_hid_stream_driver.h USB\endlink data.
  If received data contains a valid command, this command is \link dispatcher.c dispatched\endlink. 

  So the main function looks like this:
  \code
int main(void)
{
    // init functions as above....

    while (1)
    {
        ProcessIO(); // perform USB processing provided by Microchip USB stack

        dispatcherInterruptHandler();
    }
}
  \endcode

  The main call graph looks like this:

  \dot
  digraph main_loop{
      subgraph clusterDispatcher
      { 
          label="dispatcher.c";
          applProcessCommand [label="applProcessCommand()"];
          processCommand [label="processCommand()"];
          processProtocols [label="processProtocols()"];
          processISO14443A [label="processISO14443A()"];
          processISO14443B [label="processISO14443B()"];
          processISO15693 [label="processISO15693()"];
      }
  USBStream [label="usb_hid_stream_driver",shape=box];
  ProcessIO [label="ProcessIO()"];
  USBStack [label="USBStack from Microchip"];
  init [label="",shape=plaintext];
  init -> ProcessIO;
  USBStack [shape=box];
  ProcessIO ->  USBStack;
  USBStack ->  USBStream;
  USBStream ->  applProcessCommand;
  applProcessCommand -> processCommand;
  processCommand->processProtocols;
  processProtocols->processISO14443A;
  processProtocols->processISO14443B;
  processProtocols->processISO15693;
  }
  \enddot

  \section Resources
  This firmware is built for PIC24FJ64B002. Other PIC24FJ processors should be easy to be ported.
  
  Build environment.
  <ul>
    <li> Cygwin is used for processing Makefiles. Using MPLAB X 
         should be no problem but user must create a proper project file.
         Non-standard cygwin tools which need to be installed:
         <ul>
           <li>make
           <li>doxygen
           <li>graphviz (for graphs in doxygen documentation): Please see 
               <a href="http://sourceware.org/cygwinports/">http://sourceware.org/cygwinports/</a>
               to install graphviz from their repository.
         </ul>
    <li> The used compiler is MPLAB C30. It must be specified be specified via 
         environment variables: c30dir=C:\\MPLABC30/
    <li> doxygen is used for generating this documentation you currently read
  </ul>

  Currently this firmware needs the following processor resources. Be aware that this 
  can be cut down by removing protocols, buffers, stacks, etc.

  \verbinclude linker_stdout.txt

  Detailed analysis of compiled files, library code from C30 compiler is not included:

  \verbinclude as3911_sizes.txt

*/

/**
   \file usb_descriptors.c              Source code from Microchip
   \file usb_device.c                   Source code from Microchip
   \file usb_device.h                   Source code from Microchip
   \file usb_function_generic.c         Source code from Microchip
   \file usb_function_generic.h         Source code from Microchip
   \file usb_function_hid.c             Source code from Microchip
   \file usb_function_hid.h             Source code from Microchip
   \file usb_hal_pic24.c                Source code from Microchip
   \file usb_hal_pic24.h                Source code from Microchip
*/
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#define USE_AND_OR
#include <adc.h>
#include <PPS.h>
#include "uart.h"
#include "spi_driver.h"
#include "clock.h"
#include "delay.h"
#include "as3911.h"
#include "as3911_com.h"
#include "as3911_interrupt.h"
#include "board.h"
#include "led.h"
#include "ic.h"
#include "utils.h"
#include "dispatcher.h"
#include "felica.h"
#include "iso14443a.h"
#include "iso14443_common.h"
#include "nfc.h"
#include "iso15693_2.h"
#include "iso15693_3.h"
#include "usb.h"
#include "usb_function_hid.h"
#include "usb_function_generic.h"
#include "usb_hid_stream_driver.h"
#include "stream_dispatcher.h"
#include <stdio.h>
#include <libpic30.h>
#include "beep.h"
#include "logger.h"
/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

#define BULK_DEBUG 0

#ifdef HAS_BOOTLOADER
extern void _resetPRI();
u16 bltappId __attribute__ ((space(prog), section(".blid"))) = 0xBAAE;
u16 bluserReset __attribute__ ((space(prog), section(".bladdr"))) = (u16)_resetPRI;
#else
_CONFIG1( JTAGEN_ON & GCP_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx1 )
_CONFIG2( PLL_96MHZ_ON & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_NONE & FNOSC_FRCPLL & PLLDIV_NODIV & IOL1WAY_ON)
#endif

void INTERRUPT _AddressError(void)
{
    dbgLog("FATAL: ADDRESS ERROR!");
    LED_ON(LED_1);
    LED_ON(LED_2);
    LED_ON(LED_3);
    LED_ON(LED_4);
    INTCON1 = 0;
    while(1);
}

USB_HANDLE USBGenericOutHandle;
USB_HANDLE USBGenericInHandle;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
struct gen_buf{
    int index;
    int txlen;
    int rxlen;
    int sending;
    u8 buf[1100];
};
struct gen_buf genBuf;
umword IRQ_COUNT; /*!< global interrupt count */

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void printfUartInit()
{
    __C30_UART=1;
}

#if !DOOR_BOARD
unsigned int ADCResult[3];
void readAdc()
{
    unsigned int channel,config1,config2,config3,configport,configport_hi,configscan;
    unsigned int i = 0;
    u8 oprp2 = OUT_PIN_PPS_RP2;

    OUT_PIN_PPS_RP2 = 0;
    while (i < 3)
    {
        channel= ADC_CH0_POS_SAMPLEA_AN2 << i;
        configscan = ADC_SCAN_AN2 << i;

        SetChanADC10(channel);
        /*Configure adc*/
        config1 = ADC_MODULE_OFF | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON ;
        config2 = ADC_SCAN_ON | ADC_INTR_16_CONV;
        config3 = ADC_SAMPLE_TIME_17 | ADC_CONV_CLK_254Tcy;
        configport = 0xFFe3;
        configport_hi = 0x0003;
        OpenADC10(config1,config2,config3,configport,configport_hi,configscan,0);
        EnableADC1;
        ConvertADC10();
        while(!AD1CON1bits.DONE); /*wait till conversion complete*/
        ADCResult[i] = ReadADC10(0);
        CloseADC10();
        i++;
    }
    OUT_PIN_PPS_RP2 = oprp2;
    return;
}
#endif

//struct felicaProximityCard card[16];
int main(void)
{
    u32 baudrate;
    s8 err = ERR_NONE;
    u32 i=0;
    spiConfig_t spiConfig;
#if DOOR_BOARD
    u8 wakeup_running = 0;
#endif

    /* initialize global interrupt count to 0 */
    IRQ_COUNT = 0;

    /* set up all peripherals */
#if !DOOR_BOARD
    readAdc();
#endif

    clkInitialize();
    boardInitialize();
    ledInitialize();
    delayInitialize();
    icInitialize();
    boardPeripheralPinInitialize(BOARD_PERIPHERAL_SPI1);
    /* configure spi */
    spiConfig.instance = SPI1;
    spiConfig.frequency = 4000000;
    spiConfig.clockPhase = 0;
    spiConfig.clockPolarity = 0;
    spiInitialize(SYSCLK,&spiConfig, NULL);
    stopWatchInitialize();

    /* globally enable interrupts */
    IRQ_DEC_ENABLE();

    /* enable as3911 interrupt source */
    icEnableInterrupt(IC_SOURCE_AS3911);

    for (i = 0; i<10; i++)
    {
        delayNMilliSeconds(100);
        LED_OFF(LED_1);
        LED_ON(LED_2);
        LED_OFF(LED_3);
        LED_ON(LED_4);
        delayNMilliSeconds(100);

        LED_ON(LED_1);
        LED_OFF(LED_2);
        LED_ON(LED_3);
        LED_OFF(LED_4);
    }

    uartInitialize(115200, &baudrate);
#if !DOOR_BOARD
    dbgLog("adcs : %hx %hx %hx\n",ADCResult[0],ADCResult[1],ADCResult[2]);
#endif
    printfUartInit();

    /* note: all peripherals depend on SYSCLK = 13.56MHz (output of AS3911),
       actual SYSCLK is 16MHz though. Therefore, appropriate registers
       in AS3911 needs to be set to switch to 13.56MHz SYSCLK */

    err = as3911Initialize();
    if (ERR_NONE != err)
    {
        LED_ON(LED_1);
        LED_ON(LED_2);
        LED_ON(LED_3);
        LED_ON(LED_4);
    }
    else
    {
        u8 reg;
        as3911ReadRegister(AS3911_REG_IC_IDENTITY, &reg);
        LED_OFF(LED_1);
        if (0x00 == reg)
            LED_ON(LED_2);
        else
            LED_OFF(LED_2);
        LED_OFF(LED_3);
        LED_OFF(LED_4);
    }

    /* now as3911 osc output provides 6.78MHz. Change clock tree to make use
       of this clock source. This clock gets multiplied by the internal
       PLL by factor 4 (= 27.12MHz) and divided by 2 afterwards, so SYSCLK
       is 13.56MHz */
    clkSetClockSource(CLK_SOURCE_EXTERNAL);
    delayNMilliSeconds(100);

    //On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
    //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
    //This allows the device to power up at a lower initial operating frequency, which can be
    //advantageous when powered from a source which is not guaranteed to be adequate for 32MHz
    //operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
    //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
//        CLKDIVbits.PLLEN = 1;
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.



//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = 1; // input pin
#endif

//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = 1;	// input pin
#endif

    StreamDispatcherInit( SYSCLK);
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
#if defined(USB_INTERRUPT)
    USBDeviceAttach();
#endif

    dbgLog("as3911Initialize returned %hhd\n",err);

    while (1)
    {
#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        // this function periodically.  This function will take care
        // of processing and responding to SETUP transactions
        // (such as during the enumeration process when you first
        // plug in).  USB hosts require that USB devices should accept
        // and process SETUP packets in a timely fashion.  Therefore,
        // when using polling, this function should be called
        // frequently (such as once about every 100 microseconds) at any
        // time that a SETUP packet might reasonably be expected to
        // be sent by the host to your device.  In most cases, the
        // USBDeviceTasks() function does not take very long to
        // execute (~50 instruction cycles) before it returns.
#endif
        ProcessIO();

        dispatcherInterruptHandler();
        
#if 0
        {
            u8 num_cards, num_cols, i;
            s8 err;
            felicaInitialize(AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_20percent);
            delayNMilliSeconds(20);
            num_cards = 16;
            err = felicaPoll(FELICA_16_SLOTS, 0xff, 0xff, FELICA_REQ_COM_PERFORMANCE, card, &num_cards, &num_cols); 
            //err = felicaPoll(FELICA_1_SLOT, 0xff, 0xff, FELICA_REQ_COM_PERFORMANCE, card, &num_cards, &num_cols); 
            felicaDeinitialize( 0 );
            dbgLog("***************** cards=%hhx, cols=%hhx  *********************************\n",num_cards, num_cols);
            for (i=0; i<num_cards; i++)
            {
                dbgLog("***************** %hhx*********************************\n",i);
                dbgLog("faPoll: %hhx\n", err);
                //dbgLog("Complete response:\n");
                //dbgHexDump(card+i, sizeof(card[i]));
                dbgLog("length=%hhx\n",card[i].length[0]);
                dbgLog("response_code=%hhx\n",card[i].response_code[0]);
                dbgLog("IDm=");
                dbgHexDump(card[i].IDm, sizeof(card[i].IDm));
                dbgLog("PMm=");
                dbgHexDump(card[i].PMm, sizeof(card[i].PMm));
                dbgLog("request_data=");
                dbgHexDump(card[i].request_data, sizeof(card[i].request_data));
            }
            delayNMilliSeconds(1000);
        }
#endif

        nfcReceptionTask();

        // User Application USB tasks
        if(!((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1))) 
        {
            if (genBuf.sending && !USBHandleBusy(USBGenericInHandle))
            {
                genBuf.index += USBGEN_EP_SIZE;
                if (genBuf.index >= genBuf.rxlen + 8)
                {
                    genBuf.sending = 0;
                    genBuf.index = 0;
                    USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)genBuf.buf+genBuf.index,USBGEN_EP_SIZE);
#if BULK_DEBUG
                    dbgLog("new out2 handle = %x\n",(int)USBGenericOutHandle);
#endif
                }
                else
                {
                    if(!USBHandleBusy(USBGenericInHandle))		
                    {	
                        USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)genBuf.buf+genBuf.index,USBGEN_EP_SIZE);	
                    }
#if BULK_DEBUG
                    else
                        dbgLog("gen in1 busy\n");
#endif
                }
            }
            if(!genBuf.sending && !USBHandleBusy(USBGenericOutHandle))		//Check if the endpoint has received any data from the host.
            {
                if (genBuf.index == 0)
                {
                    genBuf.txlen = genBuf.buf[1] | (genBuf.buf[2]<<8);
                    genBuf.rxlen = genBuf.buf[3] | (genBuf.buf[4]<<8);
                    if (genBuf.txlen > sizeof(genBuf.buf)-5)
                        genBuf.txlen = sizeof(genBuf.buf)-5;
                    if (genBuf.rxlen > sizeof(genBuf.buf)-8)
                        genBuf.rxlen = sizeof(genBuf.buf)-8;
#if BULK_DEBUG
                    dbgLog("txlen = %hx, rxlen = %hx\n",genBuf.txlen,genBuf.rxlen);
#endif
                }

#if BULK_DEBUG
                dbgLog("received............received ..........\n");
                dbgHexDump(genBuf.buf+genBuf.index,64);
#endif
                genBuf.index += USBGEN_EP_SIZE;

                if (genBuf.index >= genBuf.txlen + 5)
                {
                    unsigned actlen = 0;
                    u32 us;
                    s8 status;
                    stopWatchStart();
                    status = iso14443TransmitAndReceive(genBuf.buf+5, genBuf.txlen, genBuf.buf+8, genBuf.rxlen, &actlen);
                    us = stopWatchMeasure();
                    genBuf.buf[1] = status;
                    genBuf.rxlen = actlen;
                    genBuf.buf[2] = actlen & 0xff;
                    genBuf.buf[3] = (actlen>>8) & 0xff;
                    genBuf.buf[4] = (us >>  0) & 0xff;
                    genBuf.buf[5] = (us >>  8) & 0xff;
                    genBuf.buf[6] = (us >> 16) & 0xff;
                    genBuf.buf[7] = (us >> 24) & 0xff;
                    genBuf.sending = 1;
                    genBuf.index = 0;
                    if(!USBHandleBusy(USBGenericInHandle))		
                    {	
                        USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)genBuf.buf,USBGEN_EP_SIZE);	
                    }
#if BULK_DEBUG
                    else
                        dbgLog("gen in2 busy\n");
#endif
                }
                else
                {
                    USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)genBuf.buf+genBuf.index,USBGEN_EP_SIZE);
#if BULK_DEBUG
                    dbgLog("new out handle = %x\n",(int)USBGenericOutHandle);
#endif
                }
            }
        }
#if BULK_DEBUG
        else
            dbgLog("gen not configured\n");
#endif

#if DOOR_BOARD
        if(USBDeviceState != CONFIGURED_STATE)
        {
            LED_OFF(LED_1);
            LED_OFF(LED_2);

            if (!wakeup_running)
            {
                u8 val;
                //as3911StartWakeup(AS3911_WAKEUP_CAP, 50);

                as3911WriteRegister(AS3911_REG_CAP_SENSOR_CONTROL,0x01); /* maximal gain, auto calibration */

                as3911ExecuteCommandAndGetResult(AS3911_CMD_CALIBRATE_C_SENSOR,
                        AS3911_REG_CAP_SENSOR_RESULT, 255, &val);

                as3911ExecuteCommandAndGetResult(AS3911_CMD_MEASURE_CAPACITANCE,
                        AS3911_REG_AD_RESULT, 100, &val);

                as3911WriteRegister(AS3911_REG_CAPACITANCE_MEASURE_REF, val);
                as3911WriteRegister(AS3911_REG_CAPACITANCE_MEASURE_CONF,0x20); /* delta = 2, no aam */
                as3911WriteRegister(AS3911_REG_WUP_TIMER_CONTROL,0xc0 | AS3911_REG_WUP_TIMER_CONTROL_wcap ); /* 50 ms */
                as3911WriteRegister(AS3911_REG_OP_CONTROL, AS3911_REG_OP_CONTROL_wu);
                as3911EnableInterrupts(AS3911_IRQ_MASK_WCAP);
                wakeup_running = 1;
            }


            if (as3911GetInterrupt(AS3911_IRQ_MASK_WCAP))
            {
                s8 err;
                iso14443AProximityCard_t card = {{0}};

                LED_ON(LED_1);
                err = iso14443AInitialize();
                delayNMilliSeconds(5);
                err = iso14443ASelect(ISO14443A_CMD_REQA, &card);
                if (ERR_NONE == err)
                { /* Some tag was found */
                    LED_ON(LED_2);
                    beep();
                }
                err = iso14443ADeinitialize(0);
                delayNMilliSeconds(30);
                LED_OFF(LED_1);
                LED_OFF(LED_2);
            }
        }
        else 
        {
            if (wakeup_running)
            {
                //as3911StopWakeup();
                as3911WriteRegister(AS3911_REG_OP_CONTROL, 0);
                as3911DisableInterrupts(AS3911_IRQ_MASK_WCAP);
                wakeup_running = 0;
            }
        }
#endif
    }
    return 0;
}

void applUSBCBHandler(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
    case EVENT_CONFIGURED:
        genBuf.sending = 0;
        genBuf.index = 0;
        //enable the GEN endpoint
        USBEnableEndpoint(USBGEN_EP_NUM,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
        //Re-arm the OUT endpoint for the next packet
        USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)genBuf.buf,USBGEN_EP_SIZE);
        break;
    case EVENT_SET_DESCRIPTOR:
        break;
    case EVENT_EP0_REQUEST:
        break;
    case EVENT_SOF:
        break;
    case EVENT_SUSPEND:
        break;
    case EVENT_RESUME:
        break;
    case EVENT_BUS_ERROR:
        break;
    case EVENT_TRANSFER:
        break;
    default:
        break;
    }
}
