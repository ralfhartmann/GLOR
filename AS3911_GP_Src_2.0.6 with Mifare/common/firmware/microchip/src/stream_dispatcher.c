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
 *      PROJECT:   ASxxxx firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file stream_dispatcher.c
 *
 *  \author M. Arpa originated by W. Reichart
 *
 *  \brief main dispatcher for streaming protocol uses a stream_driver 
 *   for the io.
 *
 */


/* --------- includes ------------------------------------------------------- */

#include "system_clock.h"
#include "ams_stream.h"
#include "stream_dispatcher.h"
#include "stream_driver.h"
#include "usb_hid_stream_driver.h"
#include "uart_stream_driver.h"
#include "bootloadable.h"
#include "ams_types.h"
#include "i2c_driver.h"
#include "spi_driver.h"
#include "logger.h"
/* FCY needed by libpic30.h, pessimistically assume the maximum, needed for __delay functions */
#ifndef FCY
#define FCY 16000000ULL
#endif
#include <libpic30.h>


/* ------------- local variables -------------------------------------------- */

static u8 rxBuffer[ AMS_STREAM_BUFFER_SIZE ]; /*! buffer to store protocol packets received from the Host */
static u8 txBuffer[ AMS_STREAM_BUFFER_SIZE ]; /*! buffer to store protocol packets which are transmitted to the Host */

static u32 systemClock;
static u8 lastError; /* flag inicating different types of errors that cannot be reported in the protocol status field */


/* ------------- local functions --------------------------------------------- */

static u8 handleI2c ( u16 rxed, u8 * rxData, u16 * toTx, u8 * txData )
{
    return i2cRxTx( rxed, rxData, *toTx, txData, TRUE, TRUE );
}

static u8 handleI2cConfig ( u16 rxed, u8 * rxData )
{ 
    i2cConfig_t config;
    i2cDeserialiseConfig( &config, rxData );
    return i2cInitialize( systemClock, &config, 0 /* oldConfig */ );
}

static u8 handleSpi ( u16 rxed, u8 * rxData, u16 * toTx, u8 * txData )
{
    u8 status;
    spiActivateSEN();
    status = spiTxRx( rxData, txData, rxed );
    /* We could also transmit only the last rxed bytes requested,
       but this way it is more SPI like */
    *toTx = rxed;
    spiDeactivateSEN();
    return status;
}

static u8 handleSpiConfig ( u16 rxed, u8 * rxData )
{    
    spiConfig_t config;
    spiDeserialiseConfig( &config, rxData );
    return spiInitialize( systemClock, &config, 0 );
}

static u8 handleReset ( u16 rxed, u8 * rxData )
{
    u8 status = AMS_STREAM_NO_ERROR;

    DBG_ASSERT( rxed == 1); /* a reset command consists of 2 bytes, command byte and flags  */
    /* at position 0 after the reset command we find the device selection flag */
    switch( *rxData )
    {
    case AMS_COM_RESET_MCU: /* reset the PIC */
        INFO_LOG( "Reset MCU\n" );
#ifdef __DEBUG
        DBG_ASSERT( 0 );
#endif
        StreamDisconnect();
        __delay_ms( 100 ); /* to give host time to recognize detach event */
        Reset();
        break;
    case AMS_COM_RESET_PERIPHERAL: /* reset the peripheral(s) */
        INFO_LOG( "Reset Peripherals\n" );
	status = applPeripheralReset( );
        break;
    default:
        INFO_LOG( "Reset N/A\n");
        status = AMS_STREAM_UNHANDLED_PROTOCOL;
        break;
    }
    return status;
}

static u8 handleFirmwareInformation ( u16 * toTx, u8 * txData )
{
    u16 size = strlen( applFirmwareInformation( ) );
    INFO_LOG( "FirmwareInfo\n" );
    if ( size >= *toTx - 1 )
    {
	size = *toTx - 1;
    }
    if ( size >= AMS_STREAM_SHORT_STRING - 1 )
    {
	size = AMS_STREAM_SHORT_STRING - 1;
    }
    *toTx = size + 1; /* for zero terminator */
    memcpy( txData , applFirmwareInformation(), size + 1 );
    return AMS_STREAM_NO_ERROR;
}

static u8 handleFirmwareNumber ( u16 * toTx, u8 * txData )
{
    INFO_LOG( "FirmwareNumber\n" );
    *toTx = 3; /* 1-byte major, 1-byte minor, 1-byte release marker */
    txData[0] = ( firmwareNumber >> 16 ) & 0xFF;
    txData[1] = ( firmwareNumber >>  8 ) & 0xFF;
    txData[2] =   firmwareNumber         & 0xFF;
    return AMS_STREAM_NO_ERROR;
}

static u8 handleEnterBootloader ( )
{
    INFO_LOG( "Enable Bootloader\n" );
    enableBootloader(); /* never returns */
    return AMS_STREAM_NO_ERROR;
}

static u16 processReceivedPackets ( )
{
    /* every time we enter this function, the last txBuffer was already sent.
       So we fill a new transfer buffer */
    u8 * txEnd = txBuffer; /* the txEnd always points to the next position to be filled with data */
    u16 txSize = 0;

    INFO_LOG( "ProcessReceivedPackets\n" );

    while ( StreamHasAnotherPacket( ) )
    {
        /* read out protocol header data */
        u8 protocol = AMS_STREAM_DR_GET_PROTOCOL( rxBuffer );
        u16 rxed     = AMS_STREAM_DR_GET_RX_LENGTH( rxBuffer );
        u16 toTx     = AMS_STREAM_DR_GET_TX_LENGTH( rxBuffer );
        u8 * rxData = AMS_STREAM_PAYLOAD( rxBuffer );
        /* set up tx pointer for any data to be transmitted back to the host */
        u8 * txData = AMS_STREAM_PAYLOAD( txEnd );
        u8 status = AMS_STREAM_NO_ERROR;
	s8 isReadCommand = ! ( protocol & AMS_COM_WRITE_READ_NOT );
	protocol &= ~AMS_COM_WRITE_READ_NOT; /* remove direction flag */

        /* decide which protocol to execute */
        INFO_LOG( "RX-Packet: Prot=%hhx, rxed=%u, toTx=%u\n", protocol, rxed, toTx );
#if STREAM_DEBUG
        LOGDUMP( rxBuffer, AMS_STREAM_HEADER_SIZE + rxed );
#endif
        switch ( protocol )
        {
  	    case AMS_COM_CONFIG:
	      // FIXME mar: implement me
	        break;
            case AMS_COM_I2C:
	        status = handleI2c( rxed, rxData, &toTx, txData );
                break;
            case AMS_COM_I2C_CONFIG:
	        status = handleI2cConfig( rxed, rxData );
                break;
            case AMS_COM_SPI:
	        status = handleSpi( rxed, rxData, &toTx, txData );
                break;
            case AMS_COM_SPI_CONFIG:
	        status = handleSpiConfig( rxed, rxData );
                break;
            case AMS_COM_CTRL_CMD_RESET:
	        status = handleReset( rxed, rxData );
                break;
	    case AMS_COM_CTRL_CMD_FW_INFORMATION:
	        status = handleFirmwareInformation( &toTx, txData );
                break;
	    case AMS_COM_CTRL_CMD_FW_NUMBER:
	        status = handleFirmwareNumber( &toTx, txData );
                break;
	    case AMS_COM_CTRL_CMD_ENTER_BOOTLOADER:
	        status = handleEnterBootloader(  );
                break;
	    case AMS_COM_WRITE_REG:
		status = applWriteReg( rxed, rxData, &toTx, txData );
		break;
	    case AMS_COM_READ_REG:
                status = applReadReg( rxed, rxData, &toTx, txData );
                break;
            default:
	        if ( protocol > AMS_COM_CONFIG )
		{ /* reserved protocol value and not handled so far */
		  status = AMS_STREAM_UNHANDLED_PROTOCOL;
		}
		else /* application protocol */
		{
		    status = applProcessCmd( protocol, rxed, rxData, &toTx, txData );
		}
                break;
        }

	INFO_LOG( "PacketStatus=%hhx\n", status );

        /* fill out transmit packet header if any data was produced */
        if ( toTx > 0 || isReadCommand )
        {
	    AMS_STREAM_DT_SET_PROTOCOL( txEnd, protocol );
            AMS_STREAM_DT_SET_STATUS( txEnd, status );
            AMS_STREAM_DT_SET_TX_LENGTH( txEnd,toTx );
	    
            INFO_LOG( "TX-Packet: Prot=%hhx stat=%hhx toTx=%u\n", protocol, status, toTx );
#if STREAM_DEBUG
            LOGDUMP(txEnd, toTx + AMS_STREAM_HEADER_SIZE );
#endif
            /* adjust pointer to the enter next data, and total size */
            txEnd += ( toTx + AMS_STREAM_HEADER_SIZE );
            txSize += (toTx + AMS_STREAM_HEADER_SIZE );
        }
	else if ( status != AMS_STREAM_NO_ERROR ) /* protocol failed, we indicate an error if command itself does not send back */
	{
	    INFO_LOG( "ERROR:Prot=%hhx stat=%hhx\n", protocol, status ); 
	    lastError = status;
	}
	  
        /* remove the handled packet, and move on to next packet */
        StreamPacketProcessed( rxed );
    }

    INFO_LOG( "LeaveProcessReceivedPackets (txSize=%u)\n", txSize );

    return txSize;
}

static u16 processCyclic ( )
{
    /* every time we enter this function, the last txBuffer was already sent.
       So we fill a new transfer buffer */
    u8 * txEnd = txBuffer; /* the txEnd always points to the next position to be filled with data */
    u16 txSize = 0;
    u16 toTx;
    u8 protocol;
    u8 status = AMS_STREAM_NO_ERROR;

    do
    {
        /* set up tx pointer for any data to be transmitted back to the host */
        u8 * txData = AMS_STREAM_PAYLOAD( txEnd );
        toTx = 0;
        status = applProcessCyclic( &protocol, &toTx, txData, AMS_STREAM_MAX_DATA_SIZE - txSize );

        /* fill out transmit packet header if any data was produced */
        if ( toTx > 0 )
        {
            INFO_LOG( "ProcessCyclic\n" );

            AMS_STREAM_DT_SET_PROTOCOL( txEnd, protocol );
            AMS_STREAM_DT_SET_STATUS( txEnd, status );
            AMS_STREAM_DT_SET_TX_LENGTH( txEnd, toTx );

            INFO_LOG( "TX-Packet: protocol=%hhx status=%hhx toTx=%u\n", protocol, status, toTx );
#if STREAM_DEBUG
	    LOGDUMP( txEnd, toTx + AMS_STREAM_HEADER_SIZE );
#endif
            /* adjust pointer to the enter next data, and total size */
            txEnd += ( toTx + AMS_STREAM_HEADER_SIZE );
            txSize += (toTx + AMS_STREAM_HEADER_SIZE );
        }
	else if ( status != AMS_STREAM_NO_ERROR ) /* protocol failed, we indicate an error if command itself does not send back */
	{
	    lastError = status;
	}
    }
    while ( toTx > 0 );

    return txSize;
}


/* --------- global functions ------------------------------------------------------------- */

void StreamDispatcherInitAndConnect ( u32 sysClk )
{
    StreamDispatcherInit( sysClk );
    StreamConnect();
}

void StreamDispatcherInit ( u32 sysClk )
{
    StreamDispatcherGetLastError();
    systemClock = sysClk;
    StreamInitialize( rxBuffer, txBuffer );
}

u8 StreamDispatcherGetLastError( )
{
    u8 temp = lastError;
    lastError = AMS_STREAM_NO_ERROR;
    return temp;
}

void ProcessIO(void)
{
    u16 txSize;

    if ( StreamReady() )
    {
        /* read out data from stream driver, and move it to module-local buffer */
        if ( StreamReceive() > 0 )
        {
            /* if we have at least one fully received packet, we start execution */
  
            /* interpret one (or more) packets in the module-local buffer */
            txSize = processReceivedPackets( );
  
            /* transmit any data waiting in the module-local buffer */
            StreamTransmit( txSize );
        }
        
        /* we need to call the processCyclic function for all applications that
           have any data to send (without receiving a hid packet). The data to
           be sent is written into the module-local buffer */
        txSize = processCyclic( );
        
        /* transmit any data waiting in the module-local buffer */
        StreamTransmit( txSize );
    }
}

