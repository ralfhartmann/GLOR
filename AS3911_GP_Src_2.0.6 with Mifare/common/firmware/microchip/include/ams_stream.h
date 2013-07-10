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
 *      PROJECT:   AMS Streaming Communication 
 *      $Revision: $
 *      LANGUAGE: C++
 */

/*! \file ams_stream.h
 *
 *  \author M. Arpa
 *
 *  \brief Constants shared between GUI and firmware concerning the ams
 *         streaming communication.
 */

#ifndef AMS_STREAM_H
#define AMS_STREAM_H




/* This is an example describing the streaming communication between
   a PC Host and a microchip device using USB HID for transporting
   the streaming packets.

   If you do not use USB HID but some other protocol your byte stream
   will look different in point 3.
   
   Point 3. was added to show a full stack.


    =============================================================================

    From Host to Device:

    ------------------------------------------------------

    On the physical line the following byte-stream is seen: 

    Byte 0    1        2         3         4         5         6         7      8            (8 + tx-prot) (9+tx-prot) (10+tx-prot)                
    +------+-------+-------+----------+---------+---------+---------+---------+-------...---+-------------+-----------+-----------+-------
    | TID  |payload|re-    | protocol | tx-prot | tx-prot | rx-prot | rx-prot | data        | protocol B  | tx-prot B | tx-prot B |                
    |      |       | served|          |  MSB    |  LSB    |   MSB   |   LSB   |             |             |    MSB    |   LSB     |
    +------+-------+-------+----------+---------+---------+---------+---------+-------...---+-------------+-----------+-----------+-------

    TID = the transaction ID is changed with every report sent.
    payload = the amount of data in this report (sent from host to device) - a packet can be longer than
              this size (i.e. the packet is sent with several reports - note *)
    tx-prot = number of bytes transmitted (for this protocol packet) from host to device
    rx-prot = number ob bytes expected to be received (for this protocol packet) from device

    *) if a packet cannot be sent in one report the next report will not have the "protocol header" set again,
    but continue with the raw data. 


    This bytestream is build through the following 3 steps:

    ------------------------------------------------------

    1. The AmsComObject class provides a single data packet:

        Byte 0         1   ...                   tx-size -1
    +------------+-------- ... ----------------------------+
    |   data                                               |
    +--------------------- ... ----------------------------+


    ------------------------------------------------------

    2. The stream driver on the Host side takes a single data packet and adds the protocol header :

    Byte 0         1          2         3        4          5                                   4 + tx-size
    +------------+---------+---------+---------+---------+-------------------------- ... ------------------+
    | protocol   | tx-prot | tx-prot | rx-prot | rx-prot |  data                                           |
    |            |  MSB    |  LSB    |  MSB    |  LSB    |                                                 |
    +------------+---------+---------+---------+---------+-------------------------- ... ------------------+
    ^                                                    ^
    |                                                    |
    +----- this is the protocol header ------------------+

    ^                                                                                                      ^
    |                                                                                                      |
    +----this is given to the HID driver as a single packet -----------------------------------------------+
         

    Steps 1. and 2. can be repeated several times for sending more than 1 packet in a single HID report.

    ------------------------------------------------------

    3. The Hid driver on the Host side adds for each Hid Report (the following report header):

    a) if the data-buffer fits in one HID-report:

    Byte 0      1     2       3                 2+ payload         ....       63
    +------+-------+------+----------+---... +-------------+-------     -----------+
    | TID  |payload|res.  | packet(s)                      | padding if necessary  | 
    +------+-------+------+----------+---...-+-------------+-------.... -----------+
    ^                     ^
    |                     |
    +---------------------+
    this is the HID
     driver header

    b) if the packet(s) need more than 1 HID-report

    Byte 0      1     2       3                                    ....       63
    +------+-------+------+----------+---... +-------------+-------     -----------+
    | TID  |payload|res.  | packet(s)                                              | 
    |      | = 61  |      |                                                        |
    +------+-------+------+----------+---...-+-------------+-------.... -----------+
		           
    +------+-------+------+----------+---... +-------------+-------     -----------+
    | new  |payload|res.  | packet(s)                                              | 
    | TID  | = 61  |      |      continued                                         |
    +------+-------+------+----------+---...-+-------------+-------.... -----------+

    .... as many HID reports as needed until the packet(s) are totally
         transmitted (here is how the final report might look like):

    +------+-------+------+----------+---... +-------------+-------     -----------+
    | next |payload|res.  | packet(s)                      | padding if necessary  | 
    | TID  | <= 61 |      |      continued                 |                       |
    +------+-------+------+----------+---...-+-------------+-------.... -----------+


    */



/*  =============================================================================

    From Device to Host:

    ------------------------------------------------------

    On the physical line the following byte-stream is seen: 

    Byte 0    1        2         3         4         5         6         7      8            (8 + tx-prot) (9+tx-prot) (10+tx-prot)                
    +------+-------+-------+----------+---------+---------+---------+---------+-------...---+-------------+-----------+-----------+-------
    | TID  |payload|HID    | protocol | reserved| protocol| tx-prot | tx-prot | data        | protocol B  | reserved  | status  B |                
    |      |       | status|          |         |  status |   MSB   |  LSB    |             |             |           |
    +------+-------+-------+----------+---------+---------+---------+---------+-------...---+-------------+-----------+-----------+-------

    TID = the transaction ID is reflected th every report sent.
    payload = the amount of data in this report (sent from host to device) - a packet can be longer than
              this size (i.e. the packet is sent with several reports - note *)
    HID status = if a protocol was not processed (because e.g. the protocol id was unknown) the next HID
              packet that is sent back will contain a HID status byte <> 0 indicating that an error 
              occurred (some time in the back).
    protocol status = a single byte representing the result of the command if the command generated either data to
              be sent back or is a read command (has the MSBit cleared)
    tx-prot = number of bytes transmitted (for this protocol packet) from device to host

    *) if a packet cannot be sent in one report the next report will not have the "protocol header" set again,
    but continue with the raw data. 


    This bytestream is build by through the following 3 steps:

    ------------------------------------------------------

    1. The Firmware-Application provides a single data packet containing the result:

	        Byte 0         1   ...                   tx-size -1
    +------------+-------- ... ----------------------------+
    |   data                                               |
    +--------------------- ... ----------------------------+

	2. The function processReceivedPackets of file stream_appl_handler.c in the firmware adds the protocol byte,
       the reserved and the status byte and the tx-prot 16-bit word (from the information provided by the 
	   firmware application):

    Byte 0         1          2         3        4          5                                   4 + tx-size
    +------------+---------+---------+---------+---------+-------------------------- ... ------------------+
    | protocol   | reserved| status  | tx-prot | tx-prot |  data                                           |
    |            |         |         |  MSB    |  LSB    |                                                 |
    +------------+---------+---------+---------+---------+-------------------------- ... ------------------+
    ^                                                    ^
    |                                                    |
    +----- this is the protocol header ------------------+


    Steps 1. and 2. can be repeated several times before handing a single buffer (containing all packets) 
	to the HID driver.

    ------------------------------------------------------

    3. The Hid driver on the Device side splits the buffer into Hid Reports and adds for each Hid Report 
       (the following report header):

    a) if the data-buffer fits in one HID-report:

    Byte 0      1      3       4                 3+ payload         ....       63
    +------+-------+-------+----------+---... +-------------+-------     -----------+
    | TID  |payload|status | packet(s)                      | padding if necessary  | 
    +------+-------+-------+----------+---...-+-------------+-------.... -----------+
    ^                      ^
    |                      |
    +----------------------+
    this is the HID
     driver header

    b) if the packet(s) need more than 1 HID-report

    Byte 0      1     2       3                                    ....       63
    +------+-------+------+----------+---... +-------------+-------     -----------+
    | TID  |payload|status| packet(s)                                              | 
    |      | = 61  |      |                                                        |
    +------+-------+------+----------+---...-+-------------+-------.... -----------+
		           
    +------+-------+------+----------+---... +-------------+-------     -----------+
    | new  |payload|status| packet(s)                                              | 
    | TID  | = 61  |      |      continued                                         |
    +------+-------+------+----------+---...-+-------------+-------.... -----------+

    .... as many HID reports as needed until the packet(s) are totally
         transmitted (here is how the final report might look like):

    +------+-------+------+----------+---... +-------------+-------     -----------+
    | next |payload|status| packet(s)                      | padding if necessary  | 
    | TID  | <= 61 |      |      continued                 |                       |
    +------+-------+------+----------+---...-+-------------+-------.... -----------+

    */


/*  =============================================================================
    TID-Rules:

    On the host side, the TID is generated as a 4-bit number counting from 0 to 0x0F 
    and than rolling over to 0. 

    The device side takes the TID received from the Host and moves it to the upper
    nibble, increments its own TID counter by one (range is again 0 to 0x0F) and 
    adds its own TID counter.
        txTID = (rxTID << 4) | ( ++txTID & 0xF )

    */

/*  =============================================================================
    Protocol-Rules:

    The MSBit of the protocol byte defines the direction: 1 = write, 0 = read.
	This bit is set by the Ams Stream class itself during sending or receiving.
	
    The following number range is special:
	0x60 - 0x7F: reserved for generic commands (part of the common firmware)
	0x60: is a configuration protocol for the firmware itself 
	0x6B: is reserved for the bootloader
	0x7F: is reserved for the flush
	
	An application can use the numbers: 0x00 - 0x5F for its own commands.

    */



/* ------------- defines and macros ---------------------------------------- */

/* the stream adds a header to each packet of this size */    
#define AMS_STREAM_HEADER_SIZE          5 

/* macros to write the packet on the host side:  HT = Host Transmitter */
#define AMS_STREAM_HT_SET_PROTOCOL( buf, prot )    do { (buf)[0] = (prot); } while ( 0 )
#define AMS_STREAM_HT_SET_TX_LENGTH( buf, len )    do { (buf)[1] = (len) >> 8; (buf)[2] = (len) & 0xFF; } while ( 0 )
#define AMS_STREAM_HT_SET_RX_LENGTH( buf, len )    do { (buf)[3] = (len) >> 8; (buf)[4] = (len) & 0xFF; } while ( 0 )

/* macros to read the packet on the device side: DR = Device Receiver */
#define AMS_STREAM_DR_GET_PROTOCOL( buf )          ( (buf)[0] )
#define AMS_STREAM_DR_GET_RX_LENGTH( buf )         ( ( ((unsigned short)((buf)[1])) << 8 ) | (buf)[2] )
#define AMS_STREAM_DR_GET_TX_LENGTH( buf )         ( ( ((unsigned short)((buf)[3])) << 8 ) | (buf)[4] )

/* macros to write the packet on the device side: DT = Device Transmitter */
#define AMS_STREAM_DT_SET_PROTOCOL( buf, prot )    do { (buf)[0] = (prot); } while ( 0 )
#define AMS_STREAM_DT_SET_STATUS( buf, stat )      do { (buf)[1] = 0; (buf)[2] = (stat); } while ( 0 )
#define AMS_STREAM_DT_SET_TX_LENGTH( buf, len )    do { (buf)[3] = (len) >> 8; (buf)[4] = (len) & 0xFF; } while ( 0 )

/* macros to read the packet on the host side: HR = Host Receiver */
#define AMS_STREAM_HR_GET_PROTOCOL( buf )          ( (buf)[0] )
#define AMS_STREAM_HR_GET_STATUS( buf )            ( (buf)[2] )
#define AMS_STREAM_HR_GET_RX_LENGTH( buf )         ( (((unsigned short)((buf)[3])) << 8 ) | (buf)[4] )

/* the payload starts after the header */
#define AMS_STREAM_PAYLOAD( buf )               ( (buf) + AMS_STREAM_HEADER_SIZE )


/* maximum amount of data that can be transported in a single packet */
#define AMS_STREAM_MAX_DATA_SIZE   		(1024 + 64)	
              
/* the size of a buffer to hold at least one packet + header */
#define AMS_STREAM_BUFFER_SIZE                  ( AMS_STREAM_HEADER_SIZE + AMS_STREAM_MAX_DATA_SIZE )

/* all communication protocols that use the AmsComStream for communication
   must define their protocol identifier here */

/* if the MSB bit is set this a command for write, else this is for read */   
#define AMS_COM_WRITE_READ_NOT              0x80

/* protocol ids can range from 0x60 - 0x7f */
#define AMS_COM_CONFIG                      0x60 /* reserved */                            
#define AMS_COM_I2C                         0x61
#define AMS_COM_I2C_CONFIG                  0x62
#define AMS_COM_SPI                         0x63
#define AMS_COM_SPI_CONFIG                  0x64
#define AMS_COM_CTRL_CMD_RESET              0x65
#define AMS_COM_CTRL_CMD_FW_INFORMATION     0x66 /* returns zero-terminated string with information about fw e.g. chip, board */
#define AMS_COM_CTRL_CMD_FW_NUMBER          0x67 /* returns the 3-byte FW number */
#define AMS_COM_WRITE_REG                   0x68
#define AMS_COM_READ_REG                    0x69

/* 0x6B = reserved protocol id 
   This will become 0xEB at sending because it is: 
   AMS_COM_WRITE_READ_NOT | AMS_COM_CTRL_CMD_ENTER_BOOTLOADER == 0x80 | 0x6B = 0xEB */
#define AMS_COM_CTRL_CMD_ENTER_BOOTLOADER   0x6B 

/* 0x7F = reserved protocol id */
#define AMS_COM_FLUSH                       0x7F

/* currently available reserved numbers are: 0x6A and 0x6C - 0x7E */

/* all unused numbers between 0x00 and 0x5F are forwarded in the firmware (by the stream_dispatcher.c) 
   to the function
      u8 applProcessCmd ( u8 protocol, u16 rxSize, const u8 * rxData, u16 * txSize, u8 * txData )
   A weak default implementation of this function is provided in weak_stream_functions.c
   in the firmware that just flags an error. Implement this function in your
   firmware to override the default behavior. 
*/

 
/* -------- additional defines ------------------------------------------- */

#define AMS_COM_RESET_MCU                   0x01 /* to reset the MCU use this as the objectToReset parameter */
#define AMS_COM_RESET_PERIPHERAL            0x02 /* to reset all peripherals use this */
#define AMS_STREAM_SHORT_STRING             0x40 /* a const char * must not point to something longer than this */

/* one byte for errors in the dispatcher (and protocols if the protocol
   was a write request = unconfirmed by the protocol itself)
*/
#define AMS_STREAM_UNHANDLED_PROTOCOL 0x01 /* no function implemented to handle this protocol */
#define AMS_STREAM_PROTOCOL_FAILED    0x02 /* function call returned a status <> 0 - and this was an unconfirmed protocol */
#define AMS_STREAM_NO_ERROR           0x00 /* no error at all */

/* this define is used as a special TID used by the firmware when a request in the old format
   is received */
#define AMS_STREAM_COMPATIBILITY_TID  0xDE
 
/* -------- usb hid defines ---------------------------------------------- */

#define USB_HID_REPORT_ID               0
#define USB_HID_REPORT_SIZE             64  /** the USB-HID report size is 0x40 (refer to usb_descriptors.c). */
#define USB_HID_HEADER_SIZE             3   /** the PIC USB HID stack hides the report ID at position 0 */
#define USB_HID_MAX_PAYLOAD_SIZE        (USB_HID_REPORT_SIZE - USB_HID_HEADER_SIZE)

/* HID header macros for the microchip */
#define USB_HID_TID(buf)          ((buf)[0])
#define USB_HID_PAYLOAD_SIZE(buf) ((buf)[1])
#define USB_HID_STATUS(buf)       ((buf)[2])
#define USB_HID_PAYLOAD(buf)      ((buf)+USB_HID_HEADER_SIZE)

#define USB_HID_GENERATE_TID_FOR_TX( rxtid, txtid ) \
  do {                                             \
    (txtid)++;                                     \
    (txtid) = ( (rxtid) << 4 ) | ( (txtid) & 0xF );\
  } while ( 0 )


 
/* -------- uart defines ---------------------------------------------- */

#define UART_HEADER_SIZE                 4

/* uart header macros for the microchip */
#define UART_TID(buf)                     (buf[0])
#define UART_STATUS(buf)                  ((buf)[1])
#define UART_SET_PAYLOAD_SIZE( buf, len ) do { (buf)[2] = (len) >> 8; (buf)[3] = (len) & 0xFF; } while ( 0 )
#define UART_GET_PAYLOAD_SIZE( buf )      ( ( ((buf)[2]) << 8 ) | ( ((buf)[3]) & 0xFF ) )
#define UART_PAYLOAD(buf)                 ((buf)+UART_HEADER_SIZE)

#define UART_GENERATE_TID_FOR_TX( rxtid, txtid ) USB_HID_GENERATE_TID_FOR_TX( rxtid, txtid )


#endif /* AMS_STREAM_H */
