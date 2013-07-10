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
 *  \brief Implementation of ISO-15693-2
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "iso15693_2.h"
#include "as3911_stream.h"
#include "as3911.h"
#include "as3911_com.h"
#include "crc.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/

//#define ISO_15693_DEBUG dbgLog
#define ISO_15693_DEBUG(...)

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define ISO15693_DAT_SOF_1_4     0x21 /* LSB constants */
#define ISO15693_DAT_EOF_1_4     0x04
#define ISO15693_DAT_00_1_4      0x02
#define ISO15693_DAT_01_1_4      0x08
#define ISO15693_DAT_10_1_4      0x20
#define ISO15693_DAT_11_1_4      0x80

#define ISO15693_DAT_SOF_1_256   0x81
#define ISO15693_DAT_EOF_1_256   0x04
#define ISO15693_DAT_SLOT0_1_256 0x02
#define ISO15693_DAT_SLOT1_1_256 0x08
#define ISO15693_DAT_SLOT2_1_256 0x20
#define ISO15693_DAT_SLOT3_1_256 0x80

#define ISO15693_PHY_DAT_MANCHESTER_1 0xaaaa

#define ISO15693_PHY_BIT_BUFFER_SIZE 1000 /*!< 
                                size of the receiving buffer. Might be adjusted
                                if longer datastreams are expected. */


/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static u8 iso15693PhyTransbuf[64]; /*!< 
                            buffer used for sending coded data */
static u8 iso15693PhyBitBuffer[ISO15693_PHY_BIT_BUFFER_SIZE]; /*!< 
                         buffer used for receiving data in stream mode.
                        Might be adjusted if longer datastreams are expected. */
static iso15693PhyConfig_t iso15693PhyConfig; /*!< 
                                                 current phy configuration */

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static s8 iso15693PhySend1Of4(bool_t sendSof, const u8* buffer, u16 length);
static s8 iso15693PhySend1Of256(bool_t sendSof, const u8* buffer, u16 length);
static struct as3911StreamConfig stream_config = {
    .useBPSK = 0, /* 0: subcarrier, 1:BPSK */
    .din = 5, /* 2^5*fc = 423750 Hz: divider for the in subcarrier frequency */
    .dout = 7, /*!< 2^7*fc = 105937 : divider for the in subcarrier frequency */
    .report_period_length = 3, /*!< 8=2^3 the length of the reporting period */
};

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 iso15693PhyInitialize(const iso15693PhyConfig_t* config)
{
    s8 err;
    u8 value;

    /* make a copy of the configuration */
    AMS_MEMCPY(&iso15693PhyConfig, (u8*)config, sizeof(iso15693PhyConfig_t));

    if (ISO15693_MODULATION_INDEX_OOK == config->mi)
    {
        /* enable ook */
        value = 0;
    }
    else
    {
        /* enable am */
        value = AS3911_REG_AUX_tr_am;
        /* set modulation depth */
        as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONTROL, ((u8)config->mi));
    }
    /* write effective settings to register */
    as3911ModifyRegister(AS3911_REG_AUX, AS3911_REG_AUX_tr_am, value);

    as3911ModifyRegister(AS3911_REG_RX_CONF1, 0x3f, 0x0c);

    /* finally, execute calibrate modulation depth */
    as3911ExecuteCommand(AS3911_CMD_CALIBRATE_MODULATION);

    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, ISO15693_MASK_RECEIVE_TIME);
    as3911SetNoResponseTime_64fcs(ISO15693_NO_RESPONSE_TIME);

    err = as3911StreamInitialize(&stream_config);

    return err;
}

s8 iso15693PhyDeinitialize(u8 keep_on)
{
    return as3911StreamDeinitialize(keep_on);
}

s8 iso15693PhyGetConfiguration(iso15693PhyConfig_t* config)
{
    AMS_MEMCPY(config, &iso15693PhyConfig, sizeof(iso15693PhyConfig_t));

    return ERR_NONE;
}

s8 iso15693PhySetNoResponseTime_64fcs(u32 nrt_64fcs)
{
    return as3911SetNoResponseTime_64fcs(nrt_64fcs);
}

s8 iso15693PhySendEof()
{
    s8 err;
    u8 eof;

    if (ISO15693_VCD_CODING_1_4 == iso15693PhyConfig.coding)
    {
        eof = ISO15693_DAT_EOF_1_4;
    }
    else
    {
        eof = ISO15693_DAT_EOF_1_256;
    }
    as3911StreamTransferPrepare(8);
    err = as3911StreamTxNBytes(&eof, 1);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
out:
    return err;
}

s8 iso15693PhySendFrame(u8* buffer, u16 length, bool_t sendCrc, bool_t sendFlags)
{
    s8 err = ERR_NONE;
    u8 eof;
    u8 transbuf[2];
    u16 crc = 0;
    s8 (*txFunc)(bool_t, const u8*, u16);
    u8 crc_len = ((sendCrc)?2:0);

    if (ISO15693_VCD_CODING_1_4 == iso15693PhyConfig.coding)
    {
        eof = ISO15693_DAT_EOF_1_4;
        txFunc = iso15693PhySend1Of4;
        as3911StreamTransferPrepare( 8 *
                ( 1  /* SOF */
                  + (length + crc_len) * 4 
                  + 1) /* EOF */
                );
    }
    else
    {
        eof = ISO15693_DAT_EOF_1_256;
        txFunc = iso15693PhySend1Of256;
        as3911StreamTransferPrepare( 8 *
                ( 1  /* SOF */
                  + (length + crc_len) * 64 
                  + 1) /* EOF */
                );
    }

    if (sendFlags)
    {
        /* set high datarate flag */
        buffer[0] |= ISO15693_REQ_FLAG_HIGH_DATARATE;
        /* clear sub-carrier flag - we only support single sub-carrier */
        buffer[0] &= ~ISO15693_REQ_FLAG_TWO_SUBCARRIERS;
    }

    if (sendCrc)
    {
        crc = ~crcCalculateCcitt(0xffff, buffer, length);
    }

    /* send data including SOF */
    err = txFunc(TRUE, buffer, length);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    if (sendCrc)
    {
        /* send crc */
        transbuf[0] = crc & 0xff;
        transbuf[1] = (crc >> 8) & 0xff;
        err = txFunc(FALSE, transbuf, 2);
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
    }

    err = as3911StreamTxNBytes(&eof, 1);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

out:
    return err;
}

s8 iso15693PhyReceiveFrame(u8* buffer,
                        u16 length,
                        u16* actLength,
                        u8* bitsBeforeCol)
{
    s8 err;
    u16 al, crc;
    u16 mp; /* Current bit position in manchester bit buffer */
    u16 bp; /* Current bit postion in buffer */

    *bitsBeforeCol = 0;
    *actLength = 0;

    err = as3911StreamRxNBytesCont(iso15693PhyBitBuffer, ISO15693_PHY_BIT_BUFFER_SIZE, &al);
    as3911StreamRxStop();
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* first check for valid SOF. Since it starts with 3 unmodulated pulses it is 0x17. */
    if ((iso15693PhyBitBuffer[0] & 0x1f) != 0x17)
    {
        ISO_15693_DEBUG("0x%x\n", iso15693PhyBitBuffer[0]);
        err = ERR_NOTFOUND;
        goto out;
    }
    ISO_15693_DEBUG("SOF\n");

    if (!length)
    {
        goto out;
    }

    mp = 5; /* 5 bits were SOF, now manchester starts: 2 bits per payload bit */
    bp = 0;

    memset(buffer,0,length);

    for ( ; mp < al * 8 - 2; mp+=2 )
    {
        u8 man;
        man  = (iso15693PhyBitBuffer[mp/8] >> mp%8) & 0x1;
        man |= ((iso15693PhyBitBuffer[(mp+1)/8] >> (mp+1)%8) & 0x1) << 1;
        if (1 == man)
        {
            bp++;
        }
        if (2 == man)
        {
            buffer[bp/8] |= 1 << (bp%8);
            bp++;
        }
        if (0 == man)
        {
                ISO_15693_DEBUG("COLLISION!\n");
                err = ERR_COLLISION;
                goto out;
        }
        if (3 == man)
        { 
            if (0 == bp)
            {
                ISO_15693_DEBUG("premature!\n");
                err = ERR_COLLISION;
                break;
            }
            if (bp%8 != 1)
            { /* EOF has to start at byte boundary, 11 would be at bit 1. */
                ISO_15693_DEBUG("EOF not on boundary!\n");
                err = ERR_COLLISION;
                break;
            }
            /* check if this was EOF */
            if ((buffer[(bp-1)/8] & (1<<((bp-1)%8))))
            {
                ISO_15693_DEBUG("previous payload bit not 1 -> no EOF!\n");
                err = ERR_COLLISION;
                break;
            } /* Now we know that it was a 0 : 10 followed by 11 which may be the start of EOF*/
            if (((iso15693PhyBitBuffer[(mp+2)/8] >> (mp+2)%8) & 0x1))
            { /* another 1, building 10111==EOF */
                ISO_15693_DEBUG("EOF\n");
                bp--; /* Last bit was already part of EOF */
                break;
            }
            err = ERR_COLLISION;
            break;
        }
    }

    if (bp%8 != 0)
    {
        err = ERR_CRC;
        goto out;
    }

    *actLength = bp / 8;
    *bitsBeforeCol = bp % 8;

    if (ERR_COLLISION == err) goto out;

    if (*actLength > 2)
    {
        /* finally, check crc */
        ISO_15693_DEBUG("Calculate CRC, val: 0x%x, length: ", *buffer);
        ISO_15693_DEBUG("0x%x ", *actLength - 2);
        crc = ~crcCalculateCcitt(0xffff, buffer, *actLength - 2);
        if (((crc & 0xff) == buffer[*actLength-2]) &&
                (((crc >> 8) & 0xff) == buffer[*actLength-1]))
        {
            ISO_15693_DEBUG("OK\n");
            *actLength -= 2;
        }
        else
        {
            ISO_15693_DEBUG("error! Expected: 0x%x, got ", crc);
            ISO_15693_DEBUG("0x%hhx 0x%hhx\n", buffer[*actLength-2], buffer[*actLength-1]);
            err = ERR_CRC;
        }
    }
    else
    {
        err = ERR_CRC;
    }
out:
    return err;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Perform 1 of 4 coding and send coded data
 *
 *  This function takes \a length bytes from \a buffer, perform 1 of 4 coding
 *  (see ISO15693-2 specification) and sends the data using stream mode.
 *
 *  \param[in] sendSof : send SOF prior to data.
 *  \param[in] buffer : data to send.
 *  \param[in] length : number of bytes to send.
 *
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
static s8 iso15693PhySend1Of4(bool_t sendSof, const u8* buffer, u16 length)
{
    u16 i;
    u8 tmp;
    s8 err = ERR_NONE;
    u8 sof = ISO15693_DAT_SOF_1_4;
    bool_t first = sendSof;
    umword a;

    for (i = 0; i < length; i++)
    {
        tmp = *buffer;
        for (a = 0; a < 4; a++)
        {
            switch (tmp & 0x3)
            {
                case 0:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_00_1_4;
                    break;
                case 1:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_01_1_4;
                    break;
                case 2:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_10_1_4;
                    break;
                case 3:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_11_1_4;
                    break;
            }
            tmp >>= 2;
        }
        buffer++;
        if (first)
        {
            /* send SOF first! */
            err = as3911StreamTxNBytes(&sof, 1);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
            first = FALSE;
        }
        err = as3911StreamTxNBytes(iso15693PhyTransbuf, 4);
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
    }
out:
    return err;
}

/*! 
 *****************************************************************************
 *  \brief  Perform 1 of 256 coding and send coded data
 *
 *  This function takes \a length bytes from \a buffer, perform 1 of 256 coding
 *  (see ISO15693-2 specification) and sends the data using stream mode.
 *  \note This function sends SOF prior to the data.
 *
 *  \param[in] sendSof : send SOF prior to data.
 *  \param[in] buffer : data to send.
 *  \param[in] length : number of bytes to send.
 *
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
static s8 iso15693PhySend1Of256(bool_t sendSof, const u8* buffer, u16 length)
{
    u16 i;
    u8 tmp;
    u8 sof = ISO15693_DAT_SOF_1_256;
    bool_t first = sendSof;
    s8 err = ERR_NONE;
    umword a;

    for (i = 0; i < length; i++)
    {
        tmp = *buffer;
        for (a = 0; a < 64; a++)
        {
            switch (tmp)
            {
                case 0:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_SLOT0_1_256;
                    break;
                case 1:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_SLOT1_1_256;
                    break;
                case 2:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_SLOT2_1_256;
                    break;
                case 3:
                    iso15693PhyTransbuf[a] = ISO15693_DAT_SLOT3_1_256;
                    break;
                default:
                    iso15693PhyTransbuf[a] = 0;
            }
            tmp -= 4;
        }
        if (first)
        {
            /* send SOF first! */
            err = as3911StreamTxNBytes(&sof, 1);
            EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
            first = FALSE;
        }

        /* send out the coded byte */
        err = as3911StreamTxNBytes(iso15693PhyTransbuf, 64);
        EVAL_ERR_NE_GOTO(ERR_NONE, err, out);
        buffer++;

    }
out:
    return err;
}

