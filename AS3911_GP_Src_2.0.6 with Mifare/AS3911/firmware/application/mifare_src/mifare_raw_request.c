/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "as3911.h"
#include "as3911_com.h"
#include "delay.h"
#include "mifare_parity_data_t.h"
#include "mifare.h"
#include <string.h>
#include "logger.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static u8 mifareRawBuffer[((MIFARE_TRANSMIT_BUFFER_SIZE*9)+7)/8];

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static u16 mifareCopyToRawBuffer(const parity_data_t *message, u16 length);
static u16 mifareExtractMessage(u8* response, u16 responseLength);

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 mifareSendRawRequest(const parity_data_t *request,
        u16 requestLength,
        u8 *response,
        u16 maxResponseLength,
        u16 *responseLength,
        u16 timeout,
        bool_t fourBitResponse)
{
    s8 err = ERR_NONE;
    u16 len_bits;

    /* Setup receive operation. */
    err = as3911PrepareReceive(TRUE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* Mask data reception. */
    err = as3911ExecuteCommand(AS3911_CMD_MASK_RECEIVE_DATA);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    len_bits = mifareCopyToRawBuffer(request, requestLength);

    err = as3911TxNBytes(mifareRawBuffer, len_bits/8, len_bits%8, AS3911_TX_FLAG_NONE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    /* Receive response. */
    err = as3911RxNBytes(mifareRawBuffer, sizeof(mifareRawBuffer), responseLength, timeout);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    if (((*responseLength * 8) / 9) > maxResponseLength)
    {
        dbgLog("limiting l=%hx ,ml=%hx\n",*responseLength,maxResponseLength);
        *responseLength = maxResponseLength / 8 * 9;
        err = ERR_NONE; /* This will but an existing CRC */
    }
    *responseLength = mifareExtractMessage(response, *responseLength);

out:
    return err;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
static u16 mifareCopyToRawBuffer(const parity_data_t *message, u16 length)
{
    int i, bytepos = 0;
    int bitpos = 0;
    memset(mifareRawBuffer,0,sizeof(mifareRawBuffer));
    dbgLog("transmitting: ");
    for(i = 0; i<length; i++)
    {
        dbgLog("%hx,",message[i]);
    }
    dbgLog("\n");


    for (i = 0; i < length; i++)
    {
        u16 m = message[i];
        mifareRawBuffer[bytepos] |= (m & ((1<<(8 - bitpos))-1)) << bitpos;
        bytepos++;
        mifareRawBuffer[bytepos] |= (m >> (8-bitpos));

        bitpos += 1;
        if (bitpos >=8 )
        {
            bitpos -= 8;
            bytepos++;
        }
    }
    dbgLog("  raw: ");
    for ( i= 0; i< ((length*9)+7)/8;i++)
    {
        dbgLog("%hhx,",mifareRawBuffer[i]);
    }
    dbgLog("\n");
    return length*9;
}
static u16 mifareExtractMessage(u8* response, u16 responseLength)
{
    int bytes = responseLength * 8 / 9;
    int i, bytepos = 0;
    int bitpos = 0;
    dbgLog("extracting ");
    dbgHexDump(mifareRawBuffer,responseLength);
    if (responseLength==1)
    {
        response[0] = mifareRawBuffer[0];
        return 1;
    }
    for (i = 0; i < bytes; i++)
    {
        u8 m;
        m = (mifareRawBuffer[bytepos] >> bitpos);
        bytepos++;
        m |= (mifareRawBuffer[bytepos] << (8-bitpos));

        bitpos += 1;
        if (bitpos >=8 )
        {
            bitpos -= 8;
            bytepos++;
        }

        response[i] = m;
    }
    dbgLog(" extracted: ");
    dbgHexDump(response,bytes);
    return bytes;
}
