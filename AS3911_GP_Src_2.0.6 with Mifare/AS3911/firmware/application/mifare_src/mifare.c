/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "debug.h"
#include "platform.h"
#include "as3911_com.h"
#include "../src/crc.h"
#include "mifare_crypto1.h"
#include "mifare.h"
#include "mifare_raw_request.h"
#include "iso14443a.h"
#include "logger.h"

/*
******************************************************************************
* MACROS
******************************************************************************
*/
#if 1
#define MIFARE_DEBUG dbgLog
#else
#define MIFARE_DEBUG(...)
#endif

/*
******************************************************************************
* DEFINES
******************************************************************************
*/
/*! Reader nonce used by the MiFare single step authentication code. */
#define MIFARE_DEFAULT_READER_NONCE             0xAA55AA55

/*! Timeout for mifare authentication step 1 in milliseconds. */
#define MIFARE_AUTHENTICATION_STEP1_TIMEOUT     10

/*! Timeout for mifare authentication step 2 in milliseconds. */
#define MIFARE_AUTHENTICATION_STEP2_TIMEOUT     10

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
/*!
 *****************************************************************************
 * Internal buffer for transmitted and received messages in the
 * partiy_data_t format.
 * This buffer is also used as intermediate storage buffer during encryption
 * and decryption.
 *****************************************************************************
 */
static parity_data_t transceive_buffer[MIFARE_TRANSMIT_BUFFER_SIZE];

/*! Global crypto1 cipher state object. */
static crypto1_state mifareCipherState;

/*!
 *****************************************************************************
 * If \a mifareCipherActive is \a TRUE, then a block authentication has been
 * successfully performed and all data transmission will be encrypted.
 *****************************************************************************
 */
static u8 mifareCipherActive;

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
u16 mifareCalculateIso14443aCrc(const u8 *buffer, u16 length);

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 mifareInitialize()
{
    s8 err = ERR_NONE;

    err = iso14443AInitialize();

    mifareCipherActive = FALSE;

    err |= as3911WriteRegister(AS3911_REG_ISO14443A_NFC, AS3911_REG_ISO14443A_NFC_no_tx_par
                                                       | AS3911_REG_ISO14443A_NFC_no_rx_par);
    return err;
}

s8 mifareDeinitialize(u8 keep_on)
{
    s8 err;

    err = iso14443ADeinitialize(keep_on);

    err |= as3911WriteRegister(AS3911_REG_ISO14443A_NFC, 0);

    return err;
}

void mifareSetKey(const u8 *key)
{
    s16 i;

    if (!crypto1_new(&mifareCipherState, CRYPTO1_CIPHER_READER, CRYPTO1_IMPLEMENTATION_CLEAN))
    {
        MIFARE_DEBUG("Initialization failed \n");
    }

    uint64emu_storage_t corrected_key;

    for (i = 0; i < 6; i++)
    {
        uint64emu_setbyte(&corrected_key, 5 - i, key[i]);
    }

    crypto1_init(&mifareCipherState, &corrected_key);

    MIFARE_DEBUG("Initialization ok\n");
}

void mifareResetCipher(void)
{
    AMS_MEMSET(&mifareCipherState, 0, sizeof(mifareCipherState));
    mifareCipherActive = FALSE;
}

s8 mifareAuthenticate(u8 keySelect,
        u8 block,
        const u8 *uid,
        u8 uidLength,
        const u8 *key)
{
    s8 err;

    err = mifareAuthenticateStep1(keySelect, block, uid, uidLength, key);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    err = mifareAuthenticateStep2(MIFARE_DEFAULT_READER_NONCE);

out:
    return err;
}

s8 mifareAuthenticateStep1(u8 keySelect,
        u8 block,
        const u8 *uid,
        u8 uidLength,
        const u8 *key)
{
    s8 err = ERR_NONE;
    u8 authenticationCommand[4];
    u8 authenticationResponse[5];
    u16 crc;
    size_t index;
    u16 numReceivedBytes;
    u32 uid_as_u32;
    u32 tag_nonce;

    authenticationCommand[0] = keySelect;
    authenticationCommand[1] = block;
    /* Append CRC. */
    crc = mifareCalculateIso14443aCrc(authenticationCommand, 2);
    authenticationCommand[2] = crc & 0xFF;
    authenticationCommand[3] = (crc >> 8) & 0xFF;

    /* Convert authenticate_command to parity_data_t. */
    for(index = 0; index < 4; index++)
        transceive_buffer[index] = authenticationCommand[index];

    calculateParity(transceive_buffer, 4);
  
    if(uidLength < 4)
    {
        MIFARE_DEBUG("!! UID too short. Abort.\n");

        return ERR_PARAM;
    }

    if(mifareCipherActive)
        crypto1_transcrypt(&mifareCipherState, transceive_buffer, 4);

    err = mifareSendRawRequest(transceive_buffer, 4, authenticationResponse
        , sizeof(authenticationResponse), &numReceivedBytes, MIFARE_AUTHENTICATION_STEP1_TIMEOUT, FALSE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    if(numReceivedBytes != 4)
    {
        MIFARE_DEBUG("!! ");
        MIFARE_DEBUG("Reveiced 0x%x byte and 0x%x bits", numReceivedBytes, 0);
        MIFARE_DEBUG(", expected 4 bytes and 0 bits. Abort.\n");
        dbgHexDump(authenticationResponse,numReceivedBytes);

        mifareResetCipher();
        return ERR_NOTFOUND;
    }
    dbgHexDump(authenticationResponse,numReceivedBytes);

    uid_as_u32 = ARRAY_TO_UINT32(uid);
    tag_nonce = ARRAY_TO_UINT32(authenticationResponse);

    if(mifareCipherActive)
    {
        mifareResetCipher();
        mifareSetKey(key);
        crypto1_clean_mutual_1_2(&mifareCipherState, uid_as_u32, tag_nonce);
    }
    else
    {
        mifareResetCipher();
        mifareSetKey(key);
        crypto1_mutual_1(&mifareCipherState, uid_as_u32, tag_nonce);
    }

    MIFARE_DEBUG("uid: %X%X\n", (u16) (uid_as_u32 >> 16), (u16) uid_as_u32);
    MIFARE_DEBUG("Card nonce: 0x%X%X\n", (u16) (tag_nonce >> 16), (u16) tag_nonce);
    MIFARE_DEBUG("Auth step 1 Successfull\n");

out:
    return err;
}

/*!
 *****************************************************************************
 * Perform second step of authentication (reader side) using a given random
 * number for the reader nonce.
 *
 * \param readerNonce Reader nonce to use during the authentication.
 *
 * \return ERR_NONE: The authentication step was successfull.
 * \return ERR_
 *****************************************************************************
 */
s8 mifareAuthenticateStep2(u32 readerNonce)
{
    s8 err = ERR_NONE;
    parity_data_t auth2Command[8];
    u8 auth2Response[5];
    u16 numReceivedBytes;
    umword index;

    UINT32_TO_ARRAY_WITH_PARITY( readerNonce, auth2Command);
    crypto1_mutual_2(&mifareCipherState, auth2Command);

    err = mifareSendRawRequest(auth2Command, 8, auth2Response
        , sizeof(auth2Response), &numReceivedBytes, MIFARE_AUTHENTICATION_STEP2_TIMEOUT, FALSE);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    if (numReceivedBytes != 4)
    {
        MIFARE_DEBUG("!! received 0x%x bytes, expected 4 bytes and 0 bits. Abort.\n", numReceivedBytes);

        mifareResetCipher();
        return ERR_NOTFOUND;
    }

    // Convert auth2_response to parity_data_t
    for(index = 0; index < 5; index++)
        transceive_buffer[index] = auth2Response[index];

    if(!crypto1_mutual_3(&mifareCipherState, transceive_buffer))
    {
        MIFARE_DEBUG("!! Invalid card response. Abort\n");

        mifareResetCipher();
        return ERR_NOMSG;
    }
  
    mifareCipherActive = TRUE;
    MIFARE_DEBUG("Authentication ok. Success\n");

out:
    return err;
}

/*!
 *****************************************************************************
 * \brief Send a request to a MiFare card.
 * 
 * Send a request to a MiFare card. If a sector has been authenticated for
 * then all data transmissions will be encrypted. If no authentication has
 * been performed yet then all data transmission is done unencrypted.
 * 
 * \note \a request and \a response always contain unencrypted data.
 * \note The CRC of the request is automatically calculated and appended
 * to the transmitted data by this routine. The CRC of the response is
 * included in the response data field and NOT checked by this function
 * as some MiFare responses do not contain a CRC.
 * \note \a four_bit_response is ignored right now.
 * \note If a four bit frame is received then the number of received bytes
 * returned will be zero and no data will be stored in the receive buffer.
 *
 * \param request Pointer to a buffer holding the request.
 * \param requestLength Length of the request in bytes.
 * \param response Pointer to the buffer used for storing the response.
 * \param maxResponseLength Maximum Length of the response buffer.
 * \param responseLength Length of the received response.
 * \param timeout Timeout in microseconds.
 * \param fourBitResponse If this is \a TRUE, then a 4 bit response is
 * expected.
 *
 * \return ERR_NONE: 
 *****************************************************************************
 */
s8 mifareSendRequest(const u8 *request, u16 requestLength
    , u8 *response, u16 maxResponseLength, u16 *responseLength, u16 timeout, bool_t fourBitResponse)
{
    s8 err = ERR_NONE;
    size_t index;
    u16 crc;

    /* Copy request into parity_data_t transmit buffer. */
    for(index = 0; index < requestLength; index++)
    {
        transceive_buffer[index] = request[index];
    }

    /* Append CRC. */
    crc = mifareCalculateIso14443aCrc(request, requestLength);
    transceive_buffer[requestLength] = ((u8) (crc & 0xFF));
    transceive_buffer[requestLength+1] = ((u8) ((crc>>8) & 0xFF));

    // Calculate Parity
    calculateParity(&transceive_buffer[0], requestLength+2);

    // Encrypt if cipher is in use
    if(mifareCipherActive)
        crypto1_transcrypt(&mifareCipherState, transceive_buffer, requestLength+2);
  
    err = mifareSendRawRequest(transceive_buffer, requestLength+2
        , response, maxResponseLength, responseLength, timeout, fourBitResponse);
    dbgLog("mifareSendRequest[%hhx]\n",err);
    EVAL_ERR_NE_GOTO(ERR_NONE, err, out);

    // Copy response into transceive buffer for decryption.
    for(index = 0; index < *responseLength; index++)
        transceive_buffer[index] = response[index];
  
    // Decrypt message in transceive buffer if cipher is in use.
    if(mifareCipherActive)
    {
        // If a response with a length of 0 or 1 byte is received it is asumed that this
        // actually was an ACK,NACK and 4 bits are fed into the cipher
        if((*responseLength == 0) || (*responseLength == 1))
        {
            // The AS3911 stores the 4 bit response in the high nibble but the
            // crypto1 implementation expects it in the low nibble. We also need
            // to reverse the bit order.

            transceive_buffer[0] >>= 4;
            // Reverse bit order of the low nibble.
            /*transceive_buffer[1] = 0;
            for(unsigned int index = 0; index < 4; index++)
            {
                transceive_buffer[1] <<= 1;

                if(transceive_buffer[0] & 0x01)
                    transceive_buffer[1] & 0x01;

                transceive_buffer[0] >>= 1;
            }
            transceive_buffer[0] = transceive_buffer[1];*/

            crypto1_transcrypt_bits(&mifareCipherState, &transceive_buffer[0], 0, 4);
        }
        else
            crypto1_transcrypt_bits(&mifareCipherState, transceive_buffer, *responseLength, 0);
    }

    /* Copy decrypted message back into response buffer. */
    /* ToDo: Check why not only numReceivedBytes bytes are copied. */
    for(index = 0; index < *responseLength; index++)
        response[index] = (unsigned char) transceive_buffer[index];

out:
    return err;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

/*
 *****************************************************************************
 * \brief Calculate the ISO14443A CRC of a message.
 *
 * \param[in] buffer : Buffer holding the message.
 * \param[in] length : Length of the message in bytes.
 *
 * \return ISO14443A CRC of the message stored in the buffer.
 *****************************************************************************
 */
u16 mifareCalculateIso14443aCrc(const u8 *buffer, u16 length)
{
    return crcCalculateCcitt(0x6363, buffer, length);
}
