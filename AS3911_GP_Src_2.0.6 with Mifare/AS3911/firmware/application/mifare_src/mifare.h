#ifndef MIFARE_H
#define MIFARE_H

/*!
  \file
  Mifare Classic implmementation taken from Karsten Nohl, Henryk Ploetz and Sean O'Neil.
  This no complete implementation but shows that AS3911 is able to correctly talk to 
  Mifare Classic cards.
*/

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! Size of the intermal MiFare transmit buffer. FIXME: Should be reduced to 18 */
#define MIFARE_TRANSMIT_BUFFER_SIZE         64

#define MIFARE_AUTH_KEY_A                   0x60
#define MIFARE_AUTH_KEY_B                   0x61
#define MIFARE_READ_BLOCK                   0x30
#define MIFARE_WRITE_BLOCK                  0xA0
#define MIFARE_ACK                          0x0A
#define MIFARE_NACK_NOT_ALLOWED             0x04
#define MIFARE_NACK_TRANSMISSION_ERROR      0x04

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
s8 mifareInitialize();
s8 mifareDeinitialize(u8 keep_on);

/*!
 *****************************************************************************
 * Set the MiFare key.
 *
 * \note: The array pointed to by \a key must be at least 6 bytes long.
 *
 * \param key Pointer to an array storing the key.
 *****************************************************************************
 */
void mifareSetKey(const u8 *key);

/*!
 *****************************************************************************
 * \brief Reset the MiFare cipher.
 *
 * Reset the MiFare crypto1 engine and deactive encryption/decryption for
 * communications. This must be called when prior to communicating with a
 * new card.
 *****************************************************************************
 */
void mifareResetCipher();

/*!
 *****************************************************************************
 * \brief Perform a complete MiFare authentication.
 *
 * This function provides a convenient interface to MIFARE_authenticateStep1 and
 * MIFARE_authenticateStep2.
 * The reader nonce used in the second authentication step is defined by
 * MIFARE_DEFAULT_READER_NONCE.
 *
 * \param keySelect Select key for authentication. 0x60 selects key A and 0x61 selects key B.
 * \param block Block to authenticate for.
 * \param uid UID of the card we authenticate to.
 * \param uidLength Length of the UID pointed to by \a uid in bytes.
 * \param key Pointer to the key.
 *
 * \return 0 if the authentication step was successfull. Otherwise a negative error code is returend.
 *****************************************************************************
 */
s8 mifareAuthenticate(u8 key_select,
        u8 block,
        const u8 *uid,
        u8 uidLength,
        const u8 *key);

/*!
 *****************************************************************************
 * \brief First step of mifare authentication (reader side).
 *
 * \Note The array pointed to by \a uid must be at least 4 bytes long.
 *
 * \param keySelect Select key for authentication. 0x60 selects key A and
 *        0x61 selects key B.
 * \param block Block to authenticate for.
 * \param uid UID of the card we authenticate to.
 * \param uidLength Length of the UID pointed to by \a uid in bytes.
 * \param key Pointer to the key.
 *
 * \return ERR_NONE: The authentication step was successfull.
 * \return ERR_PARAM: UID too short.
 *****************************************************************************
 */
s8 mifareAuthenticateStep1(u8 key_select,
        u8 block,
        const u8 *uid,
        u8 uidLength,
        const u8 *key);

s8 mifareAuthenticateStep2(u32 readerNonce);

s8 mifareSendRequest(const u8 *request,
        u16 requestLength,
        u8 *response,
        u16 maxResponseLength,
        u16 *responseLength,
        u16 timeout,
        bool_t four_bit_response);

#endif /* MIFARE_H */
